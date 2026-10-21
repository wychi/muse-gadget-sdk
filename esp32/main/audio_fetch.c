/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "audio_fetch.h"
#include "noise_control.h"
#include "sdkconfig.h"

#if CONFIG_MUSE_ENABLED && CONFIG_MUSE_HATCH

#include <ctype.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "link.audio";

#define FETCH_STACK_BYTES 6144
// For the whole fetch: a VM that trickles ranges would otherwise hold the
// task, its buffers and the busy flag indefinitely.
#define FETCH_DEADLINE_MS 60000
#define VM_POLL_MS        20
#define VM_PATH_PREFIX    "/fs/raw/"
// A file from the VM is read in Range requests of at most
// noise_ctrl_req_max_body() bytes into one buffer: the daemon may send a
// reply in one frame, which must fit the session's receive buffers.
#if CONFIG_SPIRAM
#define VM_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#else
#define VM_CAPS MALLOC_CAP_8BIT
#endif

typedef struct {
    char *url;  // the request path, GET /fs/raw/<path>
    audio_fetch_done_cb done;
    void *user;
    int64_t deadline_us;
    uint8_t *body;        // one range, chunk bytes
    size_t chunk;
    uint8_t *mp3;         // the whole file, AUDIO_FETCH_MAX_BYTES
    size_t len;           // bytes in mp3; also the next range's offset
    bool eof;
    bool timed_out;
    bool too_large;       // the file passed AUDIO_FETCH_MAX_BYTES
} fetch_t;

static atomic_bool s_busy;

// The reply to the one GET /fs/raw in flight, written into the fetch's range
// buffer on the Noise session task. Frames of an earlier request, cancelled
// or timed out, carry an older generation and are dropped, so the fetch task
// never waits on them and they never touch the buffer. The same shape as
// image_fetch's s_vm, separate so the two fetches can't cross replies.
static struct {
    SemaphoreHandle_t lock;
    uint32_t gen;
    uint8_t *buf;        // set only while a request is in flight
    size_t cap, len;
    int status;          // HTTP status, or a noise_ctrl_req_cb failure below 0
    bool done;
    bool complete;       // the whole body arrived
    bool too_large;      // more than was asked for
} s_req;

// How one request for a range ended.
typedef struct {
    size_t len;
    int status;
    bool complete, too_large, not_sent;
} req_reply_t;

static void req_on_frame(void *ctx, int status, const uint8_t *data, size_t len, bool end) {
    xSemaphoreTake(s_req.lock, portMAX_DELAY);
    if ((uint32_t)(uintptr_t)ctx == s_req.gen && !s_req.done) {
        if (status) s_req.status = status;
        // Only a 206's body is the range, or a 200's the whole file.
        bool ok = true;
        if (len && (s_req.status == 200 || s_req.status == 206)) {
            if (len > s_req.cap - s_req.len) {
                s_req.too_large = true;
                ok = false;
            } else {
                memcpy(s_req.buf + s_req.len, data, len);
                s_req.len += len;
            }
        }
        s_req.complete = end && status >= 0 && ok;
        s_req.done = end || status < 0 || !ok;
    }
    xSemaphoreGive(s_req.lock);
}

static bool req_done(void) {
    xSemaphoreTake(s_req.lock, portMAX_DELAY);
    bool done = s_req.done;
    xSemaphoreGive(s_req.lock);
    return done;
}

// GETs the next f->chunk bytes into f->body and waits for the whole reply or
// the deadline.
static void req_get_range(fetch_t *f, req_reply_t *out) {
    memset(out, 0, sizeof(*out));
    char range[48];
    snprintf(range, sizeof(range), "bytes=%u-%u", (unsigned)f->len,
             (unsigned)(f->len + f->chunk - 1));
    const char *headers[] = {"Range", range, NULL};
    xSemaphoreTake(s_req.lock, portMAX_DELAY);
    uint32_t token = ++s_req.gen;
    s_req.buf = f->body;
    s_req.cap = f->chunk;
    s_req.len = 0;
    s_req.status = 0;
    s_req.done = s_req.complete = s_req.too_large = false;
    xSemaphoreGive(s_req.lock);

    int64_t id = noise_ctrl_req_open("GET", f->url, headers, true, req_on_frame,
                                     (void *)(uintptr_t)token);
    while (id && !req_done()) {
        if (esp_timer_get_time() >= f->deadline_us) {
            f->timed_out = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(VM_POLL_MS));
    }

    xSemaphoreTake(s_req.lock, portMAX_DELAY);
    s_req.gen++;  // drop anything still to come for this request
    s_req.buf = NULL;
    out->len = s_req.len;
    out->status = s_req.status;
    out->complete = s_req.complete;
    out->too_large = s_req.too_large;
    xSemaphoreGive(s_req.lock);
    out->not_sent = !id;
    if (id && !out->complete) noise_ctrl_req_cancel(id);
}

// Appends the next range to f->mp3. False with `msg` set if it can't.
static bool fetch_next_range(fetch_t *f, char *msg, size_t msg_len) {
    req_reply_t r;
    req_get_range(f, &r);
    if (!r.complete) {
        if (r.not_sent) {
            snprintf(msg, msg_len, "could not send the request to the VM");
        } else if (f->timed_out) {
            snprintf(msg, msg_len, "download timed out");
        } else if (r.too_large) {
            snprintf(msg, msg_len, "the VM ignored Range and sent the whole file");
        } else if (r.status == NOISE_CTRL_REQ_TOO_LARGE) {
            snprintf(msg, msg_len, "the VM sent a frame too large for this board");
        } else if (r.status < 0) {
            snprintf(msg, msg_len, "the VM connection dropped");
        } else {
            snprintf(msg, msg_len, "VM answered HTTP %d", r.status);
        }
        return false;
    }
    if (r.status == 416) {
        // Starts at the end: the last range was full, or the file is empty.
        f->eof = true;
        return true;
    }
    // A daemon ignoring Range sends the whole file, fine if it fits.
    if (r.status != 206 && !(r.status == 200 && f->len == 0)) {
        snprintf(msg, msg_len, "VM answered HTTP %d", r.status);
        return false;
    }
    if (r.len > AUDIO_FETCH_MAX_BYTES - f->len) {
        f->too_large = true;
        snprintf(msg, msg_len, "file is larger than %u bytes",
                 (unsigned)AUDIO_FETCH_MAX_BYTES);
        return false;
    }
    memcpy(f->mp3 + f->len, f->body, r.len);
    f->len += r.len;
    if (r.status == 200 || r.len < f->chunk) f->eof = true;
    return true;
}

static void fetch_task(void *arg) {
    fetch_t *f = (fetch_t *)arg;
    int64_t start = esp_timer_get_time();
    f->deadline_us = start + (int64_t)FETCH_DEADLINE_MS * 1000;
    ESP_LOGI(TAG, "GET %s, %u bytes at a time", f->url, (unsigned)f->chunk);

    audio_fetch_result_t result = {0};
    char msg[64];
    while (!f->eof) {
        if (!fetch_next_range(f, msg, sizeof(msg))) break;
    }
    if (f->eof && f->len) {
        result.ok = true;
    } else if (f->eof) {
        result.code = "fetch_failed";
        result.message = "the file is empty";
    } else if (f->too_large) {
        result.code = "too_large";
        result.message = msg;
    } else {
        result.code = "fetch_failed";
        result.message = msg;
    }
    result.bytes = f->len;
    result.ms = (int)((esp_timer_get_time() - start) / 1000);
    ESP_LOGI(TAG, "audio download %s: %u bytes in %d ms, stack %u free",
             result.ok ? "done" : result.message, (unsigned)result.bytes,
             result.ms, (unsigned)uxTaskGetStackHighWaterMark(NULL));
    uint8_t *mp3 = NULL;
    size_t mp3_len = 0;
    if (result.ok) {
        mp3 = f->mp3;
        mp3_len = f->len;
        f->mp3 = NULL;  // the callback takes it
    }
    f->done(&result, mp3, mp3_len, f->user);
    free(f->mp3);
    free(f->body);
    free(f->url);
    free(f);
    atomic_store(&s_busy, false);
    vTaskDelete(NULL);
}

// GET /fs/raw/ and the path, without its leading slashes, percent-encoded
// but for its slashes. NULL if out of memory. (image_fetch's vm_request_path,
// duplicated: it is static there.)
static char *vm_request_path(const char *path) {
    while (*path == '/') path++;
    size_t prefix = strlen(VM_PATH_PREFIX);
    char *out = malloc(prefix + strlen(path) * 3 + 1);
    if (!out) return NULL;
    memcpy(out, VM_PATH_PREFIX, prefix);
    char *p = out + prefix;
    for (const unsigned char *c = (const unsigned char *)path; *c; c++) {
        if (isalnum(*c) || strchr("-._~/", *c)) {
            *p++ = (char)*c;
        } else {
            *p++ = '%';
            *p++ = "0123456789ABCDEF"[*c >> 4];
            *p++ = "0123456789ABCDEF"[*c & 15];
        }
    }
    *p = '\0';
    return out;
}

bool audio_fetch_start_vm(const char *path, audio_fetch_done_cb done,
                          void *user, const char **code, const char **message) {
    const char *name = path + strspn(path, "/");
    if (!*name) {
        *code = "invalid_params";
        *message = "path names no file";
        return false;
    }
    bool expected = false;
    if (!atomic_compare_exchange_strong(&s_busy, &expected, true)) {
        *code = "busy";
        *message = "another audio file is still downloading";
        return false;
    }
    if (!s_req.lock && !(s_req.lock = xSemaphoreCreateMutex())) {
        atomic_store(&s_busy, false);
        *code = "out_of_memory";
        *message = "failed to allocate";
        return false;
    }

    fetch_t *f = calloc(1, sizeof(*f));
    if (f) {
        f->url = vm_request_path(path);
        f->chunk = noise_ctrl_req_max_body();
        if (f->chunk > AUDIO_FETCH_MAX_BYTES) f->chunk = AUDIO_FETCH_MAX_BYTES;
        f->body = heap_caps_malloc(f->chunk, VM_CAPS);
        f->mp3 = heap_caps_malloc(AUDIO_FETCH_MAX_BYTES, VM_CAPS);
    }
    if (!f || !f->url || !f->body || !f->mp3) {
        if (f) {
            free(f->mp3);
            free(f->body);
            free(f->url);
        }
        free(f);
        atomic_store(&s_busy, false);
        *code = "out_of_memory";
        *message = "failed to allocate";
        return false;
    }
    f->done = done;
    f->user = user;
    if (xTaskCreate(fetch_task, "audio", FETCH_STACK_BYTES, f, 4, NULL) != pdPASS) {
        free(f->mp3);
        free(f->body);
        free(f->url);
        free(f);
        atomic_store(&s_busy, false);
        *code = "out_of_memory";
        *message = "failed to start the download task";
        return false;
    }
    return true;
}

#else

bool audio_fetch_start_vm(const char *path, audio_fetch_done_cb done,
                          void *user, const char **code, const char **message) {
    (void)path;
    (void)done;
    (void)user;
    *code = "unsupported";
    *message = "no Muse voice session on this board";
    return false;
}

#endif
