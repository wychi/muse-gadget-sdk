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

#include "boot_diary.h"

#include "sdkconfig.h"

#if CONFIG_HOMEHUB_BOOT_DIARY

#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_attr.h"
#include "esp_log_write.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define BOOT_DIARY_CAPACITY 4096
#define BOOT_DIARY_MAGIC 0xD1A6B007u
#define BOOT_DIARY_RECORD_BYTES 512

/* The whole store lives in RTC no-init memory: written once per log
 * line, read back by the next boot. Validation is deliberately light
 * (magic pair + a mirrored boot counter + clamped offsets): a crash
 * mid-append can garble a few tail bytes, but it must not make the
 * next boot throw the diary away. */
typedef struct {
    uint32_t magic;
    uint32_t magic_inv;
    uint32_t boots;
    uint32_t boots_inv;
    uint32_t head; /* offset of the oldest byte held */
    uint32_t len;  /* bytes held */
    char buf[BOOT_DIARY_CAPACITY];
} boot_diary_store_t;

static RTC_NOINIT_ATTR boot_diary_store_t s_store;

static SemaphoreHandle_t s_lock = NULL;
static _Atomic(vprintf_like_t) s_previous_vprintf = ATOMIC_VAR_INIT(vprintf);
static atomic_bool s_installed = ATOMIC_VAR_INIT(false);

static const char *reset_name(esp_reset_reason_t reason) {
    switch (reason) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "UNKNOWN";
    }
}

static bool store_valid(void) {
    return s_store.magic == BOOT_DIARY_MAGIC
           && s_store.magic_inv == ~BOOT_DIARY_MAGIC
           && s_store.boots_inv == ~s_store.boots
           && s_store.head < BOOT_DIARY_CAPACITY
           && s_store.len <= BOOT_DIARY_CAPACITY;
}

/* Caller holds s_lock (or is boot_diary_init before the sink is live). */
static void diary_append_locked(const char *data, size_t len) {
    if (len == 0) return;
    if (len > BOOT_DIARY_CAPACITY) {
        data += len - BOOT_DIARY_CAPACITY;
        len = BOOT_DIARY_CAPACITY;
    }
    if (s_store.len + len > BOOT_DIARY_CAPACITY) {
        uint32_t drop = (uint32_t)(s_store.len + len - BOOT_DIARY_CAPACITY);
        s_store.head = (s_store.head + drop) % BOOT_DIARY_CAPACITY;
        s_store.len -= drop;
    }
    uint32_t tail = (s_store.head + s_store.len) % BOOT_DIARY_CAPACITY;
    size_t first = BOOT_DIARY_CAPACITY - tail;
    if (first > len) first = len;
    memcpy(&s_store.buf[tail], data, first);
    if (first < len) memcpy(&s_store.buf[0], data + first, len - first);
    s_store.len += (uint32_t)len;
}

/* Same record shape as the support log (main/diagnostic_log.c): one
 * line per record, control characters flattened, 512-byte cap. */
static size_t normalize_record(char record[BOOT_DIARY_RECORD_BYTES],
                               int formatted_len) {
    if (formatted_len <= 0) return 0;
    size_t len = (size_t)formatted_len;
    if (len >= BOOT_DIARY_RECORD_BYTES) len = BOOT_DIARY_RECORD_BYTES - 1;
    while (len > 0 && (record[len - 1] == '\n' || record[len - 1] == '\r')) {
        len--;
    }
    if (len > BOOT_DIARY_RECORD_BYTES - 2) len = BOOT_DIARY_RECORD_BYTES - 2;
    for (size_t i = 0; i < len; i++) {
        unsigned char c = (unsigned char)record[i];
        if (c == '\r' || c == '\n') {
            record[i] = ' ';
        } else if ((c < 0x20 && c != '\t') || c == 0x7f) {
            record[i] = '.';
        }
    }
    record[len++] = '\n';
    record[len] = '\0';
    return len;
}

static int boot_diary_vprintf(const char *format, va_list args) {
    va_list output_args;
    va_list capture_args;
    va_copy(output_args, args);
    va_copy(capture_args, args);

    vprintf_like_t output = atomic_load_explicit(
        &s_previous_vprintf, memory_order_acquire);
    if (!output) output = vprintf;
    int output_result = output(format, output_args);
    va_end(output_args);

    if (!xPortInIsrContext()) {
        char record[BOOT_DIARY_RECORD_BYTES];
        int formatted_len = vsnprintf(record, sizeof(record), format,
                                      capture_args);
        size_t record_len = normalize_record(record, formatted_len);
        if (record_len > 0 && s_lock
            && xSemaphoreTake(s_lock, 0) == pdTRUE) {
            diary_append_locked(record, record_len);
            xSemaphoreGive(s_lock);
        }
    }
    va_end(capture_args);
    return output_result;
}

bool boot_diary_init(void) {
    if (atomic_load_explicit(&s_installed, memory_order_acquire)) {
        return true;
    }
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return false;

    if (!store_valid()) {
        memset(&s_store, 0, sizeof(s_store));
        s_store.magic = BOOT_DIARY_MAGIC;
        s_store.magic_inv = ~BOOT_DIARY_MAGIC;
        s_store.boots = 0;
        s_store.boots_inv = ~0u;
        s_store.head = 0;
        s_store.len = 0;
    }
    s_store.boots++;
    s_store.boots_inv = ~s_store.boots;

    const esp_app_desc_t *desc = esp_app_get_description();
    char marker[128];
    int marker_len = snprintf(marker, sizeof(marker),
                              "\n=== boot #%lu reset=%s ver=%s ===\n",
                              (unsigned long)s_store.boots,
                              reset_name(esp_reset_reason()),
                              desc ? desc->version : "?");
    if (marker_len > 0) {
        if ((size_t)marker_len >= sizeof(marker)) {
            marker_len = (int)sizeof(marker) - 1;
        }
        diary_append_locked(marker, (size_t)marker_len);
    }

    // Keep a valid fallback during the sink swap, as diagnostic_log does.
    atomic_store_explicit(&s_previous_vprintf, vprintf,
                          memory_order_release);
    vprintf_like_t previous = esp_log_set_vprintf(boot_diary_vprintf);
    if (previous) {
        atomic_store_explicit(&s_previous_vprintf, previous,
                              memory_order_release);
    }
    atomic_store_explicit(&s_installed, true, memory_order_release);
    return true;
}

uint32_t boot_diary_total(void) {
    if (!atomic_load_explicit(&s_installed, memory_order_acquire)) return 0;
    return s_store.len;
}

uint32_t boot_diary_boots(void) {
    if (!atomic_load_explicit(&s_installed, memory_order_acquire)) return 0;
    return s_store.boots;
}

size_t boot_diary_read(uint32_t offset, char *out, size_t out_cap) {
    if (!out || out_cap == 0) return 0;
    out[0] = '\0';
    if (!atomic_load_explicit(&s_installed, memory_order_acquire)
        || !s_lock) {
        return 0;
    }
    if (xSemaphoreTake(s_lock, portMAX_DELAY) != pdTRUE) return 0;
    size_t copied = 0;
    if (offset < s_store.len) {
        uint32_t want = s_store.len - offset;
        if (want > out_cap - 1) want = (uint32_t)(out_cap - 1);
        uint32_t start = (s_store.head + offset) % BOOT_DIARY_CAPACITY;
        uint32_t first = BOOT_DIARY_CAPACITY - start;
        if (first > want) first = want;
        memcpy(out, &s_store.buf[start], first);
        if (first < want) memcpy(out + first, &s_store.buf[0], want - first);
        copied = want;
        out[copied] = '\0';
    }
    xSemaphoreGive(s_lock);
    return copied;
}

#endif /* CONFIG_HOMEHUB_BOOT_DIARY */
