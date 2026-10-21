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

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// The largest MP3 audio.play_file will fetch: the chat path's MP3 buffer on
// boards with PSRAM (MP3_BUF in muse_chat_session.cpp). At speech bitrates
// that is minutes of audio, far past any single spoken reply.
#define AUDIO_FETCH_MAX_BYTES (512 * 1024)

typedef struct {
    bool ok;
    const char *code;     // error code when !ok, e.g. "fetch_failed"
    const char *message;  // error detail when !ok
    size_t bytes;         // bytes fetched
    int ms;               // fetch time
} audio_fetch_result_t;

// Runs on the fetch task once the whole file is in. On success mp3 is a
// buffer of mp3_len bytes the callback takes ownership of (free() it once,
// whatever it does with it); on failure mp3 is NULL.
typedef void (*audio_fetch_done_cb)(const audio_fetch_result_t *result,
                                    uint8_t *mp3, size_t mp3_len, void *user);

// Fetch an MP3 file from the VM, with GET /fs/raw/<path> over the Noise
// session in Range requests, the way image_fetch reads a file for
// display.draw_file, but accumulating the whole file in PSRAM instead of
// drawing it as it arrives. One fetch runs at a time.
//
// Returns false with `code`/`message` set if the fetch cannot start (busy,
// bad path, out of memory); otherwise `done` runs later on the fetch task.
bool audio_fetch_start_vm(const char *path, audio_fetch_done_cb done,
                          void *user, const char **code, const char **message);

#ifdef __cplusplus
}
#endif
