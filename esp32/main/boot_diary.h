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

/* Boot diary (CONFIG_HOMEHUB_BOOT_DIARY): mirrors the firmware log into
 * a ring buffer in RTC memory from the first line of app_main. RTC
 * memory survives software and watchdog resets (only a full power loss
 * erases it), so after a boot that panics or hangs and is rolled back,
 * the NEXT boot can hand the dead boot's last lines to the
 * device.bootlog command -- no serial cable needed. Each boot appends
 * a marker line ("=== boot #N reset=... ver=... ==="); the text
 * between two markers is one boot's tail. */

/* Install the log sink and open a new boot segment. Call once, as
 * early in app_main as possible. Returns false when the sink could
 * not be installed. */
bool boot_diary_init(void);

/* Bytes currently held, all boot segments together. */
uint32_t boot_diary_total(void);

/* Boots recorded since the last full power loss. */
uint32_t boot_diary_boots(void);

/* Copy up to out_cap - 1 bytes starting at linearized offset (0 is the
 * oldest byte held) into out and NUL-terminate it. Returns the number
 * of bytes copied, excluding the terminator. */
size_t boot_diary_read(uint32_t offset, char *out, size_t out_cap);

#ifdef __cplusplus
}
#endif
