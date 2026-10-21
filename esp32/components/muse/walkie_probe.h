/* Copyright (c) Meta Platforms, Inc. and affiliates.
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

#include "host/ble_gatt.h"
#include "host/ble_uuid.h"

struct ble_gap_event;

/*
 * Walkie-talkie BLE link probe (CONFIG_MUSE_WALKIE_PROBE, bench only).
 *
 * Two gadgets running a probe build find each other over BLE and stream
 * sequenced pattern data one way (the lower BLE address initiates and
 * counts; the peer notifies as fast as the stack allows) so the direct
 * gadget-to-gadget link can be measured: sustained throughput, packet
 * loss, ping round-trip time, connection stability. Nothing goes through
 * the Muse backend; the normal Muse session keeps running beside it.
 *
 * muse_ble.c appends muse_walkie_chrs (under muse_walkie_svc_uuid) to the
 * Muse companion's service list and forwards GAP events to
 * muse_walkie_gap_event(); muse_glue_start() calls
 * muse_walkie_probe_start() once at boot.
 */

extern const ble_uuid128_t muse_walkie_svc_uuid;
extern const struct ble_gatt_chr_def muse_walkie_chrs[];

/* Forwards every GAP event (the connection the phone setup accepted
 * included); the probe picks out its own connection and subscriptions. */
void muse_walkie_gap_event(struct ble_gap_event *ev);

/* Creates the probe's tasks. Safe to call before the BLE host is up:
 * the tasks wait for sync. */
void muse_walkie_probe_start(void);
