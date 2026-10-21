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

#include "sdkconfig.h"

#if CONFIG_MUSE_WALKIE_PROBE

#include "walkie_probe.h"

#include <stdatomic.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_uuid.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"

#include "muse_state.h"

static const char *TAG = "muse.walkie";

/* ---- Wire format -------------------------------------------------------
 * One DATA characteristic carries everything, first byte is the type:
 *   0x00 data   : [0]=0x00, [1..4]=seq LE32, rest = pattern bytes.
 *                 Sent by the source as notifications, as fast as the
 *                 stack takes them.
 *   0x01 start  : written by the counter, [1..4]=total value bytes to
 *                 send LE32 (0 = until told to stop / disconnected).
 *   0x02 stop   : written by the counter.
 *   0x03 ping   : written by the counter, [1..4]=seq; the source echoes
 *                 the same 5 bytes back as a notification at once.
 * The counter derives throughput from value bytes received, loss from
 * seq gaps, and round-trip time from ping echoes. */

/* 57414c4b-000X-4000-8000-6c696e6b6267 ("WALK"), little-endian. */
#define WALK_UUID(x) BLE_UUID128_INIT(0x67, 0x62, 0x6b, 0x6e, 0x69, 0x6c, 0x00, 0x80, \
                                      0x00, 0x40, x, 0x00, 0x4b, 0x4c, 0x41, 0x57)
const ble_uuid128_t muse_walkie_svc_uuid = WALK_UUID(0x01);
static const ble_uuid128_t DATA_UUID = WALK_UUID(0x02);

#define TYPE_DATA 0x00
#define TYPE_START 0x01
#define TYPE_STOP 0x02
#define TYPE_PING 0x03

static uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void wp_put_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint32_t wp_get_le32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

/* ---- Source side (GATT server): notify pattern data to the counter ---- */

static uint16_t s_data_handle;
static uint16_t s_srv_conn = BLE_HS_CONN_HANDLE_NONE;
static bool s_srv_subscribed;
static _Atomic uint32_t s_tx_pending;   /* value bytes still to send */
static _Atomic uint32_t s_tx_seq;
static _Atomic uint32_t s_tx_sent;      /* value bytes notified, this run */
static TaskHandle_t s_tx_task;
static uint8_t s_ping_echo[5];
static _Atomic bool s_ping_waiting;

static int data_access_cb(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    (void)conn;
    (void)attr;
    (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    uint8_t buf[8];
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len) != 0 || len < 1) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    switch (buf[0]) {
    case TYPE_START: {
        uint32_t total = len >= 5 ? wp_get_le32(buf + 1) : 0;
        atomic_store(&s_tx_seq, 0);
        atomic_store(&s_tx_sent, 0);
        atomic_store(&s_tx_pending, total ? total : UINT32_MAX);
        ESP_LOGI(TAG, "source: start (%lu bytes)", (unsigned long)total);
        if (s_tx_task) xTaskNotifyGive(s_tx_task);
        return 0;
    }
    case TYPE_STOP:
        atomic_store(&s_tx_pending, 0);
        ESP_LOGI(TAG, "source: stop");
        return 0;
    case TYPE_PING:
        if (len >= 5) {
            memcpy(s_ping_echo, buf, 5);
            atomic_store(&s_ping_waiting, true);
            if (s_tx_task) xTaskNotifyGive(s_tx_task);
        }
        return 0;
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

const struct ble_gatt_chr_def muse_walkie_chrs[] = {
    {
        .uuid = &DATA_UUID.u,
        .access_cb = data_access_cb,
        .val_handle = &s_data_handle,
        .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE,
    },
    { 0 },
};

static void tx_task(void *arg) {
    (void)arg;
    static uint8_t chunk[512];
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        for (;;) {
            if (atomic_exchange(&s_ping_waiting, false)) {
                if (s_srv_conn != BLE_HS_CONN_HANDLE_NONE) {
                    struct os_mbuf *om = ble_hs_mbuf_from_flat(s_ping_echo, 5);
                    if (om) ble_gatts_notify_custom(s_srv_conn, s_data_handle, om);
                }
                continue;
            }
            uint32_t left = atomic_load(&s_tx_pending);
            if (!left || s_srv_conn == BLE_HS_CONN_HANDLE_NONE || !s_srv_subscribed) break;
            uint16_t mtu = ble_att_mtu(s_srv_conn);
            uint32_t cap = mtu > 3 ? mtu - 3 : 20;
            if (cap > sizeof(chunk)) cap = sizeof(chunk);
            uint32_t len = left < cap ? left : cap;
            if (len < 5) len = 5;   /* header must fit */
            for (uint32_t i = 5; i < len; i++) chunk[i] = (uint8_t)i;
            chunk[0] = TYPE_DATA;
            wp_put_le32(chunk + 1, atomic_fetch_add(&s_tx_seq, 1));
            struct os_mbuf *om = ble_hs_mbuf_from_flat(chunk, len);
            if (!om) {
                vTaskDelay(1);
                continue;
            }
            int rc = ble_gatts_notify_custom(s_srv_conn, s_data_handle, om);
            if (rc == BLE_HS_ENOMEM || rc == BLE_HS_EBUSY) {
                vTaskDelay(1);
                continue;   /* same chunk again, like ble_basic's tput */
            }
            if (rc != 0) {
                ESP_LOGW(TAG, "source: notify rc=%d, run paused", rc);
                break;
            }
            atomic_fetch_add(&s_tx_sent, len);
            if (left != UINT32_MAX) {
                atomic_store(&s_tx_pending, left - len < left ? left - len : 0);
            }
        }
    }
}

/* ---- Counter side (GATT client): scan, connect, count ------------------ */

static SemaphoreHandle_t s_done;
static int s_step_status;
static ble_addr_t s_peer_addr;
static bool s_peer_found;
static uint8_t s_own_addr[6];
static bool s_own_addr_known;

static uint16_t s_cen_conn = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_cen_data_val;   /* peer's DATA value handle */
static uint32_t s_cen_connected_at;
static uint32_t s_cen_connect_ms; /* how long the last setup took */

/* Receive accounting, written from the GAP callback. */
static _Atomic uint32_t s_rx_bytes;
static _Atomic uint32_t s_rx_pkts;
static _Atomic uint32_t s_rx_lost;
static _Atomic uint32_t s_rx_expected;
static _Atomic bool s_rx_started;
static _Atomic uint32_t s_ping_out_seq;
static _Atomic bool s_ping_out;
static _Atomic uint32_t s_ping_sent_ms;
static _Atomic uint32_t s_rtt_last;
static _Atomic uint32_t s_rtt_count;
static _Atomic uint32_t s_rtt_sum;
static _Atomic uint32_t s_rtt_max;
static _Atomic bool s_cen_connecting;   /* a probe-initiated connect is in flight */

static void rx_packet(const uint8_t *buf, uint16_t len) {
    if (len < 5) return;
    if (buf[0] == TYPE_DATA) {
        uint32_t seq = wp_get_le32(buf + 1);
        atomic_fetch_add(&s_rx_bytes, len);
        atomic_fetch_add(&s_rx_pkts, 1);
        if (!atomic_load(&s_rx_started)) {
            atomic_store(&s_rx_expected, seq + 1);
            atomic_store(&s_rx_started, true);
            return;
        }
        uint32_t exp = atomic_load(&s_rx_expected);
        if (seq > exp) atomic_fetch_add(&s_rx_lost, seq - exp);
        if (seq >= exp) atomic_store(&s_rx_expected, seq + 1);
    } else if (buf[0] == TYPE_PING) {
        uint32_t seq = wp_get_le32(buf + 1);
        if (atomic_load(&s_ping_out) && seq == atomic_load(&s_ping_out_seq)) {
            uint32_t rtt = now_ms() - atomic_load(&s_ping_sent_ms);
            atomic_store(&s_rtt_last, rtt);
            atomic_fetch_add(&s_rtt_count, 1);
            atomic_fetch_add(&s_rtt_sum, rtt);
            if (rtt > atomic_load(&s_rtt_max)) atomic_store(&s_rtt_max, rtt);
            atomic_store(&s_ping_out, false);
        }
    }
}

/* One event callback for both roles: connections the probe initiates get
 * it from ble_gap_connect, connections it accepts get it when the probe
 * started the advertising, and everything else arrives again via
 * muse_ble_gap_event() forwarding (accepted on Link's advertising). */
static int walkie_gap_event(struct ble_gap_event *ev, void *arg);

void muse_walkie_gap_event(struct ble_gap_event *ev) {
    walkie_gap_event(ev, NULL);
}

static int walkie_gap_event(struct ble_gap_event *ev, void *arg) {
    (void)arg;
    switch (ev->type) {
    case BLE_GAP_EVENT_DISC: {
        if (s_peer_found) return 0;
        struct ble_hs_adv_fields f;
        if (ble_hs_adv_parse_fields(&f, ev->disc.data, ev->disc.length_data) != 0
            || !f.name || !f.name_len) {
            return 0;
        }
        /* The other gadget: a MuseGadget-* advertiser whose address is
         * higher than ours -- the lower address is always the initiator,
         * so exactly one side ever connects. */
        if (f.name_len < 11 || memcmp(f.name, "MuseGadget-", 11) != 0) return 0;
        if (!s_own_addr_known || memcmp(ev->disc.addr.val, s_own_addr, 6) == 0) return 0;
        if (memcmp(ev->disc.addr.val, s_own_addr, 6) < 0) return 0;
        if (!(ev->disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_ADV_IND
              || ev->disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_DIR_IND)) {
            return 0;
        }
        s_peer_addr = ev->disc.addr;
        s_peer_found = true;
        ble_gap_disc_cancel();
        return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        if (s_done) xSemaphoreGive(s_done);
        return 0;
    case BLE_GAP_EVENT_CONNECT:
        /* Only the probe's own outgoing connect completes a central step;
         * other connections (the phone's, via Link's advertising) must not
         * disturb an in-flight wait. */
        if (atomic_load(&s_cen_connecting)) {
            s_step_status = ev->connect.status;
            if (s_done) xSemaphoreGive(s_done);
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        if (ev->disconnect.conn.conn_handle == s_srv_conn) {
            s_srv_conn = BLE_HS_CONN_HANDLE_NONE;
            s_srv_subscribed = false;
            atomic_store(&s_tx_pending, 0);
            ESP_LOGI(TAG, "source: peer gone (sent %lu B)", (unsigned long)atomic_load(&s_tx_sent));
        }
        if (ev->disconnect.conn.conn_handle == s_cen_conn) {
            ESP_LOGI(TAG, "counter: link down after %lu ms, reason=%d",
                     (unsigned long)(now_ms() - s_cen_connected_at), ev->disconnect.reason);
            s_cen_conn = BLE_HS_CONN_HANDLE_NONE;
        }
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (ev->subscribe.attr_handle == s_data_handle) {
            if (ev->subscribe.cur_notify) {
                s_srv_conn = ev->subscribe.conn_handle;
                s_srv_subscribed = true;
                ESP_LOGI(TAG, "source: counter subscribed");
                if (s_tx_task) xTaskNotifyGive(s_tx_task);
            } else if (ev->subscribe.conn_handle == s_srv_conn) {
                s_srv_subscribed = false;
            }
        }
        return 0;
    case BLE_GAP_EVENT_NOTIFY_RX: {
        if (ev->notify_rx.conn_handle != s_cen_conn) return 0;
        uint8_t buf[512];
        uint16_t len = OS_MBUF_PKTLEN(ev->notify_rx.om);
        if (len > sizeof(buf)) len = sizeof(buf);
        if (ble_hs_mbuf_to_flat(ev->notify_rx.om, buf, len, NULL) == 0) {
            rx_packet(buf, len);
        }
        return 0;
    }
    case BLE_GAP_EVENT_MTU:
        return 0;
    default:
        return 0;
    }
}

/* Blocking step helpers, the ble_central.c pattern. */
static bool wait_step(uint32_t timeout_ms) {
    return xSemaphoreTake(s_done, pdMS_TO_TICKS(timeout_ms)) == pdTRUE && s_step_status == 0;
}

static int gatt_err_cb(uint16_t conn, const struct ble_gatt_error *err, void *arg) {
    (void)conn;
    (void)arg;
    if (err->status == BLE_HS_EDONE) {
        /* procedure finished normally */
    } else if (err->status != 0) {
        s_step_status = err->status;
    }
    if (s_done) xSemaphoreGive(s_done);
    return 0;
}

static uint16_t s_disc_svc_start, s_disc_svc_end;
static int svc_cb(uint16_t conn, const struct ble_gatt_error *err, const struct ble_gatt_svc *svc, void *arg) {
    (void)conn;
    (void)arg;
    if (err->status == 0 && svc) {
        s_disc_svc_start = svc->start_handle;
        s_disc_svc_end = svc->end_handle;
        return 0;
    }
    return gatt_err_cb(conn, err, NULL);
}

static int chr_cb(uint16_t conn, const struct ble_gatt_error *err, const struct ble_gatt_chr *chr, void *arg) {
    (void)conn;
    (void)arg;
    if (err->status == 0 && chr) {
        if (ble_uuid_cmp(&chr->uuid.u, &DATA_UUID.u) == 0) {
            s_cen_data_val = chr->val_handle;
        }
        return 0;
    }
    return gatt_err_cb(conn, err, NULL);
}

static uint16_t s_cccd_handle;
static int dsc_cb(uint16_t conn, const struct ble_gatt_error *err, uint16_t chr_val,
                  const struct ble_gatt_dsc *dsc, void *arg) {
    (void)conn;
    (void)chr_val;
    (void)arg;
    if (err->status == 0 && dsc) {
        if (ble_uuid_u16(&dsc->uuid.u) == 0x2902) s_cccd_handle = dsc->handle;
        return 0;
    }
    return gatt_err_cb(conn, err, NULL);
}

static int attr_cb(uint16_t conn, const struct ble_gatt_error *err, struct ble_gatt_attr *attr, void *arg) {
    (void)attr;
    return gatt_err_cb(conn, err, arg);
}

static int mtu_cb(uint16_t conn, const struct ble_gatt_error *err, uint16_t mtu, void *arg) {
    (void)conn;
    (void)arg;
    if (err->status != 0) s_step_status = err->status;
    if (s_done) xSemaphoreGive(s_done);
    return 0;
}

static bool scan_for_peer(uint8_t own_type) {
    s_peer_found = false;
    struct ble_gap_disc_params p = {0};
    p.itvl = 96;      /* 60 ms */
    p.window = 48;    /* 30 ms */
    p.passive = 0;    /* active: the name rides in the scan response */
    p.filter_duplicates = 0;
    xSemaphoreTake(s_done, 0);
    s_step_status = 0;
    int rc = ble_gap_disc(own_type, 30000, &p, walkie_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "counter: disc rc=%d", rc);
        return false;
    }
    uint32_t t0 = now_ms();
    while (!s_peer_found && now_ms() - t0 < 32000) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (ble_gap_disc_active()) ble_gap_disc_cancel();
    return s_peer_found;
}

static bool connect_and_setup(uint8_t own_type) {
    struct ble_gap_conn_params cp = {0};
    cp.scan_itvl = 16;
    cp.scan_window = 16;
    cp.itvl_min = 12;   /* 15 ms */
    cp.itvl_max = 12;
    cp.latency = 0;
    cp.supervision_timeout = 400;   /* 4 s */
    uint32_t t0 = now_ms();
    xSemaphoreTake(s_done, 0);
    s_step_status = -1;
    atomic_store(&s_cen_connecting, true);
    int rc = ble_gap_connect(own_type, &s_peer_addr, 30000, &cp, walkie_gap_event, NULL);
    if (rc != 0) {
        atomic_store(&s_cen_connecting, false);
        ESP_LOGW(TAG, "counter: connect rc=%d", rc);
        return false;
    }
    /* The CONNECT event goes to walkie_gap_event directly; recover the
     * handle from the connection table once the step completes. */
    if (!wait_step(31000)) {
        atomic_store(&s_cen_connecting, false);
        ESP_LOGW(TAG, "counter: connect step failed status=%d", s_step_status);
        ble_gap_conn_cancel();
        return false;
    }
    atomic_store(&s_cen_connecting, false);
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find_by_addr(&s_peer_addr, &desc) != 0) {
        ESP_LOGW(TAG, "counter: connection not found after connect");
        return false;
    }
    s_cen_conn = desc.conn_handle;
    s_cen_connected_at = now_ms();

    xSemaphoreTake(s_done, 0);
    s_step_status = 0;
    rc = ble_gattc_exchange_mtu(s_cen_conn, mtu_cb, NULL);
    if (rc != 0 || !wait_step(5000)) goto fail;

    s_disc_svc_start = s_disc_svc_end = 0;
    xSemaphoreTake(s_done, 0);
    s_step_status = 0;
    rc = ble_gattc_disc_svc_by_uuid(s_cen_conn, &muse_walkie_svc_uuid.u, svc_cb, NULL);
    if (rc != 0 || !wait_step(10000) || !s_disc_svc_start) goto fail;

    s_cen_data_val = 0;
    xSemaphoreTake(s_done, 0);
    s_step_status = 0;
    rc = ble_gattc_disc_all_chrs(s_cen_conn, s_disc_svc_start, s_disc_svc_end, chr_cb, NULL);
    if (rc != 0 || !wait_step(10000) || !s_cen_data_val) goto fail;

    s_cccd_handle = 0;
    xSemaphoreTake(s_done, 0);
    s_step_status = 0;
    rc = ble_gattc_disc_all_dscs(s_cen_conn, s_cen_data_val, s_disc_svc_end, dsc_cb, NULL);
    if (rc != 0 || !wait_step(10000) || !s_cccd_handle) goto fail;

    {
        uint8_t sub[2] = {1, 0};
        xSemaphoreTake(s_done, 0);
        s_step_status = 0;
        rc = ble_gattc_write_flat(s_cen_conn, s_cccd_handle, sub, 2, attr_cb, NULL);
        if (rc != 0 || !wait_step(5000)) goto fail;
    }
    {
        uint8_t start[5] = {TYPE_START, 0, 0, 0, 0};   /* 0 = run open-ended */
        rc = ble_gattc_write_no_rsp_flat(s_cen_conn, s_cen_data_val, start, sizeof(start));
        if (rc != 0) goto fail;
    }
    s_cen_connect_ms = now_ms() - t0;
    ESP_LOGI(TAG, "counter: link up to %02x:%02x:%02x:%02x:%02x:%02x in %lu ms, mtu=%u",
             s_peer_addr.val[5], s_peer_addr.val[4], s_peer_addr.val[3],
             s_peer_addr.val[2], s_peer_addr.val[1], s_peer_addr.val[0],
             (unsigned long)s_cen_connect_ms, ble_att_mtu(s_cen_conn));
    return true;

fail:
    ESP_LOGW(TAG, "counter: setup failed rc=%d status=%d", rc, s_step_status);
    if (s_cen_conn != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_cen_conn, BLE_ERR_REM_USER_CONN_TERM);
        s_cen_conn = BLE_HS_CONN_HANDLE_NONE;
    }
    return false;
}

static void central_task(void *arg) {
    (void)arg;
    uint8_t own_type = 0;
    for (;;) {
        if (!ble_hs_synced()) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (!s_own_addr_known) {
            if (ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, s_own_addr, NULL) == 0) {
                s_own_addr_known = true;
                ESP_LOGI(TAG, "own addr %02x:%02x:%02x:%02x:%02x:%02x",
                         s_own_addr[5], s_own_addr[4], s_own_addr[3],
                         s_own_addr[2], s_own_addr[1], s_own_addr[0]);
            }
        }
        if (ble_hs_id_infer_auto(0, &own_type) != 0) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (!scan_for_peer(own_type)) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        /* Fresh counters before the START inside connect_and_setup can
         * bring the first notifications in. */
        atomic_store(&s_rx_started, false);
        atomic_store(&s_rx_bytes, 0);
        atomic_store(&s_rx_pkts, 0);
        atomic_store(&s_rx_lost, 0);
        atomic_store(&s_rtt_count, 0);
        atomic_store(&s_rtt_sum, 0);
        atomic_store(&s_rtt_max, 0);
        atomic_store(&s_rtt_last, 0);
        atomic_store(&s_ping_out, false);
        if (!connect_and_setup(own_type)) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        /* Run: one ping a second until the link drops. */
        uint32_t seq = 1;
        while (s_cen_conn != BLE_HS_CONN_HANDLE_NONE) {
            if (!atomic_load(&s_ping_out)) {
                uint8_t ping[5];
                ping[0] = TYPE_PING;
                wp_put_le32(ping + 1, seq);
                atomic_store(&s_ping_out_seq, seq);
                atomic_store(&s_ping_sent_ms, now_ms());
                atomic_store(&s_ping_out, true);
                if (ble_gattc_write_no_rsp_flat(s_cen_conn, s_cen_data_val, ping, sizeof(ping)) != 0) {
                    atomic_store(&s_ping_out, false);
                }
                seq++;
            } else if (now_ms() - atomic_load(&s_ping_sent_ms) > 900) {
                atomic_store(&s_ping_out, false);   /* echo lost; count via stats */
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ---- Stats + keeping the source connectable ---------------------------- */

static void ensure_advertising(void) {
    if (s_srv_conn != BLE_HS_CONN_HANDLE_NONE || ble_gap_adv_active()) return;
    uint8_t own_type;
    if (ble_hs_id_infer_auto(0, &own_type) != 0) return;
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t *)&muse_walkie_svc_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    if (ble_gap_adv_set_fields(&fields) != 0) return;
    struct ble_hs_adv_fields rsp = {0};
    const char *name = ble_svc_gap_device_name();
    rsp.name = (uint8_t *)name;
    rsp.name_len = (uint8_t)strlen(name);
    rsp.name_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp);
    struct ble_gap_adv_params p = {0};
    p.conn_mode = BLE_GAP_CONN_MODE_UND;
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;
    p.itvl_min = BLE_GAP_ADV_ITVL_MS(100);
    p.itvl_max = BLE_GAP_ADV_ITVL_MS(100);
    int rc = ble_gap_adv_start(own_type, NULL, BLE_HS_FOREVER, &p, walkie_gap_event, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "source: probe advertising on");
    }
}

static void stats_task(void *arg) {
    (void)arg;
    uint32_t prev_bytes = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (!ble_hs_synced()) continue;
        ensure_advertising();
        uint32_t bytes = atomic_load(&s_rx_bytes);
        uint32_t rate = (bytes - prev_bytes) * 8 / 5;   /* bit/s over the window */
        prev_bytes = bytes;
        if (s_cen_conn != BLE_HS_CONN_HANDLE_NONE) {
            uint32_t lost = atomic_load(&s_rx_lost);
            uint32_t pkts = atomic_load(&s_rx_pkts);
            uint32_t rcvd = pkts + lost;
            uint32_t loss_pm = rcvd ? (uint32_t)((uint64_t)lost * 1000 / rcvd) : 0;
            uint32_t rttc = atomic_load(&s_rtt_count);
            uint32_t avg = rttc ? (uint32_t)(atomic_load(&s_rtt_sum) / rttc) : 0;
            ESP_LOGI(TAG, "counter: %lu kbit/s, %lu B total, loss %lu.%lu%%, rtt last %lu avg %lu max %lu ms",
                     (unsigned long)(rate / 1000), (unsigned long)bytes,
                     (unsigned long)(loss_pm / 10), (unsigned long)(loss_pm % 10),
                     (unsigned long)atomic_load(&s_rtt_last), (unsigned long)avg,
                     (unsigned long)atomic_load(&s_rtt_max));
            muse_state_set_caption("WALKIE %lu kbit/s loss %lu.%lu%% rtt %lu ms",
                                   (unsigned long)(rate / 1000),
                                   (unsigned long)(loss_pm / 10), (unsigned long)(loss_pm % 10),
                                   (unsigned long)atomic_load(&s_rtt_last));
        } else if (s_srv_conn != BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGI(TAG, "source: sent %lu B to counter", (unsigned long)atomic_load(&s_tx_sent));
            muse_state_set_caption("WALKIE source: sent %lu KB", (unsigned long)(atomic_load(&s_tx_sent) / 1024));
        } else {
            muse_state_set_caption("WALKIE probe: looking for the other gadget");
        }
    }
}

void muse_walkie_probe_start(void) {
    s_done = xSemaphoreCreateBinary();
    if (!s_done) {
        ESP_LOGE(TAG, "no semaphore");
        return;
    }
    if (xTaskCreate(tx_task, "walk_tx", 4096, NULL, 4, &s_tx_task) != pdPASS
        || xTaskCreate(central_task, "walk_cen", 6144, NULL, 4, NULL) != pdPASS
        || xTaskCreate(stats_task, "walk_stat", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        return;
    }
    ESP_LOGI(TAG, "walkie-talkie link probe on (bench build)");
}

#endif   /* CONFIG_MUSE_WALKIE_PROBE */
