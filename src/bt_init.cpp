// ═══════════════════════════════════════════════════════════════════════════
// bt_init.cpp — BTstack HID Host layer
//
// ISOLATION RULE: Never include Adafruit_TinyUSB.h here.
// hid_report_type_t conflict between btstack_hid.h and tusb hid.h is avoided
// by keeping them in separate translation units.
//
// Connection strategy: SCAN ONLY
//   - On boot (or disconnect): start 10s inquiry scan
//   - Find any device with gamepad CoD (0x0005xx) → connect
//   - No saved-address logic — always scan fresh
//   - This is reliable because the WiiMote advertises itself when 1+2 is held
//
// Extension detection (after HID connect):
//   Sends Wii memory write/read sequence to probe extension port
//   Results in on_extension_detected() callback to main.cpp
// ═══════════════════════════════════════════════════════════════════════════

#include <Arduino.h>

extern "C" {
#include "btstack.h"
}

// ── Shared globals (extern'd in main.cpp) ────────────────────────────────
volatile uint16_t g_btn_cur    = 0;
volatile int16_t  g_raw_ax     = 0, g_raw_ay = 0, g_raw_az = 0;
volatile bool     g_new_report = false;
volatile bool     g_connected  = false;
volatile uint8_t  g_ext_type   = 0;   // 0=none 1=nunchuk 2=guitar 3=balance_board
volatile int8_t   g_nc_jx      = 0, g_nc_jy = 0;
volatile int16_t  g_nc_ax      = 0, g_nc_ay = 0, g_nc_az = 0;
volatile bool     g_nc_btn_c   = false, g_nc_btn_z = false;
volatile uint16_t g_gh_btns    = 0;
volatile uint8_t  g_gh_whammy  = 0;
volatile int8_t   g_gh_touch   = 0, g_gh_tilt_x = 0, g_gh_tilt_z = 0;
// Balance Board: 4 sensors, raw 16-bit values (0 = no weight, ~10000 = ~34kg each)
volatile uint16_t g_bb_tl = 0, g_bb_tr = 0, g_bb_bl = 0, g_bb_br = 0;

// ── Callbacks in main.cpp ────────────────────────────────────────────────
extern void on_wiimote_connected(const bd_addr_t addr);
extern void on_wiimote_disconnected();
extern void on_extension_detected(uint8_t ext_type, const uint8_t* id6);

// ── Internal state ────────────────────────────────────────────────────────
#define MAX_DESC 2048  // Balance Board descriptor can be large; print actual size below
static uint8_t   hid_desc[MAX_DESC];
static uint16_t  hid_cid  = 0;
static bool      desc_ok  = false;
static bd_addr_t remote_addr;

static btstack_packet_callback_registration_t hci_cb_reg;

typedef enum { ST_IDLE, ST_SCANNING, ST_CONNECTING, ST_CONNECTED } AppState;
static AppState app_state = ST_IDLE;

// Extension detection state machine
typedef enum {
    EXT_IDLE, EXT_WROTE_INIT, EXT_WROTE_NOENC, EXT_READING, EXT_DONE
} ExtState;
static ExtState ext_state = EXT_IDLE;

// ── WiiMote output helpers ────────────────────────────────────────────────

static void wm_send(uint8_t rid, const uint8_t* d, uint8_t len) {
    if (hid_cid) hid_host_send_report(hid_cid, rid, (uint8_t*)d, len);
}

static void wm_set_leds(uint8_t mask) {
    uint8_t d = mask & 0xF0;
    wm_send(0x11, &d, 1);
}

static void wm_set_report_mode(uint8_t mode) {
    uint8_t d[2] = {0x00, mode};
    wm_send(0x12, d, 2);
}

// Write 1 byte to WiiMote register space.
// Report 0x16: [addr_space(1)] [addr(3)] [size(1)] [data(16)] = 21 bytes total
// addr_space: 0x04 = extension controller registers
static void wm_write_reg(uint32_t addr, uint8_t val) {
    uint8_t d[21] = {};
    d[0] = 0x04;                       // register address space
    d[1] = (addr >> 16) & 0xFF;
    d[2] = (addr >>  8) & 0xFF;
    d[3] = (addr      ) & 0xFF;
    d[4] = 0x01;                       // 1 byte
    d[5] = val;
    wm_send(0x16, d, 21);
}

// Read N bytes from WiiMote register space.
// Report 0x17: [addr_space(1)] [addr(3)] [size(2)] = 6 bytes
static void wm_read_reg(uint32_t addr, uint16_t size) {
    uint8_t d[6] = {};
    d[0] = 0x04;
    d[1] = (addr >> 16) & 0xFF;
    d[2] = (addr >>  8) & 0xFF;
    d[3] = (addr      ) & 0xFF;
    d[4] = (size >>  8) & 0xFF;
    d[5] = (size      ) & 0xFF;
    wm_send(0x17, d, 6);
}

// Start extension init+identify sequence.
// New init sequence (works on all WiiMote hardware revisions):
//   Write 0x55 to 0xA400F0  (step 1 — reset extension)
//   Write 0x00 to 0xA400FB  (step 2 — disable encryption)
//   Read  6 bytes at 0xA400FA (step 3 — read extension ID)
//
// Old encrypted sequence (pre-2008, original Nunchuk):
//   Write 0x40 to 0xA400F0
//   Write 0x00 to 0xA400F1 .. 0xA400F5  (key bytes)
// We use the new unencrypted sequence which works universally.
//
// Error 0x03 in ack means extension port not ready — add delay before first write.
static bool ext_detect_pending = false;
static uint32_t ext_detect_pending_ms = 0;
#define EXT_SETTLE_MS 300

static void wm_detect_extension_internal() {
    if (ext_state == EXT_WROTE_INIT || ext_state == EXT_WROTE_NOENC || ext_state == EXT_READING) {
        Serial.println("[EXT] Detection already in progress — skipping");
        Serial.flush();
        return;
    }
    Serial.println("[EXT] Step 1: write 0x55 to 0xA400F0");
    Serial.flush();
    ext_state = EXT_WROTE_INIT;
    wm_write_reg(0xA400F0, 0x55);
}

// Public: trigger detection from terminal command 'e'
void bt_detect_extension() {
    if (!desc_ok) { Serial.println("[EXT] Not connected"); Serial.flush(); return; }
    ext_state = EXT_IDLE;
    ext_detect_pending = false;
    wm_detect_extension_internal();
}

static void wm_configure() {
    delay(80);
    wm_set_leds(0x10);
    delay(80);
    wm_set_report_mode(0x31);
    Serial.println("[WM] LED1 on, mode 0x31 — waiting for status report...");
    Serial.flush();
}

// ── Extension decoders ────────────────────────────────────────────────────

static void decode_nunchuk(const uint8_t* ext) {
    g_nc_jx    = (int8_t)(ext[0] - 128);
    g_nc_jy    = (int8_t)(ext[1] - 128);
    g_nc_ax    = (int16_t)(((uint16_t)ext[2]<<2) | ((ext[5]>>2)&3)) - 512;
    g_nc_ay    = (int16_t)(((uint16_t)ext[3]<<2) | ((ext[5]>>4)&3)) - 512;
    g_nc_az    = (int16_t)(((uint16_t)ext[4]<<2) | ((ext[5]>>6)&3)) - 512;
    g_nc_btn_c = !(ext[5] & 0x02);
    g_nc_btn_z = !(ext[5] & 0x01);
}

static void decode_guitar_hero(const uint8_t* ext) {
    uint8_t b0 = ~ext[0], b1 = ~ext[1];
    g_gh_btns = 0;
    if (b0 & 0x40) g_gh_btns |= (1<<0);
    if (b0 & 0x01) g_gh_btns |= (1<<1);
    if (b0 & 0x04) g_gh_btns |= (1<<2);
    if (b0 & 0x10) g_gh_btns |= (1<<3);
    if (b1 & 0x20) g_gh_btns |= (1<<4);
    if (b1 & 0x40) g_gh_btns |= (1<<5);
    if (b1 & 0x10) g_gh_btns |= (1<<6);
    if (b1 & 0x08) g_gh_btns |= (1<<7);
    if (b1 & 0x04) g_gh_btns |= (1<<8);
    g_gh_whammy = (ext[3] >> 3) & 0x1F;
    g_gh_touch  = (int8_t)(ext[2] & 0x1F) - 15;
    g_gh_tilt_x = (int8_t)ext[4];
    g_gh_tilt_z = (int8_t)ext[5];
}

// Balance Board: 8 extension bytes in report 0x32
// Layout: [TL_hi][TL_lo][TR_hi][TR_lo][BL_hi][BL_lo][BR_hi][BR_lo]
// Raw values: 0 = no weight, ~10000 per sensor at full body weight (~34kg each)
// Top/Bottom Left/Right from WiiMote perspective (placed in front of TV)
static void decode_balance_board(const uint8_t* ext) {
    g_bb_tl = ((uint16_t)ext[0]<<8) | ext[1];
    g_bb_tr = ((uint16_t)ext[2]<<8) | ext[3];
    g_bb_bl = ((uint16_t)ext[4]<<8) | ext[5];
    g_bb_br = ((uint16_t)ext[6]<<8) | ext[7];
}

// ── Report handler ────────────────────────────────────────────────────────

static void handle_report(const uint8_t* r, uint16_t len) {
    if (len < 2) return;
    uint8_t rid = r[1];

    if (rid == 0x20 && len >= 7) {
        bool ext_connected = (r[4] & 0x02) != 0;
        Serial.printf("[WM] Status: ext=%d flags=0x%02X batt=0x%02X\n",
                      ext_connected?1:0, r[4], r[6]);
        Serial.flush();
        if (ext_state == EXT_IDLE || ext_state == EXT_DONE) {
            wm_set_report_mode(g_ext_type ? 0x35 : 0x31);
        }
        if (ext_connected && ext_state == EXT_IDLE) {
            // Schedule detection after settle time — do NOT delay() here
            // as that blocks the BTstack run loop
            Serial.printf("[WM] Ext connected — scheduling detection in %dms\n", EXT_SETTLE_MS);
            Serial.flush();
            ext_detect_pending = true;
            ext_detect_pending_ms = millis() + EXT_SETTLE_MS;
        }
        return;
    }

    // Write ack 0x22 — drives detection state machine
    // Observed format: A1 22 [btn0] [btn1] [addr_offset] [error]  (6 bytes)
    // error is at r[5] (the last byte in the 6-byte packet)
    if (rid == 0x22) {
        Serial.printf("[EXT] Write ack:");
        for (int i=0; i<(int)len && i<8; i++) Serial.printf(" %02X", r[i]);
        Serial.println();
        Serial.flush();
        // Error byte: last byte of packet. 0=OK.
        uint8_t err = (len >= 6) ? r[len-1] : 0xFF;
        if (err) {
            Serial.printf("[EXT] Write error 0x%02X", err);
            if (err == 0x06) Serial.print(" (ext not ready — retry in 500ms)");
            if (err == 0x07) Serial.print(" (no extension)");
            Serial.println();
            Serial.flush();
            ext_state = EXT_IDLE;
            if (err == 0x06) {
                // Schedule retry
                ext_detect_pending = true;
                ext_detect_pending_ms = millis() + 500;
            }
            return;
        }
        if (ext_state == EXT_WROTE_INIT) {
            Serial.println("[EXT] Step 2: write 0x00 to 0xA400FB (no encryption)");
            Serial.flush();
            delay(10);
            wm_write_reg(0xA400FB, 0x00);
            ext_state = EXT_WROTE_NOENC;
        } else if (ext_state == EXT_WROTE_NOENC) {
            Serial.println("[EXT] Step 3: read 6 bytes from 0xA400FA");
            Serial.flush();
            delay(10);
            wm_read_reg(0xA400FA, 6);
            ext_state = EXT_READING;
        }
        return;
    }

    // Memory read response 0x21
    // Format: A1 21 [btn0] [btn1] [size_err] [addr_hi] [addr_lo] [data x16]
    // byte[4]: upper nibble = size-1, lower nibble = error
    if (rid == 0x21) {
        Serial.printf("[EXT] Read resp:");
        for (int i=0; i<(int)len && i<14; i++) Serial.printf(" %02X", r[i]);
        Serial.println();
        Serial.flush();
        uint8_t err  = r[4] & 0x0F;
        uint8_t size = ((r[4] >> 4) & 0x0F) + 1;
        if (ext_state == EXT_READING) {
            if (err) {
                Serial.printf("[EXT] Read error 0x%X — no extension?\n", err);
                Serial.flush();
                ext_state = EXT_DONE;
                on_extension_detected(0, nullptr);
                return;
            }
            const uint8_t* id = &r[7];
            Serial.printf("[EXT] ID: %02X:%02X:%02X:%02X:%02X:%02X\n",
                          id[0],id[1],id[2],id[3],id[4],id[5]);
            uint8_t detected = 0;
            if      (id[2]==0xA4 && id[3]==0x20 && id[4]==0x00 && id[5]==0x00)
                { Serial.println("[EXT] Nunchuk!"); detected = 1; }
            else if (id[2]==0xA4 && id[3]==0x20 && id[4]==0x01 && id[5]==0x03)
                { Serial.println("[EXT] Guitar Hero 3!"); detected = 2; }
            else if (id[2]==0xA4 && id[3]==0x20 && id[4]==0x04 && id[5]==0x02)
                { Serial.println("[EXT] Balance Board!"); detected = 3; }
            else if (id[2]==0xA4 && id[3]==0x20 && id[4]==0x01 && id[5]==0x01)
                { Serial.println("[EXT] Classic Controller (unsupported)"); }
            else if (id[0]==0xFF)
                { Serial.println("[EXT] No extension"); }
            else
                { Serial.println("[EXT] Unknown extension"); }
            Serial.flush();
            ext_state  = EXT_DONE;
            g_ext_type = detected;
            if (detected) {
                delay(10);
                wm_set_report_mode(0x35); // switch to ext report mode
            }
            on_extension_detected(detected, id);
        }
        return;
    }

    // Skip non-data reports
    if (rid < 0x30 || rid > 0x37) {
        Serial.printf("[WM] Report 0x%02X (%d bytes):", rid, len);
        for (int i=0; i<(int)len && i<10; i++) Serial.printf(" %02X", r[i]);
        Serial.println(); Serial.flush();
        return;
    }

    // Data reports: decode buttons + accel + extension
    if (len < 4) return;
    uint8_t b0=r[2], b1=r[3];

    uint16_t btns = 0;
    if (b0&0x01) btns|=(1<<8);   if (b0&0x02) btns|=(1<<9);
    if (b0&0x04) btns|=(1<<10);  if (b0&0x08) btns|=(1<<11);
    if (b0&0x10) btns|=(1<<12);  if (b1&0x01) btns|=(1<<0);
    if (b1&0x02) btns|=(1<<1);   if (b1&0x04) btns|=(1<<2);
    if (b1&0x08) btns|=(1<<3);   if (b1&0x10) btns|=(1<<4);
    if (b1&0x80) btns|=(1<<7);
    g_btn_cur = btns;

    if ((rid==0x31||rid==0x35) && len>=7) {
        g_raw_ax = (int16_t)(((uint16_t)r[4]<<2)|((b0>>5)&3)) - 512;
        g_raw_ay = (int16_t)(((uint16_t)r[5]<<2)|((b1>>5)&3)) - 512;
        g_raw_az = (int16_t)(((uint16_t)r[6]<<2)|((b1>>6)&1)|((b0>>6)&1)<<1) - 512;
    }
    if (rid==0x32 && len>=10) {
        if      (g_ext_type==1) decode_nunchuk(&r[4]);
        else if (g_ext_type==2) decode_guitar_hero(&r[4]);
        else if (g_ext_type==3) decode_balance_board(&r[4]);
    }
    if (rid==0x35 && len>=21) {
        if      (g_ext_type==1) decode_nunchuk(&r[7]);
        else if (g_ext_type==2) decode_guitar_hero(&r[7]);
        else if (g_ext_type==3) decode_balance_board(&r[7]);
    }
    g_new_report = true;
}

// ── Connection helpers ────────────────────────────────────────────────────

static void start_scan() {
    Serial.println("[SCAN] Starting 10s inquiry — hold 1+2 on WiiMote");
    Serial.flush();
    app_state = ST_SCANNING;
    gap_inquiry_start(10);
}

static void do_connect() {
    if (app_state == ST_CONNECTING || app_state == ST_CONNECTED) return;
    Serial.printf("[BT] Connecting → %s\n", bd_addr_to_str(remote_addr));
    Serial.flush();
    // Try REPORT mode first; Balance Board may need BOOT mode if this fails with 0x66
    uint8_t st = hid_host_connect(remote_addr, HID_PROTOCOL_MODE_REPORT, &hid_cid);
    if (st == ERROR_CODE_SUCCESS) app_state = ST_CONNECTING;
    else { Serial.printf("[BT] Connect error 0x%02X\n", st); Serial.flush(); }
}

// ── Packet handler ────────────────────────────────────────────────────────

static void packet_handler(uint8_t ptype, uint16_t ch, uint8_t* pkt, uint16_t size) {
    (void)ch; (void)size;
    if (ptype != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(pkt)) {

    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(pkt) == HCI_STATE_WORKING) {
            bd_addr_t a; gap_local_bd_addr(a);
            Serial.printf("[BT] Stack WORKING. Pico: %s\n", bd_addr_to_str(a));
            gap_discoverable_control(1);
            gap_connectable_control(1);
            delay(200);
            start_scan();
        }
        break;

    case GAP_EVENT_INQUIRY_RESULT: {
        bd_addr_t addr;
        gap_event_inquiry_result_get_bd_addr(pkt, addr);
        uint32_t cod = (uint32_t)gap_event_inquiry_result_get_class_of_device(pkt);
        Serial.printf("[SCAN] %s CoD=0x%06X\n", bd_addr_to_str(addr), cod);
        Serial.flush();
        if ((cod & 0x001F00) == 0x000500) {
            Serial.println("[SCAN] WiiMote found — connecting");
            Serial.flush();
            memcpy(remote_addr, addr, 6);
            gap_inquiry_stop();
            do_connect();
        }
        break;
    }

    case GAP_EVENT_INQUIRY_COMPLETE:
        Serial.println("[SCAN] Inquiry complete.");
        Serial.flush();
        if (app_state == ST_SCANNING) app_state = ST_IDLE;
        break;

    case HCI_EVENT_PIN_CODE_REQUEST: {
        bd_addr_t addr;
        hci_event_pin_code_request_get_bd_addr(pkt, addr);
        uint8_t pin[6] = {addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]};
        gap_pin_code_response_binary(addr, pin, 6);
        Serial.printf("[BT] PIN → %s\n", bd_addr_to_str(addr));
        Serial.flush();
        break;
    }

    case HCI_EVENT_HID_META:
        switch (hci_event_hid_meta_get_subevent_code(pkt)) {

        case HID_SUBEVENT_INCOMING_CONNECTION:
            hid_host_accept_connection(
                hid_subevent_incoming_connection_get_hid_cid(pkt),
                HID_PROTOCOL_MODE_REPORT);
            break;

        case HID_SUBEVENT_CONNECTION_OPENED: {
            uint8_t st = hid_subevent_connection_opened_get_status(pkt);
            if (st == ERROR_CODE_SUCCESS) {
                hid_cid     = hid_subevent_connection_opened_get_hid_cid(pkt);
                app_state   = ST_CONNECTED;
                g_connected = true;
                ext_state   = EXT_IDLE;
                on_wiimote_connected(remote_addr);
                Serial.printf("[HID] *** Connected! cid=%u ***\n", hid_cid);
            } else {
                Serial.printf("[HID] Failed 0x%02X", st);
                switch (st) {
                case 0x04: Serial.print(" (page timeout — device out of range)"); break;
                case 0x0B: Serial.print(" (rejected — bonded elsewhere, hold SYNC 15s)"); break;
                case 0x66: Serial.printf(" (unsupported — MAX_DESC=%d may be too small, or device needs different HID protocol)", MAX_DESC); break;
                default:   break;
                }
                Serial.println();
                hid_cid   = 0;
                app_state = ST_IDLE;
            }
            Serial.flush();
            break;
        }

        case HID_SUBEVENT_DESCRIPTOR_AVAILABLE:
            if (hid_subevent_descriptor_available_get_status(pkt) == ERROR_CODE_SUCCESS) {
                uint16_t cid_tmp = hid_subevent_descriptor_available_get_hid_cid(pkt);
                uint16_t dlen = hid_descriptor_storage_get_descriptor_len(cid_tmp);
                Serial.printf("[HID] Descriptor OK — %u bytes (buffer=%d)\n", dlen, MAX_DESC);
                Serial.flush();
                desc_ok = true;
                wm_configure();
            } else {
                Serial.printf("[HID] Descriptor failed 0x%02X\n",
                              hid_subevent_descriptor_available_get_status(pkt));
                Serial.flush();
            }
            break;

        case HID_SUBEVENT_REPORT:
            if (desc_ok)
                handle_report(hid_subevent_report_get_report(pkt),
                              hid_subevent_report_get_report_len(pkt));
            break;

        case HID_SUBEVENT_CONNECTION_CLOSED:
            Serial.println("[HID] Disconnected.");
            hid_cid     = 0;
            app_state   = ST_IDLE;
            desc_ok     = false;
            g_connected = false;
            g_ext_type  = 0;
            ext_state   = EXT_IDLE;
            on_wiimote_disconnected();
            gap_discoverable_control(1);
            Serial.flush();
            break;

        default: break;
        }
        break;

    default: break;
    }
}

// ── Public API ────────────────────────────────────────────────────────────

void bt_hid_init(const uint8_t* /*saved_addr*/, bool /*has_saved*/, uint8_t ext_type) {
    g_ext_type = ext_type;

    l2cap_init();
    hid_host_init(hid_desc, sizeof(hid_desc));
    hid_host_register_packet_handler(packet_handler);
    gap_set_default_link_policy_settings(
        LM_LINK_POLICY_ENABLE_SNIFF_MODE | LM_LINK_POLICY_ENABLE_ROLE_SWITCH);
    hci_set_master_slave_policy(HCI_ROLE_MASTER);
    hci_cb_reg.callback = &packet_handler;
    hci_add_event_handler(&hci_cb_reg);
    gap_discoverable_control(1);
    hci_power_control(HCI_POWER_ON);
}

void bt_hid_loop_update() {
    // Fire pending extension detection after settle delay
    if (ext_detect_pending && millis() >= ext_detect_pending_ms) {
        ext_detect_pending = false;
        wm_detect_extension_internal();
    }

    // Restart scan every 12s while idle
    static uint32_t last_scan = 0;
    if (app_state == ST_IDLE && millis() - last_scan > 12000) {
        last_scan = millis();
        start_scan();
    }
}