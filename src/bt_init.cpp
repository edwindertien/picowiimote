// bt_init.cpp — BTstack lives here, isolated from TinyUSB headers.
// This file must NOT include Adafruit_TinyUSB.h or any TinyUSB header.
// main.cpp includes TinyUSB. The two never share a translation unit.

#include <Arduino.h>

extern "C" {
#include "btstack.h"
}

// ── Shared state (extern in main.cpp) ────────────────────────────────────
volatile uint16_t g_btn_cur   = 0;
volatile int16_t  g_raw_ax    = 0, g_raw_ay = 0, g_raw_az = 0;
volatile bool     g_new_report = false;
volatile bool     g_connected  = false;

// ── Callbacks declared in main.cpp ───────────────────────────────────────
extern void on_wiimote_connected(const bd_addr_t addr);
extern void on_wiimote_disconnected();

// ── HID state ─────────────────────────────────────────────────────────────
#define MAX_DESC 300
static uint8_t   hid_desc[MAX_DESC];
static uint16_t  hid_cid = 0;
static bool      desc_ok = false;

static btstack_packet_callback_registration_t hci_cb_reg;
static bd_addr_t remote_addr;
static bool      have_addr = false;

typedef enum { ST_IDLE, ST_SCANNING, ST_CONNECTING, ST_CONNECTED } AppState;
static AppState  app_state = ST_IDLE;
static int       fail_count = 0;
#define MAX_FAILS 3

// ── WiiMote output ────────────────────────────────────────────────────────
static void wm_send(uint8_t rid, const uint8_t* d, uint8_t len) {
    if (hid_cid) hid_host_send_report(hid_cid, rid, (uint8_t*)d, len);
}
void wm_configure() {
    delay(80); uint8_t led=0x10; wm_send(0x11,&led,1);
    delay(80); uint8_t m[2]={0x00,0x31}; wm_send(0x12,m,2);
    Serial.println("[WM] LED1, mode 0x31"); Serial.flush();
}

// ── Report decode ─────────────────────────────────────────────────────────
static void handle_report(const uint8_t* r, uint16_t len) {
    if (len<4) return;
    uint8_t b0=r[2], b1=r[3]; uint16_t btns=0;
    if(b0&0x01)btns|=(1<<8);  if(b0&0x02)btns|=(1<<9);
    if(b0&0x04)btns|=(1<<10); if(b0&0x08)btns|=(1<<11);
    if(b0&0x10)btns|=(1<<12); if(b1&0x01)btns|=(1<<0);
    if(b1&0x02)btns|=(1<<1);  if(b1&0x04)btns|=(1<<2);
    if(b1&0x08)btns|=(1<<3);  if(b1&0x10)btns|=(1<<4);
    if(b1&0x80)btns|=(1<<7);
    g_btn_cur = btns;
    if(len>=7){
        g_raw_ax=(int16_t)((r[4]-0x80)<<2)|((b0&0x60)>>5);
        g_raw_ay=(int16_t)((r[5]-0x80)<<2)|((b1&0x40)>>4);
        g_raw_az=(int16_t)((r[6]-0x80)<<2)|((b1&0x20)>>5);
    }
    g_new_report = true;
}

// ── Connection ────────────────────────────────────────────────────────────
static void do_connect() {
    if (app_state==ST_CONNECTING||app_state==ST_CONNECTED) return;
    Serial.printf("[BT] Connecting → %s\n", bd_addr_to_str(remote_addr));
    Serial.flush();
    uint8_t st = hid_host_connect(remote_addr, HID_PROTOCOL_MODE_REPORT, &hid_cid);
    app_state = (st==ERROR_CODE_SUCCESS) ? ST_CONNECTING : ST_IDLE;
}

static void start_scan() {
    Serial.println("[SCAN] Scanning 10s — hold 1+2 on WiiMote");
    Serial.flush();
    app_state = ST_SCANNING;
    gap_inquiry_start(10);
}

// ── Packet handler ────────────────────────────────────────────────────────
static void packet_handler(uint8_t ptype, uint16_t ch, uint8_t* pkt, uint16_t size) {
    (void)ch; (void)size;
    if (ptype!=HCI_EVENT_PACKET) return;
    switch (hci_event_packet_get_type(pkt)) {

    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(pkt)==HCI_STATE_WORKING) {
            bd_addr_t a; gap_local_bd_addr(a);
            Serial.printf("[BT] WORKING. Pico: %s\n", bd_addr_to_str(a));
            gap_discoverable_control(1); gap_connectable_control(1);
            delay(300);
            if (have_addr && fail_count<MAX_FAILS) do_connect();
            else start_scan();
        }
        break;

    case GAP_EVENT_INQUIRY_RESULT: {
        bd_addr_t addr; gap_event_inquiry_result_get_bd_addr(pkt,addr);
        uint32_t cod=(uint32_t)gap_event_inquiry_result_get_class_of_device(pkt);
        Serial.printf("[SCAN] %s CoD=0x%06X\n", bd_addr_to_str(addr), cod);
        Serial.flush();
        if ((cod&0x001F00)==0x000500) {
            Serial.println("[SCAN] WiiMote! Connecting...");
            memcpy(remote_addr, addr, 6); have_addr=true;
            gap_inquiry_stop();
            do_connect();
        }
        break;
    }

    case GAP_EVENT_INQUIRY_COMPLETE:
        Serial.println("[SCAN] Done."); Serial.flush();
        if (app_state==ST_SCANNING) app_state=ST_IDLE;
        break;

    case HCI_EVENT_PIN_CODE_REQUEST: {
        bd_addr_t addr; hci_event_pin_code_request_get_bd_addr(pkt,addr);
        uint8_t pin[6]={addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]};
        gap_pin_code_response_binary(addr,pin,6);
        Serial.printf("[BT] PIN → %s\n", bd_addr_to_str(addr)); Serial.flush();
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
            uint8_t st=hid_subevent_connection_opened_get_status(pkt);
            if (st==ERROR_CODE_SUCCESS) {
                hid_cid=hid_subevent_connection_opened_get_hid_cid(pkt);
                app_state=ST_CONNECTED; g_connected=true; fail_count=0;
                on_wiimote_connected(remote_addr);
                Serial.printf("[HID] Connected! cid=%u\n",hid_cid);
            } else {
                Serial.printf("[HID] Failed 0x%02X",st);
                if(st==0x0B) Serial.print(" (paired elsewhere — SYNC 15s)");
                Serial.println();
                hid_cid=0; app_state=ST_IDLE; fail_count++;
                if(fail_count>=MAX_FAILS){ have_addr=false; fail_count=0; }
            }
            Serial.flush(); break;
        }

        case HID_SUBEVENT_DESCRIPTOR_AVAILABLE:
            if(hid_subevent_descriptor_available_get_status(pkt)==ERROR_CODE_SUCCESS) {
                desc_ok=true;
                Serial.println("[HID] Descriptor OK — configuring...");
                Serial.flush();
                wm_configure();
                Serial.println("[HID] Ready! Buttons→MIDI Notes, Accel→CC");
                Serial.flush();
            }
            break;

        case HID_SUBEVENT_REPORT:
            if(desc_ok)
                handle_report(hid_subevent_report_get_report(pkt),
                              hid_subevent_report_get_report_len(pkt));
            break;

        case HID_SUBEVENT_CONNECTION_CLOSED:
            Serial.println("[HID] Disconnected."); Serial.flush();
            hid_cid=0; app_state=ST_IDLE; desc_ok=false; g_connected=false;
            on_wiimote_disconnected();
            gap_discoverable_control(1);
            break;

        default: break;
        }
        break;
    default: break;
    }
}

// ── Public init ───────────────────────────────────────────────────────────
void bt_hid_init(const uint8_t* saved_addr, bool has_saved) {
    if (has_saved) {
        memcpy(remote_addr, saved_addr, 6);
        have_addr = true;
    }
    l2cap_init();
    hid_host_init(hid_desc, sizeof(hid_desc));
    hid_host_register_packet_handler(packet_handler);
    gap_set_default_link_policy_settings(
        LM_LINK_POLICY_ENABLE_SNIFF_MODE|LM_LINK_POLICY_ENABLE_ROLE_SWITCH);
    hci_set_master_slave_policy(HCI_ROLE_MASTER);
    hci_cb_reg.callback = &packet_handler;
    hci_add_event_handler(&hci_cb_reg);
    gap_discoverable_control(1);
    hci_power_control(HCI_POWER_ON);
}

// ── Public update (called from loop) ─────────────────────────────────────
void bt_hid_loop_update() {
    static uint32_t lc=0;
    if (app_state==ST_IDLE && millis()-lc>8000) {
        lc=millis();
        if (have_addr && fail_count<MAX_FAILS) do_connect();
        else start_scan();
    }
}