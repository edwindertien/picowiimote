// ═══════════════════════════════════════════════════════════════════════════
// main.cpp — WiiMote MIDI Bridge (TinyUSB MIDI + LittleFS config)
//
// ISOLATION RULE: Do NOT include btstack.h here — hid_report_type_t conflict.
// bt_init.cpp owns all BTstack code; communicate via extern volatile globals.
//
// Serial commands (single char, no newline needed):
//   's'  toggle 20Hz CSV stream (suppresses event log while active)
//   'r'  reload mapping.json from LittleFS
//   'f'  forget saved WiiMote address (forces scan on next connect)
//
// CSV columns (header printed on stream start):
//   btns,ax,ay,az,nc_jx,nc_jy,nc_ax,nc_ay,nc_az,nc_C,nc_Z,
//   gh_btns,gh_whammy,gh_touch,gh_tilt_x,gh_tilt_z
// ═══════════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// ── USB MIDI ──────────────────────────────────────────────────────────────
Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI_USB);

// ── Shared state from bt_init.cpp ────────────────────────────────────────
extern volatile uint16_t g_btn_cur;
extern volatile int16_t  g_raw_ax, g_raw_ay, g_raw_az;
extern volatile bool     g_new_report, g_connected;
extern volatile uint8_t  g_ext_type;
extern volatile int8_t   g_nc_jx, g_nc_jy;
extern volatile int16_t  g_nc_ax, g_nc_ay, g_nc_az;
extern volatile bool     g_nc_btn_c, g_nc_btn_z;
extern volatile uint16_t g_gh_btns;
extern volatile uint8_t  g_gh_whammy;
extern volatile int8_t   g_gh_touch, g_gh_tilt_x, g_gh_tilt_z;
extern volatile uint16_t g_bb_tl, g_bb_tr, g_bb_bl, g_bb_br;

extern void bt_hid_init(const uint8_t* saved_addr, bool has_saved, uint8_t ext_type);
extern void bt_hid_loop_update();
extern void bt_detect_extension();

// ── Stream mode flag (checked by process_buttons to suppress event log) ───
static bool stream_mode = false;

// ═══════════════════════════════════════════════════════════════════════════
// LITTLEFS — mount once, do all operations, unmount once
// Calling begin()/end() multiple times in sequence is unreliable on the
// Earle framework. We mount at setup(), keep it mounted, unmount only on
// explicit need (e.g. before a long delay).
// ═══════════════════════════════════════════════════════════════════════════
static bool fs_mounted = false;

static bool fs_mount() {
    if (fs_mounted) return true;
    fs_mounted = LittleFS.begin();
    if (!fs_mounted) Serial.println("[FS] Mount FAILED");
    return fs_mounted;
}

// ── Persistent WiiMote address ────────────────────────────────────────────
#define ADDR_FILE "/wiimote_addr.txt"
static uint8_t saved_addr[6];
static bool    have_saved = false;

static bool loadAddr() {
    if (!fs_mount()) return false;
    File f = LittleFS.open(ADDR_FILE, "r");
    if (!f) {
        Serial.println("[PAIR] No saved address found — will scan");
        Serial.flush();
        return false;
    }
    String s = f.readStringUntil('\n');
    f.close();
    s.trim();
    if (s.length() < 17) {
        Serial.println("[PAIR] Saved address file corrupt");
        Serial.flush();
        return false;
    }
    unsigned int b[6];
    if (sscanf(s.c_str(), "%x:%x:%x:%x:%x:%x",
               &b[0],&b[1],&b[2],&b[3],&b[4],&b[5]) != 6) {
        Serial.println("[PAIR] Saved address parse failed");
        Serial.flush();
        return false;
    }
    for (int i=0;i<6;i++) saved_addr[i] = (uint8_t)b[i];
    Serial.printf("[PAIR] Loaded: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  saved_addr[0],saved_addr[1],saved_addr[2],
                  saved_addr[3],saved_addr[4],saved_addr[5]);
    Serial.flush();
    return true;
}

static void forgetAddr() {
    if (!fs_mount()) return;
    LittleFS.remove(ADDR_FILE);
    have_saved = false;
    Serial.println("[PAIR] Saved address forgotten — will scan next boot");
    Serial.flush();
}

// Called from bt_init.cpp on successful connection — log only, no direct connect
void on_wiimote_connected(const uint8_t* addr) {
    Serial.printf("[BT] Connected to %02X:%02X:%02X:%02X:%02X:%02X\n",
                  addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
    Serial.flush();
}

void on_wiimote_disconnected() {
    Serial.println("[BT] WiiMote disconnected");
    Serial.flush();
}

// ═══════════════════════════════════════════════════════════════════════════
// MAPPING CONFIG
// ═══════════════════════════════════════════════════════════════════════════

struct BtnMap {
    bool    active   = false;
    bool    isNote   = true;
    uint8_t note     = 0;
    uint8_t cc       = 0;
    uint8_t velocity = 100;
    uint8_t value_on = 127, value_off = 0;
};

struct AccelAxis {
    uint8_t cc      = 0;
    int16_t min_raw = -256;
    int16_t max_raw =  256;
};

struct NunchukMap {
    bool      enabled = false;
    AccelAxis joy_x, joy_y;
    AccelAxis nc_ax, nc_ay, nc_az;
    BtnMap    btn_c, btn_z;
};

struct GuitarMap {
    bool    enabled   = false;
    BtnMap  btn[9];
    uint8_t whammy_cc = 0;
    uint8_t touch_cc  = 0;
    uint8_t tilt_x_cc = 0;
    uint8_t tilt_z_cc = 0;
};

struct BalanceBoardMap {
    bool     enabled  = false;
    uint8_t  tl_cc    = 0;      // top-left raw sensor
    uint8_t  tr_cc    = 0;      // top-right raw sensor
    uint8_t  bl_cc    = 0;      // bottom-left raw sensor
    uint8_t  br_cc    = 0;      // bottom-right raw sensor
    uint8_t  total_cc = 0;      // sum of all 4 (total weight)
    uint8_t  x_cc     = 0;      // centre-of-gravity left/right
    uint8_t  y_cc     = 0;      // centre-of-gravity front/back
    uint16_t max_raw  = 40000;  // raw sum at max expected weight
};

struct Config {
    uint8_t       midi_ch    = 1;
    BtnMap        btn[13];
    bool          accel_en   = true;
    uint8_t       accel_gate = 0xFF;
    uint32_t      accel_ms   = 50;
    AccelAxis     accel[3];
    NunchukMap    nunchuk;
    GuitarMap     guitar;
    BalanceBoardMap balance;
    uint8_t       ext_type   = 0;  // 0=none 1=nunchuk 2=guitar 3=balance
} cfg;

// Called from bt_init.cpp after extension detection sequence completes
void on_extension_detected(uint8_t ext_type, const uint8_t* id6) {
    cfg.ext_type = ext_type;

    if (ext_type == 1) {
        cfg.nunchuk.enabled = true;  // always enable decoding
        if (cfg.nunchuk.joy_x.cc == 0 && cfg.nunchuk.joy_y.cc == 0) {
            // No MIDI mapping loaded from JSON — use defaults
            Serial.println("[EXT] Nunchuk: no mapping in JSON, using defaults (CC30-34)");
            cfg.nunchuk.joy_x  = {30, -128, 127};
            cfg.nunchuk.joy_y  = {31, -128, 127};
            cfg.nunchuk.nc_ax  = {32, -256, 256};
            cfg.nunchuk.nc_ay  = {33, -256, 256};
            cfg.nunchuk.nc_az  = {34, -256, 256};
            cfg.nunchuk.btn_c  = {true, true, 48, 0, 100, 127, 0};
            cfg.nunchuk.btn_z  = {true, true, 50, 0, 100, 127, 0};
        } else {
            Serial.println("[EXT] Nunchuk: using mapping from mapping.json");
        }
    }
    if (ext_type == 2) {
        cfg.guitar.enabled = true;
        Serial.println("[EXT] Guitar Hero: enabled");
    }
    if (ext_type == 3) {
        cfg.balance.enabled = true;
        if (cfg.balance.total_cc == 0) {
            // Defaults: total weight → CC40, CoG X → CC41, CoG Y → CC42
            Serial.println("[EXT] Balance Board: using defaults (total=CC40, x=CC41, y=CC42)");
            cfg.balance.total_cc = 40;
            cfg.balance.x_cc     = 41;
            cfg.balance.y_cc     = 42;
            cfg.balance.max_raw  = 40000;
        } else {
            Serial.println("[EXT] Balance Board: using mapping from mapping.json");
        }
    }

    Serial.printf("[EXT] Extension ready: type=%d (%s)\n",
                  ext_type,
                  ext_type==1?"Nunchuk":ext_type==2?"Guitar Hero":"none");
    Serial.flush();
}

static const char* BNAMES[13] = {
    "TWO","ONE","B","A","MINUS","","","HOME",
    "LEFT","RIGHT","DOWN","UP","PLUS"
};
static int bidx(const char* n) {
    if (!n || !n[0]) return -1;
    for (int i=0;i<13;i++) if (strcmp(BNAMES[i],n)==0) return i;
    return -1;
}

static BtnMap parseBtnMap(JsonObject b) {
    BtnMap bm;
    bm.active    = true;
    bm.isNote    = (strcmp(b["type"]|"note","note")==0);
    bm.note      = b["note"]      | 0;
    bm.cc        = b["cc"]        | 0;
    bm.velocity  = b["velocity"]  | 100;
    bm.value_on  = b["value_on"]  | 127;
    bm.value_off = b["value_off"] | 0;
    return bm;
}

static AccelAxis parseAxis(JsonObject a, uint8_t default_cc) {
    AccelAxis ax;
    ax.cc      = a["cc"]      | default_cc;
    ax.min_raw = a["min_raw"] | -256;
    ax.max_raw = a["max_raw"] |  256;
    return ax;
}

static void loadMapping() {
    // Reset to defaults
    cfg = Config{};
    cfg.accel[0] = {20, -256, 256};
    cfg.accel[1] = {21, -256, 256};
    cfg.accel[2] = {22, -256, 256};
    const struct{const char*n;uint8_t v;}defs[]={
        {"A",65},{"B",64},{"ONE",60},{"TWO",62},{"UP",74},
        {"DOWN",71},{"LEFT",69},{"RIGHT",70},{"PLUS",76},{"MINUS",67},{"HOME",72}
    };
    for (auto& d:defs) {
        int i=bidx(d.n);
        if (i>=0){ cfg.btn[i].active=true; cfg.btn[i].note=d.v; }
    }

    // LittleFS is already mounted by the time loadMapping is called
    if (!fs_mounted) {
        Serial.println("[FS] Not mounted — using built-in defaults");
        Serial.flush();
        return;
    }

    File f = LittleFS.open("/mapping.json","r");
    if (!f) {
        Serial.println("[FS] No mapping.json — using built-in defaults");
        Serial.flush();
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err != DeserializationError::Ok) {
        Serial.printf("[FS] JSON error: %s — using defaults\n", err.c_str());
        Serial.flush();
        return;
    }

    cfg.midi_ch = doc["midi_channel"] | 1;

    for (JsonPair kv : doc["buttons"].as<JsonObject>()) {
        int i = bidx(kv.key().c_str());
        if (i < 0) continue;
        cfg.btn[i] = parseBtnMap(kv.value().as<JsonObject>());
    }

    JsonObject ac = doc["accel"];
    if (!ac.isNull()) {
        cfg.accel_en   = ac["enabled"] | true;
        cfg.accel_ms   = (uint32_t)(ac["update_rate_ms"] | 50);
        int gi = bidx(ac["gate_button"]|"");
        cfg.accel_gate = (gi >= 0) ? (uint8_t)gi : 0xFF;
        cfg.accel[0]   = parseAxis(ac["x"].as<JsonObject>(), 20);
        cfg.accel[1]   = parseAxis(ac["y"].as<JsonObject>(), 21);
        cfg.accel[2]   = parseAxis(ac["z"].as<JsonObject>(), 22);
    }

    JsonObject nc = doc["nunchuk"];
    if (!nc.isNull() && (nc["enabled"]|false)) {
        cfg.nunchuk.enabled = true;
        cfg.nunchuk.joy_x   = parseAxis(nc["joy_x"].as<JsonObject>(),   30);
        cfg.nunchuk.joy_y   = parseAxis(nc["joy_y"].as<JsonObject>(),   31);
        cfg.nunchuk.nc_ax   = parseAxis(nc["accel_x"].as<JsonObject>(), 32);
        cfg.nunchuk.nc_ay   = parseAxis(nc["accel_y"].as<JsonObject>(), 33);
        cfg.nunchuk.nc_az   = parseAxis(nc["accel_z"].as<JsonObject>(), 34);
        if (!nc["C"].isNull()) cfg.nunchuk.btn_c = parseBtnMap(nc["C"].as<JsonObject>());
        if (!nc["Z"].isNull()) cfg.nunchuk.btn_z = parseBtnMap(nc["Z"].as<JsonObject>());
        cfg.ext_type = 1;
        Serial.println("[FS] Nunchuk enabled → ext_type=1, report mode 0x35");
    }

    JsonObject gh = doc["guitar_hero"];
    if (!gh.isNull() && (gh["enabled"]|false)) {
        cfg.guitar.enabled   = true;
        cfg.guitar.whammy_cc = gh["whammy_cc"] | 0;
        cfg.guitar.touch_cc  = gh["touch_cc"]  | 0;
        cfg.guitar.tilt_x_cc = gh["tilt_x_cc"] | 0;
        cfg.guitar.tilt_z_cc = gh["tilt_z_cc"] | 0;
        const char* gnames[] = {
            "strum_down","strum_up","minus","plus",
            "green","red","yellow","blue","orange"
        };
        JsonObject gbtns = gh["buttons"];
        for (int i=0;i<9;i++)
            if (!gbtns[gnames[i]].isNull())
                cfg.guitar.btn[i] = parseBtnMap(gbtns[gnames[i]].as<JsonObject>());
        cfg.ext_type = 2;
        Serial.println("[FS] Guitar Hero enabled → ext_type=2, report mode 0x35");
    }

    JsonObject bb = doc["balance_board"];
    if (!bb.isNull() && (bb["enabled"]|false)) {
        cfg.balance.enabled  = true;
        cfg.balance.total_cc = bb["total_cc"] | 40;
        cfg.balance.x_cc     = bb["x_cc"]     | 41;
        cfg.balance.y_cc     = bb["y_cc"]     | 42;
        cfg.balance.tl_cc    = bb["tl_cc"]    | 0;
        cfg.balance.tr_cc    = bb["tr_cc"]    | 0;
        cfg.balance.bl_cc    = bb["bl_cc"]    | 0;
        cfg.balance.br_cc    = bb["br_cc"]    | 0;
        cfg.balance.max_raw  = bb["max_raw"]  | 40000;
        cfg.ext_type = 3;
        Serial.println("[FS] Balance Board enabled → ext_type=3");
    }

    Serial.printf("[FS] Mapping OK: ch=%d accel=%s gate=%d rate=%lums ext=%d\n",
                  cfg.midi_ch, cfg.accel_en?"on":"off",
                  cfg.accel_gate==0xFF ? -1 : (int)cfg.accel_gate,
                  cfg.accel_ms, cfg.ext_type);
    Serial.flush();
}

// ═══════════════════════════════════════════════════════════════════════════
// MIDI OUTPUT
// ═══════════════════════════════════════════════════════════════════════════

static uint8_t rawToCC(int16_t v, int16_t mn, int16_t mx) {
    v = constrain(v, mn, mx);
    return (uint8_t)(((int32_t)(v-mn) * 127) / (mx-mn));
}

// ── WiiMote core buttons ──────────────────────────────────────────────────
static uint16_t btn_prev = 0;

static void process_buttons(uint16_t cur) {
    uint16_t changed = cur ^ btn_prev;
    if (!changed) return;
    for (int b=0; b<13; b++) {
        if (!(changed & (1<<b))) continue;
        BtnMap& bm = cfg.btn[b];
        if (!bm.active) continue;
        bool pressed = (cur >> b) & 1;
        if (bm.isNote) {
            if (pressed) MIDI_USB.sendNoteOn (bm.note, bm.velocity, cfg.midi_ch);
            else         MIDI_USB.sendNoteOff(bm.note, 0,           cfg.midi_ch);
        } else {
            MIDI_USB.sendControlChange(bm.cc,
                pressed ? bm.value_on : bm.value_off, cfg.midi_ch);
        }
        // Event log — suppressed in stream mode to keep CSV clean
        if (!stream_mode) {
            if (bm.isNote)
                Serial.printf("[BTN] %-5s %s → Note%s %d ch%d\n",
                              BNAMES[b], pressed?"▼":"▲",
                              pressed?"On ":"Off", bm.note, cfg.midi_ch);
            else
                Serial.printf("[BTN] %-5s %s → CC%d=%d ch%d\n",
                              BNAMES[b], pressed?"▼":"▲",
                              bm.cc, pressed?bm.value_on:bm.value_off, cfg.midi_ch);
            Serial.flush();
        }
    }
    btn_prev = cur;
}

// ── WiiMote accelerometer ─────────────────────────────────────────────────
static uint8_t cc_prev_wm[3] = {0xFF,0xFF,0xFF};

static void process_accel(uint16_t cur_btns) {
    if (!cfg.accel_en) return;
    // Gate: 0xFF = always send; else only while that button bit is set
    if (cfg.accel_gate != 0xFF && !((cur_btns >> cfg.accel_gate) & 1)) return;

    noInterrupts();
    int16_t lx=g_raw_ax, ly=g_raw_ay, lz=g_raw_az;
    interrupts();

    uint8_t cx = rawToCC(lx, cfg.accel[0].min_raw, cfg.accel[0].max_raw);
    uint8_t cy = rawToCC(ly, cfg.accel[1].min_raw, cfg.accel[1].max_raw);
    uint8_t cz = rawToCC(lz, cfg.accel[2].min_raw, cfg.accel[2].max_raw);

    if (cfg.accel[0].cc && cx != cc_prev_wm[0])
        { MIDI_USB.sendControlChange(cfg.accel[0].cc, cx, cfg.midi_ch); cc_prev_wm[0]=cx; }
    if (cfg.accel[1].cc && cy != cc_prev_wm[1])
        { MIDI_USB.sendControlChange(cfg.accel[1].cc, cy, cfg.midi_ch); cc_prev_wm[1]=cy; }
    if (cfg.accel[2].cc && cz != cc_prev_wm[2])
        { MIDI_USB.sendControlChange(cfg.accel[2].cc, cz, cfg.midi_ch); cc_prev_wm[2]=cz; }
}

// ── Nunchuk ───────────────────────────────────────────────────────────────
static uint8_t cc_prev_nc[5] = {0xFF,0xFF,0xFF,0xFF,0xFF};
static bool    nc_c_prev = false, nc_z_prev = false;

static void process_nunchuk() {
    // Guard: nunchuk must be enabled in config AND ext_type must be 1
    // g_ext_type is initialised from cfg.ext_type in bt_hid_init,
    // so this works even before extension auto-detection is implemented
    if (!cfg.nunchuk.enabled || g_ext_type != 1) return;

    noInterrupts();
    int8_t  jx=g_nc_jx, jy=g_nc_jy;
    int16_t ax=g_nc_ax,  ay=g_nc_ay,  az=g_nc_az;
    bool    bc=g_nc_btn_c, bz=g_nc_btn_z;
    interrupts();

    auto sendAxis = [&](AccelAxis& axis, int16_t val, uint8_t& prev) {
        if (!axis.cc) return;
        uint8_t v = rawToCC(val, axis.min_raw, axis.max_raw);
        if (v != prev) {
            MIDI_USB.sendControlChange(axis.cc, v, cfg.midi_ch);
            prev = v;
        }
    };
    sendAxis(cfg.nunchuk.joy_x, (int16_t)jx, cc_prev_nc[0]);
    sendAxis(cfg.nunchuk.joy_y, (int16_t)jy, cc_prev_nc[1]);
    sendAxis(cfg.nunchuk.nc_ax, ax,           cc_prev_nc[2]);
    sendAxis(cfg.nunchuk.nc_ay, ay,           cc_prev_nc[3]);
    sendAxis(cfg.nunchuk.nc_az, az,           cc_prev_nc[4]);

    auto sendBtn = [&](BtnMap& bm, bool pressed, bool& prev, const char* name) {
        if (!bm.active || pressed == prev) return;
        if (bm.isNote) {
            if (pressed) MIDI_USB.sendNoteOn (bm.note, bm.velocity, cfg.midi_ch);
            else         MIDI_USB.sendNoteOff(bm.note, 0,           cfg.midi_ch);
        } else {
            MIDI_USB.sendControlChange(bm.cc,
                pressed ? bm.value_on : bm.value_off, cfg.midi_ch);
        }
        if (!stream_mode) {
            Serial.printf("[NC] %s %s → Note%s %d\n",
                          name, pressed?"▼":"▲",
                          pressed?"On ":"Off", bm.note);
            Serial.flush();
        }
        prev = pressed;
    };
    sendBtn(cfg.nunchuk.btn_c, bc, nc_c_prev, "C");
    sendBtn(cfg.nunchuk.btn_z, bz, nc_z_prev, "Z");
}

// ── Guitar Hero ───────────────────────────────────────────────────────────
static uint16_t gh_btn_prev = 0;
static uint8_t  cc_prev_gh[4] = {0xFF,0xFF,0xFF,0xFF};

static void process_guitar() {
    if (!cfg.guitar.enabled || g_ext_type != 2) return;

    noInterrupts();
    uint16_t btns   = g_gh_btns;
    uint8_t  whammy = g_gh_whammy;
    int8_t   touch  = g_gh_touch;
    int8_t   tx     = g_gh_tilt_x, tz = g_gh_tilt_z;
    interrupts();

    uint16_t changed = btns ^ gh_btn_prev;
    for (int b=0; b<9; b++) {
        if (!(changed & (1<<b))) continue;
        BtnMap& bm = cfg.guitar.btn[b];
        if (!bm.active) continue;
        bool pressed = (btns >> b) & 1;
        if (bm.isNote) {
            if (pressed) MIDI_USB.sendNoteOn (bm.note, bm.velocity, cfg.midi_ch);
            else         MIDI_USB.sendNoteOff(bm.note, 0,           cfg.midi_ch);
        } else {
            MIDI_USB.sendControlChange(bm.cc, pressed?bm.value_on:bm.value_off, cfg.midi_ch);
        }
    }
    gh_btn_prev = btns;

    auto sendCC = [&](uint8_t cc, int16_t val, int16_t mn, int16_t mx, uint8_t& prev) {
        if (!cc) return;
        uint8_t v = rawToCC(val, mn, mx);
        if (v != prev) { MIDI_USB.sendControlChange(cc, v, cfg.midi_ch); prev=v; }
    };
    sendCC(cfg.guitar.whammy_cc, whammy,   0,  31, cc_prev_gh[0]);
    sendCC(cfg.guitar.touch_cc,  touch,  -15,  15, cc_prev_gh[1]);
    sendCC(cfg.guitar.tilt_x_cc, tx,    -128, 127, cc_prev_gh[2]);
    sendCC(cfg.guitar.tilt_z_cc, tz,    -128, 127, cc_prev_gh[3]);
}

// ── Balance Board ─────────────────────────────────────────────────────────
static uint8_t cc_prev_bb[7] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static void process_balance() {
    if (!cfg.balance.enabled || g_ext_type != 3) return;

    noInterrupts();
    uint16_t tl=g_bb_tl, tr=g_bb_tr, bl=g_bb_bl, br=g_bb_br;
    interrupts();

    uint32_t total = (uint32_t)tl + tr + bl + br;
    uint16_t mx = cfg.balance.max_raw;

    auto sendCC = [&](uint8_t cc, uint32_t val, uint32_t mn, uint32_t vmax, uint8_t& prev) {
        if (!cc) return;
        val = constrain(val, mn, vmax);
        uint8_t v = (uint8_t)((val - mn) * 127 / (vmax - mn));
        if (v != prev) { MIDI_USB.sendControlChange(cc, v, cfg.midi_ch); prev=v; }
    };

    sendCC(cfg.balance.tl_cc,    tl,    0, mx/4, cc_prev_bb[0]);
    sendCC(cfg.balance.tr_cc,    tr,    0, mx/4, cc_prev_bb[1]);
    sendCC(cfg.balance.bl_cc,    bl,    0, mx/4, cc_prev_bb[2]);
    sendCC(cfg.balance.br_cc,    br,    0, mx/4, cc_prev_bb[3]);
    sendCC(cfg.balance.total_cc, total, 0, mx,   cc_prev_bb[4]);

    // Centre-of-gravity: 0=full left/back, 64=centre, 127=full right/front
    if (cfg.balance.x_cc && total > 100) {
        uint32_t right = tr + br;
        uint8_t v = (uint8_t)(right * 127 / total);
        if (v != cc_prev_bb[5]) { MIDI_USB.sendControlChange(cfg.balance.x_cc, v, cfg.midi_ch); cc_prev_bb[5]=v; }
    }
    if (cfg.balance.y_cc && total > 100) {
        uint32_t top = tl + tr;
        uint8_t v = (uint8_t)(top * 127 / total);
        if (v != cc_prev_bb[6]) { MIDI_USB.sendControlChange(cfg.balance.y_cc, v, cfg.midi_ch); cc_prev_bb[6]=v; }
    }
}
#define STREAM_HZ 20
#define STREAM_MS (1000/STREAM_HZ)

static void print_csv_header() {
    Serial.println("btns,ax,ay,az,nc_jx,nc_jy,nc_ax,nc_ay,nc_az,nc_C,nc_Z,"
                   "gh_btns,gh_whammy,gh_touch,gh_tilt_x,gh_tilt_z,"
                   "bb_tl,bb_tr,bb_bl,bb_br");
    Serial.flush();
}

static void emit_csv() {
    noInterrupts();
    uint16_t btns=g_btn_cur;
    int16_t  ax=g_raw_ax, ay=g_raw_ay, az=g_raw_az;
    int8_t   jx=g_nc_jx,  jy=g_nc_jy;
    int16_t  nax=g_nc_ax,  nay=g_nc_ay, naz=g_nc_az;
    bool     nc=g_nc_btn_c, nz=g_nc_btn_z;
    uint16_t ghb=g_gh_btns;
    uint8_t  ghw=g_gh_whammy;
    int8_t   ght=g_gh_touch, ghtx=g_gh_tilt_x, ghtz=g_gh_tilt_z;
    uint16_t tl=g_bb_tl, tr=g_bb_tr, bl=g_bb_bl, br=g_bb_br;
    interrupts();

    Serial.printf("%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%d,%d,%d,%u,%u,%u,%u\n",
                  btns, ax, ay, az,
                  jx, jy, nax, nay, naz, nc?1:0, nz?1:0,
                  ghb, ghw, ght, ghtx, ghtz,
                  tl, tr, bl, br);
    Serial.flush();
}

// ═══════════════════════════════════════════════════════════════════════════
// SERIAL COMMANDS
// ═══════════════════════════════════════════════════════════════════════════
static void handle_serial_cmd(char c) {
    switch (c) {
    case 's':
        stream_mode = !stream_mode;
        if (stream_mode) {
            Serial.println("[DBG] Stream ON — event log suppressed");
            print_csv_header();
        } else {
            Serial.println("[DBG] Stream OFF — event log active");
        }
        Serial.flush();
        break;
    case 'r':
        if (!stream_mode) {
            Serial.println("[DBG] Reloading mapping.json...");
            loadMapping();
        }
        break;
    case 'f':
        if (!stream_mode) forgetAddr();
        break;
    case 'e':
        if (!stream_mode) {
            Serial.println("[DBG] Running extension detection...");
            Serial.flush();
            bt_detect_extension();
        }
        break;
    default:
        if (!stream_mode) {
            Serial.printf("[DBG] Cmds: s=stream r=reload f=forget-addr\n");
            Serial.flush();
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP / LOOP
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // USB MIDI must init before Serial
    usb_midi.begin();
    MIDI_USB.begin(MIDI_CHANNEL_OMNI);

    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis()-t0 < 3000) {
        delay(100);
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.println("\n=== WiiMote MIDI Bridge ===");
    Serial.println("[CMD] s=stream  r=reload  f=forget-addr  e=detect-extension");
    Serial.flush();

    // Mount LittleFS for mapping.json (stays mounted)
    if (fs_mount()) {
        Serial.println("[FS] Mounted OK");
        Serial.flush();
    }

    loadMapping();

    // Scan-only mode — no saved address needed
    bt_hid_init(nullptr, false, cfg.ext_type);
    Serial.println("[BT] Power on — scanning for WiiMote...");
    Serial.println("[BT] Hold 1+2 on WiiMote when LEDs are blinking.");
    Serial.flush();
}

void loop() {
    MIDI_USB.read();

    // LED: fast=connected, slow=idle
    {
        static uint32_t lt=0; static bool lon=false;
        if (millis()-lt > (g_connected?100:700))
            { lt=millis(); lon=!lon; digitalWrite(LED_BUILTIN,lon); }
    }

    bt_hid_loop_update();

    // Serial commands
    while (Serial.available())
        handle_serial_cmd((char)Serial.read());

    // CSV stream (20Hz)
    if (stream_mode && g_connected) {
        static uint32_t ls=0;
        if (millis()-ls >= STREAM_MS) { ls=millis(); emit_csv(); }
    }

    // Process new HID report → MIDI
    if (g_new_report) {
        noInterrupts();
        uint16_t bc = g_btn_cur;
        g_new_report = false;
        interrupts();

        process_buttons(bc);
        process_nunchuk();
        process_guitar();
        process_balance();
    }

    // Accel CC on timer
    if (g_connected) {
        static uint32_t la=0;
        if (millis()-la >= cfg.accel_ms) {
            la=millis();
            noInterrupts(); uint16_t bc=g_btn_cur; interrupts();
            process_accel(bc);
        }
    }
}