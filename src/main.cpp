// main.cpp — TinyUSB MIDI + LittleFS + mapping config
// BTstack lives in bt_init.cpp — never included here (avoids hid_report_type_t conflict)

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>   // No btstack.h in this file = no conflict
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

// Declared in bt_init.cpp
extern void bt_hid_init(const uint8_t* saved_addr, bool has_saved);
extern void bt_hid_loop_update();

// ── Persistent pairing ────────────────────────────────────────────────────
#define ADDR_FILE "/wiimote_addr.txt"
static uint8_t saved_addr[6];
static bool    have_saved = false;

static bool loadAddr() {
    if (!LittleFS.begin()) return false;
    File f = LittleFS.open(ADDR_FILE,"r");
    if (!f) { LittleFS.end(); return false; }
    String s = f.readStringUntil('\n'); f.close(); LittleFS.end();
    s.trim();
    if (s.length()<17) return false;
    // Parse "AA:BB:CC:DD:EE:FF"
    unsigned int b[6];
    if (sscanf(s.c_str(),"%x:%x:%x:%x:%x:%x",&b[0],&b[1],&b[2],&b[3],&b[4],&b[5])!=6)
        return false;
    for(int i=0;i<6;i++) saved_addr[i]=(uint8_t)b[i];
    Serial.printf("[PAIR] Loaded: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  saved_addr[0],saved_addr[1],saved_addr[2],
                  saved_addr[3],saved_addr[4],saved_addr[5]);
    Serial.flush();
    return true;
}

// Called from bt_init.cpp when connected
void on_wiimote_connected(const uint8_t* addr) {
    if (!LittleFS.begin()) return;
    File f = LittleFS.open(ADDR_FILE,"w");
    if (f) {
        f.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                 addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
        f.close();
    }
    LittleFS.end();
    Serial.println("[PAIR] Address saved."); Serial.flush();
}

void on_wiimote_disconnected() {
    Serial.println("[BT] Disconnected."); Serial.flush();
}

// ── Mapping config ────────────────────────────────────────────────────────
struct BtnMap {
    bool active=false,isNote=true;
    uint8_t note=0,cc=0,velocity=100,value_on=127,value_off=0;
};
struct AccelAxis { uint8_t cc=0; int16_t min_raw=-256,max_raw=256; };
struct Config {
    uint8_t  midi_ch=1;
    BtnMap   btn[13];
    bool     accel_en=true;
    uint8_t  accel_gate=0xFF;
    uint32_t accel_ms=50;
    AccelAxis accel[3];
} cfg;

static const char* BNAMES[13]={"TWO","ONE","B","A","MINUS","","","HOME","LEFT","RIGHT","DOWN","UP","PLUS"};
static int bidx(const char*n){ for(int i=0;i<13;i++) if(strcmp(BNAMES[i],n)==0)return i; return -1; }

static void loadMapping() {
    cfg=Config{};
    cfg.accel[0].cc=20; cfg.accel[1].cc=21; cfg.accel[2].cc=22;
    const struct{const char*n;uint8_t v;}defs[]={
        {"A",65},{"B",64},{"ONE",60},{"TWO",62},{"UP",74},
        {"DOWN",71},{"LEFT",69},{"RIGHT",70},{"PLUS",76},{"MINUS",67},{"HOME",72}};
    for(auto&d:defs){int i=bidx(d.n);if(i>=0){cfg.btn[i].active=true;cfg.btn[i].note=d.v;}}

    if(!LittleFS.begin()){Serial.println("[FS] defaults");return;}
    File f=LittleFS.open("/mapping.json","r");
    if(!f){LittleFS.end();Serial.println("[FS] No mapping.json");return;}
    JsonDocument doc;
    bool ok=(deserializeJson(doc,f)==DeserializationError::Ok);
    f.close(); LittleFS.end();
    if(!ok){Serial.println("[FS] JSON error");return;}

    cfg.midi_ch=doc["midi_channel"]|1;
    for(JsonPair kv:doc["buttons"].as<JsonObject>()){
        int i=bidx(kv.key().c_str()); if(i<0)continue;
        JsonObject b=kv.value();
        cfg.btn[i].active=true;
        cfg.btn[i].isNote=(strcmp(b["type"]|"note","note")==0);
        cfg.btn[i].note=b["note"]|0; cfg.btn[i].cc=b["cc"]|0;
        cfg.btn[i].velocity=b["velocity"]|100;
        cfg.btn[i].value_on=b["value_on"]|127;
        cfg.btn[i].value_off=b["value_off"]|0;
    }
    JsonObject ac=doc["accel"];
    if(!ac.isNull()){
        cfg.accel_en=ac["enabled"]|true;
        cfg.accel_ms=(uint32_t)(ac["update_rate_ms"]|50);
        int gi=bidx(ac["gate_button"]|"");
        cfg.accel_gate=(gi>=0)?(uint8_t)gi:0xFF;
        auto ax=ac["x"],ay=ac["y"],az=ac["z"];
        cfg.accel[0]={(uint8_t)(ax["cc"]|20),(int16_t)(ax["min_raw"]|-256),(int16_t)(ax["max_raw"]|256)};
        cfg.accel[1]={(uint8_t)(ay["cc"]|21),(int16_t)(ay["min_raw"]|-256),(int16_t)(ay["max_raw"]|256)};
        cfg.accel[2]={(uint8_t)(az["cc"]|22),(int16_t)(az["min_raw"]|-256),(int16_t)(az["max_raw"]|256)};
    }
    Serial.printf("[FS] ch=%d accel=%s gate=%d rate=%lums\n",
                  cfg.midi_ch,cfg.accel_en?"on":"off",
                  cfg.accel_gate==0xFF?-1:(int)cfg.accel_gate,cfg.accel_ms);
    Serial.flush();
}

// ── MIDI output ───────────────────────────────────────────────────────────
static uint16_t btn_prev=0;
static uint8_t  cc_prev[3]={0xFF,0xFF,0xFF};

static uint8_t rawToCC(int16_t v,int16_t mn,int16_t mx){
    v=constrain(v,mn,mx);
    return(uint8_t)(((int32_t)(v-mn)*127)/(mx-mn));}

static void process_buttons(uint16_t cur) {
    uint16_t changed=cur^btn_prev; if(!changed)return;
    for(int b=0;b<13;b++){
        if(!(changed&(1<<b)))continue;
        BtnMap&bm=cfg.btn[b]; if(!bm.active)continue;
        bool pressed=(cur>>b)&1;
        if(bm.isNote){
            if(pressed)MIDI_USB.sendNoteOn(bm.note,bm.velocity,cfg.midi_ch);
            else       MIDI_USB.sendNoteOff(bm.note,0,cfg.midi_ch);
        } else {
            MIDI_USB.sendControlChange(bm.cc,pressed?bm.value_on:bm.value_off,cfg.midi_ch);
        }
    }
    btn_prev=cur;
}

static void process_accel(uint16_t cur_btns) {
    if(!cfg.accel_en)return;
    if(cfg.accel_gate!=0xFF&&!((cur_btns>>cfg.accel_gate)&1))return;
    noInterrupts(); int16_t lx=g_raw_ax,ly=g_raw_ay,lz=g_raw_az; interrupts();
    uint8_t cx=rawToCC(lx,cfg.accel[0].min_raw,cfg.accel[0].max_raw);
    uint8_t cy=rawToCC(ly,cfg.accel[1].min_raw,cfg.accel[1].max_raw);
    uint8_t cz=rawToCC(lz,cfg.accel[2].min_raw,cfg.accel[2].max_raw);
    if(cx!=cc_prev[0]){MIDI_USB.sendControlChange(cfg.accel[0].cc,cx,cfg.midi_ch);cc_prev[0]=cx;}
    if(cy!=cc_prev[1]){MIDI_USB.sendControlChange(cfg.accel[1].cc,cy,cfg.midi_ch);cc_prev[1]=cy;}
    if(cz!=cc_prev[2]){MIDI_USB.sendControlChange(cfg.accel[2].cc,cz,cfg.midi_ch);cc_prev[2]=cz;}
}

// ── setup ─────────────────────────────────────────────────────────────────
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    usb_midi.begin();
    MIDI_USB.begin(MIDI_CHANNEL_OMNI);

    Serial.begin(115200);
    uint32_t t0=millis();
    while(!Serial&&millis()-t0<3000){delay(100);digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));}
    digitalWrite(LED_BUILTIN,HIGH);

    Serial.println("\n=== WiiMote MIDI Bridge ===");
    loadMapping();
    have_saved = loadAddr();

    bt_hid_init(saved_addr, have_saved);
    Serial.println("[BT] Power on..."); Serial.flush();
}

// ── loop ──────────────────────────────────────────────────────────────────
void loop() {
    MIDI_USB.read();

    // LED
    {static uint32_t lt=0;static bool lon=false;
     if(millis()-lt>(g_connected?100:700)){lt=millis();lon=!lon;digitalWrite(LED_BUILTIN,lon);}}

    bt_hid_loop_update();

    // Buttons
    if(g_new_report){
        noInterrupts(); uint16_t bc=g_btn_cur; g_new_report=false; interrupts();
        process_buttons(bc);
    }

    // Accel timer
    {static uint32_t la=0;
     if(g_connected&&millis()-la>=cfg.accel_ms){
         la=millis();
         noInterrupts(); uint16_t bc=g_btn_cur; interrupts();
         process_accel(bc);
     }}
}