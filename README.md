# WiiMote MIDI Bridge

Raspberry Pi Pico W firmware that bridges a Nintendo WiiMote (and Nunchuk /
Guitar Hero extension) to USB MIDI. The Pico W appears as **"WiiMote MIDI"**
in your OS device list alongside a CDC Serial port for debug output.

Information and inspiration from [WiiBrew](https://wiibrew.org/wiki/Wiimote/Extension_Controllers) and also from thie [Cornell university student project](https://github.com/EmithU/PicoWiimote-public)

## Hardware

| Item | Detail |
|------|--------|
| Board | Raspberry Pi Pico W (RP2040 + CYW43439) |
| WiiMote | Nintendo RVL-CNT-01 (classic, not Motion Plus) |
| Extensions | Nunchuk ✓, Guitar Hero 3 controller (partial) |

## Features

- **Auto scan**: scans for any WiiMote (CoD 0x002504) on boot and reconnects every 12s
- **Extension auto-detect**: probes extension port after connection, identifies Nunchuk or GH3 automatically
- **JSON mapping**: `data/mapping.json` configures all MIDI assignments without recompiling
- **Accel gate**: accelerometer CC output only while a named button is held (e.g. B), or always-on
- **Nunchuk**: joystick X/Y, accel X/Y/Z, buttons C/Z → MIDI CC + notes (fully working ✓)
- **Guitar Hero 3**: frets, strum, whammy, touch bar, tilt → MIDI (structural, byte layout needs field verification)
- **CSV stream**: send `'s'` over Serial for 20Hz comma-separated sensor data (for plotting)
- **USB identity**: shows as "WiiMote MIDI / MyStudio" — edit `platformio.ini` to customise

## Build & Flash

Requires PlatformIO with the maxgerhardt raspberrypi platform.

```bash
# Full flash (filesystem + firmware in one step):
pio run -t upload

# The pre-script upload_fs_pre.py uploads data/mapping.json to LittleFS
# automatically before flashing firmware, so you never need to run
# uploadfs separately.

# Serial monitor
pio device monitor
```

## First Connection

1. Flash and open Serial Monitor
2. Pico scans automatically — `[SCAN] Starting 10s inquiry`
3. Hold **1+2** on WiiMote (all 4 LEDs blink rapidly)
4. Connection happens within seconds
5. If Nunchuk is plugged in, extension detection runs automatically

**If connection is rejected** (`0x0B` error): the WiiMote is bonded to another
host (Mac, Wii console). Open the battery cover and hold the red **SYNC** button
for 15 seconds to clear bonding, then hold 1+2.

## Serial Commands

Send a single character in the Serial monitor (no newline needed):

| Key | Action |
|-----|--------|
| `s` | Toggle 20Hz CSV data stream (suppresses event log while active) |
| `r` | Reload `mapping.json` from LittleFS |
| `f` | Forget saved WiiMote address (unused in current scan-only mode) |
| `e` | Re-run extension detection (useful if you plug Nunchuk while connected) |

## CSV Stream Format

```
btns,ax,ay,az,nc_jx,nc_jy,nc_ax,nc_ay,nc_az,nc_C,nc_Z,gh_btns,gh_whammy,gh_touch,gh_tilt_x,gh_tilt_z
```

`btns` is the raw WiiMote button bitmask (bit 0=TWO, 1=ONE, 2=B, 3=A, 4=MINUS,
7=HOME, 8=LEFT, 9=RIGHT, 10=DOWN, 11=UP, 12=PLUS).

## mapping.json Reference

Location: `data/mapping.json` — uploaded to LittleFS automatically on flash.

### Button mapping
```json
"buttons": {
  "A":    { "type": "note", "note": 65, "velocity": 100 },
  "B":    { "type": "cc",   "cc": 10,   "value_on": 127, "value_off": 0 }
}
```
Button names: `A B ONE TWO UP DOWN LEFT RIGHT PLUS MINUS HOME`

### Accelerometer
```json
"accel": {
  "enabled": true,
  "gate_button": "B",       // hold B to enable accel CC; "" = always on
  "update_rate_ms": 50,
  "x": { "cc": 20, "min_raw": -256, "max_raw": 256 },
  "y": { "cc": 21, "min_raw": -256, "max_raw": 256 },
  "z": { "cc": 22, "min_raw": -256, "max_raw": 256 }
}
```

### Nunchuk
```json
"nunchuk": {
  "enabled": true,
  "joy_x":   { "cc": 30, "min_raw": -128, "max_raw": 127 },
  "joy_y":   { "cc": 31, "min_raw": -128, "max_raw": 127 },
  "accel_x": { "cc": 32, "min_raw": -256, "max_raw": 256 },
  "accel_y": { "cc": 33, "min_raw": -256, "max_raw": 256 },
  "accel_z": { "cc": 34, "min_raw": -256, "max_raw": 256 },
  "C": { "type": "note", "note": 48, "velocity": 100 },
  "Z": { "type": "note", "note": 50, "velocity": 100 }
}
```
Extension auto-detected at runtime — `enabled: false` in JSON is overridden
automatically when Nunchuk is found.

### Guitar Hero 3
```json
"guitar_hero": {
  "enabled": true,
  "whammy_cc": 23,
  "buttons": { "green": { "type": "note", "note": 48 }, ... }
}
```

## Project Structure

```
├── platformio.ini          Build config, USB VID/PID/name
├── upload_fs_pre.py        Pre-upload script (auto uploadfs)
├── data/
│   └── mapping.json        MIDI mapping (auto-uploaded with firmware)
└── src/
    ├── btstack_config.h    BTstack compile options
    ├── bt_init.cpp         All BTstack code (isolated from TinyUSB)
    └── main.cpp            TinyUSB MIDI + LittleFS + MIDI logic
```

## Known Limitations

See `context.md` for full technical details.

- Scan-only pairing: always needs 1+2 pressed on WiiMote at boot
- Guitar Hero byte layout needs field verification (use CSV stream to check)
- IR camera not implemented (needs sensor bar + report mode 0x33)
- Motion Plus not supported