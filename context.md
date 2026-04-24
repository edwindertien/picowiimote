# WiiMote MIDI Bridge — Development Context

Starting point for a new development session. Describes current state,
key decisions, confirmed working behaviour, and remaining bottlenecks.

---

## Working State (end of session)

| Feature | Status |
|---------|--------|
| BT scan + connect | ✓ Working |
| WiiMote buttons → MIDI Note On/Off | ✓ Working |
| WiiMote accel → MIDI CC (with gate) | ✓ Working |
| LittleFS mapping.json | ✓ Working |
| Nunchuk auto-detect | ✓ Working |
| Nunchuk joystick + accel + buttons → MIDI | ✓ Working |
| Serial CSV stream (20Hz) | ✓ Working |
| Extension auto-detect (write/read registers) | ✓ Working |
| Guitar Hero decode | Structural (byte layout unverified) |
| IR camera | Not implemented |
| Persistent pairing (saved address) | Removed — scan-only |

---

## Hardware Confirmed

- Pico W BD address: `2C:CF:67:9E:A4:08`
- WiiMote BD address: `00:21:47:C5:CA:89` (Nintendo RVL-CNT-01)
- Nunchuk: detected, ID bytes `00:00:A4:20:00:00` ✓

---

## Architecture: Split-File BTstack / TinyUSB

**The most important constraint in this codebase.**

`btstack_hid.h` and TinyUSB's `class/hid/hid.h` both define `hid_report_type_t`.
Including both in one translation unit = fatal compiler error. No macro workaround
works because enums aren't macros.

**Rule**: `btstack.h` only in `bt_init.cpp`. TinyUSB only in `main.cpp`.
They communicate via `extern volatile` globals and a small set of callback functions.

This was the single hardest problem in the session and is confirmed solved.

---

## BTstack Key Decisions

### HID Host API (not raw L2CAP)
Use `hid_host_init()` / `hid_host_connect()` / `hid_host_accept_connection()`.
Raw L2CAP was tried first and failed. The HID host layer handles L2CAP channels,
SDP, and protocol mode negotiation internally.

### WiiMote pairing PIN
`HCI_EVENT_PIN_CODE_REQUEST` → `gap_pin_code_response_binary(addr, reversed_addr, 6)`
The PIN is the WiiMote's own BD address in reverse byte order (binary, not ASCII).

### Connection strategy: scan-only
`gap_inquiry_start(10)` scans for 10 seconds. On finding a device with CoD
`0x0005xx` (gamepad class), connects immediately. Retries every 12s.

Persistent pairing (saved BD address → direct connect) was attempted but
caused `0x0B` rejection errors because the WiiMote was bonded to another host.
Removed in favour of scan-only which is reliable. The user holds 1+2 on the
WiiMote to make it connectable.

### Master role
`hci_set_master_slave_policy(HCI_ROLE_MASTER)` is required. WiiMote requires
the host to be Bluetooth master.

### No delay() in BTstack callbacks
`delay()` inside a packet handler blocks the BTstack async context on the
Pico W (runs in `async_context_threadsafe_background`). This prevents
subsequent packets from being processed, causing timeouts and errors.

Instead, use the pending timer pattern:
```cpp
// In packet handler:
ext_detect_pending = true;
ext_detect_pending_ms = millis() + 300;

// In bt_hid_loop_update() called from loop():
if (ext_detect_pending && millis() >= ext_detect_pending_ms) {
    ext_detect_pending = false;
    wm_detect_extension_internal();
}
```

---

## Extension Detection Sequence

Confirmed working sequence:

1. Receive status report `0x20` with `ext_connected=1` (bit 1 of flags byte)
2. Schedule detection 300ms later (settle time — writing too soon = error `0x06`)
3. Write `0x55` to `0xA400F0` (report `0x16`, address space `0x04`)
4. Receive write ack `0x22` — error in `r[len-1]` (last byte). `0x00` = OK.
   - First ack may show `0x05` (for the `wm_set_report_mode` that raced) — ignore
5. Write `0x00` to `0xA400FB` (disable encryption)
6. Receive write ack `0x22` — must be `0x00`
7. Read 6 bytes from `0xA400FA` (report `0x17`)
8. Receive memory read response `0x21`:
   - `r[4]` upper nibble = size-1, lower nibble = error
   - `r[7..12]` = 6-byte extension ID

### Extension IDs
| ID bytes | Device |
|----------|--------|
| `00:00:A4:20:00:00` | Nunchuk ✓ confirmed |
| `00:00:A4:20:01:03` | Guitar Hero 3 |
| `00:00:A4:20:01:01` | Classic Controller |
| `FF:FF:FF:FF:FF:FF` | No extension |

### Write Report Format (0x16)
21-byte payload sent via `hid_host_send_report(cid, 0x16, data, 21)`:
```
[0x04]          address space (0x04 = extension registers)
[addr >> 16]    3-byte address, big-endian
[addr >> 8]
[addr & 0xFF]
[0x01]          size (1 byte)
[val]           data byte
[0x00 x 15]    padding to 16 data bytes
```

### Read Report Format (0x17)
6-byte payload:
```
[0x04]          address space
[addr >> 16]    3-byte address
[addr >> 8]
[addr & 0xFF]
[size >> 8]     2-byte size, big-endian
[size & 0xFF]
```

### Write Ack Format (0x22)
Observed: `A1 22 [btn0] [btn1] [report_id_echo] [error]` = 6 bytes total.
Error at `r[len-1]`. Known errors:
- `0x00` = success
- `0x05` = race with another output report (retry)
- `0x06` = extension not ready (retry after 500ms)
- `0x07` = no extension

---

## Report Modes

| Mode | Content | When used |
|------|---------|-----------|
| `0x31` | Buttons + accel | WiiMote alone |
| `0x35` | Buttons + accel + 16 ext bytes | With Nunchuk/GH |

Switched to `0x35` automatically after extension detected.

---

## Button Decode

Bits are **1 = pressed** in our globals (BTstack HID host applies HID
descriptor inversion). Direct bit test: `if (b0 & mask)`.

Bit positions in `g_btn_cur`:
```
bit  0 = TWO    bit  1 = ONE    bit  2 = B      bit  3 = A
bit  4 = MINUS  bit  7 = HOME   bit  8 = LEFT   bit  9 = RIGHT
bit 10 = DOWN   bit 11 = UP     bit 12 = PLUS
```

---

## LittleFS

**Mount once, stay mounted.** The Earle framework's LittleFS doesn't reliably
re-mount after `end()`. `fs_mount()` is called once in `setup()` and the
filesystem stays open for the session.

`data/mapping.json` is uploaded via `pio run -t uploadfs`. The pre-script
`upload_fs_pre.py` runs this automatically before firmware upload, so a single
`pio run -t upload` does both.

---

## Remaining Bottlenecks

### 1. Persistent pairing
Saved-address direct connect was removed. Could be re-added with a longer
timeout before declaring failure, but the SYNC-button bonding issue with
other hosts makes it unreliable in practice. Scan-only is simpler.

### 2. Guitar Hero byte layout
The GH3 decode in `bt_init.cpp` is based on Wiibrew documentation but has
not been field-verified. Use CSV stream mode (`'s'`) and press each button
to verify which bits change in `gh_btns`. Byte layout may vary between
controller firmware versions.

### 3. IR camera
Requires:
- Report mode `0x33` (buttons + accel + 12 IR bytes)
- IR sensitivity registers written via `0x16` (addresses `0xB00030`, `0xB00000`, etc.)
- Enable IR via output reports `0x13` and `0x1A`
- A sensor bar (two clusters of ~850nm IR LEDs, ~20cm apart)
- Blob decode: 4 blobs × 3 bytes, 10-bit X/Y in 1024×768 space

The reference PicoSDK code in `wiimote_bt.c` implements this — see
`wiimote_init_step()` state machine there.

### 4. Motion Plus
Different extension ID, different register space (`0xA600xx`), gyroscope
data. Requires pass-through mode for simultaneous Nunchuk use.
Not planned.

### 5. Multiple WiiMotes
Would require multiple `hid_host_connect` instances and CID demultiplexing.
Not planned.

---

## Build Environment

```
Platform:   maxgerhardt/platform-raspberrypi (wraps Earle Philhower arduino-pico)
Board:      rpipicow
Framework:  arduino (Earle core)
BTstack:    bundled with pico-sdk inside framework
TinyUSB:    Adafruit library bundled in framework
LittleFS:   0.5M partition
```

Key build flags: `ENABLE_BLUETOOTH`, `ENABLE_IPV4`, `USE_TINYUSB`

The `USE_TINYUSB` flag enables Adafruit TinyUSB as the USB stack. Without it
the USB MIDI device doesn't enumerate. With it and `btstack.h` in the same
file, `hid_report_type_t` conflicts — hence the split-file architecture.