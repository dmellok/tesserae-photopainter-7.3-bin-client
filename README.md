# tesserae-photopainter-7.3-bin-client

Battery-powered ESP32-S3 firmware that's the embedded client for the [Tesserae](https://github.com/dmellok/tesserae) server, ported to the [Waveshare ESP32-S3 PhotoPainter](https://www.waveshare.com/wiki/ESP32-S3-PhotoPainter) (7.3" 800×480 6-colour Spectra E6 e-paper panel on an ESP32-S3-WROOM-1-N16R8 module, AXP2101 PMIC, microSD slot).

The wake state machine, MQTT contract, captive-portal provisioning, and NVS schema are the same as the 13.3" client ([dmellok/tesserae-esp32-bin-client](https://github.com/dmellok/tesserae-esp32-bin-client)). What's different is the panel (single-CS, smaller, different init sequence), power management (AXP2101 over I2C instead of a GPIO panel-power gate and an ADC battery divider), and the heartbeat `kind` so the server can route the right frame format.

## Hardware mapping

| Component | Source firmware (13.3" E6) | This port (7.3" PhotoPainter) |
| --- | --- | --- |
| Module | ESP32-S3-WROOM-2-N32R16V | ESP32-S3-WROOM-1-N16R8 |
| Panel | 13.3" 1200×1600, dual-CS | 7.3" 800×480, single-CS |
| Frame size | 960 000 bytes | **192 000 bytes** |
| Panel power | GPIO1 (active-high) | AXP2101 ALDOs (via I²C) |
| Battery sense | ADC1 ch7 (GPIO8) + divider | AXP2101 fuel gauge (via I²C) |
| User button | RESET (double-tap) | BOOT (hold) + RESET (double-tap) |
| Status LED | none | Red GPIO45, Green GPIO42 |
| Extras | — | microSD slot, SHTC3, ES8311+ES7210 (all unused in v1) |

Pin map lives in [`include/app_config.h`](include/app_config.h); sourced from Waveshare's reference repo [waveshareteam/ESP32-S3-PhotoPainter](https://github.com/waveshareteam/ESP32-S3-PhotoPainter).

## MQTT contract

Same three topics as the 13.3" client under `tesserae/<device_id>/`:

| Topic | Direction | Retained | Purpose |
| --- | --- | --- | --- |
| `frame/bin` | server → device | yes | URL of the next `.bin` frame |
| `config` | server → device | yes | Runtime device settings |
| `status` | device → broker | yes | Wake-time heartbeat + LWT |

**Default `device_id` is `photopainter-73`** (not `esp32`) so the two panel kinds don't collide on a shared broker.

### Frame format

Raw, headerless, exactly **192 000 bytes** (`800 × 480 ÷ 2`), scanline order, two pixels per byte, high nibble = even column. Palette nibbles: `0x0`=Black, `0x1`=White, `0x2`=Yellow, `0x3`=Red, `0x5`=Blue, `0x6`=Green.

The heartbeat publishes `kind: "esp32_client"` and `panel_w: 800, panel_h: 480` — same `kind` as the 13.3" client. Tesserae's `esp32_bin` renderer dispatches on the panel dimensions, so no server-side change is required for the 7.3" panel.

### Heartbeat schema

Published at the **end** of every wake (after the paint, if any) to `tesserae/<device_id>/status`, retained QoS 1. Wire-contract matches [dmellok/tesserae-esp32-bin-client v0.6.0](https://github.com/dmellok/tesserae-esp32-bin-client) byte-for-byte so server-side handlers don't need to branch on panel type.

```json
{
  "battery_mv": 4180,
  "battery_pct": 92,
  "rssi": -41,
  "ip": "192.168.50.137",
  "fw_version": "0.2.0",
  "kind": "esp32_client",
  "panel_w": 800,
  "panel_h": 480,
  "sleep_interval_s": 300,
  "next_sleep_s": 300,
  "wake_reason": "timer",
  "sleep_until": 1780887503
}
```

Two of these feed Tesserae's [smart-sync scheduler](https://github.com/dmellok/tesserae/issues/10) and are worth calling out:

- **`next_sleep_s`** — duration of the deep sleep the device is **about to enter** (vs. `sleep_interval_s` which is the *configured* cadence). Same value for this firmware since it always sleeps for the configured interval; a future firmware that extends sleep on low battery could differ.
- **`sleep_until`** — absolute unix timestamp (UTC seconds) of the intended wake. **Omitted entirely** when NTP hasn't synced yet (first cold boot before the 5 s SNTP window completes). The server treats absent and `0` differently — absent triggers a tolerance-window fallback rather than getting recorded as "device claims to wake at 1970-01-01".

The remaining fields are unchanged from the v1 wire contract and present on every heartbeat.

## Build & flash

Requires [PlatformIO](https://platformio.org/). ESP-IDF 5.x and the Xtensa toolchain are pulled automatically on first build.

```bash
pio run                                              # build
pio run -e tesserae-photopainter-73-bin-client -t upload   # flash via USB-C
pio device monitor                                   # 115200 baud
```

For the dev shortcut, copy [`include/secrets.example.h`](include/secrets.example.h) to `include/secrets.h` and bake in WiFi/MQTT defaults. `secrets.h` is git-ignored.

## Provisioning

Two ways to enter the setup form (same form for both):

- **First boot / no creds.** SoftAP `Tesserae-Setup` (password `tesserae`) comes up; phone's captive-portal prompt opens the form.
- **Always-on editor.** Either **hold BOOT while pressing RESET** *or* **double-tap RESET within one wake window**. The device serves the form on its STA IP and advertises it over mDNS at `http://tesserae-<device_id>.local/`.

The double-tap path relies on the AXP2101 preserving RTC slow-memory across a PEK-triggered reset; if your unit clears it, the BOOT-hold path always works.

## Power notes

- **WiFi off before paint** — `wifi_sta_stop()` runs before the ~25 s panel refresh so the radio isn't holding ~80 mA during the silent stretch. Same approach as the 13.3" client; biggest single battery saving in the render path.
- **Panel power via PMIC** — `pmic_rails_set(false)` is what gets the average current down before deep sleep, since the PhotoPainter has no dedicated GPIO panel-power gate. All four ALDO rails get dropped together (the Waveshare schematic doesn't publish the per-ALDO mapping; this mirrors their factory firmware behaviour).
- **Battery via I²C** — `pmic_battery_mv()` and `pmic_battery_pct()` come straight from the AXP2101 fuel gauge; no curve-fit ADC calibration, no Li-Po SoC table to maintain.

### Smart-sync overhead (v0.2.0)

Since v0.2.0 the heartbeat publishes at the **end** of every wake (so `sleep_until` is wall-clock-accurate). On render wakes this means a brief WiFi reconnect after the paint — measurable but small:

| Wake type | Δ per wake |
|---|---|
| Hash-skip (URL unchanged) — most common in steady state | **0 mAh** (heartbeat shares the existing MQTT session, just publishes at the end) |
| Render wake (new URL or BOOT tap) | **+0.07–0.11 mAh** (~3–5 s WiFi reconnect + one-shot MQTT publish at ~80 mA) |
| First cold boot (NTP not yet cached) | **+0.11 mAh** one-time (5 s SNTP window; RTC carries the synced clock across deep sleep so subsequent wakes are instant) |

Impact on the documented use cases (15-min default sleep interval, 96 wakes/day):

| Use case | Render wakes/day | Pre-v0.2.0 daily draw | Post-v0.2.0 | Δ |
|---|---|---|---|---|
| Photo frame (1 update/day) | 1 | ~14 mAh | ~14.1 mAh | +0.8% |
| Dashboard (hourly updates) | 24 | ~30 mAh | ~32.6 mAh | +9% |
| Frequent (every wake renders) | 96 | ~75 mAh | ~85.6 mAh | +14% |

The alternative (keep WiFi up through the 25 s paint) would have cost ~2 mAh per render wake — **20× worse** than the reconnect approach. What you get for the overhead: Tesserae JIT-renders the next frame to land in the retained slot right before each wake, instead of pre-staging and risking staleness.

If a high-frequency deployment ever finds the cost unacceptable, the heartbeat path is the natural place to add an "every Nth wake" gate — the server tolerates absent heartbeats within its window.

## Project layout

```
tesserae-photopainter-7.3-bin-client/
├── platformio.ini                # board, partitions, monitor, FW_VERSION
├── partitions.csv                # 14 MB factory app + NVS
├── sdkconfig.defaults            # PSRAM-octal, mbedTLS bundle, MQTT 3.1.1
├── include/
│   ├── app_config.h              # pinout + behaviour tunables
│   └── secrets.example.h         # template for credential overrides
└── src/
    ├── main.c                    # boot -> settings? -> splash -> connect -> ntp -> mqtt fetch -> render -> publish heartbeat -> sleep
    ├── idf_component.yml         # managed deps (espressif/mdns)
    ├── epd_driver.{c,h}          # 7.3" Spectra E6 panel driver
    ├── pmic.{c,h}                # AXP2101 wrapper (battery + LDO rails)
    ├── heartbeat.{c,h}           # battery / RSSI / IP / panel size JSON
    ├── wifi_manager.{c,h}        # NVS-backed STA connect
    ├── provisioning.{c,h}        # captive portal + always-on settings server
    ├── mqtt_config.{c,h}         # NVS-backed broker URI / device_id
    ├── mqtt_handler.{c,h}        # single-shot subscribe + dispatch
    ├── image_fetcher.{c,h}       # HTTP download into PSRAM
    └── image_decoder.{c,h}       # strict 192000-byte panel-native validation
```

No tests -- smoke-test on real hardware. Recommended validation after any change to the wake state machine:

1. Flash a fresh board, walk it through the captive portal.
2. `mosquitto_pub -t tesserae/photopainter-73/frame/bin -r -m '{"url":"http://.../test.bin"}'` and confirm the panel paints.
3. Hold BOOT, press RESET, confirm `http://tesserae-photopainter-73.local/` (or the device IP) serves the settings form pre-filled with live values.

## Release process

[`tools/release.sh`](tools/release.sh) cuts a leak-safe GitHub release for the version in `platformio.ini`. **Do not** `pio run` straight into `gh release create` — `include/secrets.h` is `#include`d at compile time and its `#define` values land in `.rodata`. A 3-second `strings firmware.bin | grep` would walk away with your WiFi password, MQTT credentials, and broker URI.

The script defends in two layers, both required:

1. **Move-aside with restore trap.** Before building, `include/secrets.h` is renamed to a unique `.secrets.h.release-backup.<pid>` sibling and an `EXIT` trap is installed to restore it. The build then sees no `secrets.h`, the `__has_include` guard in `include/app_config.h` skips the include, and `WIFI_DEFAULT_*` / `MQTT_DEFAULT_*` all fall back to `""`.
2. **Post-build leak scan.** Every quoted string literal of ≥ 4 chars is parsed out of the stashed backup and grep'd against each built `.bin`. Any match aborts the release with a `LEAK:` line naming the value. This is the actual safety net — layer 1 is cheap prevention.

```bash
tools/release.sh                 # build, scan, tag, release
tools/release.sh --notes-only    # print suggested release notes and exit
```

The script refuses to run on a dirty tree or when `origin/main` is ahead of `HEAD`. Release artifacts are staged to `release/<version>/` (gitignored) and uploaded as GitHub release assets alongside a `SHA256SUMS` file.

If you change `tools/release.sh`, validate it by temporarily commenting out the `mv "$SECRETS" "$BAK"` line and re-running — the leak scan must catch the leak and abort with a `LEAK:` line before the upload step. Restore the `mv` after the test.

## Credits

The wake state machine, captive-portal provisioning, NVS schema, and MQTT contract are forked verbatim from [`dmellok/tesserae-esp32-bin-client`](https://github.com/dmellok/tesserae-esp32-bin-client) (the 13.3" Spectra E6 sibling).

The PhotoPainter-specific code owes nearly everything that *works* to **[aitjcize/esp32-photoframe](https://github.com/aitjcize/esp32-photoframe)**, whose mature open-source firmware for this exact device was the only reference that actually drove the panel cleanly on our hardware. Three load-bearing pieces are ported from it:

- **AXP2101 wake pulse** in [`src/pmic.c`](src/pmic.c) — pulsing the PMIC's IRQ pin (GPIO21) LOW for >16 ms to wake its I²C interface after a sleep state. Without this, every I²C transaction returns `ESP_ERR_INVALID_STATE` and battery telemetry / rail control are dead. Ported from [`components/pmic_driver_axp2101/src/axp2101.cpp`](https://github.com/aitjcize/esp32-photoframe/blob/main/components/pmic_driver_axp2101/src/axp2101.cpp).
- **I²C bus rescue** in [`src/pmic.c`](src/pmic.c) — 9 SCL pulses + STOP condition via open-drain GPIO before `i2c_new_master_bus`, in case a slave is mid-byte from a previous power cycle. Ported from [`components/board_hal/src/driver_waveshare_photopainter_73.c`](https://github.com/aitjcize/esp32-photoframe/blob/main/components/board_hal/src/driver_waveshare_photopainter_73.c).
- **Panel driver protocol** in [`src/epd_driver.c`](src/epd_driver.c) — sending the command byte through the SPI peripheral's command phase (`SPI_TRANS_VARIABLE_CMD`), streaming the frame in 128-byte stack-buffered chunks (PSRAM is unreliable as a direct SPI DMA source on ESP32-S3), and ordering the refresh as `reset → init → DTM → data → PON → DRF → POF → DSLP`. Our first attempt — which followed Waveshare's `xiaozhi-esp32` reference at [`waveshareteam/ESP32-S3-PhotoPainter`](https://github.com/waveshareteam/ESP32-S3-PhotoPainter) — issued PON during init and DMA'd directly from PSRAM, and the panel hung at every refresh. Aitjcize's pattern recovers reliably. Ported from [`components/epaper_driver_ed2208_gca/src/driver_ed2208_gca.c`](https://github.com/aitjcize/esp32-photoframe/blob/main/components/epaper_driver_ed2208_gca/src/driver_ed2208_gca.c).

The panel command-byte values themselves still come from Waveshare's [`waveshareteam/ESP32-S3-PhotoPainter`](https://github.com/waveshareteam/ESP32-S3-PhotoPainter) reference.

## License

MIT.
