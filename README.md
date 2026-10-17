# ESP32 Biogas Greenhouse Controller

Firmware for an ESP32 that runs a small biogas digester plus a CO₂-stripping
water loop and a hydroponics feed, with latching safety supervision on every
actuator.

The controller heats a mesophilic digester, heats a CO₂-stripping water tank
on demand, tops the digester up from a reservoir, and feeds digestate to
hydroponics — while refusing to do anything unsafe when a sensor lies to it.

---

## Sensors

| Purpose | Device | ESP32 pin | Notes |
|---|---|---|---|
| Digester temperature | DS18B20 | GPIO 5 | OneWire bus, 10-bit |
| CO₂-water temperature | DS18B20 | GPIO 5 | Same bus, index 1 |
| Greenhouse temp/humidity | DHT22 | GPIO 4 | Display only, never used for control |
| CO₂ (raw) | MQ-135 | GPIO 35 | ADC1, input-only |
| Methane (raw) | MQ-4 | GPIO 34 | ADC1, logged only, unused |
| Digester water level | analog float/level | GPIO 32 | ADC1 |
| Hydroponics water level | analog float/level | GPIO 33 | ADC1 |

All analog inputs are on **ADC1** deliberately: ADC2 is unusable while WiFi
is active.

> **MQ-135 caveat.** CO₂ demand is triggered off a raw ADC count with no
> temperature/humidity compensation and no R0 calibration. Treat the CO₂
> threshold as a relative "how stale is the air" proxy, not a ppm reading.
> The MQ-4 is read and logged but never drives anything — it needs a 24–48 h
> burn-in and an R0 calibration to be meaningful.

## Actuators

All relays are **active LOW** (`LOW` = ON).

| Relay | GPIO |
|---|---|
| Digester heater | 25 |
| CO₂-water heater | 26 |
| Reservoir → digester valve | 27 |
| Digester → hydroponics valve | 13 |
| Spare valve (digester gas) | 21 |
| Spare valve (digester CO₂) | 22 |

Display is a 16×2 HD44780 LCD in 4-bit mode: `RS=19, E=23, D4=18, D5=17, D6=16, D7=15`.

### Strapping-pin hazards

Two GPIO choices deserve attention on any PCB respin:

- **GPIO 12 was deliberately removed** from the relay map. It is an MTDI
  strapping pin; a relay board pulling it can select the wrong flash voltage
  and break boot/flash. The two spares were moved to 21/22.
- **LCD D7 sits on GPIO 15**, also a strapping pin. It works as wired, but
  move the LCD off 15 on the next revision.
- GPIO 34/35 are input-only and have no internal pull-ups — fine for ADC, but
  don't expect to drive anything from them.

---

## Control behaviour

Read interval is **5 s**; the LCD alternates between two pages every **3 s**.

**Page 0** — process values: `D:` digester °C, `W:` CO₂-water °C,
`H:` greenhouse °C / humidity %, MQ-135 raw.
**Page 1** — actuator states (`D/C/W/H` = 1 if energised) and either
`FAULT …` flags or `WD:`/`WH:` raw level counts.

Every LCD line is padded to exactly 16 characters and redrawn only when its
content changes, so there is no flicker and no ghosting from leftover digits
(that is why humidity is clamped to 0–99 — `100` would overflow the line).

### Thresholds

| Quantity | Low | High | Hysteresis |
|---|---|---|---|
| Digester temp | 36.0 °C (heat) | 40.0 °C (stop) | 4 °C |
| CO₂-water temp | 55.0 °C (heat) | 62.0 °C (stop) | 7 °C |
| Digester level | < 1100 (fill) | > 1600 (stop) | 500 counts |
| Hydroponics level | < 1000 (fill) | > 1500 (stop) | 500 counts |
| MQ-135 CO₂ | < 1500 (demand) | > 1700 (satisfied) | 200 counts |

Inside a deadband the actuator holds its previous state rather than dithering.

### Safety supervision

- **Fail-safe on fault, not fail-on.** A failed DS18B20 does not just get
  ignored — it is substituted with a value *above* the threshold, so the
  heater's "want" resolves to OFF. The heater can never be latched on by a
  broken probe. (Readings are validated against disconnect, the 85 °C
  power-on-reset sentinel, and a sanity range.)
- **Latched timeouts.** Heaters trip out after 30 min continuous, valves
  after 5 min. A trip *latches*: the actuator stays off until the underlying
  condition genuinely clears (temperature recovered, tank full) — not merely
  until the next loop. Timers are only evaluated while the actuator is
  actually ON, so their zero-initialised timestamps can never cause a
  spurious trip.
- **A fault cannot clear a lockout.** The lockout-clear paths are guarded by
  the fault flags, so the fail-safe placeholder value can't be mistaken for
  a real recovery.
- **Interlock.** The hydroponics feed valve is interlocked against the
  digester level: it will not open below 1300 counts, so the system can never
  drain the digester to irrigate. The fill and drain valves also cannot be
  open simultaneously.
- **Over-temperature cutout** at 72 °C forces both heaters off immediately.
- **Railed-ADC detection.** A level input pinned at either rail (≤ 5 or
  ≥ 4090) is treated as an open/short wiring fault rather than a real level.

ADC reads are averaged over 16 samples to suppress jitter.

---

## Building and flashing

Requires [PlatformIO](https://platformio.org/) (CLI or the VS Code extension).

```bash
pio run              # build
pio run -t upload    # flash over /dev/ttyUSB0
pio device monitor   # 115200 baud serial log
```

The upload and monitor port default to `/dev/ttyUSB0` in `platformio.ini`;
adjust for your host.

Dependencies are pinned by PlatformIO in `platformio.ini` and fetched
automatically:

- `milesburton/DallasTemperature@^4.0.6`
- `paulstoffregen/OneWire@^2.3.8`
- `adafruit/DHT sensor library@^1.4.7`
- `adafruit/Adafruit Unified Sensor@^1.1.15`
- `arduino-libraries/LiquidCrystal@^1.0.7`

Build footprint on `esp32dev`: **RAM 6.7 % (21 824 B), Flash 21.9 % (287 101 B)**.

### Serial output

One line per 5 s cycle:

```
Td:36.4 Tw:55.2 CO2:1430 CH4:812 WD:1210 WH:990 Hd:1 Hc:1 Vw:0 Vh:0 Lk:0000
```

`Td`/`Tw` = digester / CO₂-water °C, `CO2`/`CH4` = raw ADC counts,
`WD`/`WH` = raw level counts, `Hd`/`Hc`/`Vw`/`Vh` = heater/valve states,
`Lk` = the four latch flags (digester heater, CO₂ heater, fill, drain).

---

## Known limitations

Worth addressing before this supervises anything unattended:

1. **No watchdog.** If a blocking library call hangs (a shorted OneWire bus,
   an LCD/I²C stall), the main loop stops and the relays freeze in their last
   state — which for a heater is the wrong direction. Enabling `esp_task_wdt`
   or the hardware WDT would close this.
2. **Uncalibrated ADC thresholds.** The level and CO₂ cutoffs are raw counts
   against a nominal 3.3 V. The ESP32 ADC has real nonlinearity and offset, so
   these numbers are specific to this board. `analogReadMilliVolts()` or
   `esp_adc_cal` would make them portable.
3. **DS18B20 addresses are bound by discovery order**, not hardcoded. Swapping
   probes silently swaps which sensor is "digester" and which is "CO₂ water".
   The setup log prints both addresses — copy them into fixed bindings to
   remove the ambiguity.
4. **Control is time-driven, not event-driven.** The 5 s cycle means worst-case
   fault detection latency is ~5 s plus the blocking DS18B20 conversion.

## License

MIT — see [LICENSE](LICENSE).
