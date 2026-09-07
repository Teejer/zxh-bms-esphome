# esp32-zxh-bms

ESPHome component that connects an ESP32 **directly over Bluetooth LE** to the
family of "smart BMS" boards used in many rebranded LiFePO4 batteries (the
`zxhbms` / uni-app vendor app family), and exposes the readings to Home
Assistant through the native ESPHome integration.

This is the ESP32 counterpart to the [`Teejer/zxh-bms`](https://github.com/Teejer/zxh-bms)
Python CLI (`bms-cli`) — the register map, frame format, CRC, and scaling
factors are a direct port of `bms_cli/protocol.py`, validated against the
same real 16S/51.2V pack.

## Why an ESP32 instead of Home Assistant's Bluetooth Proxy?

The HA/ESPHome **Bluetooth Proxy only forwards passive advertisements** — it
cannot open a GATT connection, write characteristics, or receive
notifications. These BMS boards publish no telemetry in advertisements;
everything must be *requested* over a GATT connection (Modbus-like
request/response frames). So this component **replaces** a proxy next to the
battery rather than extending one: the ESP32 is a GATT *client* of the BMS
and a WiFi server to Home Assistant.

The ESP32's BLE radio is dedicated to the batteries while connected — do not
run `bluetooth_proxy` on the same node. One ESP32 can read a single battery
over a persistent link, or several batteries one-at-a-time via sequential
mode (see [Multiple batteries](#multiple-batteries)).

## Features

- **Read-only by design** — like the CLI, it never touches the app's
  function-6/16 parameter-write commands, so it cannot misconfigure
  protection thresholds or brick the BMS.
- Auto-detects the GATT profile (vendor `00002760...`, Nordic-UART
  `6E400001...`, or `0003CDD0...` services) exactly like the app/CLI, plus
  the same notify/write fallback heuristic.
- Auto-discovers the Modbus bus address after each connect (broadcast read of
  register 4008); override with `address:` only if discovery fails.
- Reconnects automatically (handled by `ble_client`); re-runs address/label
  discovery after every reconnect.
- Per-command retry (3 attempts, 1.5 s timeout) mirroring the app's
  `readCommandSend` loop. A failed poll cycle just keeps the last values
  published — no entity goes "unavailable" for a transient BLE hiccup.
- Sensors: pack voltage (sum of cells), current, power, SOC, min/max/delta
  cell, remaining capacity, cycles, health, MOS temp, up to 4 temp probes,
  label info (cell count, nominal/full Ah). Text sensors: per-cell voltages
  as JSON, active protection faults, firmware version, manufacture date,
  and the BMS Bluetooth device name read from GATT (0x2A00, e.g.
  `ZXH16S100A-*`) via the `device_name:` text sensor.
  Binary sensors: protection active, balancing active.

## Usage

Put the `components/` folder next to your YAML (or point
`external_components` at its path) and start from
[`example-zxh-bms.yaml`](example-zxh-bms.yaml):

```yaml
external_components:
  - source:
      type: local
      path: components

esp32_ble_tracker:            # required: ble_client uses it to find the BMS

ble_client:
  - mac_address: AA:BB:CC:DD:EE:FF
    id: bms_ble

zxh_bms:
  ble_client_id: bms_ble
  update_interval: 30s
  # address: 1                # only if auto-discovery fails

sensor:
  - platform: zxh_bms
    voltage:
      name: "BMS Pack Voltage"
    # ... see example file for all keys
```

`bms-cli scan` (from the Python tool) is the easiest way to find the MAC.

## Polling

`update_interval` on each `zxh_bms:` hub is its polling cycle: every
interval the hub runs one full read (instrument + one read per 7 cells +
basic info + temps — 6 transactions for a 16S pack, typically 1–2 s total).
Keep it comfortably longer than a full cycle (>= 15s) — overlapping cycles
are skipped and logged. The first cycle after connect also runs
address/label/manufacture-date discovery.

## Multiple batteries

One ESP32 can monitor several packs **one at a time**: give each battery its
own `ble_client` + `zxh_bms` hub and set `mode: sequential` on every hub.
The hubs share a single radio slot — when a hub's `update_interval` elapses
it queues a cycle, the scheduler connects to that battery, reads,
disconnects, and hands the radio to whoever is waiting next. Only one BLE
link is ever open, so this scales past the stack's concurrent-connection
limit and each pack spends most of its time untouched.

- `update_interval` (per hub): how often that pack is polled.
- `cycle_timeout` (per hub, default 90s): a stalled/unreachable pack is
  abandoned after this and the radio moves on — one dead battery never
  blocks the others.
- `mode: persistent` (default): keep the BLE link up between polls — use
  this for a single battery for the freshest data.

See [`example-two-batteries.yaml`](example-two-batteries.yaml); each
`sensor:`/`text_sensor:`/`binary_sensor:` platform block selects its pack
with `zxh_bms_id:`.


## Notes and caveats

- Entities without a `name:` are created hidden, like ESPHome convention.
- Unset/absent temperature probes report NaN (shown as unavailable in HA),
  matching how the CLI skips probes below −40 °C.
- Protection-event counters and serial/manufacturer-name strings from the
  CLI are not ported (unverified against real hardware in the CLI's own
  README); easy to add later.
- Current sign threshold (3-byte current, sign flips at `data[0] > 124`) is
  ported verbatim from the vendor app arithmetic, quirk included.

## Development

- The component lives entirely in `components/zxh_bms/`; `example-zxh-bms.yaml`
  validates against it (`esphome config example-zxh-bms.yaml`).
- Wire-protocol changes should be made in `lifepo4-cli/bms_cli/protocol.py`
  first, then ported to `components/zxh_bms/zxh_bms.cpp`.
- Build gotcha: if your `$HOME` (or any ancestor of the build dir) is a git
  repo without any commits, the ESP-IDF cmake step dies with
  `fatal: Needed a single revision` / `git-data/head-ref` errors. Building
  with `GIT_DIR=/nonexistent esphome compile ...` (or building outside the
  repo) works around it.
