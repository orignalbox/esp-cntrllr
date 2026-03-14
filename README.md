# esp-cntrllr

BLE gamepad using ESP32-S3. Maps 8 buttons to a standard HID gamepad — 4 for the d-pad (joystick axes) and 4 action buttons. Pairs with anything that accepts a gamepad over BT.

Built on ESP-IDF 6.1 with NimBLE.

---

## wiring

| GPIO | what |
|------|------|
| 18 | up |
| 16 | down |
| 15 | left |
| 8 | right |
| 1 | A |
| 2 | B |
| 4 | C |
| 17 | D |
| 38 | WS2812 data |

buttons are active low, pull-ups enabled so just wire GPIO to GND.

---

## LED

![LED states](docs/led_states.png)

pulsing blue = advertising, dim white = connected, bright colors on button press (A=blue, B=red, C=pink, D=green), bright white for d-pad.

---

## flow

![connection flow](docs/ble_flow.png)

polls at 50hz, only sends a report when something changes. goes back to advertising automatically on disconnect.

---

## building

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

sdkconfig is included so it should build without any menuconfig changes.

---

## pairing

just look for **ESP Gamepad** in your bluetooth settings. no PIN. bonding is enabled so it reconnects automatically after the first pair.

> on some iOS versions the gamepad axes don't register unless `CONFIG_BT_NIMBLE_SM_LVL` is set to `2`.
