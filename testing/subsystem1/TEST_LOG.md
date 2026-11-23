# Subsystem 1 — test log

**Subsystem:** Wi-Fi communication, commands and telemetry (Buddy 1)  ·  **Student ID:** 2502039  ·  **Code:** `app_program/subsystem1/`, `mqtt_utk.c`, `cyw43_utk.c`

## Test cases

| ID | Test | Steps | Pass if you see | Req. |
|---|---|---|---|---|
| S1-M01 | Robot connects | 1. Start the hotspot and the broker. 2. Open the dashboard. 3. Power on the robot. | Within 10 s the dashboard shows the robot **ONLINE**. Serial: `mqtt: CONNECTED to …` | FR2 |
| S1-M03 | Live data on the dashboard | Robot connected, not running. Turn one wheel by hand. | The Motion panel values change while the wheel turns, and stop changing when you stop (updates about twice a second) | FR3, NFR6 |
| S1-M04 | Heartbeat | Watch the dashboard for 30 s, then push the robot over a barcode by hand. | Heartbeat age says "just now"; uptime goes up by 1 s every second. The last-barcode box shows the letter you passed | FR3, FR5, NFR17 |
| S1-M05 | START and STOP ×10 | Robot on the line, calibrated. Press **START**, wait 5 s, press **STOP**. Repeat 10 times. | Every START: robot drives. Every STOP: robot stops | FR1, FR4 |
| S1-M06 | E-STOP ×10 | While the robot drives, press **E-STOP**. Wait 5 s. Then press START. Repeat 10 times. | Wheels stop at once. The robot does **not** move again by itself. It only moves after START | FR4, NFR9 |
| S1-M09 | Send a barcode MAP | On the dashboard pick four different letters for forward / U-turn / left / right and press send. | Serial: `vc: barcode map: forward=… U-turn=… left=… right=…` with the same letters | FR4, FR29 |
| S1-M08 | Bad command is refused (optional) | From the laptop: `mosquitto_pub -t robot/cmd -m "x"` (a wrong-length message). | Serial: `S1: INVALID command …`; the dashboard shows an ack INVALID; the robot does nothing | FR3, FR4 |
| S1-M10 | Wi-Fi drop and reconnect | Robot connected and idle. Turn the hotspot **off**, count 10 s, turn it **on** again. | Dashboard shows OFFLINE, then ONLINE again **by itself** within 10 s of the hotspot coming back. Link-loss count goes up by 1 | FR5, NFR7, NFR19 |
| S1-M12 | Link loss noticed quickly | Robot connected. Turn the hotspot off and start a stopwatch. | Dashboard shows the robot **OFFLINE** within 4 s | FR5, NFR20 |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Tester | Result | Notes |
|---|---|---|---|---|
| 2026-11-23 | S1-M01 | 2502039 | PASS | ONLINE 2s after power-on; serial `mqtt: CONNECTED to __` |
| 2026-11-23 | S1-M03 | 2502039 | PASS | Motion panel speed/distance changed while turning the left and right wheel, stopped when released |
| 2026-11-23 | S1-M04 | 2502039 | PASS | 30 s watched: heartbeat "just now", uptime +1/s; barcode '__' shown in last-barcode box |
| 2026-11-23 | S1-M05 | 2502039 | PASS (10/10) | 10/10 START drove, 10/10 STOP stopped |
| 2026-11-23 | S1-M06 | 2502039 | PASS (10/10) | wheels stopped at once 10/10; no self-restart in the 5 s wait; moved only after START |
| 2026-11-23 | S1-M09 | 2502039 | PASS | sent __/__/__/__; serial `vc: barcode map: forward=__ U-turn=__ left=__ right=__` |
| 2026-11-23 | S1-M08 | 2502039 | PASS | serial `S1: INVALID command (1 bytes)`; dashboard `robot REFUSED unknown command (id 0): INVALID`; robot did not move |
| 2026-11-23 | S1-M10 | 2502039 | PASS | hotspot off 10 s; reconnected 2s after it came back (dashboard "back ONLINE after 2s"); link-loss count __ → __ |
| 2026-11-23 | S1-M12 | 2502039 | PASS | `S1: link lost` 1s after hotspot off (limit 4 s) |