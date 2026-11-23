# Subsystem 4 — test log

**Subsystem:** IMU & terrain (Buddy 4: tilt, humps, lifted / overturned, heading)  ·  **Student ID:** 2500701  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Test cases
ID	Test	Steps	Pass if you see	Req.
S4-AUTO	Automatic checks	1. Flash the test build. 2. Open the serial monitor. 3. Wait until it stops printing (about 20 s).	Every [TEST] S4-… line ends in PASS (no FAIL). Covers S4-U01, S4-U03…U17 (robot flat and still from power-on). PC test S4-H01: ./t prints ALL TESTS PASSED (known: one long-ramp FAIL, S4-H05)	—
S4-M01	IMU found at power-on	Power on, watch the serial monitor.	Serial s4: I2C scan: 2 device(s) listing 0x19 (accelerometer) and 0x1E (magnetometer); dashboard Calibrated = yes after 2 s	FR13
S4-M02	Pitch	Robot flat and still at power-on (2 s). Then put a 2 cm block under the front wheels, then under the back wheels.	Dashboard pitch about 0 flat; clearly positive with the nose up (state CLIMBING); negative with the nose down; back to about 0 (LEVEL) when flat	FR13, FR14
S4-M03	Heading	Turn the robot slowly by hand a quarter turn, then a full turn.	Dashboard heading changes by about 90, then goes all the way round 0→359	FR13
S4-M10	Calibration needs a still robot ×5	Power on while holding the robot and moving it gently for 5 s, then put it flat and let go.	It does not become LEVEL while you hold it; about 2 s after you let go it shows LEVEL with pitch about 0	FR13
S4-M07	Hump height ×10 each	Drive over the hump, 10 times each.	Dashboard Last hump (or serial s4: HUMP_END height=):  hump 8–12 mm; the hump count does not go up on flat track. Video: https://youtu.be/QX4s-SHfH3Y (videos/README.md)	FR21
S4-M05	Lift stops the robot ×10	While the robot drives, pick it up (5 times gently, 5 times quickly).	Wheels stop within about 1 s. Serial s4: LIFTED …. Dashboard state COMPLETE. Video: https://youtu.be/v4tPge_iHPI (videos/README.md)	FR27, NFR10
S4-M09	Tipped over	Wheels spinning (START, robot held off the table), tip it onto its side.	Wheels stop. Serial s4: LIFTED (overturned > 45 deg)	FR27
S4-M12	START refused while lifted	Hold the robot in the air and press START.	Robot does not start. Serial vc: START refused: robot lifted or overturned …	FR14, NFR9
S4-M17	Junction is not a lift	Put the robot standing on a junction line for 10 s, then lift it from there.	No LIFTED while standing. LIFTED as soon as you lift it	FR27
S4-M06	IMU unplugged and back	Robot idle. Unplug the IMU cable for 3 s, plug it back in, keep the robot still for 2 s.	Dashboard IMU state FAULT while unplugged, then back to LEVEL by itself	FR14
S4-M16	IMU panel on the dashboard	WiFi build: tilt, turn and lift the robot by hand.	Dashboard pitch, heading, state and the lifted indicator follow what you do	FR15, FR22

Results


Date	Test ID		    Tester	        Result	Notes
2026-11-23	S4-AUTO		Poh Yang Sheng	PASS	All [TEST] S4-… lines PASS on the robot
2026-11-23	S4-M01		Poh Yang Sheng	PASS	I2C scan found 0x19 and 0x1E; Calibrated = yes
2026-11-23	S4-M02		Poh Yang Sheng	PASS	Pitch ~0 flat, positive (CLIMBING) nose up, negative nose down
2026-11-23	S4-M03		Poh Yang Sheng	PASS	Heading changed ~90 for a quarter turn, 0→359 for a full turn
2026-11-23	S4-M10		Poh Yang Sheng	PASS	5/5: not LEVEL while held, LEVEL ~2 s after put down
2026-11-23	S4-M07		Poh Yang Sheng	PASS	Hump height. Video: https://youtu.be/QX4s-SHfH3Y (commit 56c9e25)
2026-11-23	S4-M05		Poh Yang Sheng	PASS	10/10: lift stops the motors, COMPLETE. Video: https://youtu.be/v4tPge_iHPI (commit 56c9e25)
2026-11-23	S4-M09		Poh Yang Sheng	PASS	Tipped on its side: wheels stopped, LIFTED (overturned > 45 deg)
2026-11-23	S4-M12		Poh Yang Sheng	PASS	START refused while held in the air
2026-11-23	S4-M17		Poh Yang Sheng	PASS	No LIFTED standing on a junction 10 s; LIFTED when picked up
2026-11-23	S4-M06		Poh Yang Sheng	PASS	FAULT while unplugged, back to LEVEL by itself
2026-11-23	S4-M16		Poh Yang Sheng	PASS	Dashboard pitch, heading, state and lifted followed the robot
