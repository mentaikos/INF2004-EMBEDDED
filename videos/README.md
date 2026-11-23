# Videos

Short videos of tests and demos running on the real hardware. A video shows
what a log cannot: the LED blinking, the motor turning, the robot reaching
the line. List **every** video in the table at the bottom of this page.

## What to record

- **Short:** 15–60 seconds. One test or one feature per video.
- **Say what it is.** Show or say the test ID (e.g. `S2-U03`, `INT-01`) or
  the demo name at the start.
- **Show the commit.** Before recording, run `git rev-parse --short HEAD` and
  keep the terminal in shot for a moment, or put it in the table below.
- **Show the result.** Keep both the hardware and, if it matters, the
  console output in view.
- **Landscape**, steady, good light. No music needed.

## How to share it: unlisted YouTube only

1. Upload the video to YouTube with visibility set to **Unlisted**: anyone
   with the link can watch it, but it does not show up in search. Not
   *Private* (only invited accounts can watch, so the link won't work for
   your team or instructor), and not *Public*.
2. Copy the video's link (e.g. `https://youtu.be/...`).

**Do not commit video files to this repo or attach them on GitHub.** A video
in the repo stays in the git history for good, even if deleted later, and
slows every clone for the whole team. Video files are ignored by
`.gitignore` for this reason.

Then add a row to the table below. If the video is evidence for a test, also
put its link in the *Log / evidence* column of that test's row in
`testing/.../TEST_LOG.md`.

## Video index

Newest at the bottom. Never delete a row.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1 | S1-U03 | `abc1234` | | <https://youtu.be/...> | LED blinks at 2 Hz |

| 2026-11-23 | S3 | S3-U01 | `56c9e25` | | <https://youtu.be/agE2hv6Gka4> | Detect a barcode, Decode the barcode and obtain the code, Follow the instruction from the barcode

| 2026-11-23 | S4 | S4-U01 | `56c9e25` | | <https://youtu.be/v4tPge_iHPI> | Stop motors when the car is picked up

| 2026-11-23 | S4 | S4-U02 | `56c9e25` | | <https://youtu.be/QX4s-SHfH3Y> | Measure hump height

| 2026-11-23 | S5 | S5-U01 | `56c9e25` | | <https://youtu.be/eVq0_ZQNGTY> | Obstacle Detection, Navigate around the obstacle and rejoin the line, Measure obstacle width