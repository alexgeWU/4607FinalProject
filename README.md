# ASL Glove + Air Mouse (4607 Final Project)

A glove that reads American Sign Language letters and can act as an air mouse,
built on a **Raspberry Pi Pico 2 W**. A **BNO08x IMU** on the back of the hand
reports hand orientation; flex sensors for the fingers come next.

**Current stage:** IMU hand-pose detection. The Pico reports which way the palm
and fingers point (up / down / away / toward / left / right) and whether the
hand is holding still.

## Hardware

| BNO08x pin | Pico 2 W pin |
|------------|--------------|
| SDA        | GP16 (I2C0 SDA) |
| SCL        | GP17 (I2C0 SCL) |
| RST        | GP5          |
| VIN / GND  | 3V3 / GND    |

I2C address 0x4A, 400 kHz. The wiring lives in `k_imu_cfg` at the top of
`4607FinalProject.c`.

## Build and flash

Open the folder in VS Code with the **Raspberry Pi Pico** extension (Pico SDK
2.3.1), then use *Compile Project* and *Run Project*. If flashing fails with
"No accessible RP-series devices", hold **BOOTSEL** while plugging the Pico
in and run again.

## Serial output

Set `OUTPUT_CSV` in `4607FinalProject.c`:

- `0`: readable screen for a serial monitor (pose, still/moving, raw values)
- `1`: CSV stream for the recording tool below

While running, type `z` in the serial monitor to re-zero the heading (the
direction the fingers point becomes "away").

## Hand pose

- **Palm / finger direction** comes from the IMU's Game Rotation Vector
  (quaternion). Up/down is locked to gravity; away/toward/left/right is
  relative to the zeroed heading, which drifts slowly, so re-zero now and then.
- **Smoothing:** the pose only changes after a new pose holds briefly.
- **Stillness:** `imu.still` is true once the gyro shows the hand has stopped
  rotating. Letter logic should only accept a letter while it's true.
- Tuning constants (`POSE_MIN_COS`, `POSE_HOLD_MS`, `STILL_MS`, `STILL_DEG_S`)
  and the IMU mounting axes are in the pose section of `imu.c`.

## Recording tool

```sh
python3 -m venv .venv
.venv/bin/pip install -r tools/requirements.txt

.venv/bin/python tools/imu_logger.py record   # live plot, saves labeled episodes
.venv/bin/python tools/imu_logger.py plot     # plot the latest recording
```

Needs `OUTPUT_CSV 1` and no other serial monitor open. In the plot window: `z`
zeros the heading, `SPACE` drops a marker, `q` stops and saves to
`recordings/`. Each recording prints which poses the hand was in while still.

## Project layout

| File | What it does |
|------|--------------|
| `4607FinalProject.c` | `main()`: wiring config, start-up, output loop |
| `imu_config.h` | Which IMU reports are on, and their rates |
| `imu.c` / `imu.h` | Reads the IMU, works out hand pose and stillness, prints / CSV |
| `bno08x.c` / `bno08x.h` | Minimal BNO08x driver: I2C + reset pin glue for the SH-2 library |
| `sh2/` | CEVA (Hillcrest) SH-2 sensor hub library, unmodified, Apache-2.0 (see `sh2/include/NOTICE.txt`) |
| `tools/imu_logger.py` | Records and plots IMU data on the computer |
| `recordings/` | Saved recordings (CSV) |
