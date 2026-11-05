# ImageRaster Digital Twin

A Processing simulation of the raster scan, built on the unit's CW2026 Digital Twin
([paulodowd/RoboticsScienceAndSystems/DigitalTwin](https://github.com/paulodowd/RoboticsScienceAndSystems/tree/main/DigitalTwin)).
It runs the **same raster controller** as the robot (`Controller.pde` is a line-for-line port of
`Controller.cpp`), scans the **same printed sheet** (`surface.png`), and rebuilds the image the
same way, so the twin and the real robot can be compared run for run.

## Run it

1. Install [Processing 4](https://processing.org/download).
2. Open `DigitalTwin/DigitalTwin.pde` and press Run. Keep all the files in a folder called `DigitalTwin`.
3. The twin starts a scan at once (sandbox mode). The right panel shows the printed sheet, the twin's
   rebuild, and the real robot's rebuild.

| Key | Does |
|---|---|
| S / X | start / stop a twin scan |
| N | 10 repeats in a row (fast), for experiments |
| F | fast-forward on/off (sandbox only) |
| P | switch the physical model: **Nominal** (template assumptions) or **Calibrated** (your Phase 1 values) |
| C | copy the scan settings from the robot's dashboard (on the robot's WiFi) |
| M | sandbox / **live comparison** with the real robot |
| R / G | image rebuilds panel / telemetry graphs |
| T, +/-, arrows | track the robot, zoom, pan |

## The printed sheet

Print `test_sheet_A3.pdf` on **A3 at 100 % ("Actual size")**. Check the ruler: 0 to 200 should
measure 200 mm. Tape it flat. Put the robot's **wheel-axle centre on the cross**, facing the arrow,
with *Next row is on: the left* (rows go up the page). With the default settings (17 rows × 160 mm,
7 mm pitch) the scan covers all the test patches:

- four shapes, like our original sheet;
- grey steps 0–100 % (sensor response);
- bars **across** the rows, 1–6 mm (along-row resolution: sensor spot, delay, speed);
- bars **along** the rows, 2–6 mm (across-row resolution: row pitch, pose error).

`surface.png` is the same sheet at 1 pixel per mm, and `surface.json` says where it sits and which
scan settings cover it. The twin reads both when it starts.

## Use your own picture

Any picture works (a drawing, a logo, a photo). One command makes the printed sheet and the twin's
copy together, so they always match:

```
python3 tools/make_sheet.py my_picture.png --width 150          # grey levels kept
python3 tools/make_sheet.py my_picture.png --width 150 --bw     # pure black and white
python3 tools/make_sheet.py --use test_sheet                    # back to the test sheet
```

It writes `DigitalTwin/sheets/<name>.pdf` (print it at 100 %, "Actual size"), picks A4 or A3 for
you, and makes the picture the twin's sheet (restart the sketch). It also prints the **dashboard
settings** that cover the picture (Rows and Row length; they are printed on the sheet too):
type them into the dashboard's Scan settings and press *Send to robot*. The twin uses them
automatically. `sheets/example_house.pdf` is an example on A4.

Tips: high-contrast pictures scan best (`--bw`); details smaller than about 3 mm blur away (the
sensor spot); keep the picture under about 170 × 105 mm for A4, or up to about 290 × 190 mm on A3.
The robot itself needs about 10 cm of flat table beyond both ends of the rows.

`tools/make_test_sheet.py` regenerates the test sheet. You can also export any sheet from Inkscape
as PNG at **25.4 dpi** (= 1 px/mm) and set `SURFACE_OFFSET_X_MM/Y_MM` in `DigitalTwin.pde` by hand,
but then delete `surface.json` so it doesn't override them.

## Live comparison with the real robot

1. Upload the latest firmware (it now drops the telemetry *signal* to 0 when a scan ends, which the
   twin needs to see each new scan).
2. Join the robot's WiFi with the laptop running Processing. Press **M**. The status line shows
   "armed" once the robot's telemetry arrives (TCP port 80), and the twin copies the robot's scan
   settings from the dashboard.
3. Start a scan from the dashboard (or the robot's button). The twin starts at the same moment, from
   the same start pose. The orange ghost and trail are the robot's own odometry.
4. When both finish, the twin saves both runs and the **twin vs robot** image difference.

The twin records the robot from its 20 Hz telemetry, which is enough for the comparison. Keep the
dashboard's **Download CSV** (50 Hz) as your main robot data.

## Update the twin with Phase 1 (`ModelParameters.pde`)

Everything physical is in one file, in two sets: `nominalModel()` (the template's ideal robot, keep
it unchanged as your "before") and `calibratedModel()` (your measured robot). The controller never
sees these numbers: like the real robot it only gets encoder counts and sensor readings, and it uses
its own *beliefs* about the wheels from the scan settings. The gap between the two is what produces
pose error in the image.

| Parameter | What to measure |
|---|---|
| wheel radii, separation, rotation efficiency | ruler distance test, straight-line drift, 360° spin |
| motor gain, deadband, time constant, right/left scale | speed vs PWM, smallest PWM that moves, step response |
| sensor positions | underside of the robot (already measured: DN1 28/27, DN2 40/9, DN3 41/0, DN4 40/−9, DN5 28/−27 mm) |
| sensor white/black levels, noise | readings on white paper and black ink, robot still |
| sensor footprint (spot sigma), delay | scan across a sharp edge (the bars), dashboard Auto-calibrate |
| placement error | put the robot on the start cross 10 times |

Values marked `MEASURED` come from our robot; `ESTIMATE` and `PHASE 1` still need your data.

## Results (`runs/`)

Each run writes, in the sketch folder's `runs/`:

- `<run>_twin_<model>.csv`: the dashboard's CSV format (the dashboard and the notebooks can load it),
  plus `true_x_mm, true_y_mm, true_theta_rad`, which only a simulation knows;
- `<run>_robot.csv`: the real robot's run from its telemetry (live comparison);
- PNG images of each rebuild;
- `summary.csv`: one line per run with coverage, **error vs sheet** (mean darkness error over the
  scan area, 0–100 %), **twin vs robot** difference, pose error (mean and max, twin only) and the
  final heading error.

## Experiment ideas for Assessment 2

- **Does updating the twin make it predict the robot?** Same scan, real robot vs twin with
  *Nominal* and with *Calibrated*: compare *twin vs robot* (10 repeats each).
- **Path spacing:** row pitch 5 / 7 / 10 / 13.5 mm against the *bars along rows*.
- **Speed:** scan PWM 20 / 30 / 45 against the *bars across rows* (sensor delay and footprint).
- **Pose error:** back and forth vs U-turns; wrong vs calibrated wheel geometry.
