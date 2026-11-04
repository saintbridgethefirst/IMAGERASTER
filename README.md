# ImageRaster: 3Pi+ raster scan with a live dashboard

The robot scans a printed image in rows, logging its odometry pose and the five surface
sensors (DN1 to DN5). The StampC3 serves a web dashboard over its own WiFi that rebuilds the
image as the robot scans.

**Back and forth (default, sharper).** The robot never turns. It drives row 0 forwards, row 1 in
reverse, and so on, always facing the same way. It steers gently onto the next row line during
a short run-in past each end of the image:

```
 row 2   ~~-------------------->~~
 row 1   ~~<--------------------~~    <-- driven in reverse
 row 0     o------------------->~~    o = start, robot facing right
         |  run-in |  image  | run-in |
```

**U-turns (option).** The robot turns 90°, shifts sideways, and turns 90° again at each row end.
Turning on the spot makes the wheels slip, which spaced our rows 5–32 mm apart instead of 14 mm,
so we switched to back and forth for imaging. Keep U-turns as a comparison for your report.

## What changed from the template

| File | Status |
|---|---|
| `ImageRaster.ino` | Template `.ino` plus the dashboard. **Change the WiFi name here.** |
| `Controller.h / .cpp` | Line follower replaced by the raster state machine. Same public shape (`update()`, `setSignal()`, `odometry`). The 14-column Serial/TCP telemetry is unchanged, so the Digital Twin still reads it. |
| `ScanLog.h / .cpp` | New: keeps every sample of a run in RAM (up to 5000; reserved before WiFi starts). |
| `WebDashboard.h / .cpp` | New: web server on port **8080** (port 80 stays the template's telemetry). |
| `dashboard_html.h` | Generated from `dashboard/dashboard.html`. Do not edit by hand. |
| `dashboard/` | Dashboard source and `make_header.py`. The Arduino IDE ignores this folder. |
| Everything else | Unchanged files from the SEMTM0043 StampC3 template ([paulodowd/RoboticsScienceAndSystems](https://github.com/paulodowd/RoboticsScienceAndSystems)). |

## Set up and upload

1. Arduino IDE with the **esp32 by Espressif** package (3.x), plus the **LSM6** and **LIS3MDL** libraries by Pololu.
2. Keep all files in a folder called `ImageRaster` and open `ImageRaster.ino`.
3. In `ImageRaster.ino`, set your own WiFi name (unique in the lab) and a password of 8+ characters.
4. Board: **M5StampC3** (M5Stack board package, as in the course's Getting Started page).
   Set **USB CDC On Boot: Disabled** (the StampC3 talks over a USB-serial chip). Upload speed 115200 on macOS.
5. Switch the robot on, then upload. The Serial Monitor (115200) prints the dashboard address.

This compiles cleanly (about 85% of flash).

## Before the first scan

- **Check the sensor positions.** The 3Pi+ sensors are not a straight bar. DN2, DN3 and DN4 sit
  about 9 mm apart near the front (about 40 mm ahead of the axle). DN1 and DN5 sit further out
  (about 27 mm left/right) and further back (about 28 mm ahead). The dashboard's *Image settings →
  Sensor positions* table holds these (measured from a photo, ±2 mm). Check them with a ruler.
  The axle centre is midway between the two wheel centres. A wrong layout smears each shape into
  stripes. The table is remembered by your browser.
- **Row pitch.** The default is 7 mm, ¾ of the ~9 mm DN2–DN3 spacing. The middle sensor tracks of
  neighbouring rows then interleave every ~2.3 mm. 13.5 mm gives ~4.5 mm detail at half the time.
  The dashboard shows these options under the sensor table, and estimates whether a scan fits the
  robot's memory.
- **Wheel separation.** 82.5 mm is the value we calibrated (the template's 90 mm made every U-turn
  over-rotate and the scan drift off the paper). Wheel radii are still the nominal 16 mm.
- **Place the robot** facing along the first row, with DN3 (front centre) at the start edge of
  the image, just above the top of the shapes. With *Next row is on: the left*, the image must
  extend to the robot's left. Leave about 10 cm of free table behind the robot and past the far
  end of the image, for the reverse rows and run-ins.

## Run a scan

1. Join the robot's WiFi on a laptop or phone and open **http://192.168.4.1:8080**.
2. Press **Start scan** (or the StampC3 button). The robot waits 1 s, then scans.
3. **Stop** (or the StampC3 button again) stops the robot at any time.
4. Press **Download CSV** after each run. This is your raw data for the submission.

The dashboard also loads CSV files: its own exports, or the template's 14-column Serial log.
When it is not connected to a robot it shows a simulated scan so you can explore the controls.

## Things the dashboard lets you vary (good experiment variables)

- **Path**: back and forth vs U-turns. Compare the *Row gaps* readout and the image: same sensors,
  different pose error.
- **Row pitch** (path spacing): gaps, overlap or even interleaving between rows.
- **Scan PWM**: speed against image sharpness (motor lag, sensor timing).
- **Brightness scaling**: *each sensor to its own range* vs *one shared range*. The shared range
  shows stripes where sensors differ in offset and gain (your reflectance characterisation).
- **Sensor footprint**: how wide each reading is spread when the image is rebuilt.
- **Auto-calibrate from this scan**: readings that land in the same place should agree. This
  button fits the sensor delay (readings describe where the robot was a few ms earlier) and the
  DN1/DN5 and DN2/DN4 positions so they do, with DN3 as the reference. It writes the result into
  the sensor table. In simulation it recovered a known layout to within ±0.5 mm and a 30 ms delay
  exactly.
- **Deblur**: each reflectance sensor sees a spot several mm wide, which softens every edge.
  Deblur undoes this with Richardson–Lucy deconvolution, using *Sensor blur σ* (default 3 mm)
  plus the footprint. If halos appear, lower σ. It is skipped while a scan is running.
- **Use** column in the sensor table: leave sensors out of the image (for example DN1/DN5) to see
  what each one contributes.
- **Tone**: smooth (raw tones), high contrast, or black and white (threshold at 50%).
- **Wheel geometry**: rerun with nominal vs calibrated values and compare the rows' alignment.

## If something looks wrong

| Symptom | Likely cause |
|---|---|
| "Turn timed out" (U-turns) | *Turn min* PWM is inside the motor deadband. Raise it (the robot now also boosts a stalled turn by itself). |
| *Row gaps* far from the target | Wheel slip or wrong wheel geometry. Use back and forth, and check the separation. |
| "Row timed out" | Scan PWM too low to move the robot. |
| Shapes smeared into horizontal stripes | The sensor position table doesn't match the robot. |
| Rows shifted against each other | Wheel radius / separation not calibrated, or the sensor positions are wrong. |
| The image is mirrored or stretched | Sensor positions are wrong, or *Next row is on* is the wrong side. |
| "Log full" | Use fewer or shorter rows, or a larger `LOG_INTERVAL_MS` in `Controller.h`. |
| The page does not load | Check you are on the robot's WiFi and include `:8080` in the address. |

## Getting a crisp image (no re-scan needed)

1. After a scan, press **Auto-calibrate from this scan**.
2. Set **Deblur → Strong** (the default), with **Sensor blur σ** around 3 mm.
3. For outlines, set **Tone → Black and white**. Keep **Smooth pixels** on with a 1 mm cell.

## Editing the dashboard

Edit `dashboard/dashboard.html`, then run `python3 dashboard/make_header.py` and upload again.
