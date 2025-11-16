# Gantry Scripting

Python scripts for controlling a 3D gantry system with Marlin firmware via serial communication.

## Gantry Size Limits

- X: 130 mm
- Y: 100 mm
- Z: 95 mm

## Scripts

### measurement_grid.py

Automated 3D measurement grid with configurable spacing, lead-in, and pin control for external device triggering.

**Example Usage:**

```bash
python3 measurement_grid.py /dev/ttyUSB0 \
    --x-size 100 --y-size 100 --z-size 90 \
    --spacing 10 --lead-in 5 --reading-time 0.5 \
    --pin 4 --feedrate 1000
```

**Test Script (smaller grid):**

```bash
python3 measurement_grid.py /dev/ttyUSB0 \
    --x-size 25 --y-size 25 --z-size 25 \
    --spacing 10 --lead-in 5 --reading-time 0.5 \
    --pin 4 --feedrate 1000
```

### send_gcode.py

Simple utility for sending custom G-code commands directly to the gantry. Useful for manual testing, calibration, and debugging.

Edit the `GCODE_COMMANDS` array in the script to define custom G-code sequences, then run:

```bash
python3 send_gcode.py
```

**Note:** The script uses hardcoded commands. Modify the `GCODE_COMMANDS` list in `send_gcode.py` to customize the G-code sequence.

## Configuration

- **Default Baudrate:** 250000 (must match Marlin Configuration.h)
- **Default Serial Port:** `/dev/ttyUSB0` (Linux) or `COM3` (Windows)

## Requirements

- Python 3
- pyserial (`pip install pyserial`)
