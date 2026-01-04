# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Wavespace (Wide-area Acoustic Vector-field Estimation for Spatial Probing & Array Calibration Environment) captures high-fidelity PDM microphone data at multiple 3D positions using a computer-controlled gantry, processes it through filtering pipelines, and visualizes results as interactive 3D point clouds.

## Architecture

The system has five main layers:

1. **Firmware** (`discrete-pdm-capture/`) - Raspberry Pi Pico 2W C firmware with PIO assembly for dual-channel PDM capture. Uses PIO state machines for clock generation, trigger-based capture, and speaker tone generation.

2. **Data Capture** (`data-capture/`) - Python serial listener that reads binary PDM frames from the Pico (921600 baud) and saves per-channel .bin files with JSON metadata.

3. **Gantry Control** (`gantry-scripting/`) - Marlin G-code control via serial (250000 baud). Moves gantry to positions and sends M42 trigger signals to initiate PDM capture.

4. **Data Processing** (`data-processing/`) - FIR filtering, decimation, and channel combining. Outputs NPZ files containing xyz coordinates and filtered audio values.

5. **Visualization** (`visualization/`) - PyVista-based 3D interactive viewer with time series playback. Alternative Plotly Dash web viewer available.

## Common Commands

```bash
# Setup Python environment
python -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt

# Capture PDM data from Pico
python data-capture/capture-listener.py

# Process captured PDM data
python data-processing/post-process-pdm.py --captures pdm_captures --xyz events.csv --out filtered-captures --decimation 1

# Run automated measurement grid
python gantry-scripting/measurement_grid.py /dev/ttyUSB0 --x-size 100 --y-size 100 --z-size 90 --spacing 10

# Visualize processed data
python visualization/pyvista-viz.py --path filtered-captures/ --mode point

# Generate zigzag toolpath with preview
python data-processing/zigzag3d.py --W 120 --H 80 --Z_top 40 --stepover 5 --layer_h 2 --plot
```

## Hardware Configuration

- **Gantry limits**: X: 130mm, Y: 100mm, Z: 95mm
- **Pico pins**: GPIO 2-3 (PDM data), GPIO 4 (clock), GPIO 5 (trigger), GPIO 6 (speaker)
- **Serial baud rates**: Pico USB 921600, Marlin gantry 250000

## Build (Pico Firmware)

Use VS Code with Raspberry Pi Pico extension. Import `discrete-pdm-capture/` folder, select `pico2_w` board, then Compile Project. Flash by copying `.uf2` to Pico in bootloader mode.

## Data Flow

Gantry moves to position → M42 trigger → Pico captures dual-channel PDM via DMA → USB serial to host → FIR filter + decimation → NPZ files → 3D visualization with playback
