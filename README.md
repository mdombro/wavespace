# wavespace
Wide-area Acoustic Vector-field Estimation for Spatial Probing & Array Calibration Environment

... measuring waves, in 3D

Note right now these docs are GPT generated and kinda are not great / misleading .. Need to redo with better info
For example the pico project building doesn't need to be so manual, can just call the pico extension compile command
And for bitstream capture it doesn't need to have that many options passed in

---

To run the capture, process, visualize loop, run the scripts in this order:
Re-run capture_raw_pdm.py → stream_filter_pdm.py → make_sample_points.py and launch pyvista-viz.py against the regenerated point_*.npz files. The visualization still sees one scalar stream, but it’s now the sum of both microphones (see _process_chunk in stream_filter if you want to change how the channels are combined).

./spi_helper_wrapper.sh --device /dev/spidev0.0 --speed 30000000 --mode 3 \
    --frame-bytes 530 --frames 20000 --output raw_frames.bin

gcc -O2 -Wall -o data-capture/spi_capture_helper data-capture/spi_capture_helper.c


## Developer Environment Setup

### Python workspace
1. Install Python 3.10+ plus `pip`.
2. Create a virtual environment:
   ```bash
   python -m venv .venv
   source .venv/bin/activate
   pip install -U pip
   ```
3. Install dependencies shared across the visualization and tooling scripts:
   ```bash
   pip install -r requirements.txt
   ```
   (Add any project-specific extras as needed.)

### Pico C/C++ toolchain (Raspberry Pi Pico / Pico 2 W)
1. Install VS Code and add the official **Raspberry Pi Pico** extension.
2. Inside VS Code press `Ctrl+Shift+P` → “Raspberry Pi Pico: Configure project”.
3. When prompted for the board, select **Raspberry Pi Pico 2 W**.
4. Point the extension to your Pico SDK folder (or let it download a copy).
5. Install the ARM GCC toolchain (`sudo apt install gcc-arm-none-eabi cmake ninja-build` or use the Pico installer on Windows/macOS).
6. Open the Pico project folder (e.g., `microphone-library-for-pico`) in VS Code; the extension auto-generates CMake + build tasks for the selected board.


## Flashing and Running `hello_pdm_microphone`
1. Build the firmware via VS Code’s Pico tasks (`CMake: Configure`, `CMake: Build`), or manually:
   ```bash
   mkdir -p build && cd build
   cmake -DPICO_BOARD=pico2_w ..
   ninja hello_pdm_microphone
   ```
2. Put the Pico 2 W into UF2 bootloader mode (hold BOOTSEL while plugging in USB).
3. Copy the generated `hello_pdm_microphone.uf2` (typically under `build/`) onto the mass-storage drive that appears (`RPI-RP2`).
4. The board reboots and begins streaming audio samples over the configured interface (GPIO or USB, depending on the project).


## Capturing Raw PDM over USB
`capture_raw_pdm.py` can talk to a Pico streaming PDM bits. The primary method uses the USB vendor interface exposed by the firmware.

Example:
```bash
python capture_raw_pdm.py --usb-vendor 0x2E8A --usb-product 0x000A --seconds 10 --out pdm_capture.raw
```
Key options:
- `--usb-vendor` / `--usb-product`: USB VID/PID (defaults to Raspberry Pi 0x2E8A / sample PID).
- `--interface`: alternate transport (e.g., serial). Leave unset to stay on USB vendor channel.
- `--seconds`: duration to record.
- `--out`: output filename for the raw PDM bitstream.


## Parsing the Bit Stream (`parse_bit_stream.m`)
1. Open MATLAB or Octave.
2. Ensure the working directory contains `parse_bit_stream.m` and the raw capture (e.g., `pdm_capture.raw`).
3. Run:
   ```matlab
   out = parse_bit_stream('pdm_capture.raw', 'Fs', 1536000, 'decimation', 64);
   ```
   Adjust arguments to match your hardware (sampling rate, decimation, channel count). The script converts the interleaved bitstream into baseband audio samples for post-processing.


## Generating Visualization Sample Data
Use `generate_sample_data.py` to create synthetic point-cloud data compatible with both `plotly-viz.py` and `visualization/pyvista-viz.py`.

Per-point NPZ files (default for plotly-viz):
```bash
python generate_sample_data.py --out-dir sample_points --frames 160 --points 6000
```
Optional extras:
```bash
python generate_sample_data.py --frame-dir sample_frames --series-file sample_series.npz
```
Then run a viewer:
```bash
python plotly-viz.py --path sample_points
# or
python visualization/pyvista-viz.py --series sample_series.npz
```


## Running the Zig-Zag Toolpath Visualizer
`zigzag3d.py` produces strict XY-then-Z paths and optional plots/G-code.
Examples:
```bash
python zigzag3d.py --W 120 --H 80 --Z_top 40 --stepover 5 --layer_h 2 --plot
python zigzag3d.py --W 100 --H 100 --Z_top 50 --stepover 4 --layer_h 1.5 --export fill.nc
```
Flags:
- `--plot`: shows a 3D matplotlib preview.
- `--export`: writes a G-code file with the strict path.
- `--start-left`, `--start-bottom`: control raster direction.

Refer to `zigzag3d.py --help` for all options.
