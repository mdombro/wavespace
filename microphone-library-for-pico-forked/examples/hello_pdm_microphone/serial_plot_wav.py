#!/usr/bin/env python3
"""Plot microphone samples from a serial stream and record into a WAV file.

The script listens to a binary stream of little-endian 16-bit samples coming
from a microcontroller over the serial port. Samples are rendered in real time
using matplotlib and saved into a mono 16-bit PCM WAV file on exit.

Example:
    python serial_plot_wav.py --port /dev/ttyACM0 --baud 115200 \
        --sample-rate 16000 --outfile capture.wav
"""
from __future__ import annotations

import argparse
import queue
import signal
import sys
import threading
import time
import wave
from collections import deque
from typing import Deque

import matplotlib.pyplot as plt
import numpy as np
try:
    import serial
except ImportError as exc:
    print(
        "pyserial is required. Install it with 'pip install pyserial'.",
        file=sys.stderr,
    )
    sys.exit(1)

if not hasattr(serial, "Serial"):
    print(
        "The installed 'serial' package does not expose Serial(). "
        "This usually means the wrong package is installed. "
        "Uninstall it and install pyserial instead:\n"
        "    pip uninstall -y serial\n"
        "    pip install pyserial",
        file=sys.stderr,
    )
    sys.exit(1)


from matplotlib import animation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot and record microphone samples streamed over serial."
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Serial port providing samples (default: /dev/ttyACM0)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate for the serial port (default: 115200)",
    )
    parser.add_argument(
        "--sample-rate",
        type=int,
        default=16000,
        help="Sample rate to encode into the WAV file (default: 16000 Hz)",
    )
    parser.add_argument(
        "--outfile",
        default="recording.wav",
        help="Destination WAV filename (default: recording.wav)",
    )
    parser.add_argument(
        "--plot-window",
        type=int,
        default=2000,
        help="Number of recent samples to display (default: 2000)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="Serial read timeout in seconds (default: 1.0)",
    )
    return parser.parse_args()


try:
    SerialException = serial.SerialException  # type: ignore[attr-defined]
except AttributeError:
    try:
        from serial.serialutil import SerialException  # type: ignore
    except (ImportError, AttributeError):  # pragma: no cover - defensive fallback
        SerialException = Exception  # type: ignore[assignment]


def open_serial(port: str, baud: int, timeout: float) -> serial.Serial:
    try:
        return serial.Serial(port=port, baudrate=baud, timeout=timeout)
    except SerialException as exc:  # type: ignore[misc]
        print(f"Failed to open serial port {port}: {exc}", file=sys.stderr)
        sys.exit(1)


def serial_reader(
    ser: serial.Serial,
    sample_queue: "queue.Queue[np.ndarray]",
    stop_event: threading.Event,
) -> None:
    pending = bytearray()
    while not stop_event.is_set():
        try:
            request_size = max(ser.in_waiting, 1024)
            chunk = ser.read(request_size)
        except SerialException:  # type: ignore[misc]
            # If the port disappears, give the main thread a chance to exit.
            stop_event.set()
            break

        if not chunk:
            continue

        pending.extend(chunk)
        usable = len(pending) & ~1  # even number of bytes
        if usable == 0:
            continue

        frame = pending[:usable]  # copy so we can resize the original buffer safely
        samples = np.frombuffer(frame, dtype="<i2").astype(np.int16, copy=True)
        sample_queue.put(samples, block=True)
        del pending[:usable]


def write_wav(filename: str, sample_rate: int, data_bytes: bytes) -> None:
    with wave.open(filename, "wb") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)  # int16
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(data_bytes)


def install_sigint_handler(stop_event: threading.Event) -> None:
    def handler(signum, frame):  # type: ignore[override]
        stop_event.set()

    signal.signal(signal.SIGINT, handler)


def backend_is_interactive() -> bool:
    backend = plt.get_backend().lower()
    non_interactive = {
        "agg",
        "cairo",
        "cairoagg",
        "pdf",
        "pgf",
        "ps",
        "svg",
        "svgagg",
        "template",
    }
    return backend not in non_interactive


def main() -> None:
    args = parse_args()

    ser = open_serial(args.port, args.baud, args.timeout)
    ser.reset_input_buffer()

    sample_queue: "queue.Queue[np.ndarray]" = queue.Queue()
    stop_event = threading.Event()
    install_sigint_handler(stop_event)

    reader = threading.Thread(
        target=serial_reader, args=(ser, sample_queue, stop_event), daemon=True
    )
    reader.start()

    ring_buffer: "Deque[int]" = deque([0] * args.plot_window, maxlen=args.plot_window)
    recorded_bytes = bytearray()

    fig, ax = plt.subplots()
    line, = ax.plot(range(args.plot_window), list(ring_buffer))
    ax.set_title("Serial microphone samples")
    ax.set_xlabel("Recent sample index")
    ax.set_ylabel("Amplitude")
    ax.set_ylim(-32768, 32767)
    ax.set_xlim(0, args.plot_window - 1)
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)

    def update(_frame):
        updated = False
        while True:
            try:
                chunk = sample_queue.get_nowait()
            except queue.Empty:
                break

            if not isinstance(chunk, np.ndarray):
                continue

            clipped = np.clip(chunk, -32768, 32767).astype(np.int16, copy=False)
            recorded_bytes.extend(clipped.tobytes())

            if clipped.size >= args.plot_window:
                ring_buffer.clear()
                ring_buffer.extend(clipped[-args.plot_window :].tolist())
            else:
                ring_buffer.extend(clipped.tolist())
            updated = True

        if updated:
            line.set_ydata(np.array(ring_buffer, dtype=np.int16))
        return (line,)

    interactive_backend = backend_is_interactive()

    if interactive_backend:
        animation.FuncAnimation(
            fig, update, interval=50, blit=True, cache_frame_data=False
        )
    else:
        print(
            "Matplotlib backend is non-interactive; skipping live plot. "
            "Use a GUI backend such as TkAgg for real-time visualization.",
            file=sys.stderr,
        )

    try:
        if interactive_backend:
            plt.show(block=True)
        else:
            while not stop_event.is_set():
                update(None)
                time.sleep(0.05)
    finally:
        stop_event.set()
        reader.join(timeout=1.0)
        ser.close()

    if recorded_bytes:
        write_wav(args.outfile, args.sample_rate, recorded_bytes)
        total_samples = len(recorded_bytes) // 2
        print(
            f"Saved {total_samples} samples to {args.outfile} "
            f"at {args.sample_rate} Hz.",
        )
    else:
        print("No samples captured; nothing saved.")


if __name__ == "__main__":
    main()
