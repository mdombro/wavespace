#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import struct
import json
import os
import time
from datetime import datetime
from typing import Optional

DEFAULT_PORT = "/dev/ttyUSB0"   # fallback if auto-detect fails
BAUD = 921600

MAGIC = 0xABCD1234
HDR_FMT = "<IIII"        # little-endian: magic, index, bit_count, channel_count
HDR_SIZE = struct.calcsize(HDR_FMT)

# Directory to store captures
OUTPUT_DIR = "pdm_captures"


def auto_detect_serial_port() -> Optional[str]:
    """Pick a likely Pico serial device from available ports."""
    preferred_prefixes = ("/dev/ttyACM", "/dev/ttyUSB", "COM")
    ports = list(serial.tools.list_ports.comports())

    for prefix in preferred_prefixes:
        for port in ports:
            if port.device.startswith(prefix):
                return port.device
    return ports[0].device if ports else None


def read_exact(ser: serial.Serial, n: int, idle_timeout: float = 2.0) -> bytes:
    """Read exactly n bytes from serial, tolerating brief pauses."""
    buf = bytearray()
    last_progress = time.monotonic()

    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
            last_progress = time.monotonic()
            continue

        # No data this read: check if we've been idle for too long
        if (time.monotonic() - last_progress) >= idle_timeout:
            raise RuntimeError("Serial read timeout or device disconnected")

    return bytes(buf)


def flush_serial(ser: serial.Serial) -> None:
    """Drop any buffered bytes so the next header starts aligned."""
    try:
        ser.reset_input_buffer()
    except serial.SerialException:
        pass


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    port = os.environ.get("PDM_CAPTURE_PORT")
    if not port:
        port = auto_detect_serial_port()
    if not port:
        port = DEFAULT_PORT
        print("Warning: could not auto-detect serial port, "
              f"falling back to {DEFAULT_PORT}. "
              "Set PDM_CAPTURE_PORT to override.")

    print(f"Opening {port} at {BAUD} baud...")
    ser = None

    print("Listening for PDM captures...")
    while True:
        if ser is None or not ser.is_open:
            try:
                ser = serial.Serial(port, BAUD, timeout=1)
            except serial.SerialException as exc:
                print(f"Failed to open {port}: {exc}. Retrying in 1s...")
                time.sleep(1)
                continue

        try:
            # 1) Read header
            hdr_bytes = read_exact(ser, HDR_SIZE)
            magic, index, bit_count, channel_count = struct.unpack(HDR_FMT, hdr_bytes)
        except RuntimeError as exc:
            print(f"{exc}; flushing and waiting for next frame...")
            flush_serial(ser)
            time.sleep(1)
            continue
        except serial.SerialException as exc:
            print(f"Serial error: {exc}. Reopening port...")
            ser.close()
            time.sleep(1)
            continue

        if magic != MAGIC:
            print(f"Bad magic 0x{magic:08X}, flushing to resync...")
            flush_serial(ser)
            time.sleep(0.1)
            continue

        # 2) Compute how many bytes to read per-channel for payload
        byte_count = (bit_count + 7) // 8  # round up to full bytes
        total_payload_bytes = byte_count * channel_count

        channel_payloads = []
        try:
            # 3) Read the payload for each channel sequentially
            for ch in range(channel_count):
                payload = read_exact(ser, byte_count)
                channel_payloads.append(payload)
        except RuntimeError as exc:
            print(f"{exc} while reading payload; flushing and discarding capture...")
            flush_serial(ser)
            time.sleep(1)
            continue
        except serial.SerialException as exc:
            print(f"Serial error while reading payload: {exc}. Reopening port...")
            ser.close()
            time.sleep(1)
            continue

        # 4) Build filenames
        timestamp = datetime.utcnow().isoformat() + "Z"
        base_name = f"capture_{index:06d}"
        channel_paths = [
            os.path.join(OUTPUT_DIR, f"{base_name}_ch{ch}.bin")
            for ch in range(channel_count)
        ]
        meta_path = os.path.join(OUTPUT_DIR, base_name + ".json")

        print(
            f"Writing capture #{index}: {channel_count} channels, "
            f"{bit_count} bits/channel ({total_payload_bytes} bytes total)"
        )

        # 5) Write raw PDM bits to per-channel .bin files
        for ch, path in enumerate(channel_paths):
            with open(path, "wb") as f_data:
                f_data.write(channel_payloads[ch])
            print(f"  Channel {ch}: wrote {byte_count} bytes -> {path}")

        # 6) Write metadata JSON
        metadata = {
            "magic": f"0x{magic:08X}",
            "index": index,
            "bit_count": bit_count,
            "byte_count_per_channel": byte_count,
            "total_payload_bytes": total_payload_bytes,
            "channel_count": channel_count,
            "timestamp_utc": timestamp,
            "serial_port": port,
            "baud": BAUD,
            "channels": [
                {
                    "channel_index": ch,
                    "data_file": os.path.basename(path),
                    "byte_count": byte_count,
                }
                for ch, path in enumerate(channel_paths)
            ],
            "notes": "PDM data packed as little-endian stream of bits; "
                     "origin: Raspberry Pi Pico PIO+DMA capture",
        }
        with open(meta_path, "w", encoding="utf-8") as f_meta:
            json.dump(metadata, f_meta, indent=2)

        print(
            f"Saved capture #{index}: "
            f"{bit_count} bits/channel ({byte_count} bytes each) "
            f"-> {base_name}_ch[0-{channel_count - 1}].bin + .json"
        )
        print(f"Capture #{index} finished writing to disk.\n")


if __name__ == "__main__":
    main()
