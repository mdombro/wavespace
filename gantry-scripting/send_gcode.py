#!/usr/bin/env python3
"""
Simple script to send G-code commands to a 3D printer via serial port.

python3 test.py /dev/ttyUSB0 --x-range 0 100 --y-range 0 50 --z-range 0 20 --spacing 10
"""

import serial
import time

# Serial port configuration
PORT = '/dev/ttyUSB0'
BAUDRATE = 250000  # Must match BAUDRATE in Marlin Configuration.h
TIMEOUT = 2  # seconds

dist = "10"
speed = "500"
# Hardcoded G-code commands
GCODE_COMMANDS = [
    # "G1 X-10 Y-10 F100",  # Move to X50 Y50 at 3000mm/min
    # "G1 Z-10 F100",  # Move Z to 10mm at 1000mm/min
    # "G1 Z0 X0 Y0 F100",  # Move Z to 10mm at 1000mm/min
    
    # "G1 X-5 F5",
    # "G1 X0 F5",
    # "G1 Y-5 F5",
    # "G1 Y0 F5",
    # "G1 Z-5 F5",
    # "G1 Z0 F5",

    # f"G1 X{dist} Y{dist} Z{dist} F{speed}",
    # f"G1 X0 Y0 Z0 F{speed}",

    # f"G1 X5 F{speed}",

    f"G1 X{dist} F{speed}",
    f"G1 Y{dist} F{speed}",
    f"G1 Z{dist} F{speed}",
    f"M42 P16 S255",
    f"G1 X0 Y0 Z0 F{speed}",

    # "G1 Y100 F100",
     
    # "M114",          # Get current position
]

def send_gcode(ser, command):
    """Send a G-code command and wait for 'ok' response."""
    print(f"Sending: {command}")
    ser.write(f"{command}\n".encode('utf-8'))

    # Wait for response
    while True:
        response = ser.readline().decode('utf-8').strip()
        if response:
            print(f"  Response: {response}")
            if 'ok' in response.lower():
                break
    time.sleep(0.1)  # Small delay between commands

def main():
    try:
        # Open serial connection
        print(f"Connecting to {PORT} at {BAUDRATE} baud...")
        ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)

        # Wait for printer to initialize
        print("Waiting for printer to initialize...")
        time.sleep(2)

        # Clear any startup messages
        ser.reset_input_buffer()

        # Send each G-code command
        for command in GCODE_COMMANDS:
            send_gcode(ser, command)

        print("\nAll commands sent successfully!")

    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        print(f"Make sure {PORT} exists and you have permission to access it.")
        print("You may need to run: sudo usermod -a -G dialout $USER")

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
