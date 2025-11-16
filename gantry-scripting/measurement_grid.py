#!/usr/bin/env python3
"""
3D Measurement Grid Script for Marlin/RAMPS

Performs measurements at grid positions with lead-in approach and external pin triggering.
Designed for RAMPS 1.4 board running Marlin firmware.
"""

import serial
import time
import argparse


class MeasurementGrid:
    def __init__(self, port: str, baudrate: int = 250000):
        """Initialize the measurement grid controller."""
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
    
    def connect(self):
        """Establish serial connection."""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=2.0
            )
            time.sleep(2)  # Wait for connection to stabilize

            # Clear any startup messages from Marlin
            self.serial_conn.reset_input_buffer()

            print(f"Connected to {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Error connecting: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Disconnected")
    
    def send_gcode(self, command: str):
        """Send G-code command and wait for ok response."""
        if not self.serial_conn or not self.serial_conn.is_open:
            return

        print(f"Sending: {command}")
        self.serial_conn.write(f"{command}\n".encode('utf-8'))

        # Wait for "ok" response from Marlin
        while True:
            response = self.serial_conn.readline().decode('utf-8').strip()
            if response:
                # Remove all control characters and special characters
                cleaned_response = ''.join(char for char in response if char.isprintable())
                if cleaned_response:
                    print(f"  Response: {cleaned_response}")
                if 'ok' in response.lower():
                    break

        time.sleep(0.1)  # Small delay between commands
    
    def initialize(self, feedrate: float = 1000):
        """Initialize the machine."""
        print("=== Initializing Machine ===")
        # self.send_gcode("G21")  # Millimeters
        self.send_gcode("G90")  # Absolute positioning
        self.send_gcode(f"G1 F{feedrate}")  # Set feedrate
        self.send_gcode("M400")  # Wait for moves to complete
        print("Machine initialized")
    
    def set_pin_high(self, pin: int):
        """Set digital pin high (3.3V/5V)."""
        self.send_gcode(f"M42 P{pin} S255")
    
    def set_pin_low(self, pin: int):
        """Set digital pin low (0V)."""
        self.send_gcode(f"M42 P{pin} S0")
    
    def run_measurement_grid(
        self,
        x_size: float,
        y_size: float,
        z_size: float,
        spacing: float,
        lead_in: float,
        reading_time: float,
        pin: int = 4,
        feedrate: float = 1000,
        start_x: float = 0,
        start_y: float = 0,
        start_z: float = 0
    ):
        """
        Execute 3D measurement grid with lead-in approach.
        
        Args:
            x_size: Total X dimension (mm)
            y_size: Total Y dimension (mm)
            z_size: Total Z dimension (mm)
            spacing: Distance between measurement points (mm)
            lead_in: Distance to approach from before first point in each row (mm)
            reading_time: Duration to hold pin high at each point (seconds)
            pin: Digital pin number to control (default: 4)
            feedrate: Movement speed (mm/min)
            start_x, start_y, start_z: Starting position (mm)
        """
        print("=== Measurement Grid Parameters ===")
        print(f"Grid size: {x_size} x {y_size} x {z_size} mm")
        print(f"Spacing: {spacing} mm")
        print(f"Lead-in distance: {lead_in} mm")
        print(f"Reading time: {reading_time} s")
        print(f"Control pin: {pin}")
        print(f"Feedrate: {feedrate} mm/min")
        print(f"Start position: X{start_x} Y{start_y} Z{start_z}")
        
        # Calculate number of points
        x_points = int(x_size / spacing) + 1
        y_points = int(y_size / spacing) + 1
        z_points = int(z_size / spacing) + 1
        total_points = x_points * y_points * z_points
        
        print(f"Grid points: {x_points} x {y_points} x {z_points} = {total_points} measurements")
        print("=" * 50)
        
        if not self.connect():
            return
        
        try:
            self.initialize(feedrate)
            
            # Ensure pin is set as output and starts low
            self.send_gcode(f"M42 P{pin} S0")
            
            point_count = 0
            
            # Iterate through Z layers
            for z_idx in range(z_points):
                z = start_z + z_idx * spacing
                
                # Iterate through Y rows
                for y_idx in range(y_points):
                    y = start_y + y_idx * spacing
                    
                    # Lead-in: Move to position before first X point
                    lead_x = start_x - lead_in
                    print(f"--- Row Y={y:.2f}, Z={z:.2f} ---")
                    print(f"Lead-in to X={lead_x:.2f}")
                    self.send_gcode(f"G1 X{lead_x:.3f} Y{y:.3f} Z{z:.3f}")
                    self.send_gcode("M400")  # Wait for move to complete
                    
                    # Iterate through X points in this row
                    for x_idx in range(x_points):
                        x = start_x + x_idx * spacing
                        point_count += 1
                        
                        # Move to measurement position
                        print(f"Point {point_count}/{total_points}: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
                        self.send_gcode(f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f}")
                        self.send_gcode("M400")  # Wait for move to complete
                        
                        # Trigger measurement
                        print(f"  Measuring (pin {pin} HIGH for {reading_time}s)...")
                        self.set_pin_high(pin)
                        time.sleep(reading_time)
                        self.set_pin_low(pin)
            
            print("=== Measurement Grid Complete ===")
            print(f"Total measurements: {point_count}")

            # Return to zero position
            print("Returning to zero position...")
            self.send_gcode("G1 X0 Y0 Z0")
            self.send_gcode("M400")  # Wait for move to complete
            print("Returned to zero")

        except KeyboardInterrupt:
            print("Measurement interrupted by user")
            self.set_pin_low(pin)  # Ensure pin is low
        
        except Exception as e:
            print(f"Error during measurement: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            self.disconnect()


def main():
    parser = argparse.ArgumentParser(
        description='3D measurement grid with lead-in approach and pin control',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  python measurement_grid.py /dev/ttyUSB0 \\
      --x-size 100 --y-size 50 --z-size 20 \\
      --spacing 10 --lead-in 5 --reading-time 0.5 \\
      --pin 4 --feedrate 1000
        """
    )
    
    parser.add_argument('port', help='Serial port (e.g., /dev/ttyUSB0 or COM3)')
    parser.add_argument('--baudrate', type=int, default=250000,
                        help='Baud rate (default: 250000)')
    
    parser.add_argument('--x-size', type=float, required=True,
                        help='X dimension in mm')
    parser.add_argument('--y-size', type=float, required=True,
                        help='Y dimension in mm')
    parser.add_argument('--z-size', type=float, required=True,
                        help='Z dimension in mm')
    
    parser.add_argument('--spacing', type=float, required=True,
                        help='Distance between measurement points in mm')
    parser.add_argument('--lead-in', type=float, required=True,
                        help='Lead-in distance before first point in each row (mm)')
    parser.add_argument('--reading-time', type=float, required=True,
                        help='Duration to hold pin high at each point (seconds)')
    
    parser.add_argument('--pin', type=int, default=4,
                        help='Digital pin number to control (default: 4)')
    parser.add_argument('--feedrate', type=float, default=1000,
                        help='Movement speed in mm/min (default: 1000)')
    
    parser.add_argument('--start-x', type=float, default=0,
                        help='Starting X position in mm (default: 0)')
    parser.add_argument('--start-y', type=float, default=0,
                        help='Starting Y position in mm (default: 0)')
    parser.add_argument('--start-z', type=float, default=0,
                        help='Starting Z position in mm (default: 0)')
    
    args = parser.parse_args()
    
    # Validate inputs
    if args.spacing <= 0:
        parser.error('Spacing must be positive')
    if args.reading_time < 0:
        parser.error('Reading time must be non-negative')
    if args.lead_in < 0:
        parser.error('Lead-in distance must be non-negative')
    
    controller = MeasurementGrid(args.port, args.baudrate)
    
    controller.run_measurement_grid(
        x_size=args.x_size,
        y_size=args.y_size,
        z_size=args.z_size,
        spacing=args.spacing,
        lead_in=args.lead_in,
        reading_time=args.reading_time,
        pin=args.pin,
        feedrate=args.feedrate,
        start_x=args.start_x,
        start_y=args.start_y,
        start_z=args.start_z
    )


if __name__ == '__main__':
    main()