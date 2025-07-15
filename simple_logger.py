#!/usr/bin/env python3
"""
Simple IMU data logger - just prints received data
"""

import serial
import time

def simple_logger(port='COM6', baudrate=115200):
    """Simple serial data logger"""
    try:
        print(f"Connecting to {port}...")
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for Arduino reset
        print(f"Connected! Listening for data...")
        print("=" * 50)
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    print(f"📊 {line}")
            time.sleep(0.01)
            
    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        if 'ser' in locals():
            ser.close()
            print("Disconnected")

if __name__ == "__main__":
    simple_logger()
