#!/usr/bin/env python3
"""
Demo data generator for testing the IMU visualizer without hardware
Simulates Arduino sending CSV data over serial port
"""

import time
import math
import random

def generate_demo_data():
    """Generate simulated IMU data"""
    t = time.time()
    
    # Simulate some motion patterns
    ax = 0.1 * math.sin(t * 2) + random.uniform(-0.05, 0.05)  # Small oscillation
    ay = 0.2 * math.cos(t * 1.5) + random.uniform(-0.05, 0.05)
    az = 1.0 + 0.1 * math.sin(t * 3) + random.uniform(-0.05, 0.05)  # ~1g with variation
    
    # Gyroscope data (degrees/second)
    gx = 10 * math.sin(t * 0.5) + random.uniform(-2, 2)
    gy = 15 * math.cos(t * 0.7) + random.uniform(-2, 2)
    gz = 5 * math.sin(t * 1.2) + random.uniform(-1, 1)
    
    # Magnetometer data (micro Tesla)
    mx = 25 + 5 * math.sin(t * 0.3) + random.uniform(-1, 1)
    my = 15 + 3 * math.cos(t * 0.4) + random.uniform(-1, 1)
    mz = -35 + 4 * math.sin(t * 0.6) + random.uniform(-1, 1)
    
    return ax, ay, az, gx, gy, gz, mx, my, mz

def main():
    """Main demo loop"""
    print("WearGo IMU Demo Data Generator")
    print("Generating simulated CSV data...")
    print("Press Ctrl+C to stop")
    print("=" * 50)
    
    # Print header
    print("ax,ay,az,gx,gy,gz,mx,my,mz")
    
    try:
        while True:
            ax, ay, az, gx, gy, gz, mx, my, mz = generate_demo_data()
            
            # Format as CSV with 4 decimal places
            csv_line = f"{ax:.4f},{ay:.4f},{az:.4f},{gx:.4f},{gy:.4f},{gz:.4f},{mx:.4f},{my:.4f},{mz:.4f}"
            print(csv_line)
            
            time.sleep(0.05)  # 20 Hz update rate
            
    except KeyboardInterrupt:
        print("\nDemo stopped.")

if __name__ == "__main__":
    main()
