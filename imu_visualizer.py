#!/usr/bin/env python3
"""
WearGo IMU Visualizer
Real-time visualization of IMU data from Arduino hardware
"""

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import time
import threading
from collections import deque
import argparse
import sys


class SingleIMUVisualizer:
    def __init__(self, port=None, baudrate=115200, buffer_size=100):
        self.port = port
        self.baudrate = baudrate
        self.buffer_size = buffer_size

        # Data buffers for IMU (BMI270/BMM150)
        self.time_buffer = deque(maxlen=buffer_size)
        self.accel_buffer = {'x': deque(maxlen=buffer_size), 
                             'y': deque(maxlen=buffer_size), 
                             'z': deque(maxlen=buffer_size)}
        self.gyro_buffer = {'x': deque(maxlen=buffer_size), 
                            'y': deque(maxlen=buffer_size), 
                            'z': deque(maxlen=buffer_size)}
        self.mag_buffer = {'x': deque(maxlen=buffer_size), 
                           'y': deque(maxlen=buffer_size), 
                           'z': deque(maxlen=buffer_size)}

        # Orientation estimation
        self.orientation = {'roll': 0, 'pitch': 0, 'yaw': 0}

        # Serial connection
        self.serial_conn = None
        self.running = False
        self.data_thread = None

        # Start time for relative timestamps
        self.start_time = time.time()

    def create_simple_orientation_model(self):
        """Create a simple, clear 3D model for orientation visualization"""
        
        # Create a simple rectangular device representation
        # Like a phone or Arduino board - easy to see orientation
        
        # Device dimensions (like a phone/board)
        length = 0.4   # X direction
        width = 0.25   # Y direction  
        height = 0.05  # Z direction (thin)
        
        # Define the 8 corners of the rectangular device
        vertices = np.array([
            # Bottom face
            [-length/2, -width/2, -height/2],  # 0
            [length/2, -width/2, -height/2],   # 1
            [length/2, width/2, -height/2],    # 2
            [-length/2, width/2, -height/2],   # 3
            # Top face
            [-length/2, -width/2, height/2],   # 4
            [length/2, -width/2, height/2],    # 5
            [length/2, width/2, height/2],     # 6
            [-length/2, width/2, height/2],    # 7
        ])
        
        # Define faces (rectangles as pairs of triangles)
        faces = [
            # Bottom face
            [0, 1, 2], [0, 2, 3],
            # Top face  
            [4, 6, 5], [4, 7, 6],
            # Front face
            [0, 5, 1], [0, 4, 5],
            # Back face
            [2, 7, 3], [2, 6, 7],
            # Left face
            [0, 7, 4], [0, 3, 7],
            # Right face
            [1, 6, 2], [1, 5, 6],
        ]
        
        return vertices, faces
        
    def find_arduino_port(self):
        """Auto-detect Arduino port"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Look for common Arduino identifiers
            if any(identifier in port.description.lower() for identifier in 
                   ['arduino', 'ch340', 'cp210', 'ftdi', 'usb serial']):
                print(f"Found potential Arduino on {port.device}: {port.description}")
                return port.device
        return None
    
    def connect(self):
        """Connect to serial port"""
        if not self.port:
            self.port = self.find_arduino_port()
            if not self.port:
                print("No Arduino found. Please specify port manually.")
                return False
        
        print(f"Attempting to connect to {self.port}...")
        
        # Try multiple connection attempts
        for attempt in range(3):
            try:
                # Close any existing connection first
                if self.serial_conn:
                    self.serial_conn.close()
                    time.sleep(1)
                    self.serial_conn.close()
                    time.sleep(1)
                
                print(f"Connection attempt {attempt + 1}/3...")
                self.serial_conn = serial.Serial(
                    port=self.port, 
                    baudrate=self.baudrate, 
                    timeout=1,
                    exclusive=True  # Try to get exclusive access
                )
                time.sleep(2)  # Wait for Arduino to reset
                print(f"✓ Connected to {self.port} at {self.baudrate} baud")
                return True
                
            except serial.SerialException as e:
                if "PermissionError" in str(e) or "Access is denied" in str(e):
                    print(f"✗ Attempt {attempt + 1}: Port {self.port} is busy or access denied")
                    if attempt < 2:
                        print("  Waiting 3 seconds before retry...")
                        time.sleep(3)
                    else:
                        print("\n🚨 TROUBLESHOOTING STEPS:")
                        print("1. Close Arduino IDE Serial Monitor")
                        print("2. Close PlatformIO Serial Monitor") 
                        print("3. Close any terminal programs (PuTTY, TeraTerm)")
                        print("4. Disconnect and reconnect USB cable")
                        print("5. Try a different USB port")
                        print(f"6. Run as administrator: python imu_visualizer.py --port {self.port}")
                else:
                    print(f"✗ Connection error: {e}")
                    
        return False
    
    def disconnect(self):
        """Disconnect from serial port"""
        self.running = False
        if self.data_thread:
            self.data_thread.join()
        if self.serial_conn:
            self.serial_conn.close()
            print("Disconnected from Arduino")
    
    def parse_csv_line(self, line):
        """Parse IMU CSV line: ax,ay,az,gx,gy,gz,mx,my,mz"""
        try:
            values = [float(x.strip()) for x in line.split(',')]
            if len(values) == 9:  # 9 values from IMU
                return {
                    'accel': {'x': values[0], 'y': values[1], 'z': values[2]},
                    'gyro': {'x': values[3], 'y': values[4], 'z': values[5]},
                    'mag': {'x': values[6], 'y': values[7], 'z': values[8]}
                }
        except ValueError:
            pass
        return None

    def update_orientation(self, accel, gyro, dt):
        """Simple complementary filter for orientation estimation"""
        # Calculate roll and pitch from accelerometer
        accel_roll = np.arctan2(accel['y'], accel['z']) * 180 / np.pi
        accel_pitch = np.arctan2(-accel['x'], np.sqrt(accel['y']**2 + accel['z']**2)) * 180 / np.pi

        # Integrate gyroscope for angular rates (convert to degrees/sec)
        gyro_roll = self.orientation['roll'] + gyro['x'] * dt
        gyro_pitch = self.orientation['pitch'] + gyro['y'] * dt
        gyro_yaw = self.orientation['yaw'] + gyro['z'] * dt

        # Complementary filter (98% gyro, 2% accel for roll/pitch)
        alpha = 0.98
        self.orientation['roll'] = alpha * gyro_roll + (1 - alpha) * accel_roll
        self.orientation['pitch'] = alpha * gyro_pitch + (1 - alpha) * accel_pitch
        self.orientation['yaw'] = gyro_yaw  # Yaw from gyro only

    def read_data(self):
        """Read data from serial port in separate thread"""
        last_time = time.time()

        while self.running:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()

                    # Skip debug messages
                    if line.startswith('BMI270') or line.startswith('Failed') or line.startswith('CSV') or not line:
                        continue

                    data = self.parse_csv_line(line)
                    if data:
                        current_time = time.time()
                        dt = current_time - last_time
                        last_time = current_time

                        # Update buffers
                        rel_time = current_time - self.start_time
                        self.time_buffer.append(rel_time)

                        for axis in ['x', 'y', 'z']:
                            self.accel_buffer[axis].append(data['accel'][axis])
                            self.gyro_buffer[axis].append(data['gyro'][axis])
                            self.mag_buffer[axis].append(data['mag'][axis])

                        # Update orientation
                        self.update_orientation(data['accel'], data['gyro'], dt)

            except serial.SerialException:
                print("Serial connection lost")
                break
            except UnicodeDecodeError:
                continue  # Skip malformed lines

            time.sleep(0.01)  # Small delay to prevent excessive CPU usage
    
    def start_reading(self):
        """Start reading data in background thread"""
        if not self.serial_conn:
            print("Not connected to serial port")
            return False
        
        self.running = True
        self.data_thread = threading.Thread(target=self.read_data, daemon=True)
        self.data_thread.start()
        return True
    
    def create_visualization(self):
        """Create real-time visualization"""
        # Create figure with subplots
        fig = plt.figure(figsize=(16, 10))
        fig.suptitle('WearGo IMU Real-time Visualization', fontsize=16)
        
        # 3D device orientation visualization
        ax_3d = fig.add_subplot(1, 2, 1, projection='3d')
        ax_3d.set_title('3D Device Orientation')
        
        # Accelerometer plot
        ax_accel = fig.add_subplot(2, 2, 2)
        ax_accel.set_title('Accelerometer (g)')
        ax_accel.set_ylabel('Acceleration (g)')
        ax_accel.grid(True)
        
        # Gyroscope plot
        ax_gyro = fig.add_subplot(2, 2, 3)
        ax_gyro.set_title('Gyroscope (°/s)')
        ax_gyro.set_ylabel('Angular velocity (°/s)')
        ax_gyro.grid(True)
        
        # Magnetometer plot
        ax_mag = fig.add_subplot(2, 2, 4)
        ax_mag.set_title('Magnetometer (µT)')
        ax_mag.set_ylabel('Magnetic field (µT)')
        ax_mag.set_xlabel('Time (s)')
        ax_mag.grid(True)
        
        def animate(frame):
            if not self.time_buffer:
                return
            
            times = list(self.time_buffer)
            
            # Clear plots
            ax_accel.clear()
            ax_gyro.clear()
            ax_mag.clear()
            ax_3d.clear()
            
            # Plot accelerometer data
            ax_accel.plot(times, list(self.accel_buffer['x']), 'r-', label='X', alpha=0.7)
            ax_accel.plot(times, list(self.accel_buffer['y']), 'g-', label='Y', alpha=0.7)
            ax_accel.plot(times, list(self.accel_buffer['z']), 'b-', label='Z', alpha=0.7)
            ax_accel.set_title('Accelerometer (g)')
            ax_accel.set_ylabel('Acceleration (g)')
            ax_accel.legend()
            ax_accel.grid(True)
            
            # Plot gyroscope data
            ax_gyro.plot(times, list(self.gyro_buffer['x']), 'r-', label='X', alpha=0.7)
            ax_gyro.plot(times, list(self.gyro_buffer['y']), 'g-', label='Y', alpha=0.7)
            ax_gyro.plot(times, list(self.gyro_buffer['z']), 'b-', label='Z', alpha=0.7)
            ax_gyro.set_title('Gyroscope (°/s)')
            ax_gyro.set_ylabel('Angular velocity (°/s)')
            ax_gyro.legend()
            ax_gyro.grid(True)
            
            # Plot magnetometer data
            ax_mag.plot(times, list(self.mag_buffer['x']), 'r-', label='X', alpha=0.7)
            ax_mag.plot(times, list(self.mag_buffer['y']), 'g-', label='Y', alpha=0.7)
            ax_mag.plot(times, list(self.mag_buffer['z']), 'b-', label='Z', alpha=0.7)
            ax_mag.set_title('Magnetometer (µT)')
            ax_mag.set_ylabel('Magnetic field (µT)')
            ax_mag.set_xlabel('Time (s)')
            ax_mag.legend()
            ax_mag.grid(True)
            
            # Draw coordinate axes
            origin = np.array([0, 0, 0])
            x_axis = np.array([0.4, 0, 0])
            y_axis = np.array([0, 0.4, 0])
            z_axis = np.array([0, 0, 0.4])
            
            ax_3d.quiver(origin[0], origin[1], origin[2], 
                        x_axis[0], x_axis[1], x_axis[2], 
                        color='red', alpha=0.9, linewidth=2, label='X')
            ax_3d.quiver(origin[0], origin[1], origin[2], 
                        y_axis[0], y_axis[1], y_axis[2], 
                        color='green', alpha=0.9, linewidth=2, label='Y')
            ax_3d.quiver(origin[0], origin[1], origin[2], 
                        z_axis[0], z_axis[1], z_axis[2], 
                        color='blue', alpha=0.9, linewidth=2, label='Z')
            
            ax_3d.set_xlabel('X')
            ax_3d.set_ylabel('Y')
            ax_3d.set_zlabel('Z')
            ax_3d.set_title('3D Device Orientation')
        
        # Start animation
        ani = animation.FuncAnimation(fig, animate, interval=50, blit=False)
        plt.tight_layout()
        plt.show()
        
        return ani
    
def main():
    parser = argparse.ArgumentParser(description='WearGo IMU Visualizer')
    parser.add_argument('--port', '-p', type=str, help='Serial port (auto-detect if not specified)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--buffer-size', type=int, default=100, help='Data buffer size (default: 100)')
    args = parser.parse_args()
    
    # Create visualizer
    visualizer = SingleIMUVisualizer(port=args.port, baudrate=args.baudrate, buffer_size=args.buffer_size)
    
    try:
        # Connect to Arduino
        if not visualizer.connect():
            print("Failed to connect to Arduino")
            return 1
        
        # Start reading data
        if not visualizer.start_reading():
            print("Failed to start data reading")
            return 1
        
        print("Starting visualization... Close the plot window to exit.")
        print("Waiting for data...")
        
        # Wait a moment for data to start flowing
        time.sleep(2)
        
        # Create and show visualization
        ani = visualizer.create_visualization()
        
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        visualizer.disconnect()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
