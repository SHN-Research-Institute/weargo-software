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
import numpy as np
import time
import threading
from collections import deque
import argparse
import sys


class IMUVisualizer:
    def __init__(self, port=None, baudrate=115200, buffer_size=100):
        self.port = port
        self.baudrate = baudrate
        self.buffer_size = buffer_size
        
        # Data buffers
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
        
        # Orientation estimation (simple complementary filter)
        self.orientation = {'roll': 0, 'pitch': 0, 'yaw': 0}
        
        # Serial connection
        self.serial_conn = None
        self.running = False
        self.data_thread = None
        
        # Start time for relative timestamps
        self.start_time = time.time()
        
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
        
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
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
        """Parse CSV line: ax,ay,az,gx,gy,gz,mx,my,mz"""
        try:
            values = [float(x.strip()) for x in line.split(',')]
            if len(values) == 9:
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
        self.orientation['yaw'] = gyro_yaw  # Yaw from gyro only (needs magnetometer for absolute)
    
    def read_data(self):
        """Read data from serial port in separate thread"""
        last_time = time.time()
        
        while self.running:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    
                    # Skip debug messages
                    if line.startswith('BMI270') or line.startswith('Failed') or not line:
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
        fig = plt.figure(figsize=(15, 10))
        fig.suptitle('WearGo IMU Real-time Visualization', fontsize=16)
        
        # 3D orientation visualization
        ax_3d = fig.add_subplot(2, 3, 1, projection='3d')
        ax_3d.set_title('3D Orientation')
        
        # Accelerometer plot
        ax_accel = fig.add_subplot(2, 3, 2)
        ax_accel.set_title('Accelerometer (g)')
        ax_accel.set_ylabel('Acceleration (g)')
        ax_accel.grid(True)
        
        # Gyroscope plot
        ax_gyro = fig.add_subplot(2, 3, 3)
        ax_gyro.set_title('Gyroscope (°/s)')
        ax_gyro.set_ylabel('Angular velocity (°/s)')
        ax_gyro.grid(True)
        
        # Magnetometer plot
        ax_mag = fig.add_subplot(2, 3, 4)
        ax_mag.set_title('Magnetometer (µT)')
        ax_mag.set_ylabel('Magnetic field (µT)')
        ax_mag.set_xlabel('Time (s)')
        ax_mag.grid(True)
        
        # Orientation angles
        ax_orient = fig.add_subplot(2, 3, 5)
        ax_orient.set_title('Orientation Angles')
        ax_orient.set_ylabel('Angle (°)')
        ax_orient.set_xlabel('Time (s)')
        ax_orient.grid(True)
        
        # Text display for current values
        ax_text = fig.add_subplot(2, 3, 6)
        ax_text.axis('off')
        ax_text.set_title('Current Values')
        
        def animate(frame):
            if not self.time_buffer:
                return
            
            times = list(self.time_buffer)
            
            # Clear plots
            ax_accel.clear()
            ax_gyro.clear()
            ax_mag.clear()
            ax_orient.clear()
            ax_3d.clear()
            ax_text.clear()
            
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
            
            # Plot orientation angles
            if len(times) > 1:
                orientation_times = times
                rolls = [self.orientation['roll']] * len(times)
                pitches = [self.orientation['pitch']] * len(times)
                yaws = [self.orientation['yaw']] * len(times)
                
                ax_orient.plot(orientation_times, rolls, 'r-', label='Roll', alpha=0.7)
                ax_orient.plot(orientation_times, pitches, 'g-', label='Pitch', alpha=0.7)
                ax_orient.plot(orientation_times, yaws, 'b-', label='Yaw', alpha=0.7)
            
            ax_orient.set_title('Orientation Angles')
            ax_orient.set_ylabel('Angle (°)')
            ax_orient.set_xlabel('Time (s)')
            ax_orient.legend()
            ax_orient.grid(True)
            
            # 3D orientation visualization
            ax_3d.set_xlim([-1, 1])
            ax_3d.set_ylim([-1, 1])
            ax_3d.set_zlim([-1, 1])
            
            # Draw coordinate frame
            roll = np.radians(self.orientation['roll'])
            pitch = np.radians(self.orientation['pitch'])
            yaw = np.radians(self.orientation['yaw'])
            
            # Rotation matrix
            Rx = np.array([[1, 0, 0],
                          [0, np.cos(roll), -np.sin(roll)],
                          [0, np.sin(roll), np.cos(roll)]])
            
            Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                          [0, 1, 0],
                          [-np.sin(pitch), 0, np.cos(pitch)]])
            
            Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                          [np.sin(yaw), np.cos(yaw), 0],
                          [0, 0, 1]])
            
            R = Rz @ Ry @ Rx
            
            # Draw axes
            origin = np.array([0, 0, 0])
            x_axis = R @ np.array([0.8, 0, 0])
            y_axis = R @ np.array([0, 0.8, 0])
            z_axis = R @ np.array([0, 0, 0.8])
            
            ax_3d.quiver(origin[0], origin[1], origin[2], 
                        x_axis[0], x_axis[1], x_axis[2], 
                        color='red', alpha=0.8, linewidth=3, label='X')
            ax_3d.quiver(origin[0], origin[1], origin[2], 
                        y_axis[0], y_axis[1], y_axis[2], 
                        color='green', alpha=0.8, linewidth=3, label='Y')
            ax_3d.quiver(origin[0], origin[1], origin[2], 
                        z_axis[0], z_axis[1], z_axis[2], 
                        color='blue', alpha=0.8, linewidth=3, label='Z')
            
            ax_3d.set_xlabel('X')
            ax_3d.set_ylabel('Y')
            ax_3d.set_zlabel('Z')
            ax_3d.set_title('3D Orientation')
            
            # Current values text
            if self.accel_buffer['x'] and self.gyro_buffer['x'] and self.mag_buffer['x']:
                text = f"""Current IMU Readings:
                
Accelerometer (g):
  X: {self.accel_buffer['x'][-1]:.3f}
  Y: {self.accel_buffer['y'][-1]:.3f}
  Z: {self.accel_buffer['z'][-1]:.3f}

Gyroscope (°/s):
  X: {self.gyro_buffer['x'][-1]:.3f}
  Y: {self.gyro_buffer['y'][-1]:.3f}
  Z: {self.gyro_buffer['z'][-1]:.3f}

Magnetometer (µT):
  X: {self.mag_buffer['x'][-1]:.3f}
  Y: {self.mag_buffer['y'][-1]:.3f}
  Z: {self.mag_buffer['z'][-1]:.3f}

Orientation:
  Roll:  {self.orientation['roll']:.1f}°
  Pitch: {self.orientation['pitch']:.1f}°
  Yaw:   {self.orientation['yaw']:.1f}°
"""
                ax_text.text(0.1, 0.9, text, transform=ax_text.transAxes, 
                           fontfamily='monospace', fontsize=10, verticalalignment='top')
            
            ax_text.set_title('Current Values')
            ax_text.axis('off')
        
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
    visualizer = IMUVisualizer(port=args.port, baudrate=args.baudrate, buffer_size=args.buffer_size)
    
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
