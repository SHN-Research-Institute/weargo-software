# -*- coding: utf-8 -*-
import serial
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import serial.tools.list_ports
import numpy as np
import math

class AdvancedSensorVisualizer:
    def __init__(self, baud_rate=115200, max_points=100):
        self.baud_rate = baud_rate
        self.max_points = max_points
        
        # Multiple sensor support on single Arduino
        self.sensors = {
            'sensor1': self._initialize_sensor_data(),
            'sensor2': self._initialize_sensor_data()
        }
        
        # 3D visualization setup
        self.fig = plt.figure(figsize=(15, 10))
        
        # Subplots for different views
        self.ax_3d = self.fig.add_subplot(221, projection='3d')
        self.ax_relative = self.fig.add_subplot(222, projection='3d')
        self.ax_accel = self.fig.add_subplot(223)
        self.ax_orientation = self.fig.add_subplot(224)
        
        # Relative movement calculation
        self.relative_positions = {
            'sensor1': [0, 0, 0],
            'sensor2': [0, 0, 0]
        }
        
        self.port = self._find_arduino_port()
        if not self.port:
            raise RuntimeError("No Arduino device found!")
        
        # Open serial connection
        self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)

    def _initialize_sensor_data(self):
        return {
            'time_data': [],
            'accel_x': [], 'accel_y': [], 'accel_z': [],
            'roll': [], 'pitch': [], 'yaw': [],
            'quaternion': []
        }

    def _find_arduino_port(self):
        ports = list(serial.tools.list_ports.comports())
        arduino_ports = [
            port.device for port in ports 
            if "Arduino" in port.description or "USB Serial" in port.description
        ]
        return arduino_ports[0] if arduino_ports else None

    def _read_sensor_data(self):
        try:
            # Check serial connection status
            if not self.ser.is_open:
                print("Serial port is closed. Reopening...")
                self.ser.open()

            # Wait and read data
            if self.ser.in_waiting > 0:
                raw_data = self.ser.readline()
                data = raw_data.decode("utf-8", errors="replace").strip()
                
                print(f"Raw received data: {data}")  # Debug print
                
                try:
                    parsed_data = json.loads(data)
                    
                    # Check for error messages
                    if isinstance(parsed_data, dict) and parsed_data.get('error'):
                        print(f"Sensor Error: {parsed_data['error']}")
                        return None
                    
                    # Validate sensor data structure
                    required_keys = [
                        'sensor1_accel_x', 'sensor1_accel_y', 'sensor1_accel_z',
                        'sensor1_roll', 'sensor1_pitch', 'sensor1_yaw',
                        'sensor2_accel_x', 'sensor2_accel_y', 'sensor2_accel_z',
                        'sensor2_roll', 'sensor2_pitch', 'sensor2_yaw'
                    ]
                    
                    if not all(key in parsed_data for key in required_keys):
                        print("Incomplete sensor data: Missing required keys")
                        return None
                    
                    return parsed_data
                
                except json.JSONDecodeError:
                    print(f"Invalid JSON: {data}")
                    return None
                except Exception as json_error:
                    print(f"JSON parsing error: {json_error}")
                    return None
            else:
                # No data available
                return None
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            try:
                self.ser.close()
                self.port = self._find_arduino_port()
                self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            except Exception as reconnect_error:
                print(f"Failed to reconnect: {reconnect_error}")
            return None
        except Exception as e:
            print(f"Unexpected error reading sensor data: {e}")
            return None

    def _calculate_relative_position(self, accel, prev_pos):
        # Simple integration of acceleration to estimate position
        dt = 0.05  # Matching FuncAnimation interval
        new_pos = [
            prev_pos[i] + 0.5 * accel[i] * (dt ** 2)
            for i in range(3)
        ]
        return new_pos

    def update_plot(self, frame):
        # Clear previous plots
        for ax in [self.ax_3d, self.ax_relative, self.ax_accel, self.ax_orientation]:
            ax.clear()

        # Collect data from both sensors
        data = self._read_sensor_data()
        if not data:
            return  # Skip update if no data

        # Process data for each sensor
        for sensor_key in self.sensors.keys():
            sensor = self.sensors[sensor_key]
            
            # Update time data
            sensor['time_data'].append(frame)
            
            # Accelerometer data
            accel_data = [
                data.get(f"{sensor_key}_accel_{axis}", 0) 
                for axis in ['x', 'y', 'z']
            ]
            sensor['accel_x'].append(accel_data[0])
            sensor['accel_y'].append(accel_data[1])
            sensor['accel_z'].append(accel_data[2])
            
            # Orientation data
            sensor['roll'].append(data.get(f'{sensor_key}_roll', 0))
            sensor['pitch'].append(data.get(f'{sensor_key}_pitch', 0))
            sensor['yaw'].append(data.get(f'{sensor_key}_yaw', 0))

            # Trim data
            for key in ['time_data', 'accel_x', 'accel_y', 'accel_z', 'roll', 'pitch', 'yaw']:
                if len(sensor[key]) > self.max_points:
                    sensor[key] = sensor[key][-self.max_points:]

            # Calculate relative position
            self.relative_positions[sensor_key] = self._calculate_relative_position(
                accel_data, 
                self.relative_positions[sensor_key]
            )

        # Visualizations
        self._plot_3d_orientation(data)
        self._plot_relative_movement()
        self._plot_accelerometer_data()
        self._plot_orientation_data()
        
        plt.tight_layout()

    def _plot_3d_orientation(self, sensor_data):
        # 3D visualization of sensor orientations
        self.ax_3d.set_title("3D Sensor Orientations")
        for sensor_key in self.sensors.keys():
            roll = sensor_data.get(f'{sensor_key}_roll', 0)
            pitch = sensor_data.get(f'{sensor_key}_pitch', 0)
            yaw = sensor_data.get(f'{sensor_key}_yaw', 0)
            
            # Convert to radians
            roll, pitch, yaw = map(math.radians, [roll, pitch, yaw])
            
            # Create rotation matrix
            Rx = np.array([
                [1, 0, 0],
                [0, math.cos(roll), -math.sin(roll)],
                [0, math.sin(roll), math.cos(roll)]
            ])
            
            Ry = np.array([
                [math.cos(pitch), 0, math.sin(pitch)],
                [0, 1, 0],
                [-math.sin(pitch), 0, math.cos(pitch)]
            ])
            
            Rz = np.array([
                [math.cos(yaw), -math.sin(yaw), 0],
                [math.sin(yaw), math.cos(yaw), 0],
                [0, 0, 1]
            ])
            
            # Combined rotation
            R = Rz @ Ry @ Rx
            
            # Plot orientation
            origin = [0, 0, 0]
            axes_length = 1
            colors = ['r', 'g', 'b']
            
            for i, color in enumerate(colors):
                axis = R[:, i] * axes_length
                self.ax_3d.quiver(*origin, *axis, color=color, label=f'{sensor_key} {["X", "Y", "Z"][i]}')
        
        self.ax_3d.set_xlim([-1, 1])
        self.ax_3d.set_ylim([-1, 1])
        self.ax_3d.set_zlim([-1, 1])
        self.ax_3d.legend()

    def _plot_relative_movement(self):
        # Relative movement between sensors
        self.ax_relative.set_title("Relative Sensor Movement")
        positions = list(self.relative_positions.values())
        
        for i, (sensor_key, pos) in enumerate(self.relative_positions.items()):
            self.ax_relative.scatter(*pos, label=sensor_key, 
                                     color=['r', 'b'][i], 
                                     s=100)
        
        # Calculate and visualize distance between sensors
        if len(positions) > 1:
            distance = np.linalg.norm(np.array(positions[0]) - np.array(positions[1]))
            self.ax_relative.text(0, 0, 0, f'Distance: {distance:.2f}', 
                                  horizontalalignment='center')
        
        self.ax_relative.set_xlabel('X')
        self.ax_relative.set_ylabel('Y')
        self.ax_relative.set_zlabel('Z')
        self.ax_relative.legend()

    def _plot_accelerometer_data(self):
        # Accelerometer data for both sensors
        self.ax_accel.set_title("Accelerometer Data")
        for sensor_key, sensor in self.sensors.items():
            self.ax_accel.plot(sensor['time_data'], sensor['accel_x'], 
                                label=f'{sensor_key} Accel X')
            self.ax_accel.plot(sensor['time_data'], sensor['accel_y'], 
                                label=f'{sensor_key} Accel Y')
            self.ax_accel.plot(sensor['time_data'], sensor['accel_z'], 
                                label=f'{sensor_key} Accel Z')
        
        self.ax_accel.set_xlabel('Time')
        self.ax_accel.set_ylabel('Acceleration')
        self.ax_accel.legend()

    def _plot_orientation_data(self):
        # Orientation data for both sensors
        self.ax_orientation.set_title("Orientation Data")
        for sensor_key, sensor in self.sensors.items():
            self.ax_orientation.plot(sensor['time_data'], sensor['roll'], 
                                     label=f'{sensor_key} Roll')
            self.ax_orientation.plot(sensor['time_data'], sensor['pitch'], 
                                     label=f'{sensor_key} Pitch')
            self.ax_orientation.plot(sensor['time_data'], sensor['yaw'], 
                                     label=f'{sensor_key} Yaw')
        
        self.ax_orientation.set_xlabel('Time')
        self.ax_orientation.set_ylabel('Angle (degrees)')
        self.ax_orientation.legend()

    def start_visualization(self):
        # Create animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=50)
        plt.show()

def main():
    try:
        visualizer = AdvancedSensorVisualizer()
        visualizer.start_visualization()
    except Exception as e:
        print(f"Error initializing visualization: {e}")

if __name__ == "__main__":
    main()