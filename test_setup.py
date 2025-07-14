#!/usr/bin/env python3
"""
Test script to check if the IMU visualizer can import all dependencies
"""

def test_imports():
    """Test all required imports"""
    try:
        import serial
        print("✓ pyserial imported successfully")
        
        import matplotlib.pyplot as plt
        print("✓ matplotlib imported successfully")
        
        import numpy as np
        print("✓ numpy imported successfully")
        
        import scipy
        print("✓ scipy imported successfully")
        
        import pygame
        print("✓ pygame imported successfully")
        
        import serial.tools.list_ports
        print("✓ serial.tools.list_ports imported successfully")
        
        from mpl_toolkits.mplot3d import Axes3D
        print("✓ mplot3d imported successfully")
        
        import matplotlib.animation as animation
        print("✓ matplotlib.animation imported successfully")
        
        return True
        
    except ImportError as e:
        print(f"✗ Import error: {e}")
        return False

def list_available_ports():
    """List available serial ports"""
    import serial.tools.list_ports
    
    print("\nAvailable Serial Ports:")
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("  No serial ports found")
        return
    
    for port in ports:
        print(f"  {port.device}: {port.description}")
        if any(identifier in port.description.lower() for identifier in 
               ['arduino', 'ch340', 'cp210', 'ftdi', 'usb serial']):
            print(f"    ^ Potential Arduino device")

if __name__ == "__main__":
    print("WearGo IMU Visualizer - Dependency Test")
    print("=" * 50)
    
    if test_imports():
        print("\n✓ All dependencies imported successfully!")
        list_available_ports()
        print("\nSetup complete! You can now run: python imu_visualizer.py")
    else:
        print("\n✗ Some dependencies failed to import. Please check the installation.")
