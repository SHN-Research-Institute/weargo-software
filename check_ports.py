#!/usr/bin/env python3
"""
Port checker tool to diagnose COM port issues
"""

import serial
import serial.tools.list_ports
import time

def check_port_availability(port):
    """Check if a port can be opened"""
    try:
        with serial.Serial(port, 115200, timeout=1) as ser:
            print(f"✓ {port} is available and can be opened")
            return True
    except serial.SerialException as e:
        print(f"✗ {port} error: {e}")
        return False

def list_all_ports():
    """List all available serial ports"""
    print("All Serial Ports:")
    print("-" * 50)
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("No serial ports found")
        return []
    
    available_ports = []
    for port in ports:
        print(f"\nPort: {port.device}")
        print(f"  Description: {port.description}")
        print(f"  Hardware ID: {port.hwid}")
        
        # Check if we can open it
        if check_port_availability(port.device):
            available_ports.append(port.device)
            
    return available_ports

def main():
    print("WearGo COM Port Checker")
    print("=" * 50)
    
    # List all ports and check availability
    available = list_all_ports()
    
    print(f"\n📊 Summary:")
    print(f"Available ports: {available}")
    
    if "COM6" in available:
        print("✅ COM6 is ready to use!")
    else:
        print("❌ COM6 has issues. Try:")
        print("   1. Close Arduino IDE/PlatformIO Serial Monitor")
        print("   2. Disconnect and reconnect USB")
        print("   3. Try a different USB port")
        print("   4. Restart VS Code")

if __name__ == "__main__":
    main()
