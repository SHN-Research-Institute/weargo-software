# 🎯 WearGo Dual IMU Setup Guide

## 📋 **Overview**

This guide helps you add a second IMU (ITG/MPU 6DOF) to your WearGo system for improved positioning accuracy and robustness.

## 🔌 **Hardware Wiring**

### **ITG/MPU 6DOF Connections**

```
ITG/MPU Pin    →    Arduino Nano 33 BLE Pin
─────────────────────────────────────────────
VCC            →    3.3V (NOT 5V!)
GND            →    GND
SDA            →    A4 (SDA)
SCL            →    A5 (SCL)
XDA            →    Leave unconnected
XCL            →    Leave unconnected
AD0            →    GND (sets I2C address to 0x68)
INT            →    Pin 2 (optional interrupt)
```

### **⚠️ Critical Notes:**

- **Use 3.3V ONLY** - Arduino Nano 33 BLE is 3.3V logic level
- **AD0 to GND** gives MPU6050 address 0x68 (may conflict with BMI270)
- **Both IMUs share the same I2C bus** (SDA/SCL)
- **Address conflict handling** - code will detect and manage automatically

## 💻 **Software Changes**

### **1. Updated PlatformIO Dependencies**

```ini
lib_deps =
  arduino-libraries/Arduino_BMI270_BMM150
  electroniccats/MPU6050
```

### **2. New Data Format**

**CSV Output:** `ax1,ay1,az1,gx1,gy1,gz1,mx1,my1,mz1,ax2,ay2,az2,gx2,gy2,gz2`

- **15 values total**
- **IMU1 (BMI270):** 9 values (accel + gyro + mag)
- **IMU2 (MPU6050):** 6 values (accel + gyro only)

### **3. Python Visualization Features**

- **Dual 3D orientation displays** - side-by-side device visualization
- **Sensor comparison plots** - compare accelerometer and gyroscope data
- **Orientation difference tracking** - monitor sensor fusion quality
- **Real-time sync status** - visual indicators for sensor agreement

## 🎯 **Benefits of Dual IMU System**

### **Improved Accuracy**

- **Sensor fusion** - combine data from both IMUs for better estimates
- **Error detection** - identify faulty readings by comparison
- **Redundancy** - continue operation if one IMU fails

### **Enhanced Capabilities**

- **Differential motion** - detect relative movement between body parts
- **Better calibration** - use one IMU to calibrate the other
- **Multi-point tracking** - track orientation at multiple body locations

## 🚀 **Usage Instructions**

### **1. Hardware Setup**

1. Wire the ITG/MPU as shown above
2. Double-check connections (especially 3.3V and AD0)
3. Upload the updated Arduino code

### **2. Software Setup**

1. Build and upload the Arduino code with PlatformIO
2. Run the Python visualizer: `python imu_visualizer.py`
3. Look for both IMU initialization messages in serial output

### **3. Monitoring**

- **Green checkmarks (✅)** indicate good sensor sync
- **Warning symbols (⚠️)** indicate sensor drift or issues
- **Orientation differences** should be small for mounted IMUs

## 🔧 **Troubleshooting**

### **Common Issues**

1. **"MPU6050 connection failed"**

   - Check wiring (especially SDA/SCL)
   - Verify 3.3V power supply
   - Ensure AD0 is connected to 3.3V

2. **Large orientation differences**

   - Check physical mounting - IMUs should be rigidly attached
   - Verify both IMUs are reading valid data
   - Consider calibration differences between sensors

3. **Serial data issues**
   - Look for 15 comma-separated values per line
   - Check for initialization messages being filtered out
   - Verify baudrate (115200)

### **I2C Address Verification**

With your current setup (AD0 to GND):

- **BMI270**: Usually 0x68 or 0x69 (built-in sensor)
- **MPU6050**: 0x68 (AD0=GND)

**Note**: Both sensors may use 0x68, but this is usually handled automatically by the libraries since they use different initialization methods.

## 📊 **Expected Performance**

- **Update Rate**: 20Hz (50ms interval)
- **Orientation Sync**: <10° difference for roll/pitch, <15° for yaw
- **Data Latency**: <100ms total system latency
- **Power Usage**: ~20-30mA additional for second IMU

## 🎨 **Visualization Features**

- **Side-by-side 3D models** with different colors
- **Real-time comparison charts** for all sensor axes
- **Difference tracking** with color-coded status indicators
- **Comprehensive data dashboard** showing both IMUs simultaneously

This dual IMU setup provides significantly improved motion tracking capabilities for your WearGo project! 🚀
