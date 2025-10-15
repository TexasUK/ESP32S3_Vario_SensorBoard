# FlightCore - ESP32C3LinkBoard Variometer System

## Overview

FlightCore is an advanced variometer system designed for gliding applications. It combines multiple sensors to provide accurate flight data, automatic flight logging, and intelligent airspeed calculation using glider polar curves and wind drift compensation.

## Key Features

### 🛩️ **Intelligent Flight Detection**
- **Automatic flight detection** when airspeed exceeds 20 kts for 3 seconds
- **Debounced logic** prevents false triggers from brief speed spikes
- **Multi-mode airspeed calculation** adapts to different flight phases

### 📊 **Advanced Airspeed Calculation**
- **Hybrid algorithm** combines GPS groundspeed with glider polar curves
- **Wind drift compensation** calculates airspeed from groundspeed and bearing
- **Flight mode detection** (Cruise, Thermal, Climb, Descent) optimizes calculations
- **Thermal optimization** uses GPS groundspeed when circling (wind cancels out)

### 📝 **Automatic Flight Logging**
- **IGC file generation** with standard format for flight analysis software
- **Automatic start/stop** based on flight detection
- **Comprehensive headers** including pilot, glider, and competition information
- **GPS track logging** with pressure altitude and vario data

### 🔧 **Configuration & Control**
- **BLE interface** for wireless configuration from mobile devices
- **Persistent settings** stored in NVS memory
- **Real-time telemetry** via CSV protocol to display units
- **SD card polar files** - Load custom glider polar curves from .plr files
- **Automatic polar scanning** - System scans /data folder for .plr files on startup

### 🔄 **Over-The-Air (OTA) Updates**
- **BLE-triggered WiFi OTA** for wireless firmware updates
- **Web-based upload interface** with improved mobile-friendly design
- **Automatic WiFi access point** creation for update mode
- **Robust upload validation** prevents system crashes
- **5-minute timeout** for security and power management

## System Behavior

### **Startup Sequence**
1. **Sensor initialization** - BMP581 barometer, GPS, SD card
2. **QNH calibration** - Automatic pressure calibration for accurate altitude
3. **Configuration loading** - Restore pilot and glider settings
4. **BLE advertising** - Start "FlightCore" service for mobile configuration
5. **Boot logging** - Write system status to SD card

### **Flight Detection Logic**
- **Ground phase**: No flight detection, system monitors sensors
- **Takeoff detection**: When calculated airspeed exceeds 20 kts for 3 seconds
- **Flight phase**: IGC logging active, continuous telemetry transmission
- **Landing detection**: When airspeed drops below 20 kts for 10 seconds

### **Airspeed Calculation Modes**

#### **Cruise Mode**
- Uses glider polar curve to estimate airspeed from vario readings
- Applies wind drift compensation based on GPS bearing
- High confidence (80%) in calculations

#### **Thermal Mode**
- Detected when vario > 2.0 m/s and turning rate > 0.1 rad/s
- Uses GPS groundspeed (wind effects cancel out in circles)
- Maintains rolling average of recent airspeed values
- Medium confidence (60%) during active turning

#### **Climb Mode**
- Detected when vario > 1.0 m/s
- Uses GPS groundspeed (less wind effect in climbs)
- Medium confidence (70%)

#### **Descent Mode**
- Detected when vario < -1.0 m/s
- Uses polar curve calculation for accurate descent airspeed
- High confidence (80%)

## Performance Expectations

### **Accuracy**
- **Altitude**: ±1 meter (barometric with Kalman filtering)
- **Vario**: ±0.1 m/s (with thermal compensation)
- **Airspeed**: ±2 kts (improves with flight duration as wind data accumulates)
- **GPS**: Standard GPS accuracy (±3-5 meters)

### **Response Times**
- **Flight detection**: 3 seconds after exceeding 20 kts
- **Telemetry updates**: 25 Hz (40ms intervals)
- **IGC logging**: 1 Hz (1 second intervals)
- **BLE configuration**: Real-time updates

### **Reliability**
- **Automatic recovery** from sensor failures
- **Robust flight detection** with debounced logic
- **Persistent configuration** survives power cycles
- **Error handling** for SD card and GPS issues

## Data Outputs

### **IGC Flight Logs**
- **Standard format** compatible with flight analysis software
- **Automatic filename** with date/time stamp
- **Complete headers** with pilot, glider, and competition data
- **GPS track data** with pressure altitude and vario

### **CSV Telemetry**
- **Real-time data** to display units
- **Format**: `T,<netto>,<te>,<alt_m>,<asi_kts>,<fix>,<sats>,<mode>`
- **Flight mode** included for display unit processing
- **25 Hz update rate** for smooth display

### **BLE Configuration**
- **Wireless setup** from mobile devices
- **Real-time updates** of pilot, glider, and competition data
- **Polar curve selection** from available .plr files and built-in defaults
- **Thermal compensation settings** and audio/display preferences

### **Polar File System**
- **SD card storage** - Place .plr files in `/data` folder on SD card
- **WinPilot format** - Compatible with standard .plr polar files
- **Automatic scanning** - System loads all .plr files on startup
- **Built-in defaults** - Includes LS8, DG-800, ASG-29, and Discus polars
- **Single polar selection** - One polar active at a time
- **Startup sequence** - QNH entry → Polar selection → Normal operation
- **Display integration** - Touch screen vario can select polars via dropdown

### **OTA Update System**
- **BLE command interface** - Write "START" to OTA characteristic to begin update mode
- **WiFi access point** - "FlightCore-OTA" network (password: flightcore123)
- **Web interface** - http://192.168.4.1 for firmware upload
- **Automatic shutdown** - Write "STOP" to OTA characteristic or wait 5 minutes
- **Upload validation** - Only accepts .bin files, prevents crashes from invalid uploads

## System Requirements

### **Hardware**
- ESP32-S3 Mini microcontroller (4MB flash, 2MB PSRAM)
- BMP581 barometric pressure sensor
- GPS module (UART2)
- SD card for flight logging
- BLE for configuration and OTA triggering
- WiFi for OTA updates

### **Power**
- 3.3V operation
- Low power consumption during flight
- Automatic power management

### **Environmental**
- Operating temperature: -10°C to +60°C
- Altitude range: 0 to 15,000 meters
- Vario range: -10 to +10 kt/s

## Getting Started

1. **Power on** the system
2. **Wait for GPS fix** (1-5 minutes first time)
3. **Configure via BLE** using mobile app
4. **Set glider polar** and pilot information
5. **Ready for flight** - automatic detection and logging

## OTA Firmware Updates

### **Starting OTA Mode**
1. **Connect to BLE** using nRF Connect or Serial Bluetooth
2. **Find the OTA characteristic** (UUID: f0a0000d-6d75-4d1a-a2a9-d5a9e0a1c001)
3. **Write "START"** to the characteristic
4. **Look for WiFi network** "FlightCore-OTA" (password: flightcore123)
5. **Connect to WiFi** and open http://192.168.4.1
6. **Upload firmware.bin** file using the web interface

### **Stopping OTA Mode**
- **Write "STOP"** to the OTA characteristic, or
- **Wait 5 minutes** for automatic timeout, or
- **Power cycle** the device

### **Troubleshooting OTA**
- **No WiFi network**: Check BLE connection and try writing "START" again
- **Upload fails**: Ensure you're uploading a valid .bin file
- **System crashes**: The new validation prevents crashes from invalid uploads

## Polar File Management

### **Creating Polar Files**
1. **Create `/data` folder** on SD card (if it doesn't exist)
2. **Add .plr files** using WinPilot format:
   ```
   *GliderName WinPilot POLAR file (w. LK8000 extension) : MassDryGross[kg], MaxWaterBallast[liters], Speed1[km/h], Sink1[m/s], Speed2, Sink2, Speed3, Sink3, WingArea[m2]
   380,  0,  78.05,  -0.79, 124.03,  -1.97, 170.00,  -4.54,  17.50
   ```
3. **File naming**: Use descriptive names like `ASK13.plr`, `LS8.plr`, etc.
4. **Restart system** to scan new polar files

### **Polar File Format**
- **Header line**: Starts with `*` and describes the format
- **Data line**: Comma-separated values with speeds in km/h and sink rates in m/s
- **Automatic conversion**: System converts km/h to knots and handles units
- **Multiple points**: Supports up to 12 speed/sink rate pairs

### **Startup Sequence**
1. **QNH Entry**: Display shows numeric keyboard for 4-digit QNH entry (950-1050 hPa)
2. **Polar Selection**: Display shows dropdown list of available polars
3. **Normal Operation**: Vario screen appears and normal functionality begins

### **Dynamic Polar Integration**
- **Automatic data exchange**: Display requests polar data on connection
- **Real-time polar list**: Sensor sends comma-separated polar names
- **Performance data**: Each polar's speed/sink rate data sent separately
- **CSV protocol**: `GET_POLARS`, `POLARS,name1,name2,...`, `POLAR_DATA,index,speed1,sink1,...`

### **Selecting Polars**
- **Startup sequence**: Choose polar during initial setup
- **Display unit**: Send CSV command `SET,POLAR,<index>` to select polar
- **BLE app**: Read Polar List characteristic to see available polars
- **Automatic loading**: Selected polar is used immediately for airspeed calculations

## Support

For technical documentation, build instructions, wiring diagrams, and component specifications, see the [Technical Documentation](docs/technical.md) page.

---

*FlightCore - Intelligent variometer system for modern gliding*