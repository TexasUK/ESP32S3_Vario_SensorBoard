# ESP32C3LinkBoard - Flight Sensor & Telemetry System

A sophisticated flight sensor and telemetry system built for the ESP32-S3 Mini, designed to provide real-time flight data to a companion display unit. This system serves as the **sensor/telemetry source** in a distributed flight instrument architecture.

## 🚁 Project Overview

The ESP32C3LinkBoard is a **telemetry sensor unit** that collects flight data from multiple sources and transmits it to a display unit via UART. It features advanced sensor fusion, flight mode detection, IGC logging, and BLE configuration capabilities.

### Key Features

- **Multi-Sensor Fusion**: BMP581 barometric pressure, GPS, and calculated airspeed
- **Advanced Flight Modes**: Automatic detection of thermal, cruise, climb, and descent phases
- **IGC Flight Logging**: Automatic flight logging to SD card with IGC standard format
- **BLE Configuration**: Wireless setup via Bluetooth Low Energy
- **Polar Curve Management**: Configurable glider performance curves with TE compensation
- **Real-time Telemetry**: 25Hz data transmission to display unit
- **Wind Calculation**: Basic wind vector estimation
- **Flight Detection**: Automatic start/stop of logging based on airspeed

## 🛠️ Hardware Requirements

### Primary Hardware
- **ESP32-S3 Mini** (LOLIN S3 Mini or compatible)
- **BMP581 Barometric Pressure Sensor** (I2C)
- **GPS Module** (UART, 9600 baud)
- **SD Card Module** (SPI, for IGC logging)
- **Display Unit** (connected via UART for telemetry)

### Pin Configuration
```
I2C (BMP581):
- SDA: GPIO8
- SCL: GPIO9

GPS (UART2):
- RX: GPIO4
- TX: GPIO5

Display Link (UART1):
- RX: GPIO44 (from display TX=43)
- TX: GPIO43 (to display RX=44)

SD Card (SPI):
- SCK: GPIO6
- MOSI: GPIO7
- MISO: GPIO10
- CS: GPIO11
```

## 📋 Software Architecture

### Core Components

1. **Main Application** (`main.cpp`)
   - Sensor data collection and fusion
   - Flight mode detection
   - Telemetry transmission
   - IGC logging management

2. **Sensor Integration**
   - **BMP581**: Barometric pressure and altitude
   - **GPS**: Position, speed, and course
   - **Kalman Filter**: Advanced altitude and variometer processing

3. **Communication Systems**
   - **CSV Serial Protocol**: UART communication with display
   - **BLE Configuration**: Wireless setup and monitoring
   - **IGC Logging**: Standard flight data recording

4. **Flight Analysis**
   - **Polar Curves**: Glider performance modeling
   - **TE Compensation**: Total Energy variometer calculation
   - **Airspeed Calculation**: Multi-mode airspeed estimation
   - **Wind Vector**: Basic wind speed and direction

### Telemetry Data Structure
```cpp
struct CsvTlm {
  float netto = 0;      // Netto variometer (m/s)
  float te = 0;         // Total Energy variometer (m/s)
  float alt_m = 0;      // Altitude (meters)
  float asi_kts = 0;    // Airspeed (knots)
  int   fix = 0;        // GPS fix quality
  int   sats = 0;       // GPS satellites
  int   mode = 0;       // Flight mode
  bool  fresh = false;  // Data freshness flag
};
```

## 🎯 Flight Modes & Detection

### Automatic Flight Mode Detection
- **CRUISE**: Normal flight - uses polar-based airspeed calculation
- **THERMAL**: Circling in thermal - uses GPS groundspeed
- **CLIMB**: Strong climb - uses GPS groundspeed with wind compensation
- **DESCENT**: Descent phase - uses polar-based airspeed

### Flight Mode Criteria
```cpp
// Thermal: High vario + turning
if (vario > 2.0f && abs(turnRate) > 0.1f) return THERMAL;

// Climb: Moderate vario
if (vario > 1.0f) return CLIMB;

// Descent: Negative vario
if (vario < -1.0f) return DESCENT;

// Default: Cruise
return CRUISE;
```

## ⚙️ Configuration

### Build Configuration (`platformio.ini`)
```ini
[env:lolin_s3_mini]
platform = espressif32@6.9.0
board = lolin_s3_mini
framework = arduino
monitor_speed = 115200

build_flags =
  -D ARDUINO_USB_MODE=1
  -D ARDUINO_USB_CDC_ON_BOOT=1
  -D CONFIG_BT_NIMBLE_MAX_CONNECTIONS=3
  -D CORE_DEBUG_LEVEL=0
```

### Key Dependencies
- **Adafruit BMP5xx Library**: Barometric pressure sensor
- **TinyGPSPlus**: GPS parsing and processing
- **SdFat**: SD card file system
- **NimBLE-Arduino**: Bluetooth Low Energy

### Configuration Options
```cpp
#define STARTUP_QNH_MODE   1    // 0=fixed, 1=auto-zero, 2=GPS-alt
#define SPEED_KTS_START    20.0f // Flight detection threshold
#define SPEED_KTS_STOP     20.0f // Flight end threshold
#define START_DEBOUNCE_MS  3000  // Flight start debounce
#define STOP_DEBOUNCE_MS   10000 // Flight end debounce
```

## 🔧 Advanced Features

### Kalman Filter Processing
- **Altitude**: Smooth altitude from barometric pressure
- **Variometer**: Derivative-based vertical speed calculation
- **Bias Learning**: Automatic drift compensation during stationary periods

### Polar Curve Management
Pre-configured glider polars:
- **LS8-b**: Standard training glider
- **DG-800**: High-performance glider
- **ASG-29**: Competition glider
- **Discus**: Classic glider

### TE Compensation
Total Energy compensation accounts for glider sink rate:
```cpp
float compensate(float netto, float asi) {
  return netto + gliderSinkRate(asi);
}
```

### IGC Flight Logging
Automatic logging with IGC standard format:
- **Flight Detection**: Based on airspeed threshold
- **Standard Format**: Compatible with flight analysis software
- **Metadata**: Pilot, glider, and configuration information

## 📊 Performance Characteristics

### Sensor Performance
- **Barometric Update Rate**: ~25Hz
- **GPS Update Rate**: ~1Hz
- **Telemetry Transmission**: 25Hz
- **Altitude Accuracy**: ±1m (with proper QNH)
- **Variometer Response**: <100ms

### Flight Detection
- **Start Threshold**: 20 kts airspeed
- **Stop Threshold**: 20 kts airspeed
- **Debounce Time**: 3s start, 10s stop
- **Logging**: Automatic IGC file creation

### BLE Configuration
- **Service UUID**: `f0a00001-6d75-4d1a-a2a9-d5a9e0a1c001`
- **Device Name**: "FlightCore"
- **Range**: ~10m typical
- **Configuration**: Real-time parameter updates

## 🚀 Getting Started

### 1. Hardware Setup
1. Connect BMP581 to I2C (SDA=8, SCL=9)
2. Connect GPS to UART2 (RX=4, TX=5)
3. Connect SD card to SPI (SCK=6, MOSI=7, MISO=10, CS=11)
4. Connect display unit to UART1 (RX=44, TX=43)

### 2. Software Setup
1. Install PlatformIO
2. Clone this repository
3. Install dependencies: `pio lib install`
4. Configure pins if needed
5. Build and upload: `pio run -t upload`

### 3. Initial Configuration
1. Power on the system
2. Connect via BLE to "FlightCore"
3. Configure pilot information, glider type, and QNH
4. Select appropriate polar curve
5. Verify sensor readings in serial monitor

## 📱 BLE Configuration Interface

### Available Characteristics
- **Pilot Name**: Text configuration
- **Glider Type**: Text configuration
- **Glider ID**: Text configuration
- **Competition ID**: Text configuration
- **QNH Pressure**: 32-bit integer (Pa)
- **Polar Selection**: 8-bit index
- **TE Toggle**: Boolean enable/disable
- **Volume**: 0-10 scale
- **Brightness**: 0-10 scale
- **Flight State**: Read-only status

### Configuration Workflow
1. Connect to "FlightCore" BLE device
2. Read current configuration values
3. Update parameters as needed
4. Changes are automatically saved to NVS
5. Display unit receives updated settings

## 📈 Telemetry Protocol

### CSV Format
```
T,netto,te,alt_m,asi_kts,fix,sats,mode
```
Where:
- `netto`: Netto variometer (m/s)
- `te`: Total Energy variometer (m/s)
- `alt_m`: Altitude in meters
- `asi_kts`: Airspeed in knots
- `fix`: GPS fix quality (0-2)
- `sats`: GPS satellite count
- `mode`: Flight mode (0-3)

### Communication Flow
1. **Display → Sensor**: Settings commands (`SET,TE,1`)
2. **Sensor → Display**: Telemetry data (`T,1.2,1.5,1200.0,45.0,2,8,1`)
3. **Handshake**: Ping/Pong for connection monitoring
4. **Acknowledgments**: Confirmation of setting changes

## 🔍 Troubleshooting

### Common Issues

1. **No GPS Fix**
   - Check GPS module connections
   - Verify baud rate (9600)
   - Ensure clear sky view
   - Check antenna connection

2. **Barometric Errors**
   - Verify BMP581 I2C connections
   - Check QNH setting
   - Calibrate at known altitude
   - Ensure sensor is not blocked

3. **No Telemetry to Display**
   - Check UART connections (RX=44, TX=43)
   - Verify baud rate (115200)
   - Check display unit power
   - Monitor serial output for errors

4. **SD Card Issues**
   - Verify SPI connections
   - Check card format (FAT32)
   - Ensure card is not write-protected
   - Check power supply

### Debug Output
Enable debug logging by setting:
```cpp
#define CORE_DEBUG_LEVEL=5
```

Monitor serial output at 115200 baud for diagnostic information.

## 📝 IGC Logging Details

### File Format
- **Filename**: `YYYYMMDD_HHMMSS.igc`
- **Header**: Pilot, glider, and configuration info
- **B Records**: Position, altitude, and variometer data
- **Frequency**: 1Hz during flight

### IGC Record Example
```
B1430525212345N00234567E012345678
```
Where:
- `B`: Record type
- `143052`: Time (14:30:52)
- `5212345N`: Latitude (52°12.345'N)
- `00234567E`: Longitude (2°34.567'E)
- `012`: Pressure altitude (120m)
- `345`: GPS altitude (340m)
- `67`: Variometer (1.7 m/s)

## 🤝 Contributing

Contributions are welcome! Please ensure:
- Code follows the existing style
- New features include appropriate documentation
- Hardware compatibility is maintained
- Performance impact is considered

## 📞 Support

For technical support or questions:
- Check the troubleshooting section
- Review the hardware connections
- Verify software configuration
- Monitor serial debug output

---

**Note**: This is a telemetry sensor system. It requires a compatible display unit that can receive and process the CSV telemetry data transmitted via UART.
