#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

enum : uint8_t {
  FT_TELEMETRY        = 0x01,   // C3 -> S3
  FT_SETTINGS_SYNC    = 0x02,   // C3 -> S3 (optional)
  FT_PING             = 0x03,
  FT_PONG             = 0x04,
  FT_TOUCH            = 0x81,   // S3 -> C3
  FT_SETTING_CHANGE   = 0x82,   // S3 -> C3
  FT_UI_EVENT         = 0x83
};

// NEW: Setting IDs for polar TE compensation
enum : uint8_t { 
  SET_VOLUME=0, 
  SET_BRIGHTNESS=1, 
  SET_UNITS=2,
  C3_SET_QNH_PA = 10,      // NEW: QNH setting
  C3_SET_POLAR = 11,       // NEW: Polar selection
  C3_TE_TOGGLE = 12        // NEW: TE compensation toggle
};

// UPDATED: Telemetry message with TE vario field
struct TelemetryMsg {
  float   vario_mps;       // Netto vario
  float   te_vario_mps;    // NEW: TE-compensated vario
  float   alt_m;
  float   asi_kts;
  float   track_deg;
  uint8_t fix;             // 0=none, 1=2D, 2=3D
  uint8_t sats;
  uint8_t rsv[2];          // Changed: padding adjusted for new field
};

enum : uint16_t { TOUCH_DOWN=0, TOUCH_MOVE=1, TOUCH_UP=2 };
struct TouchMsg {
  uint16_t type;
  uint16_t x;
  uint16_t y;
};

// UPDATED: SettingChangeMsg to support polar settings
struct SettingChangeMsg {
  uint8_t  id;
  int32_t  value;          // Can store polar index, TE toggle, etc.
};

class LinkProtocol {
public:
  // UART: pins set in begin()
  void begin(uint32_t baud, int8_t rxPin, int8_t txPin);

  // ---- C3 -> S3
  void sendTelemetry(const TelemetryMsg& m);
  bool pollTelemetry(TelemetryMsg& out);

  // ---- S3 -> C3  
  void sendTouch(uint16_t type, uint16_t x, uint16_t y);
  void sendSettingChange(uint8_t id, int32_t value);

  // Generic inbound polls
  bool pollTouch(TouchMsg& out);
  bool pollSettingChange(SettingChangeMsg& out);

  // NEW: Convenience methods for polar settings
  void sendPolarSelect(uint8_t polarIndex) {
    sendSettingChange(C3_SET_POLAR, polarIndex);
  }
  
  void sendTEToggle(bool enabled) {
    sendSettingChange(C3_TE_TOGGLE, enabled ? 1 : 0);
  }
  
  void sendQNH(uint32_t qnhPa) {
    sendSettingChange(C3_SET_QNH_PA, qnhPa);
  }

private:
  HardwareSerial* ser_ = nullptr;

  // SLIP
  static constexpr uint8_t FEND=0xC0, FESC=0xDB, TFEND=0xDC, TFESC=0xDD;
  static constexpr size_t RX_MAX = 256;
  uint8_t rxbuf_[RX_MAX]; size_t rxlen_ = 0; bool in_frame_ = false;

  // utils
  static uint16_t crc16(const uint8_t* d, size_t n, uint16_t crc=0xFFFF);
  void slipPush(uint8_t c);
  void sendFrame(uint8_t type, const uint8_t* payload, size_t len);
  bool tryParse(uint8_t*& p, size_t& n, uint8_t& type, const uint8_t*& pay, size_t& plen);

  bool pollFrame(uint8_t& type, const uint8_t*& pay, size_t& plen);
};