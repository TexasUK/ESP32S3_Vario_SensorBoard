#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <string.h>

// ===================================================================================
// Telemetry ABI shim: accepts legacy (v1: 17B) and current (v2: 24B) payloads
// ===================================================================================

#define LINKPROTO_TELEMETRY_ABI 2

struct TelemetryMsg {
  float   vario_mps;     // filtered/netto (or selected stream)
  float   te_vario_mps;  // TE-compensated vario
  float   alt_m;         // altitude (m)
  float   asi_kts;       // airspeed (kt)
  float   track_deg;     // track (deg), may be NaN
  uint8_t fix;           // 0=no, 1=2D, 2=3D
  uint8_t sats;          // satellites
  uint8_t rsv[2];        // pad to 24 bytes
} __attribute__((packed));
static_assert(sizeof(TelemetryMsg) == 24, "TelemetryMsg must be 24 bytes");

struct TelemetryMsgV1 {
  float   vario_mps;
  float   alt_m;
  float   asi_kts;
  float   track_deg;
  uint8_t fix;
} __attribute__((packed));
static_assert(sizeof(TelemetryMsgV1) == 17, "TelemetryMsgV1 must be 17 bytes");

inline void upconvertTelemetryV1(const TelemetryMsgV1& in, TelemetryMsg& out) {
  out.vario_mps     = in.vario_mps;
  out.te_vario_mps  = in.vario_mps; // sensible default for legacy
  out.alt_m         = in.alt_m;
  out.asi_kts       = in.asi_kts;
  out.track_deg     = in.track_deg;
  out.fix           = in.fix;
  out.sats          = 0;
  out.rsv[0] = out.rsv[1] = 0;
}

inline bool decodeTelemetryPayload(const void* payload, size_t len, TelemetryMsg& out) {
  if (len == sizeof(TelemetryMsg)) {
    memcpy(&out, payload, sizeof(TelemetryMsg));
    return true;
  }
  if (len == sizeof(TelemetryMsgV1)) {
    TelemetryMsgV1 v1{};
    memcpy(&v1, payload, sizeof(TelemetryMsgV1));
    upconvertTelemetryV1(v1, out);
    return true;
  }
  return false;
}

// ===================================================================================
// Messages used by your apps
// ===================================================================================

struct TouchMsg {
  int16_t x = 0;
  int16_t y = 0;
  uint8_t type = 0; // 0=tap/press, 1=move, 2=release (up to you)
};

struct SettingChangeMsg {
  uint8_t id = 0;
  int32_t value = 0;
};

// Stable IDs your code already references
enum : uint8_t {
  C3_SET_QNH_PA       = 10, // uint32 (Pa)
  C3_SET_POLAR        = 11, // uint8 index
  C3_TE_TOGGLE        = 12, // bool (0/1)
  C3_SET_VOLUME       = 13, // 0..10
  C3_SET_BRIGHTNESS   = 14  // 0..10 (display maps to 0..255)
};

// Framed message IDs
enum : uint8_t {
  MSG_TELEMETRY       = 0x01,
  MSG_TOUCH           = 0x02,
  MSG_SETTING_CHANGE  = 0x03,
};

// ===================================================================================
// LinkProtocol: simple framed UART protocol
// Frame: 0xAA | id(1) | len(1) | payload(len) | csum(1)
// csum = 8-bit sum of id + len + payload bytes
// ===================================================================================

class LinkProtocol {
public:
  LinkProtocol() = default;

  void attach(HardwareSerial* s) { serial_ = s; resetRx_(); }

  // ---- TX helpers ----
  bool sendTelemetry(const TelemetryMsg& m);
  bool sendQNH(uint32_t pa);
  bool sendPolarSelect(uint8_t idx);
  bool sendTEToggle(bool enabled);
  bool sendVolume(uint8_t vol01);      // 0..10
  bool sendBrightness(uint8_t bri01);  // 0..10

  // ---- RX polling ----
  bool pollTelemetry(TelemetryMsg& out);
  bool pollTouch(TouchMsg& out);
  bool pollSettingChange(SettingChangeMsg& out);

private:
  HardwareSerial* serial_ = nullptr;

  // RX state machine
  enum RxState { RX_SYNC, RX_ID, RX_LEN, RX_PAYLOAD, RX_CSUM };
  RxState  rxState_ = RX_SYNC;
  uint8_t  rxId_    = 0;
  uint8_t  rxLen_   = 0;
  uint8_t  rxBuf_[80];
  uint8_t  rxPos_   = 0;

  // small holding latches for last-arrived messages
  bool haveTlm_ = false;
  TelemetryMsg tlm_{};

  bool haveTouch_ = false;
  TouchMsg touch_{};

  bool haveSet_ = false;
  SettingChangeMsg set_{};

private:
  // low-level
  inline uint8_t calcCsum_(uint8_t id, uint8_t len, const uint8_t* p) {
    uint16_t s = id + len;
    for (uint8_t i=0;i<len;i++) s += p[i];
    return (uint8_t)s;
  }
  bool writeFrame_(uint8_t id, const uint8_t* payload, uint8_t len);

  // pump RX until we have at least one parsed message latched
  void pump_();

  // handle one complete frame we just parsed
  void onFrame_(uint8_t id, const uint8_t* p, uint8_t len);

  // reset RX state
  void resetRx_() {
    rxState_ = RX_SYNC; rxId_ = 0; rxLen_ = 0; rxPos_ = 0;
  }
};
