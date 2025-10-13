#include "LinkProtocol.h"

// ============================ TX ===================================

bool LinkProtocol::writeFrame_(uint8_t id, const uint8_t* payload, uint8_t len) {
  if (!serial_) return false;
  uint8_t csum = calcCsum_(id, len, payload);

  serial_->write(0xAA);
  serial_->write(id);
  serial_->write(len);
  if (len && payload) serial_->write(payload, len);
  serial_->write(csum);
  serial_->flush(); // keep it simple
  return true;
}

bool LinkProtocol::sendTelemetry(const TelemetryMsg& m) {
  // Always send v2 (24B) from the *sender* firmware.
  return writeFrame_(MSG_TELEMETRY, reinterpret_cast<const uint8_t*>(&m), (uint8_t)sizeof(TelemetryMsg));
}

bool LinkProtocol::sendQNH(uint32_t pa) {
  uint8_t buf[5];
  buf[0] = C3_SET_QNH_PA;
  memcpy(&buf[1], &pa, 4);
  return writeFrame_(MSG_SETTING_CHANGE, buf, sizeof(buf));
}

bool LinkProtocol::sendPolarSelect(uint8_t idx) {
  uint8_t buf[5];
  buf[0] = C3_SET_POLAR;
  int32_t v = (int32_t)idx;
  memcpy(&buf[1], &v, 4);
  return writeFrame_(MSG_SETTING_CHANGE, buf, sizeof(buf));
}

bool LinkProtocol::sendTEToggle(bool enabled) {
  uint8_t buf[5];
  buf[0] = C3_TE_TOGGLE;
  int32_t v = enabled ? 1 : 0;
  memcpy(&buf[1], &v, 4);
  return writeFrame_(MSG_SETTING_CHANGE, buf, sizeof(buf));
}

bool LinkProtocol::sendVolume(uint8_t vol01) {
  if (vol01 > 10) vol01 = 10;
  uint8_t buf[5];
  buf[0] = C3_SET_VOLUME;
  int32_t v = (int32_t)vol01;
  memcpy(&buf[1], &v, 4);
  return writeFrame_(MSG_SETTING_CHANGE, buf, sizeof(buf));
}

bool LinkProtocol::sendBrightness(uint8_t bri01) {
  if (bri01 > 10) bri01 = 10;
  uint8_t buf[5];
  buf[0] = C3_SET_BRIGHTNESS;
  int32_t v = (int32_t)bri01;
  memcpy(&buf[1], &v, 4);
  return writeFrame_(MSG_SETTING_CHANGE, buf, sizeof(buf));
}

// ============================ RX ===================================

void LinkProtocol::pump_() {
  if (!serial_) return;

  while (serial_->available()) {
    uint8_t b = (uint8_t)serial_->read();

    switch (rxState_) {
      case RX_SYNC:
        if (b == 0xAA) {
          rxState_ = RX_ID;
        }
        break;

      case RX_ID:
        rxId_ = b;
        rxState_ = RX_LEN;
        break;

      case RX_LEN:
        rxLen_ = b;
        if (rxLen_ > sizeof(rxBuf_)) {
          // invalid length; drop frame
          resetRx_();
          break;
        }
        rxPos_ = 0;
        rxState_ = (rxLen_ == 0) ? RX_CSUM : RX_PAYLOAD;
        break;

      case RX_PAYLOAD:
        rxBuf_[rxPos_++] = b;
        if (rxPos_ >= rxLen_) {
          rxState_ = RX_CSUM;
        }
        break;

      case RX_CSUM: {
        uint8_t expect = calcCsum_(rxId_, rxLen_, rxBuf_);
        if (expect == b) {
          onFrame_(rxId_, rxBuf_, rxLen_);
        }
        // good or bad checksum, restart
        resetRx_();
        break;
      }
    }
  }
}

void LinkProtocol::onFrame_(uint8_t id, const uint8_t* p, uint8_t len) {
  switch (id) {
    case MSG_TELEMETRY: {
      TelemetryMsg t{};
      if (decodeTelemetryPayload(p, len, t)) {
        tlm_ = t;
        haveTlm_ = true;
      }
    } break;

    case MSG_TOUCH: {
      if (len == 5) {
        TouchMsg tm{};
        memcpy(&tm.x,   p + 0, 2);
        memcpy(&tm.y,   p + 2, 2);
        tm.type = p[4];
        touch_ = tm;
        haveTouch_ = true;
      }
    } break;

    case MSG_SETTING_CHANGE: {
      if (len == 5) {
        SettingChangeMsg sc{};
        sc.id = p[0];
        memcpy(&sc.value, p + 1, 4);
        set_ = sc;
        haveSet_ = true;
      }
    } break;

    default:
      // ignore unknown ids
      break;
  }
}

bool LinkProtocol::pollTelemetry(TelemetryMsg& out) {
  pump_();
  if (!haveTlm_) return false;
  out = tlm_;
  haveTlm_ = false; // consume
  return true;
}

bool LinkProtocol::pollTouch(TouchMsg& out) {
  pump_();
  if (!haveTouch_) return false;
  out = touch_;
  haveTouch_ = false;
  return true;
}

bool LinkProtocol::pollSettingChange(SettingChangeMsg& out) {
  pump_();
  if (!haveSet_) return false;
  out = set_;
  haveSet_ = false;
  return true;
}
