#include "LinkProtocol.h"

void LinkProtocol::begin(uint32_t baud, int8_t rxPin, int8_t txPin) {
  ser_ = &Serial1; // Use UART1 for link communication
  ser_->begin(baud, SERIAL_8N1, rxPin, txPin);
}

// ==================== SENDING METHODS ====================

void LinkProtocol::sendTelemetry(const TelemetryMsg& m) {
  sendFrame(FT_TELEMETRY, (const uint8_t*)&m, sizeof(TelemetryMsg));
}

void LinkProtocol::sendTouch(uint16_t type, uint16_t x, uint16_t y) {
  TouchMsg msg = {type, x, y};
  sendFrame(FT_TOUCH, (const uint8_t*)&msg, sizeof(TouchMsg));
}

void LinkProtocol::sendSettingChange(uint8_t id, int32_t value) {
  SettingChangeMsg msg = {id, value};
  sendFrame(FT_SETTING_CHANGE, (const uint8_t*)&msg, sizeof(SettingChangeMsg));
}

void LinkProtocol::sendFrame(uint8_t type, const uint8_t* payload, size_t len) {
  if (!ser_) return;
  
  // Calculate CRC (include type in CRC calculation)
  uint16_t crc = crc16(&type, 1);
  crc = crc16(payload, len, crc);
  
  // SLIP encode and send frame
  ser_->write(FEND);
  
  // Send type
  slipPush(type);
  
  // Send payload
  for (size_t i = 0; i < len; i++) {
    slipPush(payload[i]);
  }
  
  // Send CRC (little-endian)
  slipPush(crc & 0xFF);
  slipPush((crc >> 8) & 0xFF);
  
  ser_->write(FEND);
}

void LinkProtocol::slipPush(uint8_t c) {
  if (c == FEND) {
    ser_->write(FESC);
    ser_->write(TFEND);
  } else if (c == FESC) {
    ser_->write(FESC);
    ser_->write(TFESC);
  } else {
    ser_->write(c);
  }
}

// ==================== RECEIVING METHODS ====================

bool LinkProtocol::pollTelemetry(TelemetryMsg& out) {
  uint8_t type;
  const uint8_t* payload;
  size_t plen;
  
  if (pollFrame(type, payload, plen)) {
    if (type == FT_TELEMETRY && plen == sizeof(TelemetryMsg)) {
      memcpy(&out, payload, sizeof(TelemetryMsg));
      return true;
    }
  }
  return false;
}

bool LinkProtocol::pollTouch(TouchMsg& out) {
  uint8_t type;
  const uint8_t* payload;
  size_t plen;
  
  if (pollFrame(type, payload, plen)) {
    if (type == FT_TOUCH && plen == sizeof(TouchMsg)) {
      memcpy(&out, payload, sizeof(TouchMsg));
      return true;
    }
  }
  return false;
}

bool LinkProtocol::pollSettingChange(SettingChangeMsg& out) {
  uint8_t type;
  const uint8_t* payload;
  size_t plen;
  
  if (pollFrame(type, payload, plen)) {
    if (type == FT_SETTING_CHANGE && plen == sizeof(SettingChangeMsg)) {
      memcpy(&out, payload, sizeof(SettingChangeMsg));
      return true;
    }
  }
  return false;
}

// ==================== SLIP RECEIVE LOGIC ====================

bool LinkProtocol::pollFrame(uint8_t& type, const uint8_t*& pay, size_t& plen) {
  static bool esc = false;
  
  while (ser_ && ser_->available()) {
    uint8_t b = ser_->read();
    
    if (!in_frame_) {
      if (b == FEND) {
        in_frame_ = true;
        rxlen_ = 0;
        esc = false;
      }
      continue;
    }
    
    if (esc) {
      if (b == TFEND) b = FEND;
      else if (b == TFESC) b = FESC;
      else {
        // Invalid escape sequence
        in_frame_ = false;
        rxlen_ = 0;
        esc = false;
        continue;
      }
      esc = false;
    } else if (b == FESC) {
      esc = true;
      continue;
    } else if (b == FEND) {
      // End of frame
      if (rxlen_ >= 3) { // Need at least type + 2 byte CRC
        // Parse the frame
        uint8_t* p = rxbuf_;
        size_t n = rxlen_;
        if (tryParse(p, n, type, pay, plen)) {
          in_frame_ = false;
          rxlen_ = 0;
          return true;
        }
      }
      in_frame_ = false;
      rxlen_ = 0;
      esc = false;
      continue;
    }
    
    // Add to buffer if space
    if (rxlen_ < RX_MAX) {
      rxbuf_[rxlen_++] = b;
    } else {
      // Buffer overflow
      in_frame_ = false;
      rxlen_ = 0;
      esc = false;
    }
  }
  return false;
}

bool LinkProtocol::tryParse(uint8_t*& p, size_t& n, uint8_t& type, const uint8_t*& pay, size_t& plen) {
  if (n < 3) return false; // Need type + 2 byte CRC
  
  // Extract type (first byte)
  type = p[0];
  
  // Calculate payload length (everything except type and 2-byte CRC)
  plen = n - 3;
  pay = &p[1];
  
  // Verify CRC
  uint16_t calc_crc = crc16(&type, 1);
  calc_crc = crc16(pay, plen, calc_crc);
  uint16_t recv_crc = (uint16_t)p[1+plen] | ((uint16_t)p[1+plen+1] << 8);
  
  if (calc_crc != recv_crc) {
    return false;
  }
  
  return true;
}

// ==================== CRC16 IMPLEMENTATION ====================

uint16_t LinkProtocol::crc16(const uint8_t* d, size_t n, uint16_t crc) {
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (int i = 0; i < 8; ++i) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}