// CsvSerial.h — tiny CSV-over-UART helper (no LinkProtocol)

#pragma once
#include <Arduino.h>

struct CsvTlm {
  float netto = 0, te = 0, alt_m = 0, asi_kts = 0;
  int   fix = 0, sats = 0, mode = 0;
  bool  fresh = false;
};

class CsvSerial {
public:
  CsvSerial(HardwareSerial& s, int rxPin, int txPin, uint32_t baud)
  : ser(s), RX(rxPin), TX(txPin), BAUD(baud) {}

  void begin() {
    ser.begin(BAUD, SERIAL_8N1, RX, TX);
    // purge any junk
    while (ser.available()) (void)ser.read();
  }

  // Call often in loop()
  void poll() {
    while (ser.available()) {
      int b = ser.read();
      if (b == '\r' || b == '\n') {
        if (rlen) { line[ rlen ] = 0; handleLine(line); rlen = 0; }
      } else {
        if (rlen < sizeof(line)-1) line[rlen++] = char(b);
        else rlen = 0; // overflow -> reset
      }
    }
  }

  // Display → Sensor
  inline void sendPing() { ser.println("PING"); }
  inline void sendSetTE(bool en){ ser.printf("SET,TE,%d\n", en?1:0); }
  inline void sendSetPolar(int idx){ ser.printf("SET,POLAR,%d\n", idx); }
  inline void sendSetQNH(uint32_t pa){ ser.printf("SET,QNH,%u\n", pa); }
  inline void sendSetVol(uint8_t v){ ser.printf("SET,VOL,%u\n", (unsigned)v); }
  inline void sendSetBri(uint8_t v){ ser.printf("SET,BRI,%u\n", (unsigned)v); }

  // Sensor → Display
  inline void sendHelloSensor(){ ser.println("HELLO,SENSOR"); }
  inline void sendPong(){ ser.println("PONG"); }
  inline void sendTelemetry(float netto, float te, float alt_m, float asi_kts, int fix, int sats, int mode){
    ser.printf("T,%.2f,%.2f,%.2f,%.2f,%d,%d,%d\n", netto, te, alt_m, asi_kts, fix, sats, mode);
  }
  inline void sendAck(const char* key, int32_t val){ ser.printf("ACK,%s,%ld\n", key, (long)val); }

  // Display side: get latest parsed telemetry
  bool getLatest(CsvTlm& out){
    if (!tlm.fresh) return false;
    out = tlm; tlm.fresh = false; return true;
  }

  // Hooks the app can provide (optional)
  std::function<void()>          onHelloSensor; // display sees hello
  std::function<void()>          onPong;        // display sees pong
  std::function<void(const char*, long)> onAck; // display sees ACK

  // Sensor-side SET handlers (set these on the sensor project)
  std::function<void(bool)>      onSetTE;
  std::function<void(int)>       onSetPolar;
  std::function<void(uint32_t)>  onSetQNH;
  std::function<void(uint8_t)>   onSetVol;
  std::function<void(uint8_t)>   onSetBri;

private:
  HardwareSerial& ser;
  const int RX, TX; const uint32_t BAUD;
  char line[256]; size_t rlen = 0;
  CsvTlm tlm;

  void handleLine(const char* l){
    // telemetry
    if (l[0]=='T' && l[1]==','){
      CsvTlm t{};
      if (sscanf(l, "T,%f,%f,%f,%f,%d,%d,%d", &t.netto, &t.te, &t.alt_m, &t.asi_kts, &t.fix, &t.sats, &t.mode)==7){
        t.fresh = true; tlm = t;
      }
      return;
    }
    // handshake
    if (!strcmp(l,"PING")) { sendPong(); return; }
    if (!strcmp(l,"PONG")) { if (onPong) onPong(); return; }
    if (!strcmp(l,"HELLO,SENSOR")) { if (onHelloSensor) onHelloSensor(); return; }

    // ACK
    if (!strncmp(l,"ACK,",4)){
      char key[12]; long v=0;
      if (sscanf(l,"ACK,%11[^,],%ld", key, &v)==2){ if(onAck) onAck(key,v); }
      return;
    }

    // Sensor-side SETs
    if (!strncmp(l,"SET,",4)){
      char key[12]; long v=0;
      if (sscanf(l,"SET,%11[^,],%ld", key, &v)==2){
        if (!strcmp(key,"TE") && onSetTE){ onSetTE(v!=0); sendAck("TE", v); }
        else if(!strcmp(key,"POLAR") && onSetPolar){ onSetPolar((int)v); sendAck("POLAR", v); }
        else if(!strcmp(key,"QNH") && onSetQNH){ onSetQNH((uint32_t)v); sendAck("QNH", v); }
        else if(!strcmp(key,"VOL") && onSetVol){ onSetVol((uint8_t)v); sendAck("VOL", v); }
        else if(!strcmp(key,"BRI") && onSetBri){ onSetBri((uint8_t)v); sendAck("BRI", v); }
      }
    }
  }
};
