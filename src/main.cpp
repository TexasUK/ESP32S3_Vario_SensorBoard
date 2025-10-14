// ====== ESP32-S3 Mini  — SENSOR  ======
// UART1 link on header pins:  RX=44  TX=43  (115200 8N1)
// CSV line each ~40 ms:
//   T,<netto>,<te>,<alt_m>,<asi_kts>,<fix>,<sats>,<mode>\n
//
// GPS = UART2 on pins RX=4, TX=5
// BMP581 on I2C SDA=8 SCL=9
// SD card on SPI pins 6,7,10,11 for IGC flight logging
// IGC files created automatically when flight detected (speed > 20 kts)

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include "Adafruit_BMP5xx.h"
#include <TinyGPSPlus.h>
#include <Preferences.h>
#include <math.h>
#include <time.h>

#include <NimBLEDevice.h>   // === BLE (NimBLE) ===
#include "CsvSerial.h"      // CSV link helper

// === WiFi & OTA ===
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Update.h>

// -------------------- Function Declarations --------------------
static void startWiFiOTA();
static void stopWiFiOTA();
static void handleOTA();

// -------------------- Pins --------------------
static constexpr int PIN_I2C_SDA   = 8;
static constexpr int PIN_I2C_SCL   = 9;

// GPS on UART2 (pins 4/5)
static constexpr int PIN_GPS_RX    = 4;   // UART2 RX
static constexpr int PIN_GPS_TX    = 5;   // UART2 TX

// LINK on UART1 header pair: RX=44 / TX=43
static constexpr int PIN_LINK_RX   = 44;  // Sensor RX  (from display TX=43)
static constexpr int PIN_LINK_TX   = 43;  // Sensor TX  (to   display RX=44)

// Pololu SDC01A (SPI) — optional
static constexpr int PIN_SD_SCK    = 6;
static constexpr int PIN_SD_MOSI   = 7;
static constexpr int PIN_SD_MISO   = 10;
static constexpr int PIN_SD_CS     = 11;

// -------------------- Options --------------------
#define STARTUP_QNH_MODE   1
#define SPEED_KTS_START    20.0f
#define SPEED_KTS_STOP     20.0f
#define START_DEBOUNCE_MS  3000
#define STOP_DEBOUNCE_MS   10000

// -------------------- Devices --------------------
Adafruit_BMP5xx bmp;           // BMP581
TinyGPSPlus     gps;

HardwareSerial  GPSSerial(2);  // UART2 (GPS on pins 4/5)
HardwareSerial  LinkUart(1);   // UART1 (Display link on pins 44/43)
CsvSerial       csv(LinkUart, PIN_LINK_RX, PIN_LINK_TX, 115200);

SdFat           sd;
FsFile          igc;
bool            sdMounted = false;
bool            igcLogging = false;
uint32_t        lastIgcLogMs = 0;

// -------------------- Polars / TE --------------------
struct PolarPoint { float asi_kts; float sink_rate_ms; };
struct GliderPolar {
  char name[32]; char model[32];
  PolarPoint points[12];
  uint8_t point_count;
  float min_sink_rate;
  float best_glide_speed;
};
const GliderPolar defaultPolars[] = {
  {"LS8","LS8-b",{{30,-0.50},{35,-0.60},{40,-0.72},{45,-0.95},{50,-1.25},{55,-1.65},{60,-2.15}},7,-0.60,42},
  {"DG-800","DG-800",{{35,-0.55},{40,-0.68},{45,-0.92},{50,-1.25},{55,-1.68},{60,-2.22},{65,-2.85}},7,-0.55,44},
  {"ASG-29","ASG-29 18m",{{40,-0.48},{45,-0.62},{50,-0.85},{55,-1.15},{60,-1.55},{65,-2.05},{70,-2.65}},7,-0.48,47},
  {"Discus","Discus-b",{{30,-0.60},{35,-0.65},{40,-0.75},{45,-0.95},{50,-1.25},{55,-1.65}},6,-0.60,40},
};
const int defaultPolarCount = 4;

class TECompensator {
  const GliderPolar* currentPolar = nullptr;
  static float basicSinkRate(float /*asi_kts*/) { return -0.6f; } // safe fallback
public:
  void setPolar(const GliderPolar* p){ currentPolar = p; }
  const GliderPolar* getCurrentPolar() const { return currentPolar; }
  float gliderSinkRate(float asi_kts){
    if(!currentPolar || currentPolar->point_count < 2) return basicSinkRate(asi_kts);
    if(asi_kts <= currentPolar->points[0].asi_kts) return currentPolar->points[0].sink_rate_ms;
    if(asi_kts >= currentPolar->points[currentPolar->point_count-1].asi_kts) return currentPolar->points[currentPolar->point_count-1].sink_rate_ms;
    for(int i=0;i<currentPolar->point_count-1;i++){
      if(asi_kts >= currentPolar->points[i].asi_kts && asi_kts <= currentPolar->points[i+1].asi_kts){
        float r = (asi_kts - currentPolar->points[i].asi_kts) / (currentPolar->points[i+1].asi_kts - currentPolar->points[i].asi_kts);
        return currentPolar->points[i].sink_rate_ms + r * (currentPolar->points[i+1].sink_rate_ms - currentPolar->points[i].sink_rate_ms);
      }
    }
    return basicSinkRate(asi_kts);
  }
  // With negative sink values, add sink so TE→~0 in still air
  float compensate(float netto, float asi){ return netto + gliderSinkRate(asi); }
} teComp;

// -------------------- Config --------------------
Preferences prefs;
struct UserConfig {
  char pilot[32] = "PILOT";
  char gliderType[32] = "GLIDER";
  char gliderID[16] = "G-XXXX";
  char compID[16] = "XX";
  uint32_t qnhPa = 101325;
  uint8_t selectedPolarIndex = 0;
  bool useCustomPolar = false;
  bool teCompEnabled = true;
  GliderPolar customPolar;
  uint8_t audioVolume = 5;
  uint8_t displayBrightness = 5;
} cfg;

// --- NVS persistence for cfg ---
static const char* NVS_NS = "vc3";

static void saveConfig() {
  if (!prefs.begin(NVS_NS, /*readOnly=*/false)) { Serial.println("[NVS] open rw failed"); return; }
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
  Serial.println("[NVS] cfg saved");
}

static void loadConfig() {
  if (!prefs.begin(NVS_NS, /*readOnly=*/true)) { Serial.println("[NVS] open ro failed"); return; }
  size_t n = prefs.getBytesLength("cfg");
  if (n == sizeof(cfg)) {
    prefs.getBytes("cfg", &cfg, sizeof(cfg));
    Serial.println("[NVS] cfg loaded");
  } else {
    Serial.println("[NVS] no saved cfg — using defaults");
  }
  prefs.end();
}

GliderPolar polars[20]; int polarCount=0;

// -------------------- Telemetry / state --------------------
static float alt_m=0.0f, vario_mps=0.0f, asi_kts=0.0f;
static uint8_t fix=0, sats=0;
static float seaLevelPa=101325.0f, lastPressPa=101325.0f;
static uint32_t lastBaroMs=0, lastTelemMs=0;

// -------------------- Airspeed Calculation --------------------
enum FlightMode {
  CRUISE,      // Normal flight - use polar-based airspeed
  THERMAL,     // Circling in thermal - use different method
  CLIMB,       // Strong climb - use GPS groundspeed
  DESCENT      // Descent - use polar-based airspeed
};

struct AirspeedData {
  float speed_kts;
  float confidence;     // 0.0 to 1.0
  uint32_t timestamp;
  FlightMode mode;
};

struct WindVector {
  float speed_kts;
  float direction_deg;
};

static AirspeedData airspeedData = {0.0f, 0.0f, 0, CRUISE};
static WindVector windVector = {0.0f, 0.0f};
static float lastGpsTrack = 0.0f;
static float turnRate = 0.0f;
static uint32_t lastTrackMs = 0;

// --- Flight state (debounced) ---
static bool     flying       = false;
static bool     lastFlying   = false;
static uint32_t lastAboveStart = 0;
static uint32_t lastBelowStop  = 0;
static bool     initialized = false;

// --- WiFi & OTA ---
static bool     wifiEnabled = false;
static bool     otaMode = false;
static WebServer webServer(80);
static uint32_t otaStartTime = 0;
static const uint32_t OTA_TIMEOUT_MS = 300000; // 5 minutes
static bool     uploadSuccess = false;

// --- Baro smoothing/vario (EMA + derivative) ---
static float altEma = 0.0f;
static float lastAlt = 0.0f;
static float varioEma = 0.0f;
static uint32_t lastAltMs = 0;

// Tiny gated bias learner (for Kalman vertical speed)
static float v_bias = 0.0f;
static constexpr float BIAS_ALPHA = 0.003f;  // very slow
static constexpr float BIAS_GATE  = 0.25f;   // learn only when |v|<this (m/s)

// -------------------- Kalman --------------------
struct Kalman2D{
  float h=0,v=0; float P00=1,P01=0,P10=0,P11=1; float sigma_a=0.3f,sigma_h=0.2f; bool inited=false;
  void reset(float h0){ h=h0; v=0; P00=1; P01=P10=0; P11=1; inited=true; Serial.printf("[KALMAN] Reset h=%.1f\n",h); }
  void predict(float dt){
    if(!inited||dt<1e-4f) return;
    h += v*dt;
    float P00n=P00+dt*(P10+P01)+dt*dt*P11, P01n=P01+dt*P11, P10n=P10+dt*P11, P11n=P11;
    float dt2=dt*dt, dt3=dt2*dt, dt4=dt2*dt2, qa2=sigma_a*sigma_a;
    P00=P00n+0.25f*dt4*qa2; P01=P01n+0.5f*dt3*qa2; P10=P10n+0.5f*dt3*qa2; P11=P11n+dt2*qa2;
  }
  void update(float z){
    if(!inited){ reset(z); return; }
    float y=z-h, R=sigma_h*sigma_h, S=P00+R, K0=P00/S, K1=P10/S;
    h+=K0*y; v+=K1*y;
    float P00n=(1-K0)*P00, P01n=(1-K0)*P01, P10n=P10-K1*P00, P11n=P11-K1*P01;
    P00=P00n; P01=P01n; P10=P10n; P11=P11n;
  }
} kf;

static inline float baroAltFromPa(float pPa, float p0Pa){ return 44330.0f*(1.0f - powf(pPa/p0Pa, 0.19029495f)); }
static inline float toPa(float p){ return (p < 2000.0f) ? p*100.0f : p; } // hPa->Pa if needed

// -------------------- BMP581 --------------------
static void setupBMP581(){
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  if (!bmp.begin(BMP5XX_DEFAULT_ADDRESS, &Wire)) {
    if (!bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
      Serial.println(F("[BMP581] NOT FOUND (0x47/0x46)"));
      return;
    }
  }
  Serial.println(F("[BMP581] OK"));
}
static void updateBaroKalman(){
  static int warm=10; static uint32_t lastDbg=0;
  if (!bmp.performReading()) return;
  if (warm-- > 0) return;
  lastPressPa = toPa(bmp.readPressure());
  uint32_t now = millis();
  float dt = (lastBaroMs==0) ? 0.02f : (now-lastBaroMs)/1000.0f;
  if (dt<0.001f) dt=0.001f; if (dt>0.2f) dt=0.2f;
  float a = baroAltFromPa(lastPressPa, seaLevelPa);
  if (now-lastDbg > 4000){ Serial.printf("[BARO] pa=%.1f (hPa=%.1f) alt=%.2f dt=%.3f\n", lastPressPa, lastPressPa/100.0f, a, dt); lastDbg=now; }
  kf.predict(dt); kf.update(a);

  // Bias-learn when essentially still (and slow groundspeed)
  float v_raw = kf.v;
  bool stationary = (fabsf(v_raw) < BIAS_GATE) && (asi_kts < 2.0f);
  if (stationary) {
    v_bias = (1.0f - BIAS_ALPHA) * v_bias + BIAS_ALPHA * v_raw;
  }

  alt_m     = kf.h;
  vario_mps = v_raw - v_bias;   // debiased Kalman vertical speed
  lastBaroMs= now;
}

// -------------------- GPS --------------------
static uint32_t gpsBytes = 0, lastGpsLog = 0;

static void setupGPS(){
  GPSSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX); // UART2 pins 4/5
  Serial.println(F("[GPS] UART2 @9600 on 4/5"));
}
static void pollGPS(){
  while (GPSSerial.available()) {
    char c = GPSSerial.read();
    gps.encode(c);
    gpsBytes++;
  }

  sats = (uint8_t)gps.satellites.value();
  bool locOK=gps.location.isValid(), altOK=gps.altitude.isValid(), spdOK=gps.speed.isValid();
  fix = locOK ? (altOK ? 2 : 1) : 0;
  // Note: asi_kts is now set by calculateAirspeed() in main loop
  
  // Update turn rate for airspeed calculation
  // (moved to after function declarations)

  // Set system time from GPS if we have a valid date/time
  if (gps.date.isValid() && gps.time.isValid()) {
    struct tm timeinfo;
    timeinfo.tm_year = gps.date.year() - 1900;
    timeinfo.tm_mon = gps.date.month() - 1;
    timeinfo.tm_mday = gps.date.day();
    timeinfo.tm_hour = gps.time.hour();
    timeinfo.tm_min = gps.time.minute();
    timeinfo.tm_sec = gps.time.second();
    timeinfo.tm_isdst = 0;
    
    time_t epoch = mktime(&timeinfo);
    if (epoch > 0) {
      struct timeval tv = {epoch, 0};
      settimeofday(&tv, NULL);
    }
  }

  uint32_t now = millis();
  if (now - lastGpsLog > 1000) {
    Serial.printf("[GPS] bytes=%lu  locOK=%d altOK=%d spdOK=%d  sats=%u fix=%u\n",
                  (unsigned long)gpsBytes, locOK, altOK, spdOK, (unsigned)sats, (unsigned)fix);
    gpsBytes = 0;
    lastGpsLog = now;
  }
}

// -------------------- IGC Logging --------------------
static void setupSD() {
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!sd.begin(PIN_SD_CS, SD_SCK_MHZ(10))) {
    Serial.println("[SD] Card not found or failed to initialize");
    sdMounted = false;
    return;
  }
  sdMounted = true;
  Serial.println("[SD] Card initialized successfully");
}

static void startIgcLog() {
  if (!sdMounted || igcLogging) return;
  
  // Generate filename: YYYYMMDD_HHMMSS.igc
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("[IGC] Failed to get time for filename");
    return;
  }
  
  char filename[32];
  snprintf(filename, sizeof(filename), "/%04d%02d%02d_%02d%02d%02d.igc",
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  
  if (!igc.open(filename, O_WRONLY | O_CREAT | O_TRUNC)) {
    Serial.printf("[IGC] Failed to create file: %s\n", filename);
    return;
  }
  
  // Write IGC header
  igc.println("AFLY"); // Manufacturer code
  igc.printf("HFDTE%02d%02d%02d\n", timeinfo.tm_mday, timeinfo.tm_mon + 1, (timeinfo.tm_year + 1900) % 100);
  igc.printf("HFFXA010\n"); // Fix accuracy
  igc.printf("HFPLTPILOTINCHARGE:%s\n", cfg.pilot);
  igc.printf("HFCM2CREW2:\n");
  igc.printf("HFGTYGLIDERTYPE:%s\n", cfg.gliderType);
  igc.printf("HFGIDGLIDERID:%s\n", cfg.gliderID);
  igc.printf("HFCIDCOMPETITIONID:%s\n", cfg.compID);
  igc.printf("HFCCLCOMPETITIONCLASS:Club\n");
  igc.printf("HFTZNTIMEZONE:0.00\n");
  igc.printf("HFGPSGPS:%s\n", "ESP32-S3");
  igc.printf("HFPRSPRESSALTSENSOR:%s\n", "BMP581");
  igc.printf("HFRFWFIRMWAREVERSION:1.0\n");
  igc.printf("HFRHWHARDWAREVERSION:ESP32-S3\n");
  igc.printf("HFFTYFRTYPE:ESP32-S3 Vario\n");
  igc.printf("HFGPS:%s\n", "ESP32-S3");
  igc.printf("HFPRS:%s\n", "BMP581");
  igc.printf("HFALG:%s\n", "Kalman");
  igc.printf("HFALP:%s\n", "BMP581");
  igc.printf("HFTITLE:%s\n", "ESP32-S3 Flight Log");
  igc.printf("HFDTM100GPSDATUM:WGS84\n");
  igc.printf("I023638FXA3940SIU\n"); // I record - fix accuracy and satellite info
  igc.printf("J010812HDV\n"); // J record - pressure altitude and vario
  
  igcLogging = true;
  Serial.printf("[IGC] Started logging: %s\n", filename);
}

static void stopIgcLog() {
  if (!igcLogging) return;
  
  igc.close();
  igcLogging = false;
  Serial.println("[IGC] Logging stopped");
}

static void logIgcRecord() {
  if (!igcLogging || !gps.location.isValid() || !gps.altitude.isValid()) return;
  
  uint32_t now = millis();
  if (now - lastIgcLogMs < 1000) return; // Log every second
  
  lastIgcLogMs = now;
  
  // Check if file is still open
  if (!igc.isOpen()) {
    Serial.println("[IGC] File closed unexpectedly, stopping logging");
    igcLogging = false;
    return;
  }
  
  // Get current time
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;
  
  // Format B record: BHHMMSSDDMMMMNNDDDMMMMMEE
  // B = record type
  // HHMMSS = time
  // DDMMMMN = latitude (DDMM.MMMN)
  // DDDMMMMM = longitude (DDDMM.MMME)
  // E = pressure altitude (3 digits)
  // E = GPS altitude (3 digits)
  // E = vario (2 digits)
  
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  double gpsAlt = gps.altitude.meters();
  double pressAlt = alt_m;
  double vario = vario_mps;
  
  // Convert to IGC format
  int latDeg = abs((int)lat);
  double latMin = (abs(lat) - latDeg) * 60.0;
  int latMinInt = (int)latMin;
  int latMinFrac = (int)((latMin - latMinInt) * 1000);
  char latNS = (lat >= 0) ? 'N' : 'S';
  
  int lonDeg = abs((int)lon);
  double lonMin = (abs(lon) - lonDeg) * 60.0;
  int lonMinInt = (int)lonMin;
  int lonMinFrac = (int)((lonMin - lonMinInt) * 1000);
  char lonEW = (lon >= 0) ? 'E' : 'W';
  
  // Format the B record - IGC standard format
  // BHHMMSSDDMMMMNNDDDMMMMMEE
  // B = record type
  // HHMMSS = time (UTC)
  // DDMMMMN = latitude (DDMM.MMMN)
  // DDDMMMMM = longitude (DDDMM.MMME)
  // E = pressure altitude (3 digits, meters)
  // E = GPS altitude (3 digits, meters)  
  // E = vario (2 digits, 0.1 m/s units)
  
  char bRecord[64];
  snprintf(bRecord, sizeof(bRecord), 
           "B%02d%02d%02d%02d%05d%c%03d%05d%c%03d%03d%02d",
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
           latDeg, latMinFrac, latNS,
           lonDeg, lonMinFrac, lonEW,
           (int)(pressAlt + 1000) % 1000,  // Pressure altitude (3 digits)
           (int)(gpsAlt + 1000) % 1000,    // GPS altitude (3 digits)
           (int)(vario * 10 + 50) % 100    // Vario (2 digits, scaled)
  );
  
  igc.println(bRecord);
  igc.flush(); // Ensure data is written
}

static void writeBootLog() {
  if (!sdMounted) {
    Serial.println("[BOOTLOG] SD card not mounted, skipping boot log");
    return;
  }
  
  FsFile bootLog;
  if (!bootLog.open("/boot.log", O_WRONLY | O_CREAT | O_APPEND)) {
    Serial.println("[BOOTLOG] Failed to open boot.log file");
    return;
  }
  
  // Write boot information
  bootLog.println("=== ESP32C3LinkBoard Boot Log ===");
  bootLog.printf("Boot completed at: %lu ms\n", millis());
  bootLog.printf("Pilot: %s\n", cfg.pilot);
  bootLog.printf("Glider: %s (%s)\n", cfg.gliderType, cfg.gliderID);
  bootLog.printf("Competition ID: %s\n", cfg.compID);
  bootLog.printf("QNH: %u Pa\n", cfg.qnhPa);
  bootLog.printf("Selected Polar: %s\n", cfg.selectedPolarIndex < defaultPolarCount ? 
                 defaultPolars[cfg.selectedPolarIndex].name : "Unknown");
  bootLog.printf("TE Compensation: %s\n", cfg.teCompEnabled ? "Enabled" : "Disabled");
  bootLog.printf("Audio Volume: %d/10\n", cfg.audioVolume);
  bootLog.printf("Display Brightness: %d/10\n", cfg.displayBrightness);
  bootLog.printf("SD Card: %s\n", sdMounted ? "Mounted" : "Not mounted");
  bootLog.printf("BMP581: %s\n", bmp.performReading() ? "OK" : "Failed");
  bootLog.printf("GPS: UART2 @9600 on pins 4/5\n");
  bootLog.printf("CSV Link: UART1 @115200 on pins 44/43\n");
  bootLog.printf("BLE: Advertising 'FlightCore'\n");
  bootLog.printf("BLE Status: %s\n", NimBLEDevice::getAdvertising()->isAdvertising() ? "Advertising" : "Not Advertising");
  bootLog.printf("Sea Level Pressure: %.1f Pa\n", seaLevelPa);
  bootLog.printf("Initial Altitude: %.1f m\n", alt_m);
  bootLog.println("=== End Boot Log ===");
  bootLog.println();
  
  bootLog.close();
  Serial.println("[BOOTLOG] Boot log written to SD card");
}

// -------------------- Airspeed Calculation Functions --------------------
static FlightMode detectFlightMode(float vario, float airspeed, float turnRate) {
  const float THERMAL_VARIO_THRESHOLD = 2.0f;
  const float CLIMB_VARIO_THRESHOLD = 1.0f;
  const float DESCENT_VARIO_THRESHOLD = -1.0f;
  const float THERMAL_TURN_THRESHOLD = 0.1f; // rad/s
  
  if (vario > THERMAL_VARIO_THRESHOLD && abs(turnRate) > THERMAL_TURN_THRESHOLD) {
    return THERMAL;
  }
  if (vario > CLIMB_VARIO_THRESHOLD) {
    return CLIMB;
  }
  if (vario < DESCENT_VARIO_THRESHOLD) {
    return DESCENT;
  }
  return CRUISE;
}

static float calculatePolarAirspeed(float vario) {
  // Use inverse polar calculation to estimate airspeed from sink rate
  const GliderPolar* polar = teComp.getCurrentPolar();
  if (!polar || polar->point_count < 2) {
    return 40.0f; // Default airspeed if no polar
  }
  
  float minSink = polar->min_sink_rate;
  float bestGlideSpeed = polar->best_glide_speed;
  
  // Debug polar values
  static uint32_t lastPolarDebug = 0;
  if (millis() - lastPolarDebug > 10000) {
    Serial.printf("[POLAR] polar=%p, point_count=%d, minSink: %.2f m/s, bestGlide: %.1f kts\n", 
                  polar, polar ? polar->point_count : 0, minSink, bestGlideSpeed);
    lastPolarDebug = millis();
  }
  
  // For now, use a simple approach: if descending, use best glide speed
  // If climbing, estimate based on climb rate
  if (vario < -0.5f) {
    // Strong descent - use best glide speed
    return bestGlideSpeed;
  } else if (vario > 0.5f) {
    // Climbing - estimate airspeed (climbing usually at slower speeds)
    return bestGlideSpeed * 0.8f + (vario - 0.5f) * 5.0f;
  } else {
    // Near level flight - use best glide speed
    return bestGlideSpeed;
  }
}

static void updateTurnRate() {
  if (!gps.course.isValid()) return;
  
  uint32_t now = millis();
  float currentTrack = gps.course.deg();
  
  if (lastTrackMs > 0) {
    float dt = (now - lastTrackMs) / 1000.0f;
    if (dt > 0.1f && dt < 5.0f) { // Valid time interval
      float trackDiff = currentTrack - lastGpsTrack;
      
      // Handle wrap-around
      while (trackDiff > 180.0f) trackDiff -= 360.0f;
      while (trackDiff < -180.0f) trackDiff += 360.0f;
      
      turnRate = trackDiff / dt; // degrees per second
      turnRate *= 0.017453f; // convert to rad/s
    }
  }
  
  lastGpsTrack = currentTrack;
  lastTrackMs = now;
}

static float calculateAirspeed(float gpsSpeed, float vario) {
  static float lastGoodAirspeed = 40.0f;
  static uint32_t lastUpdate = 0;
  static float airspeedHistory[10] = {0};
  static int historyIndex = 0;
  
  FlightMode mode = detectFlightMode(vario, lastGoodAirspeed, turnRate);
  float calculatedAirspeed = 0.0f;
  float confidence = 0.0f;
  
  switch (mode) {
    case CRUISE:
      // Use polar-based calculation
      calculatedAirspeed = calculatePolarAirspeed(vario);
      confidence = 0.8f;
      lastUpdate = millis();
      break;
      
    case THERMAL:
      // Use GPS groundspeed (wind effects cancel out in circles)
      if (abs(turnRate) > 0.1f) {
        calculatedAirspeed = gpsSpeed;
        confidence = 0.6f;
        lastUpdate = millis();
      } else {
        // Not turning - use last known good airspeed
        calculatedAirspeed = lastGoodAirspeed;
        confidence = 0.3f;
      }
      break;
      
    case CLIMB:
      // Use GPS groundspeed (less wind effect in climbs)
      calculatedAirspeed = gpsSpeed;
      confidence = 0.7f;
      lastUpdate = millis();
      break;
      
    case DESCENT:
      // Use polar-based calculation
      calculatedAirspeed = calculatePolarAirspeed(vario);
      confidence = 0.8f;
      lastUpdate = millis();
      break;
  }
  
  // Maintain rolling average for thermal mode
  if (mode == THERMAL && calculatedAirspeed > 0) {
    airspeedHistory[historyIndex] = calculatedAirspeed;
    historyIndex = (historyIndex + 1) % 10;
    
    // Calculate average of non-zero values
    float sum = 0.0f;
    int count = 0;
    for (int i = 0; i < 10; i++) {
      if (airspeedHistory[i] > 0) {
        sum += airspeedHistory[i];
        count++;
      }
    }
    if (count > 0) {
      calculatedAirspeed = sum / count;
      confidence = 0.5f;
    }
  }
  
  // Decay confidence over time
  if (millis() - lastUpdate > 30000) { // 30 seconds
    confidence *= 0.95f;
  }
  
  // Update airspeed data
  airspeedData.speed_kts = calculatedAirspeed;
  airspeedData.confidence = confidence;
  airspeedData.timestamp = millis();
  airspeedData.mode = mode;
  
  if (calculatedAirspeed > 0 && confidence > 0.3f) {
    lastGoodAirspeed = calculatedAirspeed;
  }
  
  return calculatedAirspeed;
}

static void calculateWind() {
  // Simple wind calculation using groundspeed vs airspeed
  if (airspeedData.confidence > 0.5f && gps.course.isValid()) {
    float gpsSpeed = gps.speed.knots();
    float gpsTrack = gps.course.deg();
    float airspeed = airspeedData.speed_kts;
    
    // Vector calculation: Groundspeed = Airspeed + Wind
    // This is simplified - real calculation would be more complex
    float speedDiff = gpsSpeed - airspeed;
    if (abs(speedDiff) > 1.0f) { // Only if significant difference
      windVector.speed_kts = abs(speedDiff);
      windVector.direction_deg = gpsTrack; // Simplified
    }
  }
}

// -------------------- Startup helpers --------------------
static void primeGPS(uint32_t ms){ uint32_t t0=millis(); while(millis()-t0<ms){ while(GPSSerial.available()) gps.encode(GPSSerial.read()); delay(1);} }
static void calibrateQNH(){
  Serial.println("[CAL] QNH calibration...]");
  // warm up sensor a bit
  for (int i=0;i<10;i++){ if (bmp.performReading()) (void)toPa(bmp.readPressure()); delay(40); }

  // average pressure ~0.5s
  float ps=0; int n=0;
  for(int i=0;i<60;i++){
    if(bmp.performReading()){ ps += toPa(bmp.readPressure()); n++; }
    delay(8);
  }
  float pavg = (n>0)?(ps/n):101325.0f;
  Serial.printf("[CAL] Average pressure: %.1f Pa (%.1f hPa) (%d samples)\n", pavg, pavg/100.0f, n);

  // choose QNH source
  if (STARTUP_QNH_MODE==0){
    seaLevelPa=(float)cfg.qnhPa;
    Serial.printf("[CAL] Fixed QNH=%u Pa\n", cfg.qnhPa);
  } else if (STARTUP_QNH_MODE==1){
    seaLevelPa=pavg;
    Serial.printf("[CAL] Auto-zero QNH=%.0f Pa\n", seaLevelPa);
  } else {
    if (gps.altitude.isValid()){
      float ag=gps.altitude.meters();
      seaLevelPa = pavg / powf(1.0f - ag/44330.0f, 5.255f);
      Serial.printf("[CAL] GPS-alt=%.1f -> QNH=%.0f Pa\n", ag, seaLevelPa);
    } else {
      seaLevelPa=pavg;
      Serial.printf("[CAL] GPS alt n/a -> QNH=%.0f Pa\n", seaLevelPa);
    }
  }

  // Initialize state with ZERO startup bias
  float h0 = baroAltFromPa(pavg, seaLevelPa);
  altEma   = h0;
  lastAlt  = h0;
  lastAltMs= millis();
  alt_m    = h0;
  vario_mps= 0.0f;

  // Also reset Kalman and bias
  kf.reset(h0);
  v_bias = 0.0f;
  lastBaroMs = millis();

  Serial.printf("[CAL] EMA init: alt=%.1f m, QNH=%.0f Pa\n", h0, seaLevelPa);
}

// =======================================================================
// ============================ BLE (NimBLE) ==============================
// =======================================================================

// UUIDs
#define UUID_SVC_CFG         "f0a00001-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_PILOT       "f0a00002-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_GTYPE       "f0a00003-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_GID         "f0a00004-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_CID         "f0a00005-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_QNH_PA      "f0a00006-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_STATE       "f0a00007-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_POLAR_LIST  "f0a00008-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_POLAR_SELECT "f0a00009-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_TE_TOGGLE   "f0a0000a-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_VOLUME      "f0a0000b-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_BRIGHTNESS  "f0a0000c-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_OTA         "f0a0000d-6d75-4d1a-a2a9-d5a9e0a1c001"

NimBLEServer*         bleServer     = nullptr;
NimBLECharacteristic* chPilot       = nullptr;
NimBLECharacteristic* chGType       = nullptr;
NimBLECharacteristic* chGID         = nullptr;
NimBLECharacteristic* chCID         = nullptr;
NimBLECharacteristic* chQNH         = nullptr;
NimBLECharacteristic* chState       = nullptr;
NimBLECharacteristic* chPolarList   = nullptr;
NimBLECharacteristic* chPolarSelect = nullptr;
NimBLECharacteristic* chTEToggle    = nullptr;
NimBLECharacteristic* chVolume      = nullptr;
NimBLECharacteristic* chBrightness  = nullptr;
NimBLECharacteristic* chOTA         = nullptr;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer) {
    Serial.println("[BLE] *** CLIENT CONNECTED ***");
  }

  void onDisconnect(NimBLEServer* pServer) {
    Serial.println("[BLE] *** CLIENT DISCONNECTED *** - restarting advertising");
    // Restart advertising after disconnection
    NimBLEDevice::startAdvertising();
  }
};

class CfgWriteCB : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic* c) /*no override keyword for max compatibility*/ {
    std::string v = c->getValue();
    Serial.printf("[BLE] *** CALLBACK TRIGGERED *** Characteristic: %p, value: '%s', length: %d\n", c, v.c_str(), v.length());

    if (c == chPilot) {
      strlcpy(cfg.pilot, v.c_str(), sizeof(cfg.pilot));
      saveConfig();
      Serial.printf("[BLE] Pilot='%s'\n", cfg.pilot);

    } else if (c == chGType) {
      strlcpy(cfg.gliderType, v.c_str(), sizeof(cfg.gliderType));
      saveConfig();
      Serial.printf("[BLE] GliderType='%s'\n", cfg.gliderType);

    } else if (c == chGID) {
      strlcpy(cfg.gliderID, v.c_str(), sizeof(cfg.gliderID));
      saveConfig();
      Serial.printf("[BLE] GliderID='%s'\n", cfg.gliderID);

    } else if (c == chCID) {
      strlcpy(cfg.compID, v.c_str(), sizeof(cfg.compID));
      saveConfig();
      Serial.printf("[BLE] CompID='%s'\n", cfg.compID);

    } else if (c == chQNH) {
      if (v.size() >= 4) {
        uint32_t pa;
        memcpy(&pa, v.data(), 4);
        if (pa > 80000 && pa < 110000) {
          cfg.qnhPa = pa;
          seaLevelPa = (float)cfg.qnhPa;
          saveConfig();
          Serial.printf("[BLE] QNH set to %u Pa\n", cfg.qnhPa);
        }
      }

    } else if (c == chPolarSelect) {
      if (v.size() == 1) {
        uint8_t idx = (uint8_t)v[0];
        if (idx < defaultPolarCount) {
          cfg.selectedPolarIndex = idx;
          teComp.setPolar(&defaultPolars[idx]);
          saveConfig();
          Serial.printf("[BLE] Selected polar: %s\n", defaultPolars[idx].name);
        }
      }

    } else if (c == chTEToggle) {
      if (v.size() == 1) {
        cfg.teCompEnabled = (v[0] != 0);
        saveConfig();
        Serial.printf("[BLE] TE Compensation %s\n", cfg.teCompEnabled ? "ENABLED" : "DISABLED");
      }

    } else if (c == chVolume) {
      Serial.printf("[BLE] Volume characteristic written to, value: '%s', size: %d\n", v.c_str(), v.size());
      if (v.size() == 1) {
        cfg.audioVolume = (uint8_t)v[0];
        if (cfg.audioVolume > 10) cfg.audioVolume = 10;
        saveConfig();
        Serial.printf("[BLE] Volume set to: %d\n", cfg.audioVolume);
      }

    } else if (c == chBrightness) {
      if (v.size() == 1) {
        cfg.displayBrightness = (uint8_t)v[0];
        if (cfg.displayBrightness > 10) cfg.displayBrightness = 10;
        saveConfig();
        Serial.printf("[BLE] Brightness set to: %d\n", cfg.displayBrightness);
      }
    } else if (c == chOTA) {
      // OTA command: "START" to begin OTA mode
      Serial.printf("[BLE] OTA characteristic written: '%s'\n", v.c_str());
      if (v == "START") {
        Serial.println("[BLE] Starting WiFi OTA mode...");
        startWiFiOTA();
      } else if (v == "STOP") {
        Serial.println("[BLE] Stopping WiFi OTA mode...");
        stopWiFiOTA();
      } else {
        Serial.printf("[BLE] Unknown OTA command: '%s'\n", v.c_str());
      }
    } else {
      Serial.printf("[BLE] Unknown characteristic written to\n");
    }
  }
};

// Global callback object to ensure it persists
static CfgWriteCB g_cfgCallback;

static void updatePolarListCharacteristic() {
  if (!chPolarList) return;
  String polarList;
  for (int i = 0; i < defaultPolarCount; i++) {
    polarList += String(i) + ":" + defaultPolars[i].name + " - " + defaultPolars[i].model + "\n";
  }
  chPolarList->setValue(polarList.c_str());
}

static void bleUpdateStateChar() {
  if (!chState) return;
  uint8_t st = flying ? 1 : 0;
  chState->setValue(&st, 1);
  chState->notify();
}

static void setupBLE() {
  Serial.println("[BLE] Starting BLE initialization...");
  
  Serial.println("[BLE] Initializing NimBLE device...");
  NimBLEDevice::init("FlightCore");                  // device name
  Serial.println("[BLE] NimBLE device initialized");
  
  Serial.println("[BLE] Setting power level...");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);           // max practical TX power
  Serial.println("[BLE] Power level set");
  
  Serial.println("[BLE] Disabling security...");
  NimBLEDevice::setSecurityAuth(false, false, false); // Disable security for easier connection
  Serial.println("[BLE] Security disabled");

  Serial.println("[BLE] Creating server...");
  bleServer = NimBLEDevice::createServer();
  Serial.printf("[BLE] Server created: %p\n", bleServer);
  
  Serial.println("[BLE] Setting server callbacks...");
  bleServer->setCallbacks(new ServerCallbacks());
  Serial.println("[BLE] Server callbacks set");
  
  Serial.println("[BLE] Creating service...");
  NimBLEService* svc = bleServer->createService(NimBLEUUID(UUID_SVC_CFG));
  Serial.printf("[BLE] Service created: %p\n", svc);

  Serial.println("[BLE] Creating characteristics...");
  chPilot       = svc->createCharacteristic(UUID_CHR_PILOT,       NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  Serial.println("[BLE] Pilot characteristic created");
  chGType       = svc->createCharacteristic(UUID_CHR_GTYPE,       NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  Serial.println("[BLE] GliderType characteristic created");
  chGID         = svc->createCharacteristic(UUID_CHR_GID,         NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  Serial.println("[BLE] GliderID characteristic created");
  chCID         = svc->createCharacteristic(UUID_CHR_CID,         NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  Serial.println("[BLE] CompID characteristic created");
  chQNH         = svc->createCharacteristic(UUID_CHR_QNH_PA,      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  Serial.println("[BLE] QNH characteristic created");
  chState       = svc->createCharacteristic(UUID_CHR_STATE,       NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  Serial.println("[BLE] State characteristic created");
  chPolarList   = svc->createCharacteristic(UUID_CHR_POLAR_LIST,  NIMBLE_PROPERTY::READ);
  Serial.println("[BLE] PolarList characteristic created");
  chPolarSelect = svc->createCharacteristic(UUID_CHR_POLAR_SELECT,NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  Serial.println("[BLE] PolarSelect characteristic created");
  chTEToggle    = svc->createCharacteristic(UUID_CHR_TE_TOGGLE,   NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  Serial.println("[BLE] TEToggle characteristic created");
  chVolume      = svc->createCharacteristic(UUID_CHR_VOLUME,      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  Serial.println("[BLE] Volume characteristic created");
  chBrightness  = svc->createCharacteristic(UUID_CHR_BRIGHTNESS,  NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  Serial.println("[BLE] Brightness characteristic created");
  chOTA         = svc->createCharacteristic(UUID_CHR_OTA,         NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  Serial.printf("[BLE] OTA characteristic created: %p\n", chOTA);

  Serial.println("[BLE] Setting up callbacks...");
  Serial.printf("[BLE] Using global callback object: %p\n", &g_cfgCallback);
  
  chPilot->setCallbacks(&g_cfgCallback);
  Serial.println("[BLE] Pilot callback set");
  chGType->setCallbacks(&g_cfgCallback);
  Serial.println("[BLE] GliderType callback set");
  chGID->setCallbacks(&g_cfgCallback);
  Serial.println("[BLE] GliderID callback set");
  chCID->setCallbacks(&g_cfgCallback);
  Serial.println("[BLE] CompID callback set");
  chQNH->setCallbacks(&g_cfgCallback);
  Serial.println("[BLE] QNH callback set");
  chPolarSelect->setCallbacks(&g_cfgCallback);
  Serial.println("[BLE] PolarSelect callback set");
  chTEToggle->setCallbacks(&g_cfgCallback);
  Serial.println("[BLE] TEToggle callback set");
  chVolume->setCallbacks(&g_cfgCallback);
  Serial.println("[BLE] Volume callback set");
  chBrightness->setCallbacks(&g_cfgCallback);
  Serial.println("[BLE] Brightness callback set");
  chOTA->setCallbacks(&g_cfgCallback);
  Serial.printf("[BLE] OTA callback set: %p\n", &g_cfgCallback);

  Serial.println("[BLE] Setting initial values...");
  chPilot->setValue((uint8_t*)cfg.pilot, strlen(cfg.pilot));
  Serial.println("[BLE] Pilot value set");
  chGType->setValue((uint8_t*)cfg.gliderType, strlen(cfg.gliderType));
  Serial.println("[BLE] GliderType value set");
  chGID->setValue((uint8_t*)cfg.gliderID, strlen(cfg.gliderID));
  Serial.println("[BLE] GliderID value set");
  chCID->setValue((uint8_t*)cfg.compID, strlen(cfg.compID));
  Serial.println("[BLE] CompID value set");
  chQNH->setValue((uint8_t*)&cfg.qnhPa, 4);
  Serial.println("[BLE] QNH value set");
  uint8_t te = cfg.teCompEnabled ? 1 : 0; chTEToggle->setValue(&te,1);
  Serial.println("[BLE] TEToggle value set");
  chVolume->setValue(&cfg.audioVolume, 1);
  Serial.println("[BLE] Volume value set");
  chBrightness->setValue(&cfg.displayBrightness, 1);
  Serial.println("[BLE] Brightness value set");
  chOTA->setValue("READY");
  Serial.println("[BLE] OTA value set to READY");
  updatePolarListCharacteristic();
  Serial.println("[BLE] Polar list updated");

  Serial.println("[BLE] Starting service...");
  svc->start();
  Serial.println("[BLE] Service started");

  Serial.println("[BLE] Configuring advertising...");
  NimBLEAdvertising* ad = NimBLEDevice::getAdvertising();
  Serial.printf("[BLE] Advertising object: %p\n", ad);
  
  Serial.println("[BLE] Creating advertisement data...");
  // Create advertisement data with device name
  NimBLEAdvertisementData advertisementData;
  advertisementData.setName("FlightCore");
  Serial.println("[BLE] Advertisement name set");
  advertisementData.setCompleteServices(NimBLEUUID(UUID_SVC_CFG));
  Serial.println("[BLE] Advertisement services set");
  
  Serial.println("[BLE] Creating scan response data...");
  // Create scan response data
  NimBLEAdvertisementData scanResponseData;
  scanResponseData.setName("FlightCore");
  Serial.println("[BLE] Scan response name set");
  
  Serial.println("[BLE] Setting advertisement data...");
  ad->setAdvertisementData(advertisementData);
  Serial.println("[BLE] Advertisement data set");
  ad->setScanResponseData(scanResponseData);
  Serial.println("[BLE] Scan response data set");
  ad->setMinInterval(0x20);  // 20ms (0x20 * 0.625ms)
  Serial.println("[BLE] Min interval set");
  ad->setMaxInterval(0x40);  // 40ms (0x40 * 0.625ms)
  Serial.println("[BLE] Max interval set");
  
  Serial.println("[BLE] Starting advertising...");
  if (ad->start()) {
    Serial.println("[BLE] Advertising 'FlightCore' with config service - STARTED");
  } else {
    Serial.println("[BLE] ERROR: Failed to start advertising");
  }
}

static void restartBLEAdvertising() {
  Serial.println("[BLE] Restarting advertising...");
  NimBLEDevice::stopAdvertising();
  delay(100);
  NimBLEDevice::startAdvertising();
  Serial.println("[BLE] Advertising restarted");
}

// -------------------- WiFi & OTA Functions --------------------

static void startWiFiOTA() {
  if (wifiEnabled) return; // Already running
  
  Serial.println("[OTA] Starting WiFi OTA mode...");
  
  // Create access point for OTA
  WiFi.mode(WIFI_AP);
  WiFi.softAP("FlightCore-OTA", "flightcore123");
  
  IPAddress apIP = WiFi.softAPIP();
  Serial.printf("[OTA] Access Point started: %s\n", apIP.toString().c_str());
  Serial.println("[OTA] Connect to WiFi: FlightCore-OTA");
  Serial.println("[OTA] Password: flightcore123");
  Serial.printf("[OTA] Open browser: http://%s\n", apIP.toString().c_str());
  
  // Setup web server
  webServer.on("/", []() {
    webServer.send(200, "text/html", 
      "<html><head><style>"
      "body { font-family: Arial, sans-serif; font-size: 24px; margin: 40px; }"
      "h1 { font-size: 36px; color: #2c3e50; margin-bottom: 30px; }"
      "form { margin: 30px 0; }"
      "input[type='file'] { font-size: 20px; margin: 10px 0; padding: 10px; }"
      "input[type='submit'] { font-size: 20px; padding: 15px 30px; background-color: #3498db; color: white; border: none; border-radius: 5px; cursor: pointer; }"
      "input[type='submit']:hover { background-color: #2980b9; }"
      "p { font-size: 20px; color: #7f8c8d; margin-top: 20px; }"
      "</style></head><body>"
      "<h1>FlightCore OTA Update</h1>"
      "<form method='POST' action='/update' enctype='multipart/form-data'>"
      "<input type='file' name='firmware' accept='.bin'>"
      "<br><br>"
      "<input type='submit' value='Upload Firmware'>"
      "</form>"
      "<p>Select firmware.bin file and click Upload</p>"
      "</body></html>"
    );
  });
  
  webServer.on("/update", HTTP_POST, []() {
    webServer.sendHeader("Connection", "close");
    if (uploadSuccess) {
      webServer.send(200, "text/plain", "OK");
      Serial.println("[OTA] Upload successful, restarting...");
      ESP.restart();
    } else {
      webServer.send(400, "text/plain", "Upload failed or no file selected");
      Serial.println("[OTA] Upload failed, staying in OTA mode");
    }
  }, []() {
    HTTPUpload& upload = webServer.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("[OTA] Update: %s\n", upload.filename.c_str());
      uploadSuccess = false; // Reset success flag
      
      // Check if filename is empty (no file selected)
      if (upload.filename.length() == 0) {
        Serial.println("[OTA] ERROR: No file selected!");
        return;
      }
      
      // Check if it's a .bin file
      if (!upload.filename.endsWith(".bin")) {
        Serial.printf("[OTA] ERROR: Invalid file type: %s\n", upload.filename.c_str());
        return;
      }
      
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
        return;
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
        uploadSuccess = false;
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("[OTA] Update Success: %u bytes\n", upload.totalSize);
        uploadSuccess = true;
      } else {
        Update.printError(Serial);
        uploadSuccess = false;
      }
    }
  });
  
  webServer.begin();
  
  wifiEnabled = true;
  otaMode = true;
  otaStartTime = millis();
  
  // Notify via BLE
  if (chState) {
    chState->setValue("OTA_MODE");
    chState->notify();
  }
}

static void stopWiFiOTA() {
  if (!wifiEnabled) return;
  
  Serial.println("[OTA] Stopping WiFi OTA mode...");
  
  webServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  
  wifiEnabled = false;
  otaMode = false;
  
  Serial.println("[OTA] WiFi disabled, returning to normal operation");
}

static void handleOTA() {
  if (otaMode && wifiEnabled) {
    webServer.handleClient();
    
    // Auto-shutdown after timeout
    if (millis() - otaStartTime > OTA_TIMEOUT_MS) {
      Serial.println("[OTA] Timeout reached, shutting down WiFi");
      stopWiFiOTA();
    }
  }
}

// -------------------- Arduino --------------------
void setup(){
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== Boot: ESP32-S3 Mini SENSOR (CSV link over UART1 44/43) ===");

  // Polars / config
  for (int i=0;i<defaultPolarCount;i++) polars[i]=defaultPolars[i];

  // Load persisted cfg (if present) before using fields
  loadConfig();

  // Ensure TEComp uses the persisted polar selection
  {
    int idx = (int)cfg.selectedPolarIndex;
    if (idx < 0 || idx >= defaultPolarCount) idx = 0;
    teComp.setPolar(&polars[idx]);
  }

  setupBMP581();
  setupGPS();
  primeGPS(1200);
  
  // Initialize SD card for IGC logging
  setupSD();

  // Use persisted QNH if you later switch to STARTUP_QNH_MODE==0
  seaLevelPa = 101325.0f;
  calibrateQNH();

  // CSV link on Hardware UART1 (pins 44/43)
  csv.begin();
  Serial.printf("[LINK] UART1 CSV  RX=%d  TX=%d @115200\n", PIN_LINK_RX, PIN_LINK_TX);

  // Optional: greet the display
  csv.sendHelloSensor(); delay(10);
  csv.sendHelloSensor(); delay(10);

  // Wire sensor-side handlers (Display -> Sensor "SET,...")
  csv.onSetTE    = [&](bool en){
    cfg.teCompEnabled = en; saveConfig();
    Serial.printf("[SET] TE=%d\n", en);
  };
  csv.onSetPolar = [&](int idx){
    if(idx>=0 && idx<defaultPolarCount){
      cfg.selectedPolarIndex = (uint8_t)idx;
      teComp.setPolar(&polars[idx]); saveConfig();
      Serial.printf("[SET] POLAR=%s\n", polars[idx].name);
    }
  };
  csv.onSetQNH   = [&](uint32_t pa){
    if(pa>80000 && pa<110000){
      cfg.qnhPa = pa; seaLevelPa=(float)pa; saveConfig();
      Serial.printf("[SET] QNH=%u\n", pa);
    }
  };
  csv.onSetVol   = [&](uint8_t v){
    if(v>10)v=10; cfg.audioVolume = v; saveConfig();
    Serial.printf("[SET] VOL=%u\n", v);
  };
  csv.onSetBri   = [&](uint8_t v){
    if(v>10)v=10; cfg.displayBrightness = v; saveConfig();
    Serial.printf("[SET] BRI=%u\n", v);
  };

  // === BLE ===
  setupBLE();
  
  // BLE system ready
  Serial.println("[BLE] BLE system initialized and ready");

  // Write boot log to SD card
  writeBootLog();

  Serial.println("[BOOT] init done.");
}

void loop(){
  handleOTA();          // Handle WiFi OTA updates if active
  csv.poll();           // handle inbound SET/PING/PONG
  updateBaroKalman();
  pollGPS();
  
  // Calculate airspeed and wind
  if (gps.speed.isValid()) {
    updateTurnRate(); // Update turn rate for airspeed calculation
    float gpsSpeed = gps.speed.knots();
    
    // Only calculate airspeed if we're actually moving (GPS speed > 5 kts)
    if (gpsSpeed > 5.0f) {
      float calculatedAirspeed = calculateAirspeed(gpsSpeed, vario_mps);
      calculateWind();
      
      // Debug output for airspeed calculation
      static uint32_t lastDebugMs = 0;
      if (millis() - lastDebugMs > 5000) {
        Serial.printf("[DEBUG] Vario: %.2f m/s, GPS: %.1f kts, AS: %.1f kts\n", 
                      vario_mps, gpsSpeed, calculatedAirspeed);
        lastDebugMs = millis();
      }
      
      // Use calculated airspeed for display
      asi_kts = calculatedAirspeed;
    } else {
      // Not moving - use GPS speed as airspeed
      asi_kts = gpsSpeed;
      
      // Debug output for stationary state
      static uint32_t lastDebugMs = 0;
      if (millis() - lastDebugMs > 5000) {
        Serial.printf("[DEBUG] Stationary: GPS: %.1f kts, AS: %.1f kts\n", gpsSpeed, asi_kts);
        lastDebugMs = millis();
      }
    }
  } else {
    // No GPS speed - set airspeed to 0
    asi_kts = 0.0f;
  }
  
  logIgcRecord();       // log GPS track data to IGC file

  uint32_t now = millis();

  // Debounced flight detection
  if (!initialized) {
    lastAboveStart = 0;  // Set to 0, not current time
    lastBelowStop = now;
    initialized = true;
  }
  
  // Use GPS groundspeed for flight detection, not calculated airspeed
  float flightDetectionSpeed = gps.speed.isValid() ? gps.speed.knots() : 0.0f;
  
  if (flightDetectionSpeed >= SPEED_KTS_START) {
    lastAboveStart = now;
    static uint32_t lastSpeedLog = 0;
    if (now - lastSpeedLog > 5000) {
      Serial.printf("[FLIGHT] Speed above threshold: %.1f kts (GPS: %.1f kts, AS: %.1f kts)\n", 
                    flightDetectionSpeed, gps.speed.knots(), asi_kts);
      lastSpeedLog = now;
    }
  }
  if (flightDetectionSpeed <= SPEED_KTS_STOP)  lastBelowStop  = now;
  
  if (!flying && lastAboveStart > 0 && (now - lastAboveStart) >= START_DEBOUNCE_MS) {
    flying = true;
    Serial.printf("[FLIGHT] FLIGHT DETECTED! GPS: %.1f kts, AS: %.1f kts\n", 
                  flightDetectionSpeed, asi_kts);
  }
  if ( flying && (now - lastBelowStop)  >= STOP_DEBOUNCE_MS) {
    flying = false;
    Serial.printf("[FLIGHT] Flight ended. GPS: %.1f kts, AS: %.1f kts\n", 
                  flightDetectionSpeed, asi_kts);
  }

  if (flying != lastFlying) {
    lastFlying = flying;
    bleUpdateStateChar(); // push state over BLE
    
    // Start/stop IGC logging based on flight state
    if (flying && !igcLogging) {
      startIgcLog();
    } else if (!flying && igcLogging) {
      stopIgcLog();
    }
  }

  // Telemetry @ ~25 Hz
  if (now - lastTelemMs >= 40){
    const float V_DEADBAND = 0.15f;
    const bool tooSlow = (asi_kts < 20.0f);   // hard floor

    // If we're below 20 kts, force zeros; otherwise use Kalman vario
    float netto = tooSlow ? 0.0f : vario_mps;
    float te_v  = tooSlow ? 0.0f : teComp.compensate(vario_mps, asi_kts);

    // small deadband
    if (fabsf(netto) < V_DEADBAND) netto = 0.0f;
    if (fabsf(te_v)  < V_DEADBAND) te_v  = 0.0f;

    csv.sendTelemetry(netto, te_v, alt_m, asi_kts, (int)fix, (int)sats, (int)airspeedData.mode);
    lastTelemMs = now;
  }

  // Debug heartbeat
  static uint32_t lastPrint=0;
  if (now - lastPrint >= 1000){
    const char* modeNames[] = {"CRUISE", "THERMAL", "CLIMB", "DESCENT"};
    Serial.printf("[TLM] netto=%.2f te=%.2f alt=%.1f as=%.1fkt gs=%.1fkt mode=%s conf=%.1f fix=%u sats=%u\n",
                  vario_mps, teComp.compensate(vario_mps, asi_kts), alt_m, asi_kts, 
                  gps.speed.isValid() ? gps.speed.knots() : 0.0f,
                  modeNames[airspeedData.mode], airspeedData.confidence, (unsigned)fix, (unsigned)sats);
    lastPrint = now;
  }

  // Periodic BLE advertising check (every 30 seconds)
  static uint32_t lastBLECheck = 0;
  if (now - lastBLECheck >= 30000) {
    if (!NimBLEDevice::getAdvertising()->isAdvertising()) {
      Serial.println("[BLE] Advertising stopped - restarting");
      restartBLEAdvertising();
    }
    lastBLECheck = now;
  }

  // Check OTA characteristic value periodically (every 2 seconds)
  static uint32_t lastOTACheck = 0;
  if (now - lastOTACheck >= 2000) {
    if (chOTA) {
      std::string otaValue = chOTA->getValue();
      static std::string lastOTAValue = "";
      if (otaValue != lastOTAValue) {
        Serial.printf("[BLE] OTA value changed: '%s' -> '%s'\n", lastOTAValue.c_str(), otaValue.c_str());
        lastOTAValue = otaValue;
        
        // Handle OTA commands
        if (otaValue == "START") {
          Serial.println("[BLE] OTA START command detected");
          startWiFiOTA();
        } else if (otaValue == "STOP") {
          Serial.println("[BLE] OTA STOP command detected");
          stopWiFiOTA();
        }
      }
    }
    lastOTACheck = now;
  }
}
