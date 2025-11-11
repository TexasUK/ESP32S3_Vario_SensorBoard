// ====== ESP32-S3 Mini  — SENSOR  ======
// UART1 link on header pins:  RX=44  TX=43  (115200 8N1)
// CSV line each ~40 ms:
//   T,<netto>,<te>,<alt_m>,<asi_kts>,<fix>,<sats>,<mode>\n
//
// GPS = UART2 on pins RX=4, TX=5
// BMP581 on I2C SDA=8 SCL=9
// SD card on SPI pins 6,7,10,11 for IGC flight logging
// IGC files created automatically when flight detected (speed > 20 kts)
//
// POLAR FILES: Place .plr files in /data folder on SD card
// Format: *ASK13 WinPilot POLAR file (w. LK8000 extension) : MassDryGross[kg], MaxWaterBallast[liters], Speed1[km/h], Sink1[m/s], Speed2, Sink2, Speed3, Sink3, WingArea[m2]
// Example: 380,  0,  78.05,  -0.79, 124.03,  -1.97, 170.00,  -4.54,  17.50

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FS.h>
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
#define SPEED_KTS_START    22.0f
#define SPEED_KTS_STOP     18.0f
#define START_DEBOUNCE_MS  3000
#define STOP_DEBOUNCE_MS   10000

// -------------------- Devices --------------------
Adafruit_BMP5xx bmp;           // BMP581
TinyGPSPlus     gps;

HardwareSerial  GPSSerial(2);  // UART2 (GPS on pins 4/5)
HardwareSerial  LinkUart(1);   // UART1 (Display link on pins 44/43)
CsvSerial       csv(LinkUart, PIN_LINK_RX, PIN_LINK_TX, 115200);

SdFs            sd;
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
// ASK13 default polar (used if no SD card polars found)
const GliderPolar ask13DefaultPolar = {
  "ASK13", "ASK13", 
  {{30,-0.60},{35,-0.70},{40,-0.80},{45,-1.00},{50,-1.30},{55,-1.70},{60,-2.20}}, 
  7, -0.60, 40
};

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

GliderPolar* polars = nullptr; // Dynamic array to hold SD card polars
int polarCount = 0;
int polarCapacity = 0;
char** polarFilenames = nullptr; // Dynamic array to store filenames

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

// --- Startup sequence ---
enum StartupState {
  STARTUP_SENSOR_CONNECTION,    // Phase 1: Wait for sensor connection
  STARTUP_DATA_LOADING,         // Phase 1: Load polar data with progress
  STARTUP_DATA_VALIDATION,      // Phase 1: Validate loaded data
  STARTUP_QNH_ENTRY,           // Phase 2: User enters QNH
  STARTUP_POLAR_SELECTION,     // Phase 2: User selects polar
  STARTUP_COMPLETE             // Normal operation
};
static StartupState startupState = STARTUP_SENSOR_CONNECTION;
static bool startupComplete = false;
static String qnhEntry = "";
static int selectedPolarIndex = 0;

// Phase 1 data loading tracking
static bool sensorConnected = false;
static bool polarDataRequested = false;
static bool polarDataComplete = false;
static int polarDataReceived = 0;
static uint32_t lastSensorCheck = 0;
static uint32_t dataLoadingStartTime = 0;

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
  static int warm=10;
  if (!bmp.performReading()) return;
  if (warm-- > 0) return;
  lastPressPa = toPa(bmp.readPressure());
  uint32_t now = millis();
  float dt = (lastBaroMs==0) ? 0.02f : (now-lastBaroMs)/1000.0f;
  if (dt<0.001f) dt=0.001f; if (dt>0.2f) dt=0.2f;
  float a = baroAltFromPa(lastPressPa, seaLevelPa);
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
    char c = (char)GPSSerial.read();
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
  // No periodic GPS debug logging for low overhead
}

// -------------------- IGC Logging --------------------
static void setupSD() {
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!sd.begin(SdSpiConfig(PIN_SD_CS, SHARED_SPI, SD_SCK_MHZ(10)))) {
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
  
  // Format standard IGC B record:
  // BHHMMSSDDMMmmmNDDDMMmmmAxxxxxYYYYY
  // - A/V flag from GPS fix (A = valid)
  // - xxxxx = pressure altitude (meters, 5 digits, 00000-99999)
  // - YYYYY = GPS altitude (meters, 5 digits, 00000-99999)
  
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  double gpsAlt = gps.altitude.meters();
  double pressAlt = alt_m;
  double vario = vario_mps;
  
  // Convert to IGC format components
  int latDeg = abs((int)lat);
  double alat = fabs(lat);
  double latMin = (alat - latDeg) * 60.0;
  int latMinInt = (int)latMin;                   // 2 digits
  int latMinFrac = (int)((latMin - latMinInt) * 1000.0 + 0.5); // 3 digits, rounded
  if (latMinFrac >= 1000) { latMinFrac = 0; latMinInt += 1; }
  char latNS = (lat >= 0) ? 'N' : 'S';

  int lonDeg = abs((int)lon);
  double alon = fabs(lon);
  double lonMin = (alon - lonDeg) * 60.0;
  int lonMinInt = (int)lonMin;                   // 2 digits
  int lonMinFrac = (int)((lonMin - lonMinInt) * 1000.0 + 0.5); // 3 digits, rounded
  if (lonMinFrac >= 1000) { lonMinFrac = 0; lonMinInt += 1; }
  char lonEW = (lon >= 0) ? 'E' : 'W';

  // Validity flag from GPS
  char validFlag = (fix > 0) ? 'A' : 'V';

  // Clamp altitudes to 0..99999
  int pressAlt_i = (int)round(pressAlt);
  if (pressAlt_i < 0) pressAlt_i = 0; if (pressAlt_i > 99999) pressAlt_i = 99999;
  int gpsAlt_i = (int)round(gpsAlt);
  if (gpsAlt_i < 0) gpsAlt_i = 0; if (gpsAlt_i > 99999) gpsAlt_i = 99999;
  
  // Assemble standard B record
  char bRecord[64];
  snprintf(bRecord, sizeof(bRecord),
           "B%02d%02d%02d%02d%02d%03d%c%03d%02d%03d%c%c%05d%05d",
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
           latDeg, latMinInt, latMinFrac, latNS,
           lonDeg, lonMinInt, lonMinFrac, lonEW,
           validFlag, pressAlt_i, gpsAlt_i);
  
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
  bootLog.printf("Selected Polar: %s\n", cfg.selectedPolarIndex < polarCount ? 
                 polars[cfg.selectedPolarIndex].name : "Unknown");
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
        if (idx < polarCount) {
          cfg.selectedPolarIndex = idx;
          teComp.setPolar(&polars[idx]);
          saveConfig();
          Serial.printf("[BLE] Selected polar: %s (index %d)\n", polars[idx].name, idx);
        } else {
          Serial.printf("[BLE] Invalid polar index: %d (max: %d)\n", idx, polarCount - 1);
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
static ServerCallbacks g_serverCallbacks; // avoid heap allocation

// Minimal line reader for SdFat FsFile (since readStringUntil may not be available)
static bool fsReadLine(FsFile &file, String &out) {
  out = "";
  while (file.available()) {
    int c = file.read();
    if (c < 0) break;
    if (c == '\r') continue;
    if (c == '\n') return true;
    out += (char)c;
    if (out.length() > 512) return true; // safety cap
  }
  return out.length() > 0;
}

static bool loadPolarFromFile(const char* filename, GliderPolar* polar) {
  if (!sdMounted) return false;
  
  FsFile file;
  if (!file.open(filename, O_RDONLY)) {
    Serial.printf("[POLAR] Failed to open file: %s\n", filename);
    return false;
  }
  
  // Read the file line by line
  String line;
  bool foundData = false;
  int pointCount = 0;
  
  while (pointCount < 12 && fsReadLine(file, line)) {
    line.trim();
    // Strip UTF-8 BOM if present
    if (line.length() >= 3 && (uint8_t)line[0] == 0xEF && (uint8_t)line[1] == 0xBB && (uint8_t)line[2] == 0xBF) {
      line = line.substring(3);
    }
    // Remove inline comments after ';' or '//' if present
    int sc = line.indexOf(';');
    if (sc >= 0) line = line.substring(0, sc);
    int dsl = line.indexOf("//");
    if (dsl >= 0) line = line.substring(0, dsl);
    line.trim();
    
    // Skip empty lines and comments
    if (line.length() == 0 || line.startsWith("*") || line.startsWith("#")) {
      continue;
    }
    
    // Look for the data line (contains numbers)
    if (line.indexOf(',') > 0) {
      // Parse the polar data line
      // Format: MassDryGross[kg], MaxWaterBallast[liters], Speed1[km/h], Sink1[m/s], Speed2, Sink2, Speed3, Sink3, WingArea[m2]
      float speeds[12] = {0};   // m/s
      float sinks[12]  = {0};   // m/s
      int pairs = 0;

      // Split by commas into tokens
      int start = 0;
      int tokenIndex = 0;
      for (int i = 0; i <= line.length(); i++) {
        if (i == line.length() || line.charAt(i) == ',') {
          String value = line.substring(start, i);
          value.trim();
          // tokens from index 0: Mass, 1: MaxWB, 2: Speed1, 3: Sink1, 4: Speed2, 5: Sink2, ...
          if (tokenIndex >= 2) {
            int idx = tokenIndex - 2; // 0-based from Speed1
            if ((idx % 2) == 0) {
              int sp = idx / 2;
              if (sp < 12) speeds[sp] = value.toFloat() / 3.6f; // km/h -> m/s
            } else {
              int sk = idx / 2;
              if (sk < 12) sinks[sk] = value.toFloat();
            }
          }
          tokenIndex++;
          start = i + 1;
        }
      }

      // Count candidate pairs
      pairs = (tokenIndex - 2) / 2;
      if (pairs < 0) pairs = 0; if (pairs > 12) pairs = 12;

      // Filter invalid values and compact
      float fspeeds[12];
      float fsinks[12];
      int valid = 0;
      for (int i = 0; i < pairs && valid < 12; i++) {
        float sp = speeds[i];
        float sk = sinks[i];
        if (isnan(sp) || isnan(sk) || isinf(sp) || isinf(sk)) continue;
        if (sp <= 0.1f || sp > 120.0f) continue;     // 0.1..120 m/s
        if (fabsf(sk) > 15.0f) continue;             // -15..+15 m/s
        fspeeds[valid] = sp;
        fsinks[valid] = sk;
        valid++;
      }

      // Require at least two points
      if (valid < 2) {
        continue;
      }

      // Sort by speed ascending (simple selection sort for small N)
      for (int i = 0; i < valid - 1; i++) {
        int minIdx = i;
        for (int j = i + 1; j < valid; j++) {
          if (fspeeds[j] < fspeeds[minIdx]) minIdx = j;
        }
        if (minIdx != i) {
          float ts = fspeeds[i]; fspeeds[i] = fspeeds[minIdx]; fspeeds[minIdx] = ts;
          float tk = fsinks[i];  fsinks[i]  = fsinks[minIdx];  fsinks[minIdx]  = tk;
        }
      }

      pointCount = valid;
      
      // Convert speeds from m/s to knots and populate polar
      for (int i = 0; i < pointCount && i < 12; i++) {
        polar->points[i].asi_kts = fspeeds[i] * 1.94384f; // m/s -> kts
        polar->points[i].sink_rate_ms = fsinks[i];        // m/s
      }
      
      polar->point_count = pointCount;
      
      // Calculate min sink and best glide
      if (pointCount > 0) {
        // Define min sink as the lowest (most negative) value.
        polar->min_sink_rate = sinks[0];
        polar->best_glide_speed = speeds[0] * 1.94384f;
        for (int i = 1; i < pointCount; i++) {
          if (sinks[i] < polar->min_sink_rate) {
            polar->min_sink_rate = sinks[i];
            polar->best_glide_speed = speeds[i] * 1.94384f;
          }
        }
      }
      
      foundData = true;
      break;
    }
  }
  
  file.close();
  
  if (foundData && pointCount > 1) {
    // Extract glider name from filename
    String fname = String(filename);
    int lastSlash = fname.lastIndexOf('/');
    int lastDot = fname.lastIndexOf('.');
    if (lastSlash >= 0 && lastDot > lastSlash) {
      fname = fname.substring(lastSlash + 1, lastDot);
    }
    strlcpy(polar->name, fname.c_str(), sizeof(polar->name));
    strlcpy(polar->model, fname.c_str(), sizeof(polar->model));
    
    Serial.printf("[POLAR] Loaded %s: %d points, min sink: %.2f m/s, best glide: %.1f kts\n", 
                  polar->name, polar->point_count, polar->min_sink_rate, polar->best_glide_speed);
    return true;
  }
  
  Serial.printf("[POLAR] No valid data found in: %s\n", filename);
  return false;
}

// Dynamic array management functions
static void expandPolarArrays() {
  int newCapacity = polarCapacity == 0 ? 10 : polarCapacity * 2;
  Serial.printf("[POLAR] Expanding arrays from %d to %d capacity\n", polarCapacity, newCapacity);
  
  GliderPolar* newPolars = (GliderPolar*)realloc(polars, newCapacity * sizeof(GliderPolar));
  char** newFilenames = (char**)realloc(polarFilenames, newCapacity * sizeof(char*));
  
  if (newPolars && newFilenames) {
    polars = newPolars;
    polarFilenames = newFilenames;
    polarCapacity = newCapacity;
    
    // Initialize new filename slots
    for (int i = polarCount; i < polarCapacity; i++) {
      polarFilenames[i] = (char*)malloc(64);
      polarFilenames[i][0] = '\0';
    }
    Serial.printf("[POLAR] Arrays expanded successfully to capacity %d\n", polarCapacity);
  } else {
    Serial.println("[POLAR] Failed to expand arrays - memory allocation error");
  }
}

static void addPolar(const GliderPolar& polar, const char* filename) {
  Serial.printf("[POLAR] Adding polar: %s (current count: %d, capacity: %d)\n", polar.name, polarCount, polarCapacity);
  
  if (polarCount >= polarCapacity) {
    Serial.println("[POLAR] Expanding arrays...");
    expandPolarArrays();
  }
  
  if (polarCount < polarCapacity) {
    polars[polarCount] = polar;
    strlcpy(polarFilenames[polarCount], filename, 64);
    polarCount++;
    Serial.printf("[POLAR] Successfully added polar: %s (new count: %d)\n", polar.name, polarCount);
  } else {
    Serial.printf("[POLAR] Failed to add polar: %s (capacity exceeded)\n", polar.name);
  }
}

static void cleanupPolarArrays() {
  if (polarFilenames) {
    for (int i = 0; i < polarCapacity; i++) {
      free(polarFilenames[i]);
    }
    free(polarFilenames);
    polarFilenames = nullptr;
  }
  if (polars) {
    free(polars);
    polars = nullptr;
  }
  polarCount = 0;
  polarCapacity = 0;
}

static void scanPolarFiles() {
  Serial.println("[POLAR] Starting polar file scan...");
  
  // Clean up any existing arrays
  cleanupPolarArrays();
  
  if (!sdMounted) {
    Serial.println("[POLAR] SD card not mounted, using ASK13 default");
    addPolar(ask13DefaultPolar, "DEFAULT");
    Serial.printf("[POLAR] Added ASK13 default, total polars: %d\n", polarCount);
    return;
  }
  
  Serial.println("[POLAR] SD card mounted, checking for /data directory...");
  
  // Check if /data directory exists
  if (!sd.exists("/data")) {
    Serial.println("[POLAR] /data directory not found, using ASK13 default");
    addPolar(ask13DefaultPolar, "DEFAULT");
    Serial.printf("[POLAR] Added ASK13 default, total polars: %d\n", polarCount);
    return;
  }
  
  Serial.println("[POLAR] /data directory found, scanning for .plr files...");
  
  // Scan for .plr files in /data directory
  FsFile dir = sd.open("/data");
  if (!dir.isDirectory()) {
    Serial.println("[POLAR] /data is not a directory, using ASK13 default");
    addPolar(ask13DefaultPolar, "DEFAULT");
    Serial.printf("[POLAR] Added ASK13 default, total polars: %d\n", polarCount);
    return;
  }
  
  int filesFound = 0;
  FsFile file;
  while (file.openNext(&dir, O_RDONLY)) {
    if (!file.isDirectory()) {
      char filename[64];
      file.getName(filename, sizeof(filename));
      String filenameStr = String(filename);
      Serial.printf("[POLAR] Found file: %s\n", filename);

      if (filenameStr.endsWith(".plr")) {
        String fullPath = String("/data/") + filenameStr;
        Serial.printf("[POLAR] Processing .plr file: %s\n", fullPath.c_str());

        // Load the polar file
        GliderPolar newPolar;
        if (loadPolarFromFile(fullPath.c_str(), &newPolar)) {
          addPolar(newPolar, fullPath.c_str());
          Serial.printf("[POLAR] Successfully loaded polar: %s\n", newPolar.name);
          filesFound++;
        } else {
          Serial.printf("[POLAR] Failed to load polar from: %s\n", fullPath.c_str());
        }
      }
    }
    file.close();
  }
  dir.close();
  
  Serial.printf("[POLAR] Found %d .plr files\n", filesFound);
  
  // If no polars were loaded, use ASK13 default
  if (polarCount == 0) {
    Serial.println("[POLAR] No polar files found, using ASK13 default");
    addPolar(ask13DefaultPolar, "DEFAULT");
  }
  
  Serial.printf("[POLAR] Loaded %d polar files total\n", polarCount);
  
  // Debug: List all loaded polars
  for (int i = 0; i < polarCount; i++) {
    Serial.printf("[POLAR] %d: %s - %s (%s)\n", i, polars[i].name, polars[i].model, polarFilenames[i]);
  }
}

static void updatePolarListCharacteristic() {
  if (!chPolarList) return;
  String polarList;
  for (int i = 0; i < polarCount; i++) {
    polarList += String(i) + ":" + polars[i].name + " - " + polars[i].model + "\n";
  }
  chPolarList->setValue(polarList.c_str());
}

// Startup sequence functions
static void handleStartupSequence() {
  if (startupComplete) return;
  
  uint32_t now = millis();
  
  switch (startupState) {
    case STARTUP_SENSOR_CONNECTION:
      // Phase 1: Wait for sensor connection
      if (now - lastSensorCheck > 1000) { // Check every second
        Serial.println("[STARTUP] Phase 1: Waiting for sensor connection...");
        lastSensorCheck = now;
      }
      
      // Check if sensor is connected (HELLO received)
      if (sensorConnected) {
        Serial.println("[STARTUP] Phase 1: Sensor connected, requesting polar data...");
        startupState = STARTUP_DATA_LOADING;
        dataLoadingStartTime = now;
      }
      break;
      
    case STARTUP_DATA_LOADING:
      // Phase 1: Load polar data with progress tracking
      if (!polarDataRequested) {
        Serial.println("[STARTUP] Phase 1: Requesting polar data from sensor...");
        // This will be triggered by the display sending GET_POLARS
        polarDataRequested = true;
      }
      
      // Check for timeout (30 seconds)
      if (now - dataLoadingStartTime > 30000) {
        Serial.println("[STARTUP] Phase 1: Data loading timeout, proceeding anyway...");
        startupState = STARTUP_DATA_VALIDATION;
      }
      
      // Check if data loading is complete
      if (polarDataComplete) {
        Serial.println("[STARTUP] Phase 1: Data loading complete, validating...");
        startupState = STARTUP_DATA_VALIDATION;
      }
      break;
      
    case STARTUP_DATA_VALIDATION:
      // Phase 1: Validate loaded data
      Serial.printf("[STARTUP] Phase 1: Validating %d polars loaded...\n", polarCount);
      if (polarCount > 0) {
        Serial.println("[STARTUP] Phase 1: Data validation complete, starting Phase 2...");
        startupState = STARTUP_QNH_ENTRY;
      } else {
        Serial.println("[STARTUP] Phase 1: No polars loaded, using default...");
        startupState = STARTUP_QNH_ENTRY;
      }
      break;
      
    case STARTUP_QNH_ENTRY:
      // Phase 2: User enters QNH (handled by display)
      Serial.println("[STARTUP] Phase 2: Waiting for QNH entry from user...");
      // This will be handled by the display sending SET,QNH command
      break;
      
    case STARTUP_POLAR_SELECTION:
      // Phase 2: User selects polar (handled by display)
      Serial.println("[STARTUP] Phase 2: Waiting for polar selection from user...");
      // This will be handled by the display sending SET,POLAR command
      break;
      
    case STARTUP_COMPLETE:
      startupComplete = true;
      Serial.println("[STARTUP] Complete: Normal operation started");
      break;
  }
}

static void processQNHEntry(String qnh) {
  if (qnh.length() == 4) {
    int qnhValue = qnh.toInt();
    if (qnhValue >= 950 && qnhValue <= 1050) {
      cfg.qnhPa = qnhValue * 100; // Convert hPa to Pa
      saveConfig();
      Serial.printf("[STARTUP] Phase 2: QNH set to: %d hPa\n", qnhValue);
      startupState = STARTUP_POLAR_SELECTION;
    }
  }
}

static void processPolarSelection(int index) {
  if (index >= 0 && index < polarCount) {
    selectedPolarIndex = index;
    teComp.setPolar(&polars[selectedPolarIndex]);
    Serial.printf("[STARTUP] Phase 2: Selected polar: %s %s\n", polars[selectedPolarIndex].name, polars[selectedPolarIndex].model);
    startupState = STARTUP_COMPLETE;
    startupComplete = true;
  }
}

// CSV polar data functions
static void sendPolarDataToDisplay(int index) {
  if (index < 0 || index >= polarCount) return;
  
  const GliderPolar& polar = polars[index];
  
  // Build speed/sink data string
  String data = "";
  for (int i = 0; i < polar.point_count; i++) {
    if (i > 0) data += ",";
    data += String(polar.points[i].asi_kts, 1); // speed in knots
    data += ",";
    data += String(polar.points[i].sink_rate_ms, 2);  // sink rate in m/s
  }
  
  // Send polar data to display
  csv.sendPolarData(index, data.c_str());
  Serial.printf("[CSV] Sent polar data for %s: %s\n", polar.name, data.c_str());
  // Keep GPS fed during heavy CSV activity
  pollGPS();
}

static void sendPolarListToDisplay() {
  if (polarCount == 0) {
    Serial.println("[CSV] No polars to send");
    return;
  }
  
  Serial.printf("[CSV] Sending %d polars to display...\n", polarCount);
  
  // Build comma-separated list of polar names
  String polarList = "";
  for (int i = 0; i < polarCount; i++) {
    if (i > 0) polarList += ",";
    polarList += polars[i].name;
  }
  
  // Check if the message is too long for CSV buffer (1024 chars)
  int messageLength = polarList.length() + 8; // +8 for "POLARS,"
  Serial.printf("[CSV] Polar list message length: %d characters\n", messageLength);
  
  if (messageLength > 900) { // Leave some buffer for 1024 byte limit
    Serial.printf("[CSV] WARNING: Polar list too long (%d chars), may be truncated!\n", messageLength);
    Serial.printf("[CSV] First 200 chars: %.200s\n", polarList.c_str());
  }
  
  // Send polar list to display
  csv.sendPolars(polarList.c_str());
  Serial.printf("[CSV] Sent polar list (%d polars)\n", polarCount);
  
  // Send polar data for each polar
  Serial.printf("[CSV] Sending polar data for %d polars...\n", polarCount);
  for (int i = 0; i < polarCount; i++) {
    sendPolarDataToDisplay(i);
    if (i % 10 == 0) { // Progress indicator every 10 polars
      Serial.printf("[CSV] Sent polar data %d/%d\n", i+1, polarCount);
    }
    // Service GPS while iterating
    pollGPS();
  }
  Serial.printf("[CSV] Completed sending all %d polar data sets\n", polarCount);
}

static void sendPolarDataChunkedToDisplay() {
  if (polarCount == 0) {
    Serial.println("[CSV] No polars to send");
    return;
  }
  
  Serial.printf("[CSV] Sending %d polars to display using chunked approach...\n", polarCount);
  
  // Build comma-separated list of polar names
  String polarList = "";
  for (int i = 0; i < polarCount; i++) {
    if (i > 0) polarList += ",";
    polarList += polars[i].name;
  }
  
  // Send polar list to display in chunks (to avoid buffer overflow)
  const int maxPolarsPerChunk = 20; // Send 20 polars per POLARS command
  int sentPolars = 0;
  
  while (sentPolars < polarCount) {
    String chunkList = "POLARS,";
    int polarsInChunk = 0;
    
    for (int i = sentPolars; i < polarCount && polarsInChunk < maxPolarsPerChunk; i++) {
      if (i > sentPolars) chunkList += ",";
      chunkList += polars[i].name;
      polarsInChunk++;
    }
    
    csv.sendPolars(chunkList.c_str());
    Serial.printf("[CSV] Sent polar list chunk (%d polars, %d-%d)\n", polarsInChunk, sentPolars, sentPolars + polarsInChunk - 1);
    sentPolars += polarsInChunk;
    
    // Small delay between polar list chunks
    delay(100);
  }
  
  Serial.printf("[CSV] Sent complete polar list (%d polars in chunks)\n", polarCount);
  
  // Give display time to process the polar list, servicing GPS
  {
    uint32_t t0 = millis();
    while (millis() - t0 < 1000) { pollGPS(); delay(10); }
  }
  
  // Send polar data in chunks of 3 polars each (reduced to prevent buffer overflow)
  const int chunkSize = 3;
  int totalChunks = (polarCount + chunkSize - 1) / chunkSize;
  
  Serial.printf("[CSV] Sending polar data in %d chunks of up to %d polars each...\n", totalChunks, chunkSize);
  
  for (int chunk = 0; chunk < totalChunks; chunk++) {
    String chunkData = String(chunk + 1) + "," + String(totalChunks) + ",";
    
    int polarsInChunk = 0;
    for (int i = 0; i < chunkSize && (chunk * chunkSize + i) < polarCount; i++) {
      int polarIndex = chunk * chunkSize + i;
      GliderPolar& polar = polars[polarIndex];
      
      // Add polar name
      chunkData += String(polar.name) + ",";
      
      // Add speed/sink pairs
      for (int j = 0; j < polar.point_count; j++) {
        chunkData += String(polar.points[j].asi_kts, 1) + ",";
        chunkData += String(polar.points[j].sink_rate_ms, 2) + ",";
      }
      
      polarsInChunk++;
    }
    
    // Remove trailing comma
    if (chunkData.endsWith(",")) {
      chunkData = chunkData.substring(0, chunkData.length() - 1);
    }
    
    // Check message length
    int messageLength = chunkData.length() + 17; // +17 for "POLAR_DATA_CHUNK,"
    Serial.printf("[CSV] Chunk %d/%d: %d polars, %d chars\n", 
                  chunk + 1, totalChunks, polarsInChunk, messageLength);
    
    if (messageLength > 1900) { // Leave some buffer for 2048 byte limit
      Serial.printf("[CSV] WARNING: Chunk %d too long (%d chars), may be truncated!\n", 
                    chunk + 1, messageLength);
    }
    
    // Send chunk
    csv.sendPolarDataChunk(chunkData.c_str());
    Serial.printf("[CSV] Sent chunk %d/%d, waiting for processing...\n", chunk + 1, totalChunks);
    
    // Longer delay between chunks while servicing GPS
    {
      uint32_t t0 = millis();
      while (millis() - t0 < 500) { pollGPS(); delay(10); }
    }
  }
  
  Serial.printf("[CSV] Completed sending all %d polars in %d chunks\n", polarCount, totalChunks);
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
  bleServer->setCallbacks(&g_serverCallbacks);
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
  
  // Notify via BLE (use OTA characteristic for textual status)
  if (chOTA) {
    chOTA->setValue("OTA_MODE");
    chOTA->notify();
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

  // Initialize SD card first for polar file scanning
  setupSD();

  // Scan for polar files on SD card
  scanPolarFiles();

  // Load persisted cfg (if present) before using fields
  loadConfig();

  // --- Initialize startup sequence ---
  startupState = STARTUP_QNH_ENTRY;
  startupComplete = false;
  selectedPolarIndex = 0;

  // Ensure TEComp uses the persisted polar selection (will be changed in startup sequence)
  {
    int idx = (int)cfg.selectedPolarIndex;
    if (idx < 0 || idx >= polarCount) idx = 0;
    teComp.setPolar(&polars[idx]);
    Serial.printf("[SETUP] Default polar: %s (index %d)\n", polars[idx].name, idx);
  }

  setupBMP581();
  setupGPS();
  primeGPS(1200);

  // Use persisted QNH if you later switch to STARTUP_QNH_MODE==0
  seaLevelPa = 101325.0f;
  calibrateQNH();

  // CSV link on Hardware UART1 (pins 44/43)
  csv.begin();
  Serial.printf("[LINK] UART1 CSV  RX=%d  TX=%d @115200\n", PIN_LINK_RX, PIN_LINK_TX);

  // Optional: greet the display
  Serial.println("[CSV] Sending HELLO,SENSOR to display...");
  csv.sendHelloSensor(); delay(10);
  Serial.println("[CSV] Sending second HELLO,SENSOR to display...");
  csv.sendHelloSensor(); delay(10);
  Serial.println("[CSV] HELLO messages sent");

  // Wire sensor-side handlers (Display -> Sensor "SET,...")
  csv.onSetTE    = [&](bool en){
    cfg.teCompEnabled = en; saveConfig();
    Serial.printf("[SET] TE=%d\n", en);
  };
  csv.onSetPolar = [&](int idx){
    if(idx>=0 && idx<polarCount){
      cfg.selectedPolarIndex = (uint8_t)idx;
      teComp.setPolar(&polars[idx]); saveConfig();
      Serial.printf("[SET] POLAR=%s (index %d)\n", polars[idx].name, idx);
      
      // If in startup sequence Phase 2, process the selection
      if (!startupComplete && startupState == STARTUP_POLAR_SELECTION) {
        processPolarSelection(idx);
      }
    } else {
      Serial.printf("[SET] Invalid polar index: %d (max: %d)\n", idx, polarCount - 1);
    }
  };
  csv.onSetQNH   = [&](uint32_t pa){
    if(pa>80000 && pa<110000){
      cfg.qnhPa = pa; seaLevelPa=(float)pa; saveConfig();
      Serial.printf("[SET] QNH=%u\n", pa);
      
      // If in startup sequence Phase 2, process the QNH entry
      if (!startupComplete && startupState == STARTUP_QNH_ENTRY) {
        processQNHEntry(String(pa / 100)); // Convert Pa to hPa
      }
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
  csv.onGetPolars = [&](){
    Serial.println("[CSV] *** GET_POLARS request received from display ***");
    sendPolarDataChunkedToDisplay(); // Use chunked approach for better reliability
    polarDataComplete = true; // Mark data loading as complete
  };
  csv.onHelloSensor = [&](){
    Serial.println("[CSV] *** Display connected - Phase 1 sensor connection established ***");
    csv.sendAck("HELLO", 1);
    sensorConnected = true; // Mark sensor as connected
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
  
  // Always poll CSV communication (even during startup)
  csv.poll();           // handle inbound SET/PING/PONG
  
  // Debug CSV communication disabled for low overhead
  
  // Manual trigger for polar data (for testing) - disabled in new startup sequence
  // static uint32_t lastPolarTest = 0;
  // if (millis() - lastPolarTest > 30000) { // Every 30 seconds
  //   Serial.println("[CSV] Manual polar data test - sending polar list...");
  //   sendPolarListToDisplay();
  //   lastPolarTest = millis();
  // }
  
  // --- Handle startup sequence ---
  if (!startupComplete) {
    // Keep reading GPS continuously during startup so fix can be acquired
    pollGPS();
    handleStartupSequence();
    delay(100);
    return;
  }
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
      
      // Debug output disabled for low overhead
      
      // Use calculated airspeed for display
      asi_kts = calculatedAirspeed;
    } else {
      // Not moving - use GPS speed as airspeed
      asi_kts = gpsSpeed;
      
      // Debug output disabled for low overhead
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
    float te_v  = tooSlow ? 0.0f : (cfg.teCompEnabled ? teComp.compensate(vario_mps, asi_kts) : netto);

    // small deadband
    if (fabsf(netto) < V_DEADBAND) netto = 0.0f;
    if (fabsf(te_v)  < V_DEADBAND) te_v  = 0.0f;

    csv.sendTelemetry(netto, te_v, alt_m, asi_kts, (int)fix, (int)sats, (int)airspeedData.mode);
    lastTelemMs = now;
  }

  // Debug heartbeat disabled for low overhead

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
