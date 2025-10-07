// ESP32-C3 Super Mini — Telemetry + IGC + BLE Config + Polar TE Compensation
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPSPlus.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <math.h>
#include "link/LinkProtocol.h"

// -------------------- Pin map (ESP32-C3 Super Mini) --------------------
static constexpr int PIN_I2C_SDA   = 8;
static constexpr int PIN_I2C_SCL   = 9;
static constexpr int PIN_GPS_RX    = 5;
static constexpr int PIN_GPS_TX    = 4;
static constexpr int PIN_LINK_RX   = 20;
static constexpr int PIN_LINK_TX   = 21;
static constexpr int PIN_SD_SCK    = 6;
static constexpr int PIN_SD_MOSI   = 7;
static constexpr int PIN_SD_MISO   = 10;
static constexpr int PIN_SD_CS     = 3;

// -------------------- Build-time options --------------------
#define STARTUP_QNH_MODE 1
#define SPEED_KTS_START 20.0f
#define SPEED_KTS_STOP  20.0f
#define START_DEBOUNCE_MS 3000
#define STOP_DEBOUNCE_MS 10000

// -------------------- Devices --------------------
Adafruit_BMP3XX bmp;
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);
LinkProtocol g_link;
SdFat sd;
FsFile igc;
bool sdMounted = false;

// -------------------- Polar Structures --------------------
struct PolarPoint {
    float asi_kts;
    float sink_rate_ms;
};

struct GliderPolar {
    char name[32];
    char model[32];
    PolarPoint points[12];
    uint8_t point_count;
    float min_sink_rate;
    float best_glide_speed;
};

// Popular glider polars (simplified examples)
const GliderPolar defaultPolars[] = {
    {
        "LS8", "LS8-b",
        {{30.0f, -0.50f}, {35.0f, -0.60f}, {40.0f, -0.72f}, {45.0f, -0.95f}, {50.0f, -1.25f}, {55.0f, -1.65f}, {60.0f, -2.15f}},
        7, -0.60f, 42.0f
    },
    {
        "DG-800", "DG-800", 
        {{35.0f, -0.55f}, {40.0f, -0.68f}, {45.0f, -0.92f}, {50.0f, -1.25f}, {55.0f, -1.68f}, {60.0f, -2.22f}, {65.0f, -2.85f}},
        7, -0.55f, 44.0f
    },
    {
        "ASG-29", "ASG-29 18m",
        {{40.0f, -0.48f}, {45.0f, -0.62f}, {50.0f, -0.85f}, {55.0f, -1.15f}, {60.0f, -1.55f}, {65.0f, -2.05f}, {70.0f, -2.65f}},
        7, -0.48f, 47.0f
    },
    {
        "Discus", "Discus-b",
        {{30.0f, -0.60f}, {35.0f, -0.65f}, {40.0f, -0.75f}, {45.0f, -0.95f}, {50.0f, -1.25f}, {55.0f, -1.65f}},
        6, -0.60f, 40.0f
    }
};
const int defaultPolarCount = 4;

// -------------------- TE Compensation Engine --------------------
class TECompensator {
private:
    const GliderPolar* currentPolar = nullptr;
    
public:
    void setPolar(const GliderPolar* polar) {
        currentPolar = polar;
    }
    
    float gliderSinkRate(float asi_kts) {
        if (!currentPolar || currentPolar->point_count < 2) {
            return basicSinkRate(asi_kts);
        }
        
        if (asi_kts <= currentPolar->points[0].asi_kts) {
            return currentPolar->points[0].sink_rate_ms;
        }
        if (asi_kts >= currentPolar->points[currentPolar->point_count-1].asi_kts) {
            return currentPolar->points[currentPolar->point_count-1].sink_rate_ms;
        }
        
        for (int i = 0; i < currentPolar->point_count - 1; i++) {
            if (asi_kts >= currentPolar->points[i].asi_kts && 
                asi_kts <= currentPolar->points[i+1].asi_kts) {
                float ratio = (asi_kts - currentPolar->points[i].asi_kts) / 
                             (currentPolar->points[i+1].asi_kts - currentPolar->points[i].asi_kts);
                return currentPolar->points[i].sink_rate_ms + 
                       ratio * (currentPolar->points[i+1].sink_rate_ms - currentPolar->points[i].sink_rate_ms);
            }
        }
        
        return basicSinkRate(asi_kts);
    }
    
    float compensate(float netto_vario, float asi_kts) {
        return netto_vario - gliderSinkRate(asi_kts);
    }
    
    const GliderPolar* getCurrentPolar() { return currentPolar; }
    
private:
    float basicSinkRate(float asi_kts) {
        return -0.5f - (asi_kts - 35.0f) * 0.05f;
    }
};

static TECompensator teComp;

// -------------------- Persistent Config --------------------
Preferences prefs;

struct UserConfig {
    char pilot[32] = "PILOT";
    char gliderType[32] = "GLIDER";
    char gliderID[16] = "G-XXXX";
    char compID[16] = "XX";
    uint32_t qnhPa = 101325;
    
    // Polar settings
    uint8_t selectedPolarIndex = 0;
    bool useCustomPolar = false;
    bool teCompEnabled = true;
    GliderPolar customPolar;
};

UserConfig cfg;
GliderPolar polars[20];
int polarCount = 0;

void loadConfig() {
    prefs.begin("c3vario", true);
    String p = prefs.getString("pilot", cfg.pilot);
    String gt = prefs.getString("gtype", cfg.gliderType);
    String gid = prefs.getString("gid", cfg.gliderID);
    String cid = prefs.getString("cid", cfg.compID);
    uint32_t q = prefs.getUInt("qnh", cfg.qnhPa);
    cfg.selectedPolarIndex = prefs.getUChar("polarIdx", 0);
    cfg.teCompEnabled = prefs.getBool("teEnabled", true);
    cfg.useCustomPolar = prefs.getBool("useCustom", false);
    
    // Load custom polar if exists
    if (cfg.useCustomPolar) {
        size_t customSize = prefs.getBytesLength("customPol");
        if (customSize == sizeof(GliderPolar)) {
            prefs.getBytes("customPol", (uint8_t*)&cfg.customPolar, sizeof(GliderPolar));
        }
    }
    
    prefs.end();
    
    strlcpy(cfg.pilot, p.c_str(), sizeof(cfg.pilot));
    strlcpy(cfg.gliderType, gt.c_str(), sizeof(cfg.gliderType));
    strlcpy(cfg.gliderID, gid.c_str(), sizeof(cfg.gliderID));
    strlcpy(cfg.compID, cid.c_str(), sizeof(cfg.compID));
    cfg.qnhPa = q;
}

void saveConfig() {
    prefs.begin("c3vario", false);
    prefs.putString("pilot", cfg.pilot);
    prefs.putString("gtype", cfg.gliderType);
    prefs.putString("gid", cfg.gliderID);
    prefs.putString("cid", cfg.compID);
    prefs.putUInt("qnh", cfg.qnhPa);
    prefs.putUChar("polarIdx", cfg.selectedPolarIndex);
    prefs.putBool("teEnabled", cfg.teCompEnabled);
    prefs.putBool("useCustom", cfg.useCustomPolar);
    
    if (cfg.useCustomPolar) {
        prefs.putBytes("customPol", (uint8_t*)&cfg.customPolar, sizeof(GliderPolar));
    }
    
    prefs.end();
}

// -------------------- Telemetry / State --------------------
static float alt_m = 0.0f;
static float vario_mps = 0.0f;
static float asi_kts = 0.0f;
static float track_deg = 0.0f;
static uint8_t fix = 0;
static uint8_t sats = 0;

static float seaLevelPa = 101325.0f;
static float lastPressPa = 101325.0f;
static uint32_t lastBaroMs = 0, lastTelemMs = 0, lastLogMs = 0;

// Flight logging state
enum LogState { IDLE, ARMED, LOGGING };
static LogState logState = IDLE;
static uint32_t speedHighSince = 0, speedLowSince = 0;
static bool pendingRename = false;
static char pendingName[40] = {0};

// ===================================================================
//                           2-state Kalman - FIXED PARAMETERS
// ===================================================================
struct Kalman2D {
    float h = 0.0f, v = 0.0f;
    float P00 = 1.0f, P01 = 0.0f, P10 = 0.0f, P11 = 1.0f;  // Reduced initial covariance
    float sigma_a = 0.3f;   // REDUCED: process accel std dev [m/s^2] - was 1.2f
    float sigma_h = 0.2f;   // REDUCED: measurement std dev [m] - was 0.8f
    bool inited = false;

    void reset(float h0) {
        h = h0; v = 0.0f;
        P00 = 1.0f; P01 = 0.0f; P10 = 0.0f; P11 = 1.0f;  // Reduced initial uncertainty
        inited = true;
        Serial.printf("[KALMAN] Reset to h=%.1f, v=%.1f\n", h, v);
    }
    
    void predict(float dt) {
        if (!inited || dt < 1e-4f) return;
        h += v * dt;
        float P00n = P00 + dt*(P10 + P01) + dt*dt*P11;
        float P01n = P01 + dt*P11;
        float P10n = P10 + dt*P11;
        float P11n = P11;
        float dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt2*dt2, qa2 = sigma_a*sigma_a;
        P00 = P00n + 0.25f*dt4*qa2;
        P01 = P01n + 0.5f*dt3*qa2;
        P10 = P10n + 0.5f*dt3*qa2;
        P11 = P11n + dt2*qa2;
    }
    
    void update(float z_h) {
        if (!inited) { reset(z_h); return; }
        float y = z_h - h;
        float R = sigma_h * sigma_h;
        float S = P00 + R;
        float K0 = P00 / S;
        float K1 = P10 / S;
        h += K0 * y; v += K1 * y;
        float P00n = (1.0f - K0) * P00;
        float P01n = (1.0f - K0) * P01;
        float P10n = P10 - K1 * P00;
        float P11n = P11 - K1 * P01;
        P00 = P00n; P01 = P01n; P10 = P10n; P11 = P11n;
    }
} kf;

static inline float baroAltFromPa(float pPa, float p0Pa) {
    return 44330.0f * (1.0f - powf(pPa / p0Pa, 0.19029495f));
}

// -------------------- BMP388 --------------------
void setupBMP388() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    if (!bmp.begin_I2C(0x77)) {
        if (!bmp.begin_I2C(0x76)) {
            Serial.println(F("[BMP388] NOT FOUND"));
            return;
        }
    }
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println(F("[BMP388] OK"));
}

void updateBaroKalman() {
    static int warmup_count = 10;  // Ignore first few readings in operation too
    static uint32_t lastDebug = 0;
    
    if (!bmp.performReading()) return;
    
    if (warmup_count > 0) {
        warmup_count--;
        return;
    }
    
    lastPressPa = bmp.readPressure();

    uint32_t now = millis();
    float dt = (lastBaroMs == 0) ? 0.02f : (now - lastBaroMs) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.2f) dt = 0.2f;

    float a_meas = baroAltFromPa(lastPressPa, seaLevelPa);
    
    // Debug output every 5 seconds
    if (now - lastDebug > 5000) {
        Serial.printf("[KALMAN] Before: h=%.3f, v=%.3f, meas=%.3f, dt=%.3f\n", 
                      kf.h, kf.v, a_meas, dt);
        lastDebug = now;
    }
    
    kf.predict(dt);
    kf.update(a_meas);

    alt_m = kf.h;
    vario_mps = kf.v;
    lastBaroMs = now;
}

// -------------------- GPS --------------------
void setupGPS() {
    GPSSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    Serial.println(F("[GPS] UART1 @ 9600"));
}

void pollGPS() {
    while (GPSSerial.available()) gps.encode(GPSSerial.read());
    sats = (uint8_t)gps.satellites.value();

    bool locOK = gps.location.isValid();
    bool altOK = gps.altitude.isValid();
    bool spdOK = gps.speed.isValid();
    bool crsOK = gps.course.isValid();

    fix = locOK ? (altOK ? 2 : 1) : 0;
    if (spdOK) asi_kts = gps.speed.knots();
    if (crsOK) {
        track_deg = gps.course.deg();
        if (track_deg < 0.0f) track_deg += 360.0f;
        if (track_deg >= 360.0f) track_deg -= 360.0f;
    }
}

// -------------------- SD Card --------------------
const char* fatName(uint8_t t) {
    switch (t) {
        case FAT_TYPE_FAT12: return "FAT12";
        case FAT_TYPE_FAT16: return "FAT16";
        case FAT_TYPE_FAT32: return "FAT32";
        case FAT_TYPE_EXFAT: return "exFAT";
        default: return "Unknown";
    }
}

void setupSD() {
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    const uint8_t mhz[] = { 25, 20, 12, 8, 4, 1 };
    for (uint8_t i=0; i<sizeof(mhz); ++i) {
        SdSpiConfig cfg(PIN_SD_CS, DEDICATED_SPI, SD_SCK_MHZ(mhz[i]));
        if (sd.begin(cfg)) {
            sdMounted = true;
            uint8_t ft = sd.fatType();
            uint64_t sectors = sd.card()->sectorCount();
            uint64_t sizeMB = (sectors * 512ULL) / (1024ULL * 1024ULL);
            Serial.printf("[SD] Mounted @ %u MHz  type=%s  size=%llu MB\n",
                         mhz[i], fatName(ft), (unsigned long long)sizeMB);
            return;
        }
    }
    Serial.println("[SD] Mount failed");
}

// -------------------- IGC Helpers --------------------
void igcCoord(double deg, bool isLat, char* out) {
    char hemi = isLat ? (deg >= 0.0 ? 'N' : 'S') : (deg >= 0.0 ? 'E' : 'W');
    double a = fabs(deg);
    int d = (int)floor(a);
    double minutes = (a - d) * 60.0;
    int mmm = (int)round(minutes * 1000.0);
    if (mmm == 60000) { d += 1; mmm = 0; }
    if (isLat) sprintf(out, "%02d%05d%c", d, mmm, hemi);
    else sprintf(out, "%03d%05d%c", d, mmm, hemi);
}

String igcDateString() {
    if (!gps.date.isValid()) return "HFDTE010100\n";
    char buf[20];
    sprintf(buf, "HFDTE%02d%02d%02d\n", gps.date.day(), gps.date.month(), gps.date.year()%100);
    return String(buf);
}

String igcTimeStringHHMMSS() {
    char buf[7];
    int hh = gps.time.isValid() ? gps.time.hour() : 0;
    int mm = gps.time.isValid() ? gps.time.minute() : 0;
    int ss = gps.time.isValid() ? gps.time.second() : 0;
    sprintf(buf, "%02d%02d%02d", hh, mm, ss);
    return String(buf);
}

int qnePressureAltM() {
    int pa = (int)lround(44330.0 * (1.0 - powf(lastPressPa / 101325.0, 0.19029495)));
    if (pa < 0) pa = 0;
    if (pa > 99999) pa = 99999;
    return pa;
}

int gpsAltM() {
    if (!gps.altitude.isValid()) return 0;
    long m = lround(gps.altitude.meters());
    if (m < 0) m = 0;
    if (m > 99999) m = 99999;
    return (int)m;
}

String makeBRecord() {
    char latbuf[9], lonbuf[10];
    double lat = gps.location.lat(), lon = gps.location.lng();
    igcCoord(lat, true, latbuf);
    igcCoord(lon, false, lonbuf);
    char line[40];
    char fixc = (fix >= 1) ? 'A' : 'V';
    int pAlt = qnePressureAltM();
    int gAlt = gpsAltM();
    String t = igcTimeStringHHMMSS();
    sprintf(line, "B%s%s%s%c%05d%05d\n", t.c_str(), latbuf, lonbuf, fixc, pAlt, gAlt);
    return String(line);
}

String igcFileNameFromGps() {
    if (!gps.date.isValid() || !gps.time.isValid()) return String("FLIGHT_TMP.igc");
    char name[32];
    sprintf(name, "%04d%02d%02d_%02d%02d%02d.igc",
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
    return String(name);
}

void writeIgcHeader(FsFile& f) {
    f.print(igcDateString());
    f.printf("HFFXA010\n");
    f.printf("HFPLTPILOTINCHARGE:%s\n", cfg.pilot);
    f.printf("HFGTYGLIDERTYPE:%s\n", cfg.gliderType);
    f.printf("HFGIDGLIDERID:%s\n", cfg.gliderID);
    f.printf("HFCIDCOMPETITIONID:%s\n", cfg.compID);
    f.printf("HFDTM100GPSDATUM:WGS-84\n");
    f.printf("HFGPS:BN-280\n");
    f.printf("HFRFWFIRMWARE:ESP32C3\n");
    f.flush();
}

void startLog() {
    if (!sdMounted) { Serial.println("[IGC] SD not mounted"); return; }
    String fname = igcFileNameFromGps();
    if (fname == "FLIGHT_TMP.igc") {
        pendingRename = true;
    } else {
        pendingRename = false;
    }
    strlcpy(pendingName, fname.c_str(), sizeof(pendingName));

    igc.close();
    if (!igc.open(fname.c_str(), O_WRONLY | O_CREAT | O_TRUNC)) {
        Serial.println("[IGC] open failed");
        return;
    }
    writeIgcHeader(igc);
    Serial.printf("[IGC] Started: %s\n", fname.c_str());
    logState = LOGGING;
    speedLowSince = 0;
}

void tryRenameIfTimeValid() {
    if (!pendingRename || !igc.isOpen()) return;
    if (!gps.date.isValid() || !gps.time.isValid()) return;
    String trueName = igcFileNameFromGps();
    if (trueName == "FLIGHT_TMP.igc") return;
    igc.close();
    sd.remove(trueName.c_str());
    sd.rename("FLIGHT_TMP.igc", trueName.c_str());
    pendingRename = false;
    Serial.printf("[IGC] Renamed to %s\n", trueName.c_str());
    igc.open(trueName.c_str(), O_WRONLY | O_APPEND);
}

void stopLog() {
    if (igc.isOpen()) { igc.flush(); igc.close(); Serial.println("[IGC] Stopped"); }
    logState = IDLE;
    speedHighSince = speedLowSince = 0;
    pendingRename = false;
}

// -------------------- BLE (NimBLE) --------------------
#define UUID_SVC_CFG       "f0a00001-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_PILOT     "f0a00002-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_GTYPE     "f0a00003-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_GID       "f0a00004-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_CID       "f0a00005-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_QNH_PA    "f0a00006-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_STATE     "f0a00007-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_POLAR_LIST "f0a00008-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_POLAR_SELECT "f0a00009-6d75-4d1a-a2a9-d5a9e0a1c001"
#define UUID_CHR_TE_TOGGLE "f0a0000a-6d75-4d1a-a2a9-d5a9e0a1c001"

NimBLEServer* bleServer = nullptr;
NimBLECharacteristic* chPilot = nullptr;
NimBLECharacteristic* chGType = nullptr;
NimBLECharacteristic* chGID = nullptr;
NimBLECharacteristic* chCID = nullptr;
NimBLECharacteristic* chQNH = nullptr;
NimBLECharacteristic* chState = nullptr;
NimBLECharacteristic* chPolarList = nullptr;
NimBLECharacteristic* chPolarSelect = nullptr;
NimBLECharacteristic* chTEToggle = nullptr;

class CfgWriteCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c) override {
        std::string v = c->getValue();
        if (c == chPilot) {
            strlcpy(cfg.pilot, v.c_str(), sizeof(cfg.pilot)); saveConfig();
            Serial.printf("[BLE] Pilot='%s'\n", cfg.pilot);
        } else if (c == chGType) {
            strlcpy(cfg.gliderType, v.c_str(), sizeof(cfg.gliderType)); saveConfig();
            Serial.printf("[BLE] GliderType='%s'\n", cfg.gliderType);
        } else if (c == chGID) {
            strlcpy(cfg.gliderID, v.c_str(), sizeof(cfg.gliderID)); saveConfig();
            Serial.printf("[BLE] GliderID='%s'\n", cfg.gliderID);
        } else if (c == chCID) {
            strlcpy(cfg.compID, v.c_str(), sizeof(cfg.compID)); saveConfig();
            Serial.printf("[BLE] CompID='%s'\n", cfg.compID);
        } else if (c == chQNH) {
            if (v.size() >= 4) {
                uint32_t pa; memcpy(&pa, v.data(), 4);
                if (pa > 80000 && pa < 110000) {
                    cfg.qnhPa = pa; seaLevelPa = (float)cfg.qnhPa; saveConfig();
                    Serial.printf("[BLE] QNH set to %u Pa\n", cfg.qnhPa);
                }
            }
        } else if (c == chPolarSelect && v.size() == 1) {
            uint8_t idx = v[0];
            if (idx < polarCount) {
                cfg.selectedPolarIndex = idx;
                cfg.useCustomPolar = false;
                teComp.setPolar(&polars[idx]);
                saveConfig();
                Serial.printf("[BLE] Selected polar: %s\n", polars[idx].name);
            }
        } else if (c == chTEToggle && v.size() == 1) {
            cfg.teCompEnabled = (v[0] != 0);
            saveConfig();
            Serial.printf("[BLE] TE Compensation %s\n", cfg.teCompEnabled ? "ENABLED" : "DISABLED");
        }
    }
};

void updatePolarListCharacteristic() {
    String polarList;
    for (int i = 0; i < polarCount; i++) {
        polarList += String(i) + ":" + polars[i].name + " - " + polars[i].model + "\n";
    }
    if (chPolarList) chPolarList->setValue(polarList.c_str());
}

void setupBLE() {
    NimBLEDevice::init("Vario-C3");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    bleServer = NimBLEDevice::createServer();

    NimBLEService* svc = bleServer->createService(UUID_SVC_CFG);
    chPilot = svc->createCharacteristic(UUID_CHR_PILOT, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    chGType = svc->createCharacteristic(UUID_CHR_GTYPE, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    chGID = svc->createCharacteristic(UUID_CHR_GID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    chCID = svc->createCharacteristic(UUID_CHR_CID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    chQNH = svc->createCharacteristic(UUID_CHR_QNH_PA, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    chState = svc->createCharacteristic(UUID_CHR_STATE, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    chPolarList = svc->createCharacteristic(UUID_CHR_POLAR_LIST, NIMBLE_PROPERTY::READ);
    chPolarSelect = svc->createCharacteristic(UUID_CHR_POLAR_SELECT, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    chTEToggle = svc->createCharacteristic(UUID_CHR_TE_TOGGLE, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    static CfgWriteCB cb;
    chPilot->setCallbacks(&cb);
    chGType->setCallbacks(&cb);
    chGID->setCallbacks(&cb);
    chCID->setCallbacks(&cb);
    chQNH->setCallbacks(&cb);
    chPolarSelect->setCallbacks(&cb);
    chTEToggle->setCallbacks(&cb);

    chPilot->setValue((uint8_t*)cfg.pilot, strlen(cfg.pilot));
    chGType->setValue((uint8_t*)cfg.gliderType, strlen(cfg.gliderType));
    chGID->setValue((uint8_t*)cfg.gliderID, strlen(cfg.gliderID));
    chCID->setValue((uint8_t*)cfg.compID, strlen(cfg.compID));
    chQNH->setValue((uint8_t*)&cfg.qnhPa, 4);
    uint8_t st = (logState == LOGGING) ? 1 : 0;
    chState->setValue(&st, 1);
    uint8_t te = cfg.teCompEnabled ? 1 : 0;
    chTEToggle->setValue(&te, 1);

    svc->start();
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(UUID_SVC_CFG);
    adv->setScanResponse(true);
    adv->start();
    
    updatePolarListCharacteristic();
    Serial.println("[BLE] Advertising 'Vario-C3' with polar support");
}

void bleUpdateStateChar() {
    if (!chState) return;
    uint8_t st = (logState == LOGGING) ? 1 : 0;
    chState->setValue(&st, 1);
    chState->notify();
}

// -------------------- Link Inbound --------------------
void applyQNH(uint32_t pa) {
    if (pa > 80000 && pa < 110000) {
        cfg.qnhPa = pa; seaLevelPa = (float)cfg.qnhPa; saveConfig();
        Serial.printf("[LINK] QNH set to %u Pa\n", cfg.qnhPa);
    }
}

void serviceLinkInbound() {
    TouchMsg tmsg;
    SettingChangeMsg sc;
    
    if (g_link.pollTouch(tmsg)) {
        Serial.printf("[LINK] Touch received: %d,%d type=%d\n", tmsg.x, tmsg.y, tmsg.type);
    }
    
    if (g_link.pollSettingChange(sc)) {
        Serial.printf("[LINK] Setting received: id=%d, value=%d\n", sc.id, (int)sc.value);
        
        uint8_t id = sc.id;
        uint32_t v = (uint32_t)sc.value;
        if (id == C3_SET_QNH_PA) {
            applyQNH(v);
        } else if (id == C3_SET_POLAR) {
            if (v < polarCount) {
                cfg.selectedPolarIndex = v;
                cfg.useCustomPolar = false;
                teComp.setPolar(&polars[cfg.selectedPolarIndex]);
                saveConfig();
                Serial.printf("[LINK] Polar changed to: %s (index %d)\n", 
                             polars[cfg.selectedPolarIndex].name, cfg.selectedPolarIndex);
            }
        } else if (id == C3_TE_TOGGLE) {
            cfg.teCompEnabled = (v > 0);
            saveConfig();
            Serial.printf("[LINK] TE Compensation %s\n", cfg.teCompEnabled ? "ENABLED" : "DISABLED");
        }
    }
    
    // Also check for simple serial commands (fallback)
    while (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd.startsWith("SETPOLAR:")) {
            int polarIndex = cmd.substring(9).toInt();
            if (polarIndex >= 0 && polarIndex < polarCount) {
                cfg.selectedPolarIndex = polarIndex;
                cfg.useCustomPolar = false;
                teComp.setPolar(&polars[cfg.selectedPolarIndex]);
                saveConfig();
                Serial.printf("[SERIAL] Polar set to: %s (index %d)\n", 
                             polars[cfg.selectedPolarIndex].name, cfg.selectedPolarIndex);
            }
        }
        else if (cmd.startsWith("SETTE:")) {
            int teEnabled = cmd.substring(6).toInt();
            cfg.teCompEnabled = (teEnabled > 0);
            saveConfig();
            Serial.printf("[SERIAL] TE Compensation %s\n", cfg.teCompEnabled ? "ENABLED" : "DISABLED");
        }
    }
}

// -------------------- Startup --------------------
void primeGPS(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) { while (GPSSerial.available()) gps.encode(GPSSerial.read()); delay(1); }
}

void calibrateQNH() {
  Serial.println("[CAL] Starting QNH calibration...");
  
  // IGNORE FIRST 10 READINGS - they're unreliable
  Serial.println("[CAL] Ignoring first 10 readings for sensor warm-up...");
  for (int i = 0; i < 10; i++) {
    if (bmp.performReading()) {
      // Just read and ignore
      bmp.readPressure();
    }
    delay(50);
  }
  
  // Now take real measurements
  float psum = 0.0f; 
  int n = 0;
  
  for (int i = 0; i < 60; ++i) {
    if (bmp.performReading()) {
      psum += bmp.readPressure();
      n++;
    }
    delay(10);
  }
  
  float p_avg = (n > 0) ? (psum / n) : 101325.0f;
  Serial.printf("[CAL] Average pressure: %.1f Pa (%d samples)\n", p_avg, n);

  if (STARTUP_QNH_MODE == 0) {
    seaLevelPa = (float)cfg.qnhPa;
    Serial.printf("[CAL] Fixed QNH=%u Pa\n", cfg.qnhPa);
  } else if (STARTUP_QNH_MODE == 1) {
    seaLevelPa = p_avg;
    Serial.printf("[CAL] Auto-zero QNH=%.0f Pa\n", seaLevelPa);
  } else {
    if (gps.altitude.isValid()) {
      float alt_gps = gps.altitude.meters();
      seaLevelPa = p_avg / powf(1.0f - alt_gps / 44330.0f, 5.255f);
      Serial.printf("[CAL] QNH from GPS: alt=%.1f m -> QNH=%.0f Pa\n", alt_gps, seaLevelPa);
    } else {
      seaLevelPa = p_avg;
      Serial.printf("[CAL] GPS alt not ready -> Auto-zero QNH=%.0f Pa\n", seaLevelPa);
    }
  }

  // Reset Kalman filter with proper initial altitude
  float initial_alt = baroAltFromPa(p_avg, seaLevelPa);
  kf.reset(initial_alt);
  alt_m = initial_alt; 
  vario_mps = 0.0f; 
  lastBaroMs = millis();
  
  Serial.printf("[CAL] Kalman reset: alt=%.1f m, QNH=%.0f Pa\n", initial_alt, seaLevelPa);
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 1500) {}

    // Initialize polar database
    polarCount = defaultPolarCount;
    for (int i = 0; i < polarCount; i++) {
        polars[i] = defaultPolars[i];
    }

    loadConfig();

    Serial.println(F("\n=== ESP32-C3 Sensor/Telemetry + IGC + BLE + Polar TE ==="));
    Serial.printf("Polar: %s, TE: %s\n", 
                  polars[cfg.selectedPolarIndex].name, 
                  cfg.teCompEnabled ? "ON" : "OFF");

    setupBMP388();
    setupGPS();
    setupSD();
    setupBLE();

    g_link.begin(115200, PIN_LINK_RX, PIN_LINK_TX);
    Serial.println(F("[LINK] UART0 @ 115200"));

    // Set initial polar
    if (cfg.useCustomPolar) {
        teComp.setPolar(&cfg.customPolar);
    } else {
        teComp.setPolar(&polars[cfg.selectedPolarIndex]);
    }

    primeGPS(1500);
    seaLevelPa = (float)cfg.qnhPa;
    calibrateQNH();
}

// -------------------- Main Loop --------------------
void loop() {
    updateBaroKalman();
    pollGPS();
    serviceLinkInbound();

    // --- Telemetry to S3 ---
uint32_t now = millis();
if (now - lastTelemMs >= 40) {
    TelemetryMsg m{};
    
    // Apply deadband to vario to eliminate false positives
    float display_vario = vario_mps;
    const float VARIO_DEADBAND = 0.15f;
    
    if (fabsf(display_vario) < VARIO_DEADBAND) {
        display_vario = 0.0f;
    }
    
    m.vario_mps = display_vario;
    
    // Calculate TE-compensated vario if enabled
    if (cfg.teCompEnabled) {
        // Only apply TE compensation when we have meaningful airspeed
        if (asi_kts >= 10.0f) {  // Minimum speed for TE compensation
            float sink_rate = teComp.gliderSinkRate(asi_kts);
            m.te_vario_mps = display_vario - sink_rate;
            
            // Debug occasionally
            static uint32_t lastDebug = 0;
            if (now - lastDebug > 3000) {
                Serial.printf("[TE] ASI=%.1fkt, polar_sink=%.3f, netto=%.3f, TE=%.3f\n", 
                             asi_kts, sink_rate, display_vario, m.te_vario_mps);
                lastDebug = now;
            }
        } else {
            // At low speeds, TE vario = netto vario
            m.te_vario_mps = display_vario;
            if (asi_kts < 2.0f) {
                static uint32_t lastWarning = 0;
                if (now - lastWarning > 5000) {
                    Serial.println("[TE] Compensation disabled - airspeed too low");
                    lastWarning = now;
                }
            }
        }
    } else {
        m.te_vario_mps = display_vario;
    }
    
    m.alt_m = alt_m;
    m.asi_kts = asi_kts;
    m.track_deg = track_deg;
    m.fix = fix;
    m.sats = sats;
    memset(m.rsv, 0, sizeof(m.rsv));
    g_link.sendTelemetry(m);
    lastTelemMs = now;
}

    // --- IGC logging logic ---
    bool speedOK = (asi_kts >= SPEED_KTS_START);
    bool speedLow = (asi_kts < SPEED_KTS_STOP);

    if (logState == IDLE) {
        if (speedOK) {
            if (speedHighSince == 0) speedHighSince = now;
            if (now - speedHighSince >= START_DEBOUNCE_MS && gps.location.isValid()) {
                startLog();
            }
        } else {
            speedHighSince = 0;
        }
    } else if (logState == LOGGING) {
        if (speedLow) {
            if (speedLowSince == 0) speedLowSince = now;
            if (now - speedLowSince >= STOP_DEBOUNCE_MS) {
                stopLog();
                bleUpdateStateChar();
            }
        } else {
            speedLowSince = 0;
        }
        tryRenameIfTimeValid();
    }

    // --- Write B-record ---
    if (logState == LOGGING && now - lastLogMs >= 1000) {
        if (igc.isOpen()) {
            String B = makeBRecord();
            igc.print(B);
            if ((now / 1000) % 3 == 0) igc.flush();
        }
        lastLogMs = now;
        bleUpdateStateChar();
    }

    // --- Console output ---
    static uint32_t lastPrint = 0;
    if (now - lastPrint >= 1000) {
        const char* teStatus = cfg.teCompEnabled ? "TE" : "NETTO";
        float display_vario = cfg.teCompEnabled ? teComp.compensate(vario_mps, asi_kts) : vario_mps;
        
        // Apply deadband for console output too
        if (fabsf(display_vario) < 0.15f) {
            display_vario = 0.0f;
        }
        
        // In the console output section:
Serial.printf("[TLM] %s=%.2f alt=%.1f gs=%.1fkt track=%.0f fix=%d polar=%s %s\n",
             teStatus, display_vario, alt_m, asi_kts, track_deg, (int)fix,
             polars[cfg.selectedPolarIndex].name,  // This should show the current polar
             (logState == LOGGING ? "LOGGING" : "-"));
        lastPrint = now;
    }
}