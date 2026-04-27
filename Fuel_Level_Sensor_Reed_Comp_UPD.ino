/*
 * Fuel Level Sensor V6.1
 * ESP32-C3  |  NMEA2000 via TWAI  |  BLE  |  OTA via BLE
 *
 * PIN MAP
 * -------
 *  GPIO 1  → ADC_A1  : Vcc monitor      (partitore 10K+1K8, ingresso 12V)
 *  GPIO 3  → ADC_A3  : Sensore analogico (partitore 6K8+2K2, sensore R18)
 *  GPIO 4  → ADC_A4  : Sensore SOLO NMEA (pull-up 330Ω a 3.3V)
 *  GPIO 5  → GPIO_Q1 : base Q1
 *  GPIO 8  → LED
 *  GPIO 20 → CAN TX
 *  GPIO 21 → CAN RX
 *
 * BLE GATT SERVICE: "FuelSensor"
 * --------------------------------
 *  CHAR_DATA   (NOTIFY) : JSON 500ms
 *  CHAR_CMD    (WRITE)  : comandi testo
 *  CHAR_LOG    (NOTIFY) : log testo
 *  CHAR_OTA    (WRITE)  : chunk binari firmware OTA
 *
 * OTA VIA BLE
 * -----------
 *  1) App scarica .bin da GitHub
 *  2) CMD: OTA_START:size,md5hex
 *  3) App invia chunk binari su CHAR_OTA (fino a 490 byte ciascuno)
 *  4) CMD: OTA_END → ESP32 verifica MD5, scrive otadata, riavvia
 *  5) CMD: OTA_ABORT → annulla
 *
 * GENERA IL .bin DA ARDUINO IDE
 * ------------------------------
 *  Sketch → Export Compiled Binary
 *  → Fuel_Level_Sensor_V6.ino.bin  (questo è il file da uplodare su GitHub)
 */

#include <Arduino.h>
#include <Preferences.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32_twai_C3.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>

// ─── VERSIONE FIRMWARE ───────────────────────────────────────────────────────
#define FW_VERSION "6.1.0"

// ─── PIN ─────────────────────────────────────────────────────────────────────
#define ADC_A1_PIN   1
#define ADC_A3_PIN   3
#define ADC_A4_PIN   4
#define GPIO_Q1_PIN  5
#define LED_PIN      8

// ─── BLE UUID ────────────────────────────────────────────────────────────────
#define BLE_SERVICE_UUID   "12345678-1234-1234-1234-123456789abc"
#define BLE_CHAR_DATA_UUID "12345678-1234-1234-1234-123456789ab0"
#define BLE_CHAR_CMD_UUID  "12345678-1234-1234-1234-123456789ab1"
#define BLE_CHAR_LOG_UUID  "12345678-1234-1234-1234-123456789ab2"
#define BLE_CHAR_OTA_UUID  "12345678-1234-1234-1234-123456789ab3"

// ─── COSTANTI DI CONFIGURAZIONE ──────────────────────────────────────────────
float ADC_OFFSET_A1 = 0.15f;
float ADC_OFFSET_A3 = 0.10f;
float ADC_OFFSET_A4 = 0.10f;
const bool DEBUG_WAIT_GO = false;

// ─── NMEA2000 ─────────────────────────────────────────────────────────────────
NMEA2000_esp32_twai NMEA2000(GPIO_NUM_20, GPIO_NUM_21);

// ─── PREFERENZE ──────────────────────────────────────────────────────────────
Preferences prefs;

float ohm_vuoto     = 240.0f;
float ohm_pieno     =  32.0f;
float R_parallel    =   0.0f;
float K_ratio       =   0.0f;
bool  rk_calibrated = false;

// ─── SENSORE REED ────────────────────────────────────────────────────────────
#define REED_MAX_STEPS 12
struct ReedStep { float ohm; int percent; };

bool      sensorIsReed   = false;
int       reedStepCount  = 0;
float     reedHysteresis = 5.0f;     // Ω
ReedStep  reedLUT[REED_MAX_STEPS];
int       reedLastStep   = 0;        // ultimo step valido

void reedSaveFlash() {
  prefs.putBool ("reed_en",  sensorIsReed);
  prefs.putInt  ("reed_n",   reedStepCount);
  prefs.putFloat("reed_hys", reedHysteresis);
  for (int i = 0; i < reedStepCount; i++) {
    prefs.putFloat(("ro" + String(i)).c_str(), reedLUT[i].ohm);
    prefs.putInt  (("rp" + String(i)).c_str(), reedLUT[i].percent);
  }
}

void reedLoadFlash() {
  sensorIsReed   = prefs.getBool ("reed_en",  false);
  reedStepCount  = prefs.getInt  ("reed_n",   0);
  reedHysteresis = prefs.getFloat("reed_hys", 5.0f);
  for (int i = 0; i < reedStepCount && i < REED_MAX_STEPS; i++) {
    reedLUT[i].ohm     = prefs.getFloat(("ro" + String(i)).c_str(), 0.0f);
    reedLUT[i].percent = prefs.getInt  (("rp" + String(i)).c_str(), 0);
  }
}

// Cerca lo step più vicino con isteresi — restituisce ultimo valido se nessun match
int reedLookup(float ohm) {
  for (int i = 0; i < reedStepCount; i++) {
    if (abs(ohm - reedLUT[i].ohm) <= reedHysteresis) {
      reedLastStep = i;
      return reedLUT[i].percent;
    }
  }
  return reedStepCount > 0 ? reedLUT[reedLastStep].percent : 0;
}

// ─── PARTITORI ADC ───────────────────────────────────────────────────────────
const float R_A1_HIGH = 10000.0f;
const float R_A1_LOW  =  1800.0f;
const float R_A3_HIGH =  6800.0f;
const float R_A3_LOW  =  2200.0f;
float DIV_A1 = (R_A1_HIGH + R_A1_LOW) / R_A1_LOW;
float DIV_A3 = (R_A3_HIGH + R_A3_LOW) / R_A3_LOW;

// ─── STATI ───────────────────────────────────────────────────────────────────
enum State {
  MODE_WAIT_START,
  MODE_RK_MEASURE,
  MODE_WAIT_CAL,
  MODE_CALIBRATION,
  MODE_NORMAL_ANALOG,
  MODE_NORMAL_NMEA
};
enum RKPhase {
  RK_WAIT_HIGH, RK_MEASURE_RPAR, RK_WAIT_GO,
  RK_WAIT_LOW, RK_WAIT_HIGH2, RK_MEASURE_K, RK_DONE
};

State   currentState = MODE_WAIT_START;
RKPhase rkPhase      = RK_WAIT_HIGH;
bool    calNmeaOnly  = false;

// ─── VARIABILI ───────────────────────────────────────────────────────────────
int   fuelTankLevel = 0;
float filteredA1    = 0.0f;
float filteredA3    = 4095.0f;
float filteredA4    = 0.0f;

const float        alpha_normal = 0.05f;
const float        alpha_cal    = 0.20f;
const unsigned long FILTER_MS   = 10;
unsigned long lastFilterTime    = 0;

unsigned long stateTimer         = 0;
unsigned long peakTimer          = 0;
unsigned long stabilizationTimer = 0;
bool          isStabilized       = false;

// accumulatori R//K
unsigned long rkTimer    = 0;
int           rkSamples  = 0;
double        rkAccA1    = 0.0;
double        rkAccA3    = 0.0;
float         rkVa1saved = 0.0f;
float         rkVa3saved = 0.0f;

bool          rkHighSeen   = false;  unsigned long rkHighTimer  = 0;
bool          rkLowSeen    = false;  unsigned long rkLowTimer   = 0;
bool          rkHigh2Seen  = false;  unsigned long rkHigh2Timer = 0;
bool          wsLowSeen    = false;  unsigned long wsLowTimer   = 0;

// ─── BLE ─────────────────────────────────────────────────────────────────────
BLEServer*         bleServer        = nullptr;
BLECharacteristic* bleCharData      = nullptr;
BLECharacteristic* bleCharCmd       = nullptr;
BLECharacteristic* bleCharLog       = nullptr;
BLECharacteristic* bleCharOta       = nullptr;
bool               bleClientConnected = false;
bool               bleActive         = false;
bool               bleConfirmed      = false;
unsigned long      bleStartTime      = 0;
const unsigned long BLE_CONFIRM_TIMEOUT = 10000; // 10s

// Buffer comando ricevuto via BLE
volatile bool   bleCmdPending = false;
String          bleCmdBuffer  = "";

// ─── OTA ─────────────────────────────────────────────────────────────────────
enum OtaState { OTA_IDLE, OTA_RECEIVING, OTA_ERROR };
OtaState               otaState        = OTA_IDLE;
esp_ota_handle_t       otaHandle       = 0;
const esp_partition_t* otaPartition    = nullptr;
size_t                 otaExpectedSize = 0;
size_t                 otaReceivedSize = 0;

// ─── BLE CALLBACKS ───────────────────────────────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleClientConnected = true;
    BLEDevice::setMTU(185);
    Serial.println("[BLE] Client connesso");
  }
  void onDisconnect(BLEServer* s) override {
    bleClientConnected = false;
    Serial.println("[BLE] Client disconnesso");
    if (bleActive) s->startAdvertising();
  }
};

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String val = c->getValue().c_str();
    val.trim();
    if (val.length() > 0) {
      bleCmdBuffer  = val;
      bleCmdPending = true;
    }
  }
};

class OtaCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    if (otaState != OTA_RECEIVING) return;
    uint8_t* data = c->getData();
    size_t   len  = c->getLength();
    if (len == 0) return;

    esp_err_t err = esp_ota_write(otaHandle, data, len);
    if (err != ESP_OK) {
      bleLog("!!! OTA write error: " + String(esp_err_to_name(err)));
      esp_ota_abort(otaHandle);
      otaState = OTA_ERROR;
      return;
    }
    otaReceivedSize += len;
    static size_t lastPct = 0;
    size_t pct = otaExpectedSize > 0 ? (otaReceivedSize * 100 / otaExpectedSize) : 0;
    if (pct / 10 != lastPct / 10) {
      bleLog("OTA " + String(pct) + "% (" + String(otaReceivedSize) + "/" + String(otaExpectedSize) + ")");
      lastPct = pct;
    }
  }
};

// ─── BLE UTILITY ─────────────────────────────────────────────────────────────
void bleLog(const String& msg) {
  Serial.println(msg);
  if (bleActive && bleClientConnected && bleCharLog) {
    bleCharLog->setValue(msg.c_str());
    bleCharLog->notify();
  }
}

void initBLE() {
  BLEDevice::init("FuelSensor");
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService* svc = bleServer->createService(BLE_SERVICE_UUID);

  // Caratteristica dati (NOTIFY)
  bleCharData = svc->createCharacteristic(
    BLE_CHAR_DATA_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  bleCharData->addDescriptor(new BLE2902());

  // Caratteristica comandi (WRITE)
  bleCharCmd = svc->createCharacteristic(
    BLE_CHAR_CMD_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  bleCharCmd->setCallbacks(new CmdCallbacks());

  // Caratteristica log (NOTIFY)
  bleCharLog = svc->createCharacteristic(
    BLE_CHAR_LOG_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  bleCharLog->addDescriptor(new BLE2902());

  // Caratteristica OTA (WRITE con response) — riceve chunk binari del firmware
  bleCharOta = svc->createCharacteristic(
    BLE_CHAR_OTA_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  bleCharOta->setCallbacks(new OtaCallbacks());

  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  bleActive    = true;
  bleStartTime = millis();
  Serial.println("[BLE] Avviato — attendo conferma da app entro 10s");
}

void stopBLE() {
  BLEDevice::stopAdvertising();
  bleActive = false;
  Serial.println("[BLE] Spento — timeout conferma");
}

// Invia JSON dati ogni 500ms se BLE attivo
void bleSendData(float vA1real, float vA3real, float vA4raw, float r18) {
  static unsigned long lastBle = 0;
  if (!bleActive || !bleClientConnected || !bleCharData) return;
  unsigned long now = millis();
  if (now - lastBle < 500) return;
  lastBle = now;

  // Stato come stringa
  const char* stateStr = "WAIT";
  switch (currentState) {
    case MODE_WAIT_START:    stateStr = "WAIT";     break;
    case MODE_RK_MEASURE:    stateStr = "RK_CAL";   break;
    case MODE_WAIT_CAL:      stateStr = "WAIT_CAL"; break;
    case MODE_CALIBRATION:   stateStr = "CAL";      break;
    case MODE_NORMAL_ANALOG: stateStr = "ANALOG";   break;
    case MODE_NORMAL_NMEA:   stateStr = "NMEA";     break;
  }

  char json[160];
  snprintf(json, sizeof(json),
    "{\"a1\":%.2f,\"a3\":%.2f,\"a4\":%.2f,\"r\":%.1f,"
    "\"lv\":%d,\"st\":\"%s\",\"rc\":%d,"
    "\"v0\":%.1f,\"p0\":%.1f,\"rp\":%.2f,\"k\":%.4f,\"rd\":%d,\"fw\":\"%s\"}",
    vA1real, vA3real,
    max((vA4raw * 3.3f / 4095.0f) - ADC_OFFSET_A4, 0.0f),
    r18, fuelTankLevel, stateStr,
    rk_calibrated ? 1 : 0,
    ohm_vuoto, ohm_pieno, R_parallel, K_ratio,
    sensorIsReed ? 1 : 0,
    FW_VERSION
  );
  bleCharData->setValue(json);
  bleCharData->notify();
}

// ─── UTILITY ─────────────────────────────────────────────────────────────────
float readVoltage(int pin) {
  float v = (analogRead(pin) * 3.3f) / 4095.0f;
  float offset = (pin == ADC_A1_PIN) ? ADC_OFFSET_A1 :
                 (pin == ADC_A3_PIN) ? ADC_OFFSET_A3 : ADC_OFFSET_A4;
  return max(v - offset, 0.0f);
}

float threshA3(float va1real, float thresh12V) {
  if (va1real < 1.0f) return thresh12V;
  return thresh12V * va1real / 12.0f;
}

void printADC(const char* label, int pin) {
  int   raw  = analogRead(pin);
  float volt = (raw * 3.3f) / 4095.0f;
  Serial.print("  "); Serial.print(label);
  Serial.print(": raw="); Serial.print(raw);
  Serial.print("  ADC="); Serial.print(volt, 4); Serial.print("V");
  if (pin == ADC_A4_PIN) {
    float r = (volt > 0.1f && volt < 3.25f) ? (volt * 330.0f / (3.3f - volt)) : 9999.0f;
    Serial.print("  R="); Serial.print(r, 1); Serial.println("Ω");
  } else {
    float div = (pin == ADC_A1_PIN) ? DIV_A1 : DIV_A3;
    Serial.print("  reale="); Serial.print(volt * div, 3); Serial.println("V");
  }
}

// ─── TABELLA CORREZIONE ──────────────────────────────────────────────────────
#define CORR_MAX_PTS 21   // max 21 punti (ogni 5% da 0 a 100)

struct CorrPoint { float raw; float corrected; };

bool      corrEnabled  = false;
int       corrCount    = 0;       // numero punti attivi
CorrPoint corrLUT[CORR_MAX_PTS];

void corrSaveFlash() {
  prefs.putBool("corr_en", corrEnabled);
  prefs.putInt ("corr_n",  corrCount);
  for (int i = 0; i < corrCount; i++) {
    prefs.putFloat(("cr" + String(i)).c_str(), corrLUT[i].raw);
    prefs.putFloat(("cc" + String(i)).c_str(), corrLUT[i].corrected);
  }
}

void corrLoadFlash() {
  corrEnabled = prefs.getBool("corr_en", false);
  corrCount   = prefs.getInt ("corr_n",  0);
  for (int i = 0; i < corrCount && i < CORR_MAX_PTS; i++) {
    corrLUT[i].raw       = prefs.getFloat(("cr" + String(i)).c_str(), 0.0f);
    corrLUT[i].corrected = prefs.getFloat(("cc" + String(i)).c_str(), 0.0f);
  }
}

// Interpolazione lineare sulla spezzata — estremi fissi a 0% e 100%
float applyCorrection(float pctRaw) {
  if (!corrEnabled || corrCount < 2) return pctRaw;
  // Cerca i due punti che racchiudono pctRaw
  if (pctRaw <= corrLUT[0].raw)              return corrLUT[0].corrected;
  if (pctRaw >= corrLUT[corrCount-1].raw)    return corrLUT[corrCount-1].corrected;
  for (int i = 0; i < corrCount - 1; i++) {
    if (pctRaw >= corrLUT[i].raw && pctRaw <= corrLUT[i+1].raw) {
      float t = (pctRaw - corrLUT[i].raw) / (corrLUT[i+1].raw - corrLUT[i].raw);
      return corrLUT[i].corrected + t * (corrLUT[i+1].corrected - corrLUT[i].corrected);
    }
  }
  return pctRaw;
}

// Genera curva da formula: type 0=lineare, 1=concava (gamma<1), 2=convessa (gamma>1)
// param è il valore di gamma (es. 0.7 per concava, 1.4 per convessa)
void corrFromFormula(int npts, int type, float param) {
  corrCount = constrain(npts, 2, CORR_MAX_PTS);
  for (int i = 0; i < corrCount; i++) {
    float raw = (float)i / (corrCount - 1) * 100.0f;
    float corrected;
    float t = raw / 100.0f;
    switch (type) {
      case 1:  corrected = pow(t, param) * 100.0f; break;  // concava
      case 2:  corrected = pow(t, param) * 100.0f; break;  // convessa (param>1)
      default: corrected = raw; break;                      // lineare
    }
    corrLUT[i].raw       = raw;
    corrLUT[i].corrected = constrain(corrected, 0.0f, 100.0f);
  }
  corrEnabled = true;
  corrSaveFlash();
}

float sensorA3toOhm(float adcRaw) {
  if (!rk_calibrated || R_parallel <= 0.0f || K_ratio <= 0.0f) {
    float v = (adcRaw * 3.3f) / 4095.0f;
    if (v >= 3.25f) return 500.0f;
    return (v * 330.0f) / (3.3f - v);
  }
  float Va3real = max((adcRaw     * 3.3f / 4095.0f) - ADC_OFFSET_A3, 0.0f) * DIV_A3;
  float Va1real = max((filteredA1 * 3.3f / 4095.0f) - ADC_OFFSET_A1, 0.0f) * DIV_A1;
  float R9  = R_parallel / K_ratio;
  float R10 = R_parallel / (1.0f - K_ratio);
  float dV  = Va1real - Va3real;
  if (dV < 0.05f) return 2000.0f;
  float I       = dV / R9;
  float R_lower = Va3real / I;
  if (R10 <= R_lower + 0.1f) return 2000.0f;
  return (R_lower * R10) / (R10 - R_lower);
}

float sensorA4toOhm(float adcRaw) {
  float v = max((adcRaw * 3.3f) / 4095.0f - ADC_OFFSET_A4, 0.0f);
  if (v >= 3.05f) return 9999.0f;
  if (v <= 0.05f) return 0.0f;
  return (v * 330.0f) / (3.3f - v);
}

void initNMEA2000() {
  NMEA2000.SetProductInformation("001", 100, "Fuel", "6.0", "6.0");
  NMEA2000.SetDeviceInformation(1, 130, 25, 699);
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 4);
  NMEA2000.Open();
}

void sendN2kLevel(int level) {
  tN2kMsg msg;
  SetN2kFluidLevel(msg, 0, N2kft_Fuel, (double)level, N2kDoubleNA);
  NMEA2000.SendMsg(msg);
}

void processLevel(float ohm) {
  float pctRaw;
  if (sensorIsReed && reedStepCount > 0) {
    pctRaw = (float)reedLookup(ohm);
  } else {
    float range = ohm_vuoto - ohm_pieno;
    if (abs(range) < 1.0f) { fuelTankLevel = 0; return; }
    pctRaw = (ohm_vuoto - ohm) / range * 100.0f;
    pctRaw = constrain(pctRaw, 0.0f, 100.0f);
  }
  // Applica tabella di correzione (sia Reed che continuo)
  float pctCorrected = applyCorrection(pctRaw);
  fuelTankLevel = (int)constrain(pctCorrected, 0.0f, 100.0f);
}

void sendN2k(float ohm, const char* mode) {
  sendN2kLevel(fuelTankLevel);
  Serial.print("LIV: ");  Serial.print(fuelTankLevel);   Serial.print("% | ");
  Serial.print("R18: ");  Serial.print(ohm, 1);          Serial.print("Ω | ");
  Serial.print("vuoto="); Serial.print(ohm_vuoto, 1);    Serial.print("Ω ");
  Serial.print("pieno="); Serial.print(ohm_pieno, 1);    Serial.print("Ω | ");
  Serial.println(mode);
}

void pulsNmea(unsigned long now) {
  static unsigned long lastPulse = 0;
  static bool tog = false;
  if (now - lastPulse > 500) {
    sendN2kLevel(tog ? 100 : 0);
    tog = !tog;
    lastPulse = now;
  }
}

// ─── GESTIONE COMANDI (seriale + BLE) ────────────────────────────────────────
void handleCommand(const String& cmd) {
  // ── Comandi OTA ──────────────────────────────────────────────────────────
  if (cmd.startsWith("OTA_START:")) {
    otaExpectedSize = cmd.substring(10).toInt();
    otaPartition    = esp_ota_get_next_update_partition(nullptr);
    if (!otaPartition) {
      bleLog("!!! OTA: partizione non trovata");
      return;
    }
    esp_err_t err = esp_ota_begin(otaPartition, otaExpectedSize, &otaHandle);
    if (err != ESP_OK) {
      bleLog("!!! OTA begin error: " + String(esp_err_to_name(err)));
      otaState = OTA_ERROR;
      return;
    }
    otaReceivedSize = 0;
    otaState        = OTA_RECEIVING;
    bleLog(">>> OTA avviato: " + String(otaExpectedSize) + " byte attesi <<<");
    bleLog("    Partizione: " + String(otaPartition->label));
    return;
  }
  if (cmd == "OTA_END") {
    if (otaState != OTA_RECEIVING) {
      bleLog("!!! OTA_END fuori sequenza (stato=" + String(otaState) + ")");
      return;
    }
    bleLog("    Ricevuti: " + String(otaReceivedSize) + "/" + String(otaExpectedSize) + " byte");
    esp_err_t err = esp_ota_end(otaHandle);
    if (err != ESP_OK) {
      bleLog("!!! OTA end error: " + String(esp_err_to_name(err)));
      otaState = OTA_ERROR;
      return;
    }
    err = esp_ota_set_boot_partition(otaPartition);
    if (err != ESP_OK) {
      bleLog("!!! OTA set boot error: " + String(esp_err_to_name(err)));
      otaState = OTA_ERROR;
      return;
    }
    otaState = OTA_IDLE;
    bleLog(">>> OTA OK — riavvio tra 3s <<<");
    delay(3000);
    ESP.restart();
    return;
  }
  // OTA_ABORT — annulla OTA in corso
  if (cmd == "OTA_ABORT") {
    if (otaState == OTA_RECEIVING) {
      esp_ota_abort(otaHandle);
      otaState = OTA_IDLE;
      bleLog(">>> OTA annullato <<<");
    }
    return;
  }
  // OTA_VER? — richiede versione firmware corrente
  if (cmd == "OTA_VER?") {
    const esp_partition_t* running = esp_ota_get_running_partition();
    bleLog("{\"ver\":\"" + String(FW_VERSION) + "\",\"part\":\"" + String(running->label) + "\"}");
    return;
  }
  // ── Fine comandi OTA ──────────────────────────────────────────────────────

  if (cmd == "CONFIRM") {
    bleConfirmed = true;
    bleLog(">>> BLE confermato — rimane attivo <<<");
    return;
  }
  if (cmd == "RESET") {
    prefs.putFloat("R_parallel", 0.0f);
    prefs.putFloat("K_ratio",    0.0f);
    prefs.putBool ("rk_cal",    false);
    bleLog(">>> Calibrazione R// e K azzerata. Riavvio... <<<");
    delay(500); ESP.restart();
  }
  if (cmd.startsWith("VA1:")) {
    float vReal = cmd.substring(4).toFloat();
    if (vReal > 0.5f) {
      float vadc = rkVa1saved > 0.0f ? rkVa1saved : readVoltage(ADC_A1_PIN);
      DIV_A1 = vReal / vadc;
      prefs.putFloat("DIV_A1", DIV_A1);
      String msg = "    DIV_A1=" + String(DIV_A1, 4) + "  (reale=" + String(vReal, 3) + "V)";
      bleLog(msg);
    }
  }
  if (cmd.startsWith("VA3:")) {
    float vReal = cmd.substring(4).toFloat();
    if (vReal > 0.5f) {
      float vadc = rkVa3saved > 0.0f ? rkVa3saved : readVoltage(ADC_A3_PIN);
      DIV_A3 = vReal / vadc;
      prefs.putFloat("DIV_A3", DIV_A3);
      String msg = "    DIV_A3=" + String(DIV_A3, 4) + "  (reale=" + String(vReal, 3) + "V)";
      bleLog(msg);
    }
  }
  // RK:r9,r10 — imposta R9 e R10 manualmente da misura multimetro
  if (cmd.startsWith("RK:")) {
    int comma = cmd.indexOf(',', 3);
    if (comma > 3) {
      float r9  = cmd.substring(3, comma).toFloat();
      float r10 = cmd.substring(comma + 1).toFloat();
      if (r9 > 1.0f && r10 > 1.0f) {
        // Ricava R// e K da R9 e R10
        R_parallel = (r9 * r10) / (r9 + r10);
        K_ratio    = r10 / (r9 + r10);
        rk_calibrated = true;
        prefs.putFloat("R_parallel", R_parallel);
        prefs.putFloat("K_ratio",    K_ratio);
        prefs.putBool ("rk_cal",     true);
        String msg = ">>> RK manuale: R9=" + String(r9, 1) + " R10=" + String(r10, 1)
                   + " R//=" + String(R_parallel, 2) + " K=" + String(K_ratio, 4) + " <<<";
        bleLog(msg);
      }
    }
  }
  if (cmd == "GO" && rkPhase == RK_WAIT_GO) {
    bleLog(">>> GO — proseguo <<<");
    bleLog("    Va1 reale=" + String(rkVa1saved * DIV_A1, 4) + "V");
    bleLog("    Va3 reale=" + String(rkVa3saved * DIV_A3, 4) + "V");
    digitalWrite(GPIO_Q1_PIN, LOW);
    rkPhase = RK_WAIT_LOW;
    bleLog(">>> Q1 OFF — scollegare 12V <<<");
  }
  if (cmd == "DIAG") {
    bleLog("[DIAGNOSTICA]");
    printADC("A1", ADC_A1_PIN);
    printADC("A3", ADC_A3_PIN);
    printADC("A4", ADC_A4_PIN);
    bleLog("[fine]");
  }
  // REED:enable,steps,hysteresis — configura tipo Reed
  if (cmd.startsWith("REED:")) {
    // formato: REED:1,12,5.0  (enable, nsteps, isteresi)
    int c1 = cmd.indexOf(',', 5);
    int c2 = cmd.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > 0) {
      sensorIsReed   = cmd.substring(5, c1).toInt() == 1;
      reedStepCount  = constrain(cmd.substring(c1+1, c2).toInt(), 0, REED_MAX_STEPS);
      reedHysteresis = cmd.substring(c2+1).toFloat();
      // Azzera LUT quando si riconfigura
      for (int i = 0; i < REED_MAX_STEPS; i++) { reedLUT[i].ohm = 0; reedLUT[i].percent = 0; }
      reedLastStep = 0;
      reedSaveFlash();
      bleLog(">>> Reed: " + String(sensorIsReed?"ON":"OFF")
           + " steps=" + String(reedStepCount)
           + " hys=" + String(reedHysteresis, 1) + "Ω <<<");
    }
  }
  // REEDSTEP:idx,percent — registra lo step corrente nella LUT
  // L'ohm viene letto dal sensore in tempo reale
  if (cmd.startsWith("REEDSTEP:")) {
    int comma = cmd.indexOf(',', 9);
    if (comma > 9) {
      int   idx  = cmd.substring(9, comma).toInt();
      int   pct  = cmd.substring(comma + 1).toInt();
      if (idx >= 0 && idx < REED_MAX_STEPS) {
        float ohm = (currentState == MODE_NORMAL_NMEA) ?
                    sensorA4toOhm(filteredA4) : sensorA3toOhm(filteredA3);
        reedLUT[idx].ohm     = ohm;
        reedLUT[idx].percent = pct;
        reedSaveFlash();
        bleLog(">>> Reed step " + String(idx) + ": " + String(ohm, 1) + "Ω = " + String(pct) + "% <<<");
      }
    }
  }
  // REEDLUT? — richiede la LUT completa (per sincronizzare l'app)
  if (cmd == "REEDLUT?") {
    String lut = "{\"reed\":" + String(sensorIsReed?1:0)
               + ",\"n\":" + String(reedStepCount)
               + ",\"hys\":" + String(reedHysteresis, 1)
               + ",\"lut\":[";
    for (int i = 0; i < reedStepCount; i++) {
      lut += "[" + String(reedLUT[i].ohm, 1) + "," + String(reedLUT[i].percent) + "]";
      if (i < reedStepCount - 1) lut += ",";
    }
    lut += "]}";
    bleLog(lut);
  }
  // ── Comandi tabella correzione ───────────────────────────────────────────
  // CORR:n — imposta numero punti e abilita correzione (punti da definire)
  if (cmd.startsWith("CORR:")) {
    int n = constrain(cmd.substring(5).toInt(), 2, CORR_MAX_PTS);
    corrCount   = n;
    corrEnabled = false; // disabilitata finché non completata
    // Inizializza con curva lineare come base
    for (int i = 0; i < n; i++) {
      corrLUT[i].raw       = (float)i / (n-1) * 100.0f;
      corrLUT[i].corrected = corrLUT[i].raw;
    }
    corrSaveFlash();
    bleLog(">>> Correzione: " + String(n) + " punti inizializzati (lineare) <<<");
  }
  // CORRPT:idx,raw,corrected — imposta un punto della tabella
  if (cmd.startsWith("CORRPT:")) {
    int c1 = cmd.indexOf(',', 7);
    int c2 = cmd.indexOf(',', c1+1);
    if (c1 > 0 && c2 > 0) {
      int   idx  = cmd.substring(7, c1).toInt();
      float raw  = cmd.substring(c1+1, c2).toFloat();
      float corr = cmd.substring(c2+1).toFloat();
      if (idx >= 0 && idx < corrCount) {
        corrLUT[idx].raw       = constrain(raw,  0.0f, 100.0f);
        corrLUT[idx].corrected = constrain(corr, 0.0f, 100.0f);
        corrEnabled = true;
        corrSaveFlash();
        bleLog(">>> Corr pt " + String(idx) + ": " + String(raw,1) + "% → " + String(corr,1) + "% <<<");
      }
    }
  }
  // CORRFN:type,param,npts — genera curva da formula
  // type: 0=lineare, 1=concava (param<1), 2=convessa (param>1)
  if (cmd.startsWith("CORRFN:")) {
    int c1 = cmd.indexOf(',', 7);
    int c2 = cmd.indexOf(',', c1+1);
    if (c1 > 0 && c2 > 0) {
      int   type  = cmd.substring(7, c1).toInt();
      float param = cmd.substring(c1+1, c2).toFloat();
      int   npts  = cmd.substring(c2+1).toInt();
      corrFromFormula(npts, type, param);
      bleLog(">>> Curva formula: tipo=" + String(type) + " gamma=" + String(param,2) + " pts=" + String(npts) + " <<<");
    }
  }
  // CORR? — invia tabella correzione all'app
  if (cmd == "CORR?") {
    String s = "{\"corr\":" + String(corrEnabled?1:0) + ",\"n\":" + String(corrCount) + ",\"pts\":[";
    for (int i = 0; i < corrCount; i++) {
      s += "[" + String(corrLUT[i].raw,1) + "," + String(corrLUT[i].corrected,1) + "]";
      if (i < corrCount-1) s += ",";
    }
    s += "]}";
    bleLog(s);
  }
  // CORR_EN:0/1 — abilita/disabilita correzione senza modificarla
  if (cmd.startsWith("CORR_EN:")) {
    corrEnabled = cmd.substring(8).toInt() == 1;
    corrSaveFlash();
    bleLog(">>> Correzione " + String(corrEnabled?"abilitata":"disabilitata") + " <<<");
  }
  // CORR_RESET — azzera tabella correzione
  if (cmd == "CORR_RESET") {
    corrEnabled = false;
    corrCount   = 0;
    corrSaveFlash();
    bleLog(">>> Tabella correzione azzerata <<<");
  }
  // REED_RESET — azzera tutta la LUT Reed
  if (cmd == "REED_RESET") {
    sensorIsReed  = false;
    reedStepCount = 0;
    for (int i = 0; i < REED_MAX_STEPS; i++) { reedLUT[i].ohm = 0; reedLUT[i].percent = 0; }
    reedSaveFlash();
    bleLog(">>> LUT Reed azzerata <<<");
  }
  // OFF:o1,o3 — imposta offset manualmente e salva subito in flash
  if (cmd.startsWith("OFF:")) {
    int comma = cmd.indexOf(',', 4);
    if (comma > 4) {
      float o1 = cmd.substring(4, comma).toFloat();
      float o3 = cmd.substring(comma + 1).toFloat();
      ADC_OFFSET_A1 = o1;
      ADC_OFFSET_A3 = o3;
      prefs.putFloat("OFF_A1", ADC_OFFSET_A1);
      prefs.putFloat("OFF_A3", ADC_OFFSET_A3);
      bleLog(">>> Offset manuali salvati: A1=" + String(ADC_OFFSET_A1, 4) + " A3=" + String(ADC_OFFSET_A3, 4) + " <<<");
    }
  }
  if (cmd == "OFFSET") {
    bleLog(">>> CALIBRAZIONE OFFSET — collegare 8V su A1 e A3 <<<");
    delay(2000);
    double accA1 = 0, accA3 = 0;
    for (int i = 0; i < 100; i++) {
      accA1 += analogRead(ADC_A1_PIN);
      accA3 += analogRead(ADC_A3_PIN);
      delay(10);
    }
    float vA1raw = (accA1 / 100.0f) * 3.3f / 4095.0f;
    float vA3raw = (accA3 / 100.0f) * 3.3f / 4095.0f;
    float vA1exp = 8.0f * R_A1_LOW / (R_A1_HIGH + R_A1_LOW);
    float vA3exp = 8.0f * R_A3_LOW / (R_A3_HIGH + R_A3_LOW);
    ADC_OFFSET_A1 = vA1raw - vA1exp;
    ADC_OFFSET_A3 = vA3raw - vA3exp;
    prefs.putFloat("OFF_A1", ADC_OFFSET_A1);
    prefs.putFloat("OFF_A3", ADC_OFFSET_A3);
    bleLog("    A1: raw=" + String(vA1raw, 4) + "V atteso=" + String(vA1exp, 4) + "V offset=" + String(ADC_OFFSET_A1, 4));
    bleLog("    A3: raw=" + String(vA3raw, 4) + "V atteso=" + String(vA3exp, 4) + "V offset=" + String(ADC_OFFSET_A3, 4));
    bleLog("    A1 corretto=" + String(vA1raw - ADC_OFFSET_A1, 4) + "V");
    bleLog("    A3 corretto=" + String(vA3raw - ADC_OFFSET_A3, 4) + "V");
    bleLog(">>> Offset salvati <<<");
  }
}

// ─── SETUP ───────────────────────────────────────────────────────────────────
void setup() {
  pinMode(LED_PIN,     OUTPUT);
  pinMode(GPIO_Q1_PIN, OUTPUT);
  digitalWrite(GPIO_Q1_PIN, LOW);
  pinMode(ADC_A1_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && millis() - t < 3000);

  prefs.begin("n2k_cal", false);
  ohm_vuoto     = prefs.getFloat("ohm_vuoto",  240.0f);
  ohm_pieno     = prefs.getFloat("ohm_pieno",   32.0f);
  R_parallel    = prefs.getFloat("R_parallel",   0.0f);
  K_ratio       = prefs.getFloat("K_ratio",      0.0f);
  rk_calibrated = prefs.getBool ("rk_cal",      false);
  DIV_A1        = prefs.getFloat("DIV_A1", (R_A1_HIGH + R_A1_LOW) / R_A1_LOW);
  DIV_A3        = prefs.getFloat("DIV_A3", (R_A3_HIGH + R_A3_LOW) / R_A3_LOW);
  ADC_OFFSET_A1 = prefs.getFloat("OFF_A1", ADC_OFFSET_A1);
  ADC_OFFSET_A3 = prefs.getFloat("OFF_A3", ADC_OFFSET_A3);
  reedLoadFlash();
  corrLoadFlash();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(ADC_A1_PIN, ADC_11db);
  analogSetPinAttenuation(ADC_A3_PIN, ADC_11db);
  analogSetPinAttenuation(ADC_A4_PIN, ADC_11db);

  // Warmup ADC
  delay(1000);
  float a1=0, a3=0, a4=0;
  for (int i = 0; i < 64; i++) {
    a1 += analogRead(ADC_A1_PIN);
    a3 += analogRead(ADC_A3_PIN);
    a4 += analogRead(ADC_A4_PIN);
    delay(5);
  }
  filteredA1 = a1 / 64.0f;
  filteredA3 = a3 / 64.0f;
  filteredA4 = a4 / 64.0f;

  Serial.println("--- FUEL SENSOR V6.0 ---");
  Serial.println("[DIAGNOSTICA AVVIO — post warmup]");
  Serial.print("  A1: filtered="); Serial.print(filteredA1, 0);
  Serial.print("  vA1="); Serial.print(max((filteredA1*3.3f/4095.0f)-ADC_OFFSET_A1,0.0f),3); Serial.println("V");
  Serial.print("  A3: filtered="); Serial.print(filteredA3, 0);
  Serial.print("  vA3="); Serial.print(max((filteredA3*3.3f/4095.0f)-ADC_OFFSET_A3,0.0f),3); Serial.println("V");
  Serial.print("  A4: filtered="); Serial.print(filteredA4, 0);
  Serial.print("  vA4="); Serial.print(max((filteredA4*3.3f/4095.0f)-ADC_OFFSET_A4,0.0f),3); Serial.println("V");
  Serial.println("[fine diagnostica]");
  Serial.print("R//="); Serial.print(R_parallel, 2);
  Serial.print("  K="); Serial.print(K_ratio, 4);
  Serial.print("  rk_cal="); Serial.println(rk_calibrated ? "YES" : "NO");
  Serial.print("vuoto="); Serial.print(ohm_vuoto, 1);
  Serial.print("Ω  pieno="); Serial.println(ohm_pieno, 1);
  Serial.print("OFF_A1="); Serial.print(ADC_OFFSET_A1, 4);
  Serial.print("  OFF_A3="); Serial.println(ADC_OFFSET_A3, 4);
  Serial.println("Comandi: RESET | VA1:xx.xx | VA3:xx.xx | GO | DIAG | OFFSET | RK:r9,r10");

  // Avvia BLE
  initBLE();

  stateTimer = millis();
  initNMEA2000();
}

// ─── LOOP ────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── Handshake BLE: spegni se non confermato entro 10s ────────────────────
  if (bleActive && !bleConfirmed && (now - bleStartTime > BLE_CONFIRM_TIMEOUT)) {
    stopBLE();
  }

  // ── Comandi seriali ───────────────────────────────────────────────────────
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handleCommand(cmd);
  }

  // ── Comandi BLE ──────────────────────────────────────────────────────────
  if (bleCmdPending) {
    bleCmdPending = false;
    handleCommand(bleCmdBuffer);
  }

  // ── Se OTA in corso: processa solo comandi BLE, sospendi tutto il resto ──
  if (otaState == OTA_RECEIVING || otaState == OTA_ERROR) {
    if (bleCmdPending) {
      bleCmdPending = false;
      handleCommand(bleCmdBuffer);
    }
    return;
  }

  // ── Filtro ADC ────────────────────────────────────────────────────────────
  if (now - lastFilterTime >= FILTER_MS) {
    bool isNormal = (currentState == MODE_NORMAL_ANALOG || currentState == MODE_NORMAL_NMEA);
    float alpha = isNormal ? alpha_normal : alpha_cal;
    filteredA1 = alpha * analogRead(ADC_A1_PIN) + (1.0f - alpha) * filteredA1;
    filteredA3 = alpha * analogRead(ADC_A3_PIN) + (1.0f - alpha) * filteredA3;
    filteredA4 = alpha * analogRead(ADC_A4_PIN) + (1.0f - alpha) * filteredA4;
    lastFilterTime = now;
  }

  float vA1 = max((filteredA1 * 3.3f) / 4095.0f - ADC_OFFSET_A1, 0.0f);
  float vA3 = max((filteredA3 * 3.3f) / 4095.0f - ADC_OFFSET_A3, 0.0f);
  float vA4 = max((filteredA4 * 3.3f) / 4095.0f - ADC_OFFSET_A4, 0.0f);

  float VA1_reale = vA1 * DIV_A1;
  float VA3_reale = vA3 * DIV_A3;
  float tA3_18   = threshA3(VA1_reale, 1.8f);
  float tA3_10   = threshA3(VA1_reale, 1.0f);

  NMEA2000.ParseMessages();

  // ── Invio dati BLE ────────────────────────────────────────────────────────
  float r18cur = (currentState == MODE_NORMAL_ANALOG) ? sensorA3toOhm(filteredA3) :
                 (currentState == MODE_NORMAL_NMEA)   ? sensorA4toOhm(filteredA4) : 0.0f;
  bleSendData(VA1_reale, VA3_reale, filteredA4, r18cur);

  // ════════════════════════════════════════════════════════════════════════════
  switch (currentState) {

  // ── MODE_WAIT_START ────────────────────────────────────────────────────────
  case MODE_WAIT_START: {
    static bool          wsSeen  = false;
    static unsigned long wsTimer = 0;
    static int           wsMode  = 0;

    int mode = 0;
    if      (vA1 < 1.0f && vA3 < tA3_10  && vA4 > 2.0f) mode = 1;
    else if (vA1 < 1.0f && vA3 > tA3_10  && vA4 > 2.0f) mode = 2;
    else if (vA1 > 1.0f && vA4 > 2.0f && vA3 < tA3_18)  mode = 3;
    else if (vA1 > 1.0f && vA4 > 2.0f && vA3 > tA3_18)  mode = 5;
    else if (vA1 < 1.0f && vA3 > tA3_18  && vA4 < 2.0f) mode = 4;

    if (mode != wsMode) {
      wsMode  = mode;
      wsSeen  = false;
      wsTimer = now;
      switch (mode) {
        case 1: bleLog(">>> A1<1V A3<1V A4>2V: attendo 4s per misura R//K <<<"); break;
        case 2: bleLog(">>> A1<1V A3>1V A4>2V: attendo 4s per calibrazione SOLO NMEA <<<"); break;
        case 3: bleLog(">>> A1>1V A3<1.8V A4>2V: attendo 1s per modalita' NMEA+Analogico <<<"); break;
        case 4: bleLog(">>> A1<1V A3>1.8V A4<2V: attendo 1s per modalita' SOLO NMEA <<<"); break;
        case 5: bleLog(">>> A3 > 1.8V all'avvio: attendo 4s per calibrazione NMEA+Analogico <<<"); break;
        default: break;
      }
    }

    if (mode == 1 || mode == 2 || mode == 5) {
      pulsNmea(now);
      digitalWrite(LED_PIN, (now / 500) % 2);
    } else {
      sendN2kLevel(0);
      digitalWrite(LED_PIN, LOW);
    }

    unsigned long timeout = (mode == 1 || mode == 2 || mode == 5) ? 4000 : 1000;
    if (mode > 0 && now - wsTimer > timeout) {
      switch (mode) {
        case 1:
          currentState = MODE_RK_MEASURE;
          rkPhase      = RK_WAIT_HIGH;
          rkHighSeen   = false;
          calNmeaOnly  = false;
          sendN2kLevel(0);
          bleLog(">>> MISURA R//K — collegare 12V con strumento cortocircuitato <<<");
          break;
        case 2:
          currentState = MODE_WAIT_CAL;
          calNmeaOnly  = true;
          bleLog(">>> CALIBRAZIONE SOLO NMEA — collegare il sensore su A4 <<<");
          break;
        case 3:
          currentState = MODE_NORMAL_ANALOG;
          bleLog(">>> MODALITA' NMEA+ANALOGICO <<<");
          break;
        case 4:
          currentState = MODE_NORMAL_NMEA;
          bleLog(">>> MODALITA' SOLO NMEA <<<");
          break;
        case 5:
          currentState = MODE_WAIT_CAL;
          calNmeaOnly  = false;
          isStabilized = false;
          bleLog(">>> CALIBRAZIONE NMEA+ANALOGICO — collegare il sensore su A3 <<<");
          break;
      }
      wsSeen = false;
      wsMode = 0;
    }
    break;
  }

  // ── MODE_RK_MEASURE ────────────────────────────────────────────────────────
  case MODE_RK_MEASURE: {

    if (rkPhase == RK_WAIT_HIGH) {
      pulsNmea(now);
      digitalWrite(LED_PIN, (now / 500) % 2);
      if (vA1 > 1.0f) {
        if (!rkHighSeen) {
          rkHighSeen = true; rkHighTimer = now;
          bleLog(">>> A1 > 1V: attendo 4s stabili poi misuro R// <<<");
        }
        if (now - rkHighTimer > 4000) {
          rkHighSeen = false;
          rkPhase    = RK_MEASURE_RPAR;
          rkTimer    = now; rkSamples = 0; rkAccA1 = 0.0; rkAccA3 = 0.0;
          sendN2kLevel(100);
          digitalWrite(GPIO_Q1_PIN, HIGH);
          bleLog(">>> Q1 ON — stabilizzazione 1s poi 100 campioni <<<");
        }
      } else { rkHighSeen = false; }
    }

    else if (rkPhase == RK_MEASURE_RPAR) {
      sendN2kLevel(100);
      digitalWrite(LED_PIN, HIGH);
      if (now - rkTimer < 1000) break;
      if (rkSamples < 100) {
        rkAccA1 += readVoltage(ADC_A1_PIN);
        rkAccA3 += readVoltage(ADC_A3_PIN);
        rkSamples++;
      } else {
        rkVa1saved = (float)(rkAccA1 / 100.0);
        rkVa3saved = (float)(rkAccA3 / 100.0);
        bleLog(">>> Fase 1 completata <<<");
        bleLog("    A1: ADC=" + String(rkVa1saved, 4) + "V  reale=" + String(rkVa1saved * DIV_A1, 3) + "V");
        bleLog("    A3: ADC=" + String(rkVa3saved, 4) + "V  reale=" + String(rkVa3saved * DIV_A3, 3) + "V");
        if (DEBUG_WAIT_GO) {
          bleLog("    Digita VA1/VA3 per correggere, poi GO per proseguire");
          rkPhase = RK_WAIT_GO;
        } else {
          digitalWrite(GPIO_Q1_PIN, LOW);
          rkPhase = RK_WAIT_LOW;
          bleLog(">>> Q1 OFF — scollegare 12V <<<");
        }
      }
    }

    else if (rkPhase == RK_WAIT_GO) {
      sendN2kLevel(100);
      digitalWrite(LED_PIN, (now / 200) % 2);
    }

    else if (rkPhase == RK_WAIT_LOW) {
      pulsNmea(now);
      digitalWrite(LED_PIN, (now / 200) % 2);
      if (vA1 < 1.0f) {
        if (!rkLowSeen) { rkLowSeen = true; rkLowTimer = now; }
        if (now - rkLowTimer > 4000) {
          rkLowSeen = false; rkPhase = RK_WAIT_HIGH2; rkHigh2Seen = false;
          bleLog(">>> A1 < 1V — collegare R11 a massa poi 12V <<<");
        }
      } else { rkLowSeen = false; }
    }

    else if (rkPhase == RK_WAIT_HIGH2) {
      pulsNmea(now);
      digitalWrite(LED_PIN, (now / 1000) % 2);
      if (vA1 > 1.0f) {
        if (!rkHigh2Seen) {
          rkHigh2Seen = true; rkHigh2Timer = now;
          bleLog(">>> A1 > 1V: attendo 4s stabili poi misuro K <<<");
        }
        if (now - rkHigh2Timer > 4000) {
          rkHigh2Seen = false;
          rkPhase = RK_MEASURE_K;
          rkTimer = now; rkSamples = 0; rkAccA1 = 0.0; rkAccA3 = 0.0;
          bleLog(">>> Stabilizzazione 1s poi 100 campioni per K <<<");
        }
      } else { rkHigh2Seen = false; }
    }

    else if (rkPhase == RK_MEASURE_K) {
      pulsNmea(now);
      digitalWrite(LED_PIN, HIGH);
      if (now - rkTimer < 1000) break;
      if (rkSamples < 100) {
        rkAccA1 += readVoltage(ADC_A1_PIN);
        rkAccA3 += readVoltage(ADC_A3_PIN);
        rkSamples++;
      } else {
        float vA1avg  = (float)(rkAccA1 / 100.0);
        float vA3avg  = (float)(rkAccA3 / 100.0);
        float vA1real = vA1avg * DIV_A1;
        float vA3real = vA3avg * DIV_A3;
        bleLog(">>> Fase 2 completata:");
        bleLog("    A1: ADC=" + String(vA1avg, 4) + "V  reale=" + String(vA1real, 4) + "V");
        bleLog("    A3: ADC=" + String(vA3avg, 4) + "V  reale=" + String(vA3real, 4) + "V");

        K_ratio    = constrain(vA3real / vA1real, 0.01f, 0.99f);
        float Va1r = rkVa1saved * DIV_A1;
        float Va3r = rkVa3saved * DIV_A3;
        R_parallel = (Va3r > 0.1f) ? (330.0f * (Va1r - Va3r) / Va3r) : 0.0f;

        bleLog(">>> R9="  + String(R_parallel / K_ratio, 2)          + " Ω");
        bleLog(">>> R10=" + String(R_parallel / (1.0f - K_ratio), 2) + " Ω");
        bleLog(">>> R//=" + String(R_parallel, 2)                     + " Ω");
        bleLog(">>> K="   + String(K_ratio, 4));

        rk_calibrated = true;
        prefs.putFloat("R_parallel", R_parallel);
        prefs.putFloat("K_ratio",    K_ratio);
        prefs.putBool ("rk_cal",     true);
        rkPhase = RK_DONE;
        sendN2kLevel(0);
        bleLog(">>> Salvato — spegnere e riavviare per taratura livello <<<");
        for (int i = 0; i < 15; i++) { digitalWrite(LED_PIN, i % 2); delay(200); }
        digitalWrite(LED_PIN, LOW);
      }
    }

    else if (rkPhase == RK_DONE) {
      sendN2kLevel(0);
      digitalWrite(LED_PIN, LOW);
    }
    break;
  }

  // ── MODE_WAIT_CAL ─────────────────────────────────────────────────────────
  case MODE_WAIT_CAL: {
    float vSens  = calNmeaOnly ? vA4 : vA3;
    float thresh = calNmeaOnly ? 2.0f : tA3_18;

    if (vSens >= thresh) {
      isStabilized = false;
      stabilizationTimer = now;
      pulsNmea(now);
      digitalWrite(LED_PIN, (now / 500) % 2);
    } else {
      sendN2kLevel(100);
      digitalWrite(LED_PIN, HIGH);
      if (!isStabilized) {
        isStabilized       = true;
        stabilizationTimer = now;
        bleLog(">>> Sensore collegato — attendo 4s in posizione VUOTO <<<");
      }
      if (now - stabilizationTimer > 4000) {
        float ohm = calNmeaOnly ? sensorA4toOhm(filteredA4) : sensorA3toOhm(filteredA3);
        ohm_vuoto    = ohm;
        ohm_pieno    = ohm;
        peakTimer    = now;
        isStabilized = false;
        currentState = MODE_CALIBRATION;
        bleLog(">>> VUOTO memorizzato: " + String(ohm_vuoto, 1) + " Ω — muovere verso PIENO <<<");
      }
    }
    break;
  }

  // ── MODE_CALIBRATION ──────────────────────────────────────────────────────
  case MODE_CALIBRATION: {
    pulsNmea(now);
    digitalWrite(LED_PIN, (now / 500) % 2);

    float ohm = calNmeaOnly ? sensorA4toOhm(filteredA4) : sensorA3toOhm(filteredA3);
    bool  nr  = false;

    if (abs(ohm - ohm_vuoto) > abs(ohm_pieno - ohm_vuoto) + 1.0f && ohm < 400.0f) {
      ohm_pieno = ohm;
      nr = true;
    }
    if (nr) {
      peakTimer = now;
      bleLog("  pieno aggiornato: " + String(ohm_pieno, 1) + " Ω");
    }

    static unsigned long lastTick = 0;
    if (now - lastTick > 1000) {
      int tLeft = 4 - (int)((now - peakTimer) / 1000);
      if (tLeft >= 0) bleLog("Fine tra: " + String(tLeft) + "s");
      lastTick = now;
    }

    if (now - peakTimer > 4000) {
      prefs.putFloat("ohm_vuoto", ohm_vuoto);
      prefs.putFloat("ohm_pieno", ohm_pieno);
      sendN2kLevel(0);
      bleLog(">>> Calibrazione completata: vuoto=" + String(ohm_vuoto, 1)
           + " Ω  pieno=" + String(ohm_pieno, 1)
           + (ohm_pieno < ohm_vuoto ? " Ω  tipo=USA" : " Ω  tipo=EUR"));
      digitalWrite(LED_PIN, HIGH); delay(2000);
      ESP.restart();
    }
    break;
  }

  // ── MODE_NORMAL_ANALOG ────────────────────────────────────────────────────
  case MODE_NORMAL_ANALOG: {
    static unsigned long disconnectTimer = 0;

    if (vA3 > tA3_18) {
      if (disconnectTimer == 0) {
        disconnectTimer = now;
        bleLog(">>> A3 > 1.8V: sensore scollegato <<<");
      }
      if (now - disconnectTimer > 4000) {
        pulsNmea(now);
        digitalWrite(LED_PIN, (now / 200) % 2);
        static unsigned long lastWarn = 0;
        if (now - lastWarn > 5000) {
          bleLog("!!! SENSORE SCOLLEGATO — ricollegare il sensore");
          lastWarn = now;
        }
      } else {
        digitalWrite(LED_PIN, (now / 500) % 2);
        sendN2kLevel(0);
      }
    } else {
      disconnectTimer = 0;
      digitalWrite(LED_PIN, (now / 1000) % 2);
      static unsigned long lastSend = 0;
      if (now - lastSend > 2500) {
        float r18 = sensorA3toOhm(filteredA3);
        processLevel(r18);
        sendN2k(r18, "NMEA+ANALOG");
        lastSend = now;
      }
    }
    break;
  }

  // ── MODE_NORMAL_NMEA ──────────────────────────────────────────────────────
  case MODE_NORMAL_NMEA: {
    float ohmMax = max(ohm_vuoto, ohm_pieno);
    float margin = abs(ohm_vuoto - ohm_pieno) * 0.5f;
    float r18    = sensorA4toOhm(filteredA4);
    bool  ok     = (r18 < ohmMax + margin);

    static unsigned long disconnectTimer4 = 0;
    static bool          wasDisconnected4 = false;
    if (!ok) {
      if (disconnectTimer4 == 0) disconnectTimer4 = now;
      if (now - disconnectTimer4 < 5000) ok = true;
    } else {
      disconnectTimer4 = 0;
      wasDisconnected4 = false;
    }
    if (!ok && now - disconnectTimer4 >= 5000) wasDisconnected4 = true;

    if (!ok) {
      pulsNmea(now);
      digitalWrite(LED_PIN, (now / 200) % 2);
      static unsigned long lastWarn4 = 0;
      if (now - lastWarn4 > 2500) {
        bleLog("!!! SENSORE SCOLLEGATO: R18=" + String(r18, 1) + "Ω");
        lastWarn4 = now;
      }
    } else {
      digitalWrite(LED_PIN, (now / 1000) % 2);
      static unsigned long lastSend4 = 0;
      if (now - lastSend4 > 2500) {
        processLevel(r18);
        sendN2k(r18, "SOLO NMEA");
        lastSend4 = now;
      }
    }
    break;
  }

  } // fine switch
} // fine loop
