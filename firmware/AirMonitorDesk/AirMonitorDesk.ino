#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_AHTX0.h>
#include "ScioSense_ENS160.h"
#include "esp_system.h"
#include <math.h>

// =====================================================
// Debug switch
// =====================================================
// 0 = formal daily-use mode.
//     Serial is completely disabled, so UART0 is free for ZE08-CH2O.
// 1 = debug mode.
//     If you set this to 1 on ESP32-C3, enable "USB CDC On Boot" when uploading.
#define ENABLE_DEBUG 0

#if ENABLE_DEBUG
  #define DBG_BEGIN(baud)      do { Serial.begin(baud); delay(1000); } while (0)
  #define DBG_PRINT(...)       Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...)     Serial.println(__VA_ARGS__)
#else
  #define DBG_BEGIN(baud)      do {} while (0)
  #define DBG_PRINT(...)       do {} while (0)
  #define DBG_PRINTLN(...)     do {} while (0)
#endif

// =====================================================
// Basic configuration
// =====================================================
#define DEVICE_NAME "AirMonitor-C3"

static const uint32_t DEBUG_BAUD        = 115200;
static const uint32_t SENSOR_BAUD       = 9600;

static const uint32_t PMS_WARMUP_MS     = 30000UL;
static const uint32_t ENV_UPDATE_MS     = 1000UL;
static const uint32_t DISPLAY_UPDATE_MS = 1500UL;
static const uint32_t BLE_NOTIFY_MS     = 2000UL;
static const uint32_t HEARTBEAT_MS      = 5000UL;

static const uint32_t I2C_CLOCK_HZ      = 100000UL;
static const uint32_t I2C_TIMEOUT_MS    = 50UL;

static const int UART_READ_BUDGET_BYTES = 128;

// =====================================================
// OLED: 1.4-inch SH1106 @ 0x3C
// =====================================================
#define OLED_ADDR      0x3C
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64

Adafruit_SH1106G oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// =====================================================
// BLE UUIDs
// =====================================================
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHAR_UUID_DATA      "12345678-1234-1234-1234-1234567890ac"
#define CHAR_UUID_STATUS    "12345678-1234-1234-1234-1234567890ad"

BLECharacteristic* pDataChar = nullptr;
BLECharacteristic* pStatusChar = nullptr;

volatile bool bleConnected = false;
volatile bool needRestartAdvertising = false;

// =====================================================
// Pin definitions
// =====================================================
static const int PIN_CH2O_RX  = 6;    // ZE08-CH2O TXD -> ESP32 RX
static const int PIN_CH2O_TX  = 7;    // ESP32 TX -> ZE08-CH2O RX

static const int PIN_PMS_RX   = 3;    // PMS7003 TXD -> ESP32 RX
static const int PIN_PMS_TX   = 4;    // PMS7003 RXD -> ESP32 TX

static const int PIN_I2C_SDA  = 0;
static const int PIN_I2C_SCL  = 1;

// =====================================================
// Hardware serial ports
// =====================================================
// UART0 is used only for ZE08-CH2O.
// UART1 is used only for PMS7003.
// Serial debug is disabled by default through ENABLE_DEBUG = 0.
HardwareSerial uartCH2O(0);
HardwareSerial uartPMS(1);

// =====================================================
// ENS160 + AHT20
// =====================================================
Adafruit_AHTX0 aht;
ScioSense_ENS160 ens160(ENS160_I2CADDR_1);

// =====================================================
// State variables
// =====================================================
bool oledOK = false;
bool ahtOK  = false;
bool ensOK  = false;

uint32_t lastEnvUpdate     = 0;
uint32_t lastDisplayUpdate = 0;
uint32_t lastBleNotify     = 0;
uint32_t lastHeartbeat     = 0;

uint32_t pmsPowerOnMs      = 0;
bool pmsWarmedUp           = false;
uint32_t bleSeq            = 0;

float envTempC = 25.0f;
float envHumRH = 0.0f;

uint16_t ensAQI  = 0;
uint16_t ensTVOC = 0;
uint16_t enseCO2 = 0;

uint16_t ch2oPpb = 0;
float ch2oPpm = 0.0f;
float ch2oMgM3 = 0.0f;

struct PMSData {
  uint16_t pm1_0_atm = 0;
  uint16_t pm2_5_atm = 0;
  uint16_t pm10_atm  = 0;
  bool valid = false;
  uint32_t lastUpdateMs = 0;
};

PMSData pmsData;

// =====================================================
// Reset reason helper
// =====================================================
const char* resetReasonToText(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "OTHER";
  }
}

// =====================================================
// BLE callbacks
// =====================================================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleConnected = true;
    needRestartAdvertising = false;

    DBG_PRINTLN("BLE client connected");

    if (pStatusChar) {
      pStatusChar->setValue("CONNECTED");
    }
  }

  void onDisconnect(BLEServer* pServer) override {
    bleConnected = false;
    needRestartAdvertising = true;

    DBG_PRINTLN("BLE client disconnected");

    if (pStatusChar) {
      pStatusChar->setValue("DISCONNECTED");
    }
  }
};

// =====================================================
// Unit conversion functions
// =====================================================
float ch2oPpmToMgM3(float ppm, float tempC = 25.0f) {
  if (ppm < 0) return 0.0f;

  const float MW = 30.026f;
  float kelvin = tempC + 273.15f;
  float molarVolume = 22.414f * kelvin / 273.15f;

  return ppm * MW / molarVolume;
}

float co2PpmToMgM3(float ppm, float tempC = 25.0f) {
  if (ppm < 0) return 0.0f;

  const float MW = 44.01f;
  float kelvin = tempC + 273.15f;
  float molarVolume = 22.414f * kelvin / 273.15f;

  return ppm * MW / molarVolume;
}

float tvocPpbToMgM3Approx(float ppb, float tempC = 25.0f) {
  if (ppb < 0) return 0.0f;

  const float MW_EQ = 92.14f;
  float kelvin = tempC + 273.15f;
  float molarVolume = 22.414f * kelvin / 273.15f;

  return (ppb / 1000.0f) * MW_EQ / molarVolume;
}

// =====================================================
// I2C scanner
// =====================================================
void scanI2C() {
#if ENABLE_DEBUG
  DBG_PRINTLN("I2C scan start...");

  uint8_t count = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      DBG_PRINT("I2C device found at 0x");
      if (addr < 16) DBG_PRINT("0");
      DBG_PRINTLN(addr, HEX);
      count++;
    }

    delay(1);
  }

  DBG_PRINT("I2C device count = ");
  DBG_PRINTLN(count);
  DBG_PRINTLN("");
#endif
}

// =====================================================
// OLED
// =====================================================
bool initOLED() {
  if (!oled.begin(OLED_ADDR, true)) {
    DBG_PRINTLN("OLED init failed");
    return false;
  }

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SH110X_WHITE);
  oled.setTextWrap(false);
  oled.cp437(true);

  oled.setCursor(0, 0);
  oled.println("Air Monitor");
  oled.println("BLE + Sensors");
  oled.println("Initializing...");
  oled.display();

  return true;
}

void updateDisplay() {
  if (!oledOK) return;

  float tvocMgM3 = tvocPpbToMgM3Approx((float)ensTVOC, envTempC);
  float eco2MgM3 = co2PpmToMgM3((float)enseCO2, envTempC);

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SH110X_WHITE);
  oled.setTextWrap(false);
  oled.cp437(true);

  char line[32];

  oled.setCursor(0, 0);
  oled.print("Temp/Hum: ");
  oled.print(envTempC, 1);
  oled.write(248);
  oled.print("C/");
  oled.print(envHumRH, 0);
  oled.print("%");

if (pmsData.valid) {
  snprintf(line, sizeof(line), "PM2.5/10: %u/%uug/m3",
           pmsData.pm2_5_atm,
           pmsData.pm10_atm);
} else {
  snprintf(line, sizeof(line), "PM2.5/10: ---/---");
}

  oled.setCursor(0, 10);
  oled.print(line);

  snprintf(line, sizeof(line), "CH2O : %.3f mg/m3", ch2oMgM3);
  oled.setCursor(0, 20);
  oled.print(line);

  snprintf(line, sizeof(line), "TVOC : %.2f mg/m3", tvocMgM3);
  oled.setCursor(0, 30);
  oled.print(line);

  snprintf(line, sizeof(line), "eCO2 : %.0f mg/m3", eco2MgM3);
  oled.setCursor(0, 40);
  oled.print(line);

  snprintf(line, sizeof(line), "AQI: Lv%u   BT: %s",
           ensAQI,
           bleConnected ? "Link" : "Ready");
  oled.setCursor(0, 50);
  oled.print(line);

  oled.display();
}

// =====================================================
// AHT20 + ENS160
// =====================================================
void initENS160AHT20() {
  if (aht.begin()) {
    ahtOK = true;
    DBG_PRINTLN("AHT20 init success");
  } else {
    ahtOK = false;
    DBG_PRINTLN("AHT20 init failed");
  }

  ens160.begin();

  if (ens160.available()) {
    ensOK = ens160.setMode(ENS160_OPMODE_STD);

    if (ensOK) {
      DBG_PRINTLN("ENS160 init success");
    } else {
      DBG_PRINTLN("ENS160 set mode failed");
    }
  } else {
    ensOK = false;
    DBG_PRINTLN("ENS160 init failed");
  }
}

void updateENS160AHT20() {
  if (ahtOK) {
    sensors_event_t humidity;
    sensors_event_t temp;

    aht.getEvent(&humidity, &temp);

    if (!isnan(temp.temperature) && !isnan(humidity.relative_humidity)) {
      envTempC = temp.temperature;
      envHumRH = humidity.relative_humidity;
    }

    DBG_PRINT("AHT20 T = ");
    DBG_PRINT(envTempC, 2);
    DBG_PRINT(" C  RH = ");
    DBG_PRINT(envHumRH, 1);
    DBG_PRINTLN(" %");
  }

  if (ensOK && ens160.available()) {
    ens160.measure(true);
    ens160.measureRaw(true);

    ensAQI  = ens160.getAQI();
    ensTVOC = ens160.getTVOC();
    enseCO2 = ens160.geteCO2();

    DBG_PRINT("ENS160 AQI = ");
    DBG_PRINT(ensAQI);
    DBG_PRINT("  TVOC = ");
    DBG_PRINT(ensTVOC);
    DBG_PRINT(" ppb  eCO2 = ");
    DBG_PRINT(enseCO2);
    DBG_PRINTLN(" ppm");
  }
}

// =====================================================
// ZE08-CH2O non-blocking parser
// =====================================================
struct ZE08Parser {
  static const uint8_t FRAME_LEN = 9;
  uint8_t buf[FRAME_LEN];
  uint8_t idx = 0;

  void reset() {
    idx = 0;
  }

  bool feed(uint8_t b, uint16_t &ppb, float &ppm) {
    if (idx == 0) {
      if (b != 0xFF) {
        return false;
      }

      buf[idx++] = b;
      return false;
    }

    buf[idx++] = b;

    if (idx == 2 && buf[1] != 0x17) {
      if (b == 0xFF) {
        buf[0] = 0xFF;
        idx = 1;
      } else {
        reset();
      }

      return false;
    }

    if (idx == 3 && buf[2] != 0x04) {
      reset();
      return false;
    }

    if (idx < FRAME_LEN) {
      return false;
    }

    uint8_t sum = 0;

    for (int i = 1; i <= 7; i++) {
      sum += buf[i];
    }

    uint8_t checksum = (uint8_t)(~sum + 1);

    if (checksum == buf[8]) {
      ppb = ((uint16_t)buf[4] << 8) | buf[5];
      ppm = ppb / 1000.0f;

      reset();
      return true;
    }

    reset();
    return false;
  }
};

ZE08Parser ze08Parser;

bool updateZE08CH2O() {
  bool gotData = false;
  int budget = UART_READ_BUDGET_BYTES;

  while (uartCH2O.available() > 0 && budget-- > 0) {
    uint8_t b = (uint8_t)uartCH2O.read();

    uint16_t ppb = 0;
    float ppm = 0.0f;

    if (ze08Parser.feed(b, ppb, ppm)) {
      ch2oPpb = ppb;
      ch2oPpm = ppm;
      ch2oMgM3 = ch2oPpmToMgM3(ch2oPpm, envTempC);

      gotData = true;
    }
  }

  if (gotData) {
    DBG_PRINT("CH2O = ");
    DBG_PRINT(ch2oMgM3, 3);
    DBG_PRINTLN(" mg/m3");
  }

  return gotData;
}

// =====================================================
// PMS7003 non-blocking parser
// =====================================================
struct PMSParser {
  static const uint8_t FRAME_LEN = 32;
  uint8_t buf[FRAME_LEN];
  uint8_t idx = 0;

  void reset() {
    idx = 0;
  }

  bool feed(uint8_t b, PMSData &out) {
    if (idx == 0) {
      if (b != 0x42) {
        return false;
      }

      buf[idx++] = b;
      return false;
    }

    if (idx == 1) {
      if (b != 0x4D) {
        if (b == 0x42) {
          buf[0] = 0x42;
          idx = 1;
        } else {
          reset();
        }

        return false;
      }

      buf[idx++] = b;
      return false;
    }

    buf[idx++] = b;

    if (idx == 4) {
      uint16_t frameLen = ((uint16_t)buf[2] << 8) | buf[3];

      if (frameLen != 28) {
        reset();
      }

      return false;
    }

    if (idx < FRAME_LEN) {
      return false;
    }

    uint16_t sum = 0;

    for (int i = 0; i < 30; i++) {
      sum += buf[i];
    }

    uint16_t checksum = ((uint16_t)buf[30] << 8) | buf[31];

    if (sum == checksum) {
      out.pm1_0_atm = ((uint16_t)buf[10] << 8) | buf[11];
      out.pm2_5_atm = ((uint16_t)buf[12] << 8) | buf[13];
      out.pm10_atm  = ((uint16_t)buf[14] << 8) | buf[15];
      out.valid = true;
      out.lastUpdateMs = millis();

      reset();
      return true;
    }

    reset();
    return false;
  }
};

PMSParser pmsParser;

void sendPMSCommand(const uint8_t *cmd, size_t len) {
  uartPMS.write(cmd, len);
  uartPMS.flush();
}

void pmsSetActiveMode() {
  static const uint8_t cmd[] = {
    0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71
  };

  sendPMSCommand(cmd, sizeof(cmd));
}

void pmsWake() {
  static const uint8_t cmd[] = {
    0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74
  };

  sendPMSCommand(cmd, sizeof(cmd));
}

bool updatePMSAlwaysOn() {
  PMSData latest;
  bool gotFrame = false;

  int budget = UART_READ_BUDGET_BYTES;

  while (uartPMS.available() > 0 && budget-- > 0) {
    uint8_t b = (uint8_t)uartPMS.read();

    if (pmsParser.feed(b, latest)) {
      gotFrame = true;
    }
  }

  if (!pmsWarmedUp) {
    if ((uint32_t)(millis() - pmsPowerOnMs) >= PMS_WARMUP_MS) {
      pmsWarmedUp = true;
      DBG_PRINTLN("PMS -> CONTINUOUS");
    }

    return false;
  }

  if (gotFrame) {
    pmsData = latest;

    DBG_PRINT("PMS PM2.5 = ");
    DBG_PRINT(pmsData.pm2_5_atm);
    DBG_PRINT("  PM10 = ");
    DBG_PRINTLN(pmsData.pm10_atm);

    return true;
  }

  return false;
}

// =====================================================
// BLE
// =====================================================
void initBLE() {
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(128);

  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pDataChar = pService->createCharacteristic(
    CHAR_UUID_DATA,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pDataChar->addDescriptor(new BLE2902());
  pDataChar->setValue("ENV,0,0,0,0,0,0,0,0,0");

  pStatusChar = pService->createCharacteristic(
    CHAR_UUID_STATUS,
    BLECharacteristic::PROPERTY_READ
  );

  pStatusChar->setValue("READY");

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();

  DBG_PRINTLN("BLE advertising started");
  DBG_PRINT("Device name: ");
  DBG_PRINTLN(DEVICE_NAME);
}

void restartAdvertisingIfNeeded() {
  if (!needRestartAdvertising) {
    return;
  }

  needRestartAdvertising = false;

  delay(200);
  BLEDevice::startAdvertising();

  DBG_PRINTLN("BLE advertising restarted");
}

void notifyBLEData() {
  if (!bleConnected || !pDataChar) {
    return;
  }

  bleSeq++;

  char msg[128];

  snprintf(
    msg,
    sizeof(msg),
    "ENV,%lu,%u,%u,%.3f,%.1f,%.0f,%u,%u,%u",
    (unsigned long)bleSeq,
    pmsData.pm2_5_atm,
    pmsData.pm10_atm,
    ch2oMgM3,
    envTempC,
    envHumRH,
    ensTVOC,
    enseCO2,
    ensAQI
  );

  pDataChar->setValue(msg);
  pDataChar->notify();

  DBG_PRINT("Notify: ");
  DBG_PRINTLN(msg);
}

// =====================================================
// Heartbeat
// =====================================================
void printHeartbeat() {
#if ENABLE_DEBUG
  DBG_PRINT("HEARTBEAT ms=");
  DBG_PRINT(millis());

  DBG_PRINT(" freeHeap=");
  DBG_PRINT(ESP.getFreeHeap());

  DBG_PRINT(" minFreeHeap=");
  DBG_PRINT(ESP.getMinFreeHeap());

  DBG_PRINT(" BLE=");
  DBG_PRINT(bleConnected ? "connected" : "idle");

  DBG_PRINT(" PMS=");
  DBG_PRINT(pmsData.valid ? "valid" : "no_data");

  DBG_PRINT(" CH2O=");
  DBG_PRINT(ch2oMgM3, 3);

  DBG_PRINTLN("");
#endif
}

// =====================================================
// setup
// =====================================================
void setup() {
  DBG_BEGIN(DEBUG_BAUD);

  delay(500);

  DBG_PRINTLN("");
  DBG_PRINTLN("=== Air Monitor + BLE Start ===");

#if ENABLE_DEBUG
  esp_reset_reason_t reason = esp_reset_reason();

  DBG_PRINT("Reset reason: ");
  DBG_PRINT((int)reason);
  DBG_PRINT(" / ");
  DBG_PRINTLN(resetReasonToText(reason));

  DBG_PRINT("Initial freeHeap = ");
  DBG_PRINTLN(ESP.getFreeHeap());
#endif

  // I2C
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  scanI2C();

  // OLED
  oledOK = initOLED();
  DBG_PRINTLN(oledOK ? "OLED init success" : "OLED init failed");

  // AHT20 + ENS160
  initENS160AHT20();

  // UART sensors
  uartCH2O.begin(SENSOR_BAUD, SERIAL_8N1, PIN_CH2O_RX, PIN_CH2O_TX);
  uartPMS.begin(SENSOR_BAUD, SERIAL_8N1, PIN_PMS_RX, PIN_PMS_TX);

  uartCH2O.setTimeout(20);
  uartPMS.setTimeout(20);

  DBG_PRINTLN("UART sensors initialized");

  // PMS7003
  pmsWake();
  delay(100);
  pmsSetActiveMode();

  pmsPowerOnMs = millis();
  pmsWarmedUp = false;

  DBG_PRINTLN("PMS always-on mode start");

  // BLE
  initBLE();

  // Initial display
  updateDisplay();

  lastEnvUpdate = millis();
  lastDisplayUpdate = millis();
  lastBleNotify = millis();
  lastHeartbeat = millis();

  DBG_PRINTLN("Setup complete");
}

// =====================================================
// loop
// =====================================================
void loop() {
  uint32_t now = millis();

  restartAdvertisingIfNeeded();

  updateZE08CH2O();
  updatePMSAlwaysOn();

  if ((uint32_t)(now - lastEnvUpdate) >= ENV_UPDATE_MS) {
    lastEnvUpdate = now;
    updateENS160AHT20();
  }

  if ((uint32_t)(now - lastDisplayUpdate) >= DISPLAY_UPDATE_MS) {
    lastDisplayUpdate = now;
    updateDisplay();
  }

  if ((uint32_t)(now - lastBleNotify) >= BLE_NOTIFY_MS) {
    lastBleNotify = now;
    notifyBLEData();
  }

  if ((uint32_t)(now - lastHeartbeat) >= HEARTBEAT_MS) {
    lastHeartbeat = now;
    printHeartbeat();
  }

  delay(5);
}
