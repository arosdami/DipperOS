// =====================================================================
//  DipperOS v12.9 - FINAL STABLE R81 SBUS + SMOOTH ROBOTPET EDITION
//  - Proven Stepper.h inside FreeRTOS motorTask (14, 26, 27, 25)
//  - Ultra-smooth speed-limited Servo (writeMicroseconds, max 2.5 deg/20ms)
//  - Physical OLED Mode Banners (AUTO PET / MANUAL RC / SIMULATOR)
//  - Dedicated FreeRTOS Tasks: sbusTask (Core 0) + motorTask (Core 1)
//  - TX12 Sticks: Right Y (CH2) = Servo | Left X (CH4) = Stepper
//  - TX12 Switches: SA(CH5)=AI, SB(CH6)=Mood, SC(CH7)=Tired/Speak, SD(CH8)=Audio/Scan
//  - Auto AI OFF by default | No AutoScan
// =====================================================================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEHIDDevice.h>
#include <esp_wifi.h>
#include <FluxGarage_RoboEyes.h>
#include <ESP32Servo.h>
#include <Stepper.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
RoboEyes<Adafruit_SSD1306> roboEyes(display);
WebServer server(80);

uint8_t webScreenBuf[1024];

// ---- Hardware & Power Pins ----
#define SBUS_RX_PIN  16
#define SBUS_BAUD    100000
#define SBUS_INVERT  true
#define BAT_ADC_PIN  35

// ---- RobotPet Hardware Pins ----
const int SERVO_PIN  = 13;
const int BUZZER_PIN = 23;
const int STEPS_PER_REVOLUTION = 2048;
const float STEPS_PER_DEGREE   = 11.38f;

Stepper tiltStepper(STEPS_PER_REVOLUTION, 14, 26, 27, 25);
Servo headServo;

const char* AP_SSID = "TP-Link_A4F8";
const char* AP_PASS = "12345678";

// ---- RobotPet State & Thread-safe Targets ----
volatile float currentServoDeg = 90.0f;
volatile float targetServoDeg  = 90.0f;
volatile long  currentStepPos  = 0;
volatile long  targetStepPos   = 0;
bool stepperEnergized          = false;

volatile bool petAutoMove   = false; // Alapból OFF
volatile bool petSoundOn    = true;
volatile int  petManualMood = 0;     // 0=DEFAULT, 1=HAPPY, 2=TIRED, 3=ANGRY

bool rcSwitchesInit = false;
int  lastSaPos = -1, lastSbPos = -1, lastScPos = -1, lastSdPos = -1;

enum PetState { PET_CURIOUS, PET_IDLE, PET_BORED, PET_TIRED };
PetState petState = PET_CURIOUS;
unsigned long petStateEnter    = 0;
unsigned long petStateDuration = 7000;

// ---- Physical OLED Mode Banner & Free Wi-Fi Alert ----
String modeBannerLine1 = "";
String modeBannerLine2 = "";
unsigned long modeBannerUntil = 0;

void showModeBanner(String line1, String line2 = "", unsigned long durMs = 2000) {
  modeBannerLine1 = line1;
  modeBannerLine2 = line2;
  modeBannerUntil = millis() + durMs;
}

String freeWifiAlertSSID = "";
int    freeWifiAlertRSSI = 0;
unsigned long freeWifiAlertUntil = 0;
String alertedOpenSSIDs = "";

// ---- NimBLE ----
NimBLEHIDDevice* hidDevice = nullptr;
NimBLECharacteristic* inputGamepad = nullptr;
bool bleHidConnected = false;
bool bleScanningNow = false;

const uint8_t HID_GAMEPAD_REPORT_MAP[] = {
  0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x85, 0x01,
  0x05, 0x09, 0x19, 0x01, 0x29, 0x08, 0x15, 0x00,
  0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
  0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32,
  0x09, 0x33, 0x09, 0x34, 0x09, 0x35, 0x15, 0x81,
  0x25, 0x7F, 0x75, 0x08, 0x95, 0x06, 0x81, 0x02,
  0xC0
};

// ---- Web UI Logging ----
String sysLogs[6] = {"", "", "", "", "", ""};
void addLog(String msg) {
  for(int i=5; i>0; i--) sysLogs[i] = sysLogs[i-1];
  sysLogs[0] = msg;
}

class BleSimCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    bleHidConnected = true;
    addLog("[BLE] HID Gamepad Connected");
    showModeBanner("SIM / BLE HID", "CONNECTED", 2000);
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    bleHidConnected = false;
    addLog("[BLE] HID Disconnected");
    NimBLEDevice::startAdvertising();
  }
};

void initNativeBleGamepad() {
  NimBLEDevice::init("TP-Link_BLE");
  NimBLEDevice::setPower(ESP_PWR_LVL_N0);
  NimBLEDevice::setSecurityAuth(true, false, true);
  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new BleSimCallbacks());
  hidDevice = new NimBLEHIDDevice(pServer);
  inputGamepad = hidDevice->getInputReport(1);
  hidDevice->setManufacturer("TP-Link");
  hidDevice->setPnp(0x02, 0xe502, 0xa111, 0x0210);
  hidDevice->setHidInfo(0x00, 0x01);
  hidDevice->setReportMap((uint8_t*)HID_GAMEPAD_REPORT_MAP, sizeof(HID_GAMEPAD_REPORT_MAP));
  hidDevice->startServices();
  NimBLEAdvertising* pAdv = pServer->getAdvertising();
  pAdv->setAppearance(HID_GAMEPAD);
  pAdv->addServiceUUID(hidDevice->getHidService()->getUUID());
  pAdv->start();
}

// ---- States ----
enum Screen { MAIN_MENU, WIFI_MENU, BLE_MENU, RC_MENU, SET_MENU, APP };
Screen screen = MAIN_MENU;
Screen parentMenu = WIFI_MENU;

const char* mainMenu[] = {"WIFI AUDIT", "BLE RADAR", "RC / SIM", "POCKET AI", "SYSTEM"};
const int mainCount = 5;
int mainIdx = 0;

const char* wifiMenu[] = {"AP Inspector", "Security Audit", "2.4G Spectrum", "EvilTwin Guard", "Probe Sniffer", "Deauth IDS Mon", "PPS Waveform", "<< Main Menu"};
const int wifiCount = 8;
int wifiIdx = 0;

const char* bleMenu[] = {"BLE Devices", "Proximity Radar", "AirTag Hunter", "<< Main Menu"};
const int bleCount = 4;
int bleIdx = 0;

const char* rcMenu[] = {"8Ch RC Matrix", "CH1-4 Precise", "R81 Telemetry", "FPV Simulator", "<< Main Menu"};
const int rcCount = 5;
int rcIdx = 0;

const char* setMenu[] = {
  "Theme: ORANGE", "HW Pad: SHOW", "Robot Pet: ON", 
  "Power & SoC", "Reset Sniffer", "Clear Guard AP", "MAZE ART", "<< Main Menu"
};
const int setCount = 8;
int setIdx = 0;

int currentTheme = 0;
const char* themeLabels[] = {"Theme: ORANGE", "Theme: CYAN", "Theme: MATRIX", "Theme: WHITE"};
bool showHwPad = true;
bool robotPetEnabled = true;

int activeApp = -1;
bool vBtnL = false, vBtnR = false, vBtnOK = false, vBtnBack = false;

unsigned long lastUserInput = 0;
bool idlePocketAIActive = false;

// Power
bool isUsbPowered = true;
int batteryPct = 100;
unsigned long lastPwrCheck = 0;
void updatePowerSupply() {
  if (millis() - lastPwrCheck < 2500) return;
  lastPwrCheck = millis();
  int mv = analogReadMilliVolts(BAT_ADC_PIN);
  float v = (mv * 2.0) / 1000.0;
  if (v < 2.80 || v > 4.35) { isUsbPowered = true; batteryPct = 100; }
  else { isUsbPowered = false; batteryPct = constrain(map((int)(v * 100), 330, 420, 0, 100), 0, 100); }
}

// WiFi
struct WifiEntry { String ssid; String bssid; int32_t rssi; uint8_t ch; wifi_auth_mode_t auth; };
WifiEntry wifiList[25];
int wifiNetCount = 0;
bool wifiScanning = false;
int wifiScroll = 0;
int channelLoad[15] = {0};
int peakChannel = 1;
String mySSID = "", myBSSID = "";

// Sniffer
volatile uint32_t pktProbe = 0, pktDeauth = 0, pktBeacon = 0, pktTotal = 0;
uint32_t lastPktTotal = 0;
uint16_t ppsHistory[60] = {0};
uint16_t currentPPS = 0;
unsigned long lastPpsCalc = 0;
bool snifferActive = false;
uint8_t snifferCh = 1;
unsigned long lastChHop = 0;

// BLE
struct BleDev { String mac; String name; int rssi; };
BleDev bleDevs[15];
int bleDevCount = 0;
int bleScroll = 0;
bool triggerBleScan = false;

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (bleDevCount < 15) {
      bleDevs[bleDevCount].mac = dev->getAddress().toString().c_str();
      bleDevs[bleDevCount].name = dev->haveName() ? dev->getName().c_str() : "(unknown)";
      bleDevs[bleDevCount].rssi = dev->getRSSI();
      bleDevCount++;
    }
  }
};

// SBUS (Thread-safe volatile vars)
uint8_t sbusBuf[25];
uint8_t sbusIdx = 0;
unsigned long sbusLastByteUs = 0;
volatile uint16_t sbusCh[16] = {992,992,992,992,172,172,172,172};
volatile bool sbusValid = false;
volatile unsigned long sbusLastRx = 0;
volatile uint32_t sbusFrameCount = 0;
bool simModeEnabled = true;
bool serialSimBridge = false;
unsigned long lastSimReport = 0;

// =====================================================================
//  LoPaka PROGMEM IMAGES
// =====================================================================
static const unsigned char PROGMEM image_paint_10_bits[] = {0x00,0x07,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xe0,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x07,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xfc,0x07,0xff,0xff,0xe0,0x40,0x00,0x00,0x02,0x07,0xff,0xff,0xe0,0x3f,0xff,0x00,0x02,0x08,0x00,0x00,0x10,0x80,0x00,0x00,0x01,0x08,0x00,0x00,0x10,0x40,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x87,0x90,0x11,0x11,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x84,0x50,0x29,0x11,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x01,0x10,0x00,0x00,0x08,0x84,0x50,0x44,0xa1,0x10,0x00,0x00,0x08,0x80,0x00,0x00,0x02,0x08,0x00,0x00,0x10,0x87,0x90,0x44,0x41,0x08,0x00,0x00,0x10,0x40,0x00,0xff,0xfc,0x07,0xff,0xff,0xe0,0x84,0x10,0x7c,0x41,0x07,0xff,0xff,0xe0,0x3f,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x84,0x10,0x44,0x41,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x1f,0x44,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xf8,0x00,0x00,0x00,0x00,0x00,0x00};
static const unsigned char PROGMEM image_paint_10_copy_1_bits[] = {0xbf,0xff,0xf0,0xa0,0x11,0x10,0xa2,0xd1,0x50,0xbe,0x5d,0x50,0x80,0x45,0x50,0xbf,0x75,0xd0,0xa0,0x14,0x10,0xaf,0xd5,0xd0,0xa8,0x54,0x50,0x81,0x57,0x50,0xbf,0x55,0x50,0x88,0x54,0x50,0xff,0x55,0xd0,0x81,0x55,0x10,0xbf,0x75,0x50,0xa0,0x01,0x10,0xbf,0xf7,0xd0,0x80,0x00,0x50,0xff,0xff,0xd0};
static const unsigned char PROGMEM image_paint_13_copy_1_bits[] = {0xf3,0xcf,0x3c,0xf3,0xcf,0x3c,0x00,0x00,0x00,0xf3,0xcf,0x3c,0xf3,0xcf,0x3c,0x00,0x00,0x00,0x1e,0x79,0xe0,0x1e,0x79,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x78,0x00,0x00,0xec,0x00,0x00,0xf4,0x00,0x00,0xfc,0x00,0x00,0xfc,0x00,0x00,0x78,0x00,0x00,0x00,0x00,0x37,0xff,0xb0,0x37,0xff,0xb0};
static const unsigned char PROGMEM image_paint_5_copy_1_bits[] = {0xc0,0x00,0x00,0xc0,0x20,0x00,0xc0,0x00,0x00,0xc0,0x20,0x00,0xc6,0x00,0x00,0xc6,0x20,0x00,0xc0,0x00,0x00,0xc0,0x20,0x00,0xc0,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x20,0x0c,0x00,0x00,0x0c,0x00,0x20,0x0c,0x00,0x00,0x0c,0x00,0x20,0x0c,0x00,0x00,0x0c,0x00,0x20,0x0c,0x00,0x00,0x0c,0x00,0x20,0x0c};
static const unsigned char PROGMEM image_paint_7_bits[] = {0x0f,0xff,0xf8,0x1b,0xff,0xfc,0x37,0xff,0xfc,0x2f,0xe3,0xc4,0x3f,0xc9,0x90,0x2f,0xc5,0x88,0x2f,0xc1,0x80,0x2f,0xe3,0xc4,0x2f,0xff,0xfc,0x2f,0xff,0xfc,0x2f,0xf7,0xdc,0x2f,0xfb,0xbc,0x2f,0xfc,0x7c,0x2f,0xff,0xfc,0x7f,0xff,0xfe,0xbf,0xff,0xf5,0x9f,0xff,0xed,0x4f,0xff,0xfb,0x20,0x00,0x04,0x13,0x00,0x02,0x20,0x07,0x01,0x1f,0x00,0x1e,0x00,0xff,0xe0};
static const unsigned char PROGMEM image_paint_9_copy_1_bits[] = {0x01,0x02,0x00,0x39,0x02,0x00,0x45,0x02,0x00,0x45,0x02,0x00,0x45,0x02,0x00,0x39,0x02,0x00,0x01,0x02,0x00,0xff,0xff,0xfc,0x01,0x02,0x00,0x01,0x02,0x00,0x01,0x02,0x00,0x01,0x02,0x00,0x01,0x02,0x00,0x01,0x02,0x00,0x01,0x02,0x00,0xff,0xff,0xfc,0x01,0x02,0x00,0x01,0x02,0x88,0x01,0x02,0x50,0x01,0x02,0x20,0x01,0x02,0x50,0x01,0x02,0x88,0x01,0x02,0x00};

void drawScreen_1(void) {
  display.clearDisplay();
  display.drawBitmap(0, 2, image_paint_10_bits, 128, 59, 1);
  display.drawBitmap(85, 32, image_paint_13_copy_1_bits, 22, 19, 1);
  display.drawBitmap(54, 29, image_paint_10_copy_1_bits, 20, 19, 1);
  display.drawBitmap(21, 30, image_paint_9_copy_1_bits, 22, 23, 1);
  display.drawBitmap(-8, 31, image_paint_5_copy_1_bits, 22, 20, 1);
  display.setTextColor(1); display.setTextWrap(false);
  display.setCursor(32, 8); display.print("MAZE RUNNER");
  display.drawBitmap(114, 30, image_paint_7_bits, 24, 23, 1);
}

// Forward decls
void startSingleWiFiScan();
void setSnifferMode(bool enable);

// =====================================================================
//  MÉLY, ÉLETHŰ DROID HANGMOTOR (GLISSANDO)
// =====================================================================
void toneGlide(int fStart, int fEnd, int durationMs) {
  if (!petSoundOn) return;
  int steps = max(4, durationMs / 6);
  float df = (float)(fEnd - fStart) / steps;
  int stepMs = max(2, durationMs / steps);
  for (int i = 0; i <= steps; i++) {
    tone(BUZZER_PIN, (int)(fStart + df * i));
    delay(stepMs);
  }
  noTone(BUZZER_PIN);
}

void playWarmStartup() {
  if (!petSoundOn) return;
  toneGlide(260, 440, 90); delay(15);
  toneGlide(440, 620, 100); delay(15);
  toneGlide(580, 740, 75);
}

void playWarmChirp() {
  if (!petSoundOn) return;
  int base = random(280, 460);
  toneGlide(base, base + random(140, 240), 65);
  delay(15);
  toneGlide(base + 160, base + random(-50, 100), 55);
}

void playAngryGrowl() {
  if (!petSoundOn) return;
  toneGlide(260, 140, 120);
}

void playFreeWifiAlertSound() {
  if (!petSoundOn) return;
  toneGlide(380, 720, 85); delay(20);
  toneGlide(480, 880, 95); delay(20);
  toneGlide(620, 960, 110);
}

void speakLifelike(String phrase) {
  if (!petSoundOn) return;
  phrase.toUpperCase();
  for (int i = 0; i < phrase.length(); i++) {
    char c = phrase[i];
    if (c == ' ') { noTone(BUZZER_PIN); delay(65); continue; }
    int f1 = 300, f2 = 420, dur = 55;
    switch (c) {
      case 'A': f1 = 290; f2 = 410; dur = 75; break;
      case 'E': f1 = 420; f2 = 580; dur = 65; break;
      case 'I': f1 = 560; f2 = 760; dur = 60; break;
      case 'O': f1 = 240; f2 = 340; dur = 80; break;
      case 'U': f1 = 210; f2 = 270; dur = 80; break;
      case 'B': case 'D': case 'G': f1 = 190; f2 = 260; dur = 38; break;
      case 'M': case 'N': f1 = 230; f2 = 250; dur = 55; break;
      case 'L': case 'R': f1 = 310; f2 = 390; dur = 50; break;
      case 'S': case 'Z': case 'C': f1 = 620; f2 = 480; dur = 42; break;
      case 'T': case 'K': case 'P': f1 = 480; f2 = 320; dur = 32; break;
      case 'H': case 'F': case 'V': f1 = 360; f2 = 520; dur = 48; break;
      default:  f1 = random(260, 420); f2 = random(380, 620); dur = 42; break;
    }
    toneGlide(f1, f2, dur);
    delay(12);
  }
  noTone(BUZZER_PIN);
}

// =====================================================================
//  OPTIMIZED FREERTOS SBUS TASK (CORE 0, PRIORITY 1 -> ZERO WIFI LAG)
// =====================================================================
void sbusTask(void* pvParameters) {
  for (;;) {
    while (Serial2.available()) {
      unsigned long nowUs = micros();
      if (sbusIdx > 0 && (nowUs - sbusLastByteUs > 3500)) {
        sbusIdx = 0;
      }
      sbusLastByteUs = nowUs;

      uint8_t b = Serial2.read();
      if (sbusIdx == 0) {
        if (b == 0x0F) sbusBuf[sbusIdx++] = b;
      } else {
        sbusBuf[sbusIdx++] = b;
        if (sbusIdx == 25) {
          if (sbusBuf[24] == 0x00 || (sbusBuf[24] & 0x0F) == 0x04) {
            bool failsafe = (sbusBuf[23] & 0x08) != 0;
            if (!failsafe) {
              sbusCh[0] = ((sbusBuf[1]     | sbusBuf[2]<<8)                   & 0x07FF);
              sbusCh[1] = ((sbusBuf[2]>>3  | sbusBuf[3]<<5)                   & 0x07FF);
              sbusCh[2] = ((sbusBuf[3]>>6  | sbusBuf[4]<<2 | sbusBuf[5]<<10)  & 0x07FF);
              sbusCh[3] = ((sbusBuf[5]>>1  | sbusBuf[6]<<7)                   & 0x07FF);
              sbusCh[4] = ((sbusBuf[6]>>4  | sbusBuf[7]<<4)                   & 0x07FF);
              sbusCh[5] = ((sbusBuf[7]>>7  | sbusBuf[8]<<1 | sbusBuf[9]<<9)   & 0x07FF);
              sbusCh[6] = ((sbusBuf[9]>>2  | sbusBuf[10]<<6)                  & 0x07FF);
              sbusCh[7] = ((sbusBuf[10]>>5 | sbusBuf[11]<<3)                  & 0x07FF);
              sbusValid = true;
              sbusLastRx = millis();
              sbusFrameCount++;
            }
          }
          sbusIdx = 0;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(4));
  }
}

// =====================================================================
//  SMOOTH MOTOR TASK (CORE 1, PRIORITY 2 -> STEPPER.H + MICROSEC SERVO)
// =====================================================================
void setTargetStepDeg(float deg) {
  deg = constrain(deg, -60.0f, 60.0f);
  targetStepPos = (long)(deg * STEPS_PER_DEGREE);
}

void motorTask(void* pvParameters) {
  unsigned long lastServoMs = 0;
  for (;;) {
    // 1. STEPPER (bevált Stepper.h: 14, 26, 27, 25)
    long diff = targetStepPos - currentStepPos;
    if (diff != 0) {
      int dir = (diff > 0) ? 1 : -1;
      tiltStepper.step(dir);
      currentStepPos += dir;
      stepperEnergized = true;
    } else if (stepperEnergized) {
      digitalWrite(14, LOW); digitalWrite(26, LOW);
      digitalWrite(27, LOW); digitalWrite(25, LOW);
      stepperEnergized = false;
    }

    // 2. FINOM, RÁNGATÁSMENTES SZERVÓ (50Hz, max 2.5° / 20ms sebességkorlát)
    unsigned long nowMs = millis();
    if (nowMs - lastServoMs >= 20) {
      lastServoMs = nowMs;
      float err = targetServoDeg - currentServoDeg;
      if (fabs(err) > 0.4f) {
        float smooth = petAutoMove ? 0.08f : 0.16f;
        float delta  = constrain(err * smooth, -2.5f, 2.5f);
        currentServoDeg += delta;
        int us = (int)(500.0f + (currentServoDeg / 180.0f) * 1900.0f);
        headServo.writeMicroseconds(constrain(us, 500, 2400));
      }
    }

    vTaskDelay(pdMS_TO_TICKS(4));
  }
}

// =====================================================================
//  R81 TX12 LOGIC + AUTONÓM FSM
// =====================================================================
int get3PosSwitch(uint16_t val) {
  if (val < 600) return 0;      // Alacsony
  if (val > 1400) return 2;     // Magas
  return 1;                     // Közép
}

void updateRobotPetLogic(unsigned long now) {
  bool rcLive = sbusValid && (now - sbusLastRx < 500);
  bool rcManualActive = false;

  if (rcLive) {
    int saPos = get3PosSwitch(sbusCh[4]); // CH5 (SA)
    int sbPos = get3PosSwitch(sbusCh[5]); // CH6 (SB)
    int scPos = get3PosSwitch(sbusCh[6]); // CH7 (SC)
    int sdPos = get3PosSwitch(sbusCh[7]); // CH8 (SD)

    if (!rcSwitchesInit) {
      rcSwitchesInit = true;
      lastSaPos = saPos; lastSbPos = sbPos; lastScPos = scPos; lastSdPos = sdPos;
    }

    // 1. SA (CH5): Átbillentésre vált AUTO PET <-> MANUAL RC között + OLED kiírás
    if (saPos != lastSaPos) {
      lastSaPos = saPos;
      if (saPos == 2) {
        petAutoMove = true;
        addLog("[SA] MODE: AUTO PET");
        showModeBanner("MODE: AUTO PET", "AI ACTIVE", 2000);
      } else {
        petAutoMove = false;
        addLog("[SA] MODE: MANUAL RC");
        showModeBanner("MODE: MANUAL RC", "TX12 STICKS", 2000);
      }
    }

    // 2. SB (CH6): Hangulat (0=NORM, 1=HAPPY, 2=ANGRY)
    if (sbPos != lastSbPos) {
      lastSbPos = sbPos;
      if (sbPos == 0)      { petManualMood = 0; addLog("[SB] Mood: NORMAL"); }
      else if (sbPos == 1) { petManualMood = 1; playWarmChirp(); addLog("[SB] Mood: HAPPY"); }
      else                 { petManualMood = 3; playAngryGrowl(); addLog("[SB] Mood: ANGRY"); }
    }

    // 3. SC (CH7): Extra & Beszéd (0=NORM, 1=TIRED, 2=SZIA beszéd)
    if (scPos != lastScPos) {
      lastScPos = scPos;
      if (scPos == 0)      { petManualMood = 0; }
      else if (scPos == 1) { petManualMood = 2; addLog("[SC] Mood: TIRED"); }
      else                 { petManualMood = 1; speakLifelike("SZIA"); addLog("[SC] Droid Speak: SZIA"); }
    }

    // 4. SD (CH8): Hang & Wi-Fi Scan (0=Audio ON, 1=Mute, 2=Wi-Fi Scan)
    if (sdPos != lastSdPos) {
      lastSdPos = sdPos;
      if (sdPos == 0)      { petSoundOn = true;  playWarmChirp(); addLog("[SD] Audio: ON"); }
      else if (sdPos == 1) { petSoundOn = false; addLog("[SD] Audio: MUTED"); }
      else                 { setSnifferMode(false); startSingleWiFiScan(); addLog("[SD] RC Trigger: Wi-Fi Scan"); }
    }

    // Karok kezelése: MINDIG aktív, ha nincs bekapcsolva az Auto AI!
    if (!petAutoMove) {
      rcManualActive = true;

      // JOBB KAR (Fel-Le = CH2 / sbusCh[1]) -> SZERVÓ (15° .. 165°)
      int ch2 = constrain((int)sbusCh[1], 172, 1811);
      if (abs(ch2 - 992) < 25) targetServoDeg = 90.0f;
      else targetServoDeg = (float)map(ch2, 172, 1811, 15, 165);

      // BAL KAR (Balra-Jobbra = CH4 / sbusCh[3]) -> STEPPER (-60° .. +60°)
      int ch4 = constrain((int)sbusCh[3], 172, 1811);
      if (abs(ch4 - 992) < 25) setTargetStepDeg(0.0f);
      else setTargetStepDeg((float)map(ch4, 172, 1811, -60, 60));
    }
  } else {
    rcSwitchesInit = false;
  }

  // Autonóm FSM (csak ha petAutoMove == true)
  if (!rcManualActive && robotPetEnabled && petAutoMove) {
    if (now - petStateEnter >= petStateDuration) {
      petStateEnter = now;
      int roll = random(0, 100);
      switch (petState) {
        case PET_IDLE:
          if (roll < 50)      { petState = PET_CURIOUS; petStateDuration = random(6000, 12000); }
          else if (roll < 80) { petState = PET_BORED;   petStateDuration = random(5000, 9000); }
          else                { petState = PET_TIRED;   petStateDuration = random(8000, 14000); }
          break;
        case PET_CURIOUS:
          if (roll < 60) { petState = PET_IDLE;  petStateDuration = random(5000, 9000); }
          else           { petState = PET_BORED; petStateDuration = random(4000, 7000); }
          break;
        case PET_BORED:
          if (roll < 50) { petState = PET_TIRED;   petStateDuration = random(7000, 12000); }
          else           { petState = PET_CURIOUS; petStateDuration = random(5000, 10000); }
          break;
        case PET_TIRED:
          petState = PET_CURIOUS;
          petStateDuration = random(5000, 8000);
          playWarmChirp();
          break;
      }
    }

    static unsigned long lastAiMove = 0;
    if (now - lastAiMove > 1000) {
      lastAiMove = now;
      if (petState == PET_CURIOUS && random(0, 100) < 35) {
        targetServoDeg = (float)random(55, 125);
        setTargetStepDeg((float)random(-40, 40));
        if (random(0, 100) < 20) playWarmChirp();
      } else if (petState == PET_IDLE && random(0, 100) < 15) {
        targetServoDeg = (float)random(75, 105);
        setTargetStepDeg((float)random(-15, 15));
      } else if (petState == PET_BORED && random(0, 100) < 15) {
        targetServoDeg = 60.0f;
        setTargetStepDeg(0.0f);
      } else if (petState == PET_TIRED) {
        targetServoDeg = 55.0f;
      }
    }
  }
}

// =====================================================================
//  Process Name
// =====================================================================
void getActiveProcessName(char* outBuf, size_t maxLen) {
  if (millis() < freeWifiAlertUntil) { snprintf(outBuf, maxLen, "ALERT: [FREE_WIFI_FOUND]"); return; }
  bool rcLive = sbusValid && (millis() - sbusLastRx < 500);
  if (bleScanningNow) { snprintf(outBuf, maxLen, "PROC: [BLE_RADAR_SWEEP]"); }
  else if (wifiScanning) { snprintf(outBuf, maxLen, "PROC: [WIFI_SCAN_DAEMON]"); }
  else if (snifferActive) { snprintf(outBuf, maxLen, "PROC: [80211_SNIFFER C%02d]", snifferCh); }
  else if (screen == APP && parentMenu == RC_MENU && activeApp == 3) { snprintf(outBuf, maxLen, "MODE: [FPV_SIMULATOR]"); }
  else if (petAutoMove) { snprintf(outBuf, maxLen, "MODE: [AUTO_PET_AI]"); }
  else if (rcLive) { snprintf(outBuf, maxLen, "MODE: [MANUAL_R81_RC]"); }
  else if (screen == MAIN_MENU) { snprintf(outBuf, maxLen, "PROC: [IDLE // MAIN_UI]"); }
  else if (screen == WIFI_MENU) { snprintf(outBuf, maxLen, "PROC: [MENU // WIFI_AUD]"); }
  else if (screen == BLE_MENU) { snprintf(outBuf, maxLen, "PROC: [MENU // BLE_RAD]"); }
  else if (screen == RC_MENU) { snprintf(outBuf, maxLen, "PROC: [MENU // RC_SIM]"); }
  else if (screen == SET_MENU) { snprintf(outBuf, maxLen, "PROC: [MENU // SYS_CFG]"); }
  else if (screen == APP) {
    const char* appTitle = "TASK";
    if (parentMenu == MAIN_MENU) appTitle = "POCKET_AI";
    else if (parentMenu == WIFI_MENU) appTitle = wifiMenu[activeApp];
    else if (parentMenu == BLE_MENU) appTitle = bleMenu[activeApp];
    else if (parentMenu == RC_MENU) appTitle = rcMenu[activeApp];
    else appTitle = setMenu[activeApp];
    snprintf(outBuf, maxLen, "RUN: [%s]", appTitle);
  } else { snprintf(outBuf, maxLen, "PROC: [DIPPER_KERNEL]"); }
}

// ---- Menu icons ----
const unsigned char PROGMEM icon16_wifi[] = {0x00,0x00, 0x07,0xe0, 0x1f,0xf8, 0x38,0x1c, 0x63,0xc6, 0x4f,0xf2, 0x1c,0x38, 0x08,0x10, 0x03,0xc0, 0x07,0xe0, 0x04,0x20, 0x00,0x00, 0x01,0x80, 0x01,0x80, 0x00,0x00, 0x00,0x00};
const unsigned char PROGMEM icon16_ble[] = {0x01,0x80, 0x01,0xc0, 0x01,0x60, 0x09,0x30, 0x05,0x60, 0x03,0xc0, 0x01,0x80, 0x03,0xc0, 0x05,0x60, 0x09,0x30, 0x01,0x60, 0x01,0xc0, 0x01,0x80, 0x00,0x00, 0x00,0x00, 0x00,0x00};
const unsigned char PROGMEM icon16_rc[] = {0x01,0x80, 0x01,0x80, 0x01,0x80, 0x03,0xc0, 0x1f,0xf8, 0x3f,0xfc, 0x79,0x9e, 0x71,0x8e, 0x79,0x9e, 0x7f,0xfe, 0x7f,0xfe, 0x63,0xc6, 0x40,0x02, 0x00,0x00, 0x00,0x00, 0x00,0x00};
const unsigned char PROGMEM icon16_ai[] = {0x00,0x00, 0x03,0xc0, 0x07,0xe0, 0x0f,0xf0, 0x1f,0xf8, 0x31,0x8c, 0x31,0x8c, 0x3f,0xfc, 0x3f,0xfc, 0x33,0xcc, 0x30,0x0c, 0x1f,0xf8, 0x0f,0xf0, 0x07,0xe0, 0x00,0x00, 0x00,0x00};
const unsigned char PROGMEM icon16_sys[] = {0x03,0xc0, 0x03,0xc0, 0x1c,0x38, 0x3f,0xfc, 0x33,0xcc, 0x67,0xe6, 0x66,0x66, 0x66,0x66, 0x67,0xe6, 0x33,0xcc, 0x3f,0xfc, 0x1c,0x38, 0x03,0xc0, 0x03,0xc0, 0x00,0x00, 0x00,0x00};
const unsigned char* mainIcons[] = {icon16_wifi, icon16_ble, icon16_rc, icon16_ai, icon16_sys};

// =====================================================================
//  WEB UI (DipperOS v12.9)
// =====================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0,maximum-scale=1.0,user-scalable=no,viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes"><meta name="theme-color" content="#101216">
<title>DipperOS v12.9 // FINAL</title>
<style>
:root{--flp:#ff8c00;--flp-dk:#b85d00;--bg:#0c0e12;--case:#181c24;--bezel:#0d0f14;--term:#00ff66;--cyan:#00f0ff;--red:#ff2a55}
*{box-sizing:border-box;margin:0;padding:0;font-family:'Courier New',monospace;user-select:none;-webkit-user-select:none;-webkit-tap-highlight-color:transparent;touch-action:manipulation}
html,body{position:fixed;inset:0;width:100%;height:100%;overflow:hidden;background:var(--bg);color:#e0e6ed;display:flex;flex-direction:column;align-items:center}
.device-frame{width:100%;max-width:480px;height:100%;background:var(--case);border-left:4px solid #262c3a;border-right:4px solid #262c3a;display:flex;flex-direction:column;justify-content:space-between;padding:7px;gap:6px;box-shadow:inset 0 0 20px rgba(0,0,0,.85)}
.hw-header{display:flex;justify-content:space-between;align-items:center;background:#101319;border:2px solid #2a3140;padding:5px 8px;box-shadow:0 3px 0 #07080b;flex-shrink:0}
.screw{width:8px;height:8px;background:#3a4252;border:1px solid #111;display:inline-block;flex-shrink:0}
.hw-proc{font-size:.7rem;font-weight:900;color:var(--flp);letter-spacing:.5px;text-shadow:1px 1px 0 #000;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;margin:0 6px;flex:1}
.led-grp{display:flex;align-items:center;gap:5px;font-size:.6rem;font-weight:bold;color:#94a3b8;flex-shrink:0}
.led{width:8px;height:8px;background:var(--term);box-shadow:0 0 6px var(--term);display:inline-block}
.mode-tabs{display:grid;grid-template-columns:repeat(4,1fr);gap:4px;flex-shrink:0}
.m-tab{background:#12151c;border:2px solid #2c3546;color:#78879c;padding:7px 2px;text-align:center;font-size:.62rem;font-weight:900;box-shadow:0 3px 0 #07090c;cursor:pointer}
.m-tab.active{background:var(--flp);color:#120900;border-color:#ffc266;box-shadow:0 3px 0 #7a3d00}
.panel{display:none;flex:1;flex-direction:column;justify-content:flex-start;overflow-y:auto;gap:6px}
.panel.active{display:flex}
.bezel-box{background:var(--bezel);border:4px solid #28303e;padding:7px;box-shadow:inset 0 4px 12px #000,0 4px 0 #08090c;display:flex;flex-direction:column;gap:4px;flex-shrink:0}
.bezel-meta{display:flex;justify-content:space-between;font-size:.6rem;color:#6c7a8e;font-weight:bold}
.oled-container{position:relative;width:100%;aspect-ratio:2/1;background:#ff8c00;border:3px solid #050505;box-shadow:0 0 18px rgba(255,140,0,.35);overflow:hidden}
.oled-container::after{content:"";position:absolute;inset:0;background-image:linear-gradient(to right,rgba(0,0,0,.18) 1px,transparent 1px),linear-gradient(to bottom,rgba(0,0,0,.18) 1px,transparent 1px);background-size:.78125% 1.5625%;pointer-events:none}
canvas#oled{width:100%;height:100%;image-rendering:pixelated;display:block}
.swipe-sub{font-size:.54rem;color:#6c7a8e;text-align:center;font-weight:bold}
.intel-deck{flex-shrink:0;display:flex;flex-direction:column;gap:5px;min-height:110px}
.signal-cards{display:grid;grid-template-columns:1fr 1fr;gap:5px;flex-shrink:0}
.sig-card{background:#090b10;border:2px solid #222a38;padding:5px 6px;display:flex;flex-direction:column;justify-content:space-between;gap:3px}
.sig-hdr{display:flex;justify-content:space-between;font-size:.54rem;color:#7c8ba1;font-weight:900;border-bottom:1px solid #19202c;padding-bottom:2px}
.sig-name{font-size:.7rem;font-weight:900;color:var(--flp);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.sig-meta{display:flex;justify-content:space-between;font-size:.58rem;color:#cbd5e1;font-weight:bold}
.rssi-track{width:100%;height:5px;background:#141923;border:1px solid #263042}
.rssi-fill{height:100%;background:var(--term);width:0%}
.kpi-strip{display:grid;grid-template-columns:repeat(4,1fr);gap:4px;flex-shrink:0}
.kpi-box{background:#090b10;border:2px solid #222a38;padding:3px 2px;text-align:center}
.kpi-box .v{font-size:.76rem;font-weight:900;color:var(--cyan)}
.kpi-box .l{font-size:.5rem;color:#78879c;font-weight:bold}
.mini-hud{flex-shrink:0;background:#090b0f;border:2px solid #222a38;padding:4px 6px;display:flex;align-items:center;justify-content:space-between;gap:8px;min-height:38px}
.hud-stats{font-size:.6rem;line-height:1.35;color:var(--flp);font-weight:bold;white-space:nowrap}
canvas#miniSpec{flex:1;height:100%;min-height:30px;max-height:54px;background:#050608;border:1px solid #1c2430;image-rendering:pixelated}
.sys-log-box{flex:1;background:#080a0d;border:2px solid #222a38;padding:6px;display:flex;flex-direction:column;gap:4px;overflow:hidden}
.log-hdr{font-size:.6rem;color:#7c8ba1;font-weight:900;border-bottom:1px solid #19202c;padding-bottom:4px;display:flex;justify-content:space-between}
.log-content{flex:1;font-size:.65rem;color:var(--term);font-family:monospace;overflow-y:auto;line-height:1.4;white-space:pre-wrap;word-break:break-all}
.hw-controls{display:flex;flex-direction:column;gap:6px;flex-shrink:0;margin-top:auto}
.hw-controls.hidden{display:none}
.dpad-grid{display:grid;grid-template-columns:1fr 1.15fr 1fr;gap:6px;align-items:stretch}
.px-btn{height:52px;background:#222834;border-top:3px solid #455066;border-left:3px solid #455066;border-right:3px solid #0e1117;border-bottom:3px solid #0e1117;box-shadow:0 4px 0 #07080c;color:#fff;font-size:1rem;font-weight:900;display:flex;flex-direction:column;align-items:center;justify-content:center;cursor:pointer}
.px-btn small{font-size:.52rem;color:#90a0b8;margin-top:2px}
.px-btn:active{transform:translateY(3px);box-shadow:0 1px 0 #07080c;background:#1b202a}
.px-btn.ok-main{background:var(--flp);color:#140900;border-top-color:#ffc470;border-left-color:#ffc470;border-right-color:#7a3e00;border-bottom-color:#7a3e00;box-shadow:0 4px 0 #4d2600}
.px-btn.ok-main small{color:#2e1600;font-weight:900}
.sub-btns{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}
.px-sub{height:34px;background:#191d26;border-top:2px solid #353e50;border-left:2px solid #353e50;border-right:2px solid #0b0d12;border-bottom:2px solid #0b0d12;box-shadow:0 3px 0 #07080c;color:#cbd5e1;font-size:.63rem;font-weight:900;display:flex;align-items:center;justify-content:center;cursor:pointer}
.px-sub:active{transform:translateY(2px);box-shadow:0 1px 0 #07080c}
.px-sub.back{color:#ff99bb;border-top-color:#5c2b3e}
.px-sub.scan{color:var(--cyan);border-top-color:#1f5561}
.px-sub.snif{color:var(--term);border-top-color:#1f5c35}
.cli-shell{flex:1;background:#050806;border:3px solid #1b3826;display:flex;flex-direction:column;overflow:hidden}
.cli-topbar{background:#0c1c12;border-bottom:2px solid #1b3826;padding:5px 8px;display:flex;justify-content:space-between;font-size:.65rem;color:var(--term);font-weight:bold}
.cli-output{flex:1;padding:8px;overflow-y:auto;font-size:.71rem;line-height:1.42;color:var(--term);white-space:pre-wrap;word-break:break-word;user-select:text;-webkit-user-select:text;touch-action:pan-y}
.cli-macros{display:grid;grid-template-columns:repeat(4,1fr);gap:4px;padding:5px;background:#08110b;border-top:1px solid #152c1d}
.m-chip{background:#0f2417;border:1px solid #235234;color:var(--term);font-size:.63rem;font-weight:bold;padding:5px 2px;text-align:center;cursor:pointer}
.cli-input-row{display:flex;background:#030504;border-top:2px solid #1b3826}
.cli-prompt{padding:9px 4px 9px 8px;color:var(--flp);font-weight:900;font-size:.76rem}
.cli-in{flex:1;background:transparent;border:none;color:#fff;font-size:.8rem;padding:9px 4px;outline:none}
.cli-exec{background:var(--term);color:#041208;border:none;padding:0 14px;font-weight:900;font-size:.74rem;cursor:pointer}
.lab-grid{flex:1;display:grid;grid-template-rows:1fr 1.15fr 1fr;gap:6px;overflow:hidden}
.inst-card{background:#090c10;border:2px solid #263042;padding:5px;display:flex;flex-direction:column;justify-content:space-between;overflow:hidden}
.inst-hdr{display:flex;justify-content:space-between;font-size:.62rem;font-weight:900;color:var(--cyan);border-bottom:1px solid #1b2330;padding-bottom:2px}
canvas.inst-cv{width:100%;height:100%;min-height:45px;background:#040608;display:block;image-rendering:pixelated}
.gim-split{display:grid;grid-template-columns:1fr 1fr;gap:6px;height:100%}
canvas.gim-cv{width:100%;height:100%;min-height:45px;background:#040608;border:1px solid #1b2330;display:block}
.pet-top{display:flex;flex-direction:column;gap:6px;flex-shrink:0}
.pet-card{background:#0d1017;border:2px solid #263042;padding:8px;display:flex;flex-direction:column;gap:6px}
.pet-lbl{display:flex;justify-content:space-between;font-size:.64rem;color:#94a3b8;font-weight:bold}
.pet-lbl span{color:var(--flp)}
.pet-row{display:flex;gap:6px}
.pet-grid4{display:grid;grid-template-columns:repeat(4,1fr);gap:5px}
.pet-btn{flex:1;background:#191f2a;border:2px solid #2e394d;color:#e0e6ed;padding:8px 3px;font-size:.64rem;font-weight:900;cursor:pointer;text-align:center}
.pet-btn.on{background:var(--flp);color:#140900;border-color:#ffc266}
.pet-in{flex:1;background:#05070a;border:2px solid #2e394d;color:#fff;padding:7px;font-size:.74rem;outline:none}
.rc-deck{flex:1;display:grid;grid-template-columns:1fr 1fr;gap:8px;align-items:stretch;min-height:210px;padding-bottom:4px}
.joy-box{background:#090c11;border:3px solid #283244;border-radius:10px;display:flex;flex-direction:column;align-items:center;justify-content:space-between;padding:8px 6px;box-shadow:inset 0 0 18px #000}
.joy-hdr{width:100%;display:flex;justify-content:space-between;font-size:.6rem;font-weight:900;color:var(--cyan)}
.joy-hdr span:last-child{color:var(--flp)}
.joy-area{position:relative;width:136px;height:136px;border-radius:50%;background:radial-gradient(circle,#141a24 0%,#06080c 75%);border:2px solid #2e3a4f;touch-action:none;display:flex;align-items:center;justify-content:center}
.joy-area::before{content:"";position:absolute;width:100%;height:1px;background:#1e293b}
.joy-area::after{content:"";position:absolute;height:100%;width:1px;background:#1e293b}
.joy-knob{position:absolute;width:44px;height:44px;border-radius:50%;background:radial-gradient(circle at 35% 35%,#ffb347,#cc6600);border:3px solid #1a0d00;box-shadow:0 4px 10px rgba(0,0,0,.85);pointer-events:none;transition:transform 0.14s ease-out}
.joy-sub{font-size:.55rem;color:#64748b;font-weight:bold}
</style></head><body>
<div class="device-frame">
<div class="hw-header">
<span class="screw"></span>
<span class="hw-proc" id="procBar">PROC: [IDLE]</span>
<div class="led-grp"><span class="led" id="rfLed"></span>
<span id="pwrMeta" style="color:#00ff66">[USB]</span><span>|</span>
<span id="topMeta">RAM:--%</span><span class="screw"></span></div></div>
<div class="mode-tabs">
<div class="m-tab active" onclick="setView(0)">[1] OLED</div>
<div class="m-tab" onclick="setView(1)">[2] CLI</div>
<div class="m-tab" onclick="setView(2)">[3] RF</div>
<div class="m-tab" onclick="setView(3)">[4] PET</div></div>

<!-- [1] DIPPER OS OLED TOOL -->
<div class="panel active" id="p0">
<div class="bezel-box">
<div class="bezel-meta"><span>DIPPER-OS // VIRTUAL OLED</span><span id="oledModeLbl">STEALTH: TP-LINK</span></div>
<div class="oled-container" id="oledTouch"><canvas id="oled" width="128" height="64"></canvas></div>
<div class="swipe-sub">SWIPE OLED: LEFT/UP (PREV) | RIGHT/DOWN (NEXT) | TAP (OK)</div></div>
<div class="intel-deck">
<div class="signal-cards">
<div class="sig-card"><div class="sig-hdr"><span>STRONGEST WIFI AP</span><span id="topApCh">CH --</span></div>
<div class="sig-name" id="topApSsid">No Scan Data</div>
<div class="sig-meta"><span id="topApAuth">---</span><span id="topApRssi">-- dBm</span></div>
<div class="rssi-track"><div class="rssi-fill" id="topApBar"></div></div></div>
<div class="sig-card"><div class="sig-hdr"><span>CLOSEST BLE NODE</span><span id="topBleCnt">0 FOUND</span></div>
<div class="sig-name" id="topBleName">No Target</div>
<div class="sig-meta"><span id="topBleMac">--:--:--</span><span id="topBleRssi">-- dBm</span></div>
<div class="rssi-track"><div class="rssi-fill" id="topBleBar" style="background:#00f0ff"></div></div></div></div>
<div class="kpi-strip">
<div class="kpi-box"><div class="v" id="kPps">0</div><div class="l">PKT / SEC</div></div>
<div class="kpi-box"><div class="v" id="kBcn">0</div><div class="l">BEACONS</div></div>
<div class="kpi-box"><div class="v" id="kPrb">0</div><div class="l">PROBES</div></div>
<div class="kpi-box"><div class="v" id="kIds" style="color:#00ff66">CLEAN</div><div class="l">IDS GUARD</div></div></div>
<div class="mini-hud">
<div class="hud-stats">
<div>APs : <span id="hAp">0</span> (PK:<span id="hCh">1</span>)</div>
<div>R81 : <span id="hRc">STBY</span></div>
<div>PWR : <span id="hPwr" style="color:#00ff66">USB</span></div></div>
<canvas id="miniSpec" width="170" height="38"></canvas></div></div>

<div class="sys-log-box">
  <div class="log-hdr"><span>SYSTEM TACTICAL LOG</span><span id="logCount">LIVE</span></div>
  <div class="log-content" id="logOut"></div>
</div>

<div class="hw-controls" id="hwPad">
<div class="dpad-grid">
<button class="px-btn" id="bL" onclick="trigBtn('L')">&lt;&lt;<small>LEFT / UP</small></button>
<button class="px-btn ok-main" id="bOK" onclick="trigBtn('OK')">[ OK ]<small>OPEN / SELECT</small></button>
<button class="px-btn" id="bR" onclick="trigBtn('R')">&gt;&gt;<small>RIGHT / DOWN</small></button></div>
<div class="sub-btns">
<button class="px-sub back" id="bB" onclick="trigBtn('BACK')">[ESC] BACK</button>
<button class="px-sub scan" onclick="execTerm('scan wifi')">[RF] WIFI SCAN</button>
<button class="px-sub snif" onclick="execTerm('sniffer')">[IDS] SNIFFER</button></div></div></div>

<!-- [2] PRO CLI -->
<div class="panel" id="p1">
<div class="cli-shell">
<div class="cli-topbar"><span>root@dipper-os:/dev/ttyUSB0</span><span id="cliUp">UP: 0s</span></div>
<div class="cli-output" id="cliOut">  ____  _                     ___  ____  
 |  _ \(_)_ __  _ __   ___ _ _|  _ \/ ___| 
 | | | | | '_ \| '_ \ / _ \ '__| | | \___ \ 
 | |_| | | |_) | |_) |  __/ |  | |_| |___) |
 |____/|_| .__/| .__/ \___|_|   \___/|____/ 
         |_|   |_|   CYBER-TOOL SHELL v12.9
----------------------------------------------
[+] Stealth AP : TP-Link_A4F8 (192.168.4.1)
[+] TX12 Sticks: L-X(CH4)=Stepper, R-Y(CH2)=Servo
[+] TX12 Sw    : SA=AI, SB=Mood, SC=Speak, SD=Scan
</div>
<div class="cli-macros">
<div class="m-chip" onclick="execTerm('neofetch')">neofetch</div>
<div class="m-chip" onclick="execTerm('wifi')">wifi</div>
<div class="m-chip" onclick="execTerm('scan wifi')">scan wifi</div>
<div class="m-chip" onclick="execTerm('sniffer')">sniffer</div>
<div class="m-chip" onclick="execTerm('ble')">ble</div>
<div class="m-chip" onclick="execTerm('scan ble')">scan ble</div>
<div class="m-chip" onclick="execTerm('sbus')">sbus</div>
<div class="m-chip" onclick="execTerm('clear')">clear</div></div>
<div class="cli-input-row">
<span class="cli-prompt">root#</span>
<input type="text" class="cli-in" id="cliIn" placeholder="Command (help, neofetch, wifi...)" autocomplete="off">
<button class="cli-exec" onclick="sendCliInput()">EXEC</button></div></div></div>

<!-- [3] RF SPECTRUM -->
<div class="panel" id="p2">
<div class="lab-grid">
<div class="inst-card"><div class="inst-hdr">
<span>802.11 RF OSCILLOSCOPE</span><span id="oscPps">0 PPS</span></div>
<canvas id="oscCv" class="inst-cv" width="320" height="72"></canvas></div>
<div class="inst-card"><div class="inst-hdr">
<span>2.4 GHz SPECTRUM &amp; WATERFALL</span><span id="wfPeak">PEAK: CH 1</span></div>
<canvas id="specCv" class="inst-cv" width="320" height="84"></canvas></div>
<div class="inst-card"><div class="inst-hdr">
<span>R81 SBUS GIMBAL MONITOR</span><span id="rcSt">STBY</span></div>
<div class="gim-split">
<canvas id="gimL" class="gim-cv" width="140" height="80"></canvas>
<canvas id="gimR" class="gim-cv" width="140" height="80"></canvas></div></div></div></div>

<!-- [4] PET RC CONTROLLER -->
<div class="panel" id="p3">
  <div class="pet-top">
    <div class="pet-card">
      <div class="pet-lbl"><span>ROBOTPET TELEMETRY LINK</span><span id="petRcStat">WEB / R81 READY</span></div>
      <div class="pet-row">
        <button id="petAutoBtn" class="pet-btn" onclick="petToggleAuto()">AUTO AI: OFF</button>
        <button id="petSndBtn" class="pet-btn on" onclick="petToggleSnd()">AUDIO: ON</button>
        <button class="pet-btn" onclick="petHome()">HOME</button>
      </div>
      <div class="pet-grid4">
        <button class="pet-btn" onclick="petMood(0)">NORM</button>
        <button class="pet-btn" onclick="petMood(1)">HAPPY</button>
        <button class="pet-btn" onclick="petMood(2)">TIRED</button>
        <button class="pet-btn" onclick="petMood(3)">ANGRY</button>
      </div>
    </div>
    <div class="pet-card">
      <div class="pet-lbl"><span>DROID VOICE SYNTH (DEEP GLISSANDO)</span></div>
      <div class="pet-row">
        <input type="text" id="petSayIn" class="pet-in" placeholder="pl. SZIA, HELLO...">
        <button class="pet-btn on" style="flex:0.35" onclick="petSay()">SAY</button>
        <button class="pet-btn" style="flex:0.35" onclick="fetch('/api/pet?chirp=1')">CHIRP</button>
      </div>
    </div>
  </div>

  <div class="rc-deck">
    <div class="joy-box">
      <div class="joy-hdr"><span>L: STEPPER</span><span id="joyLVal">0°</span></div>
      <div class="joy-area" id="joyL"><div class="joy-knob" id="knobL"></div></div>
      <div class="joy-sub">BALRA - JOBBRA</div>
    </div>
    <div class="joy-box">
      <div class="joy-hdr"><span>R: SERVO</span><span id="joyRVal">90°</span></div>
      <div class="joy-area" id="joyR"><div class="joy-knob" id="knobR"></div></div>
      <div class="joy-sub">LE - FEL</div>
    </div>
  </div>
</div>

</div>
<script>
document.addEventListener('gesturestart',e=>e.preventDefault());
document.addEventListener('dblclick',e=>e.preventDefault(),{passive:false});
document.addEventListener('touchmove',e=>{if(!e.target.closest('.cli-output')&&!e.target.closest('.log-content')&&!e.target.closest('.joy-area'))e.preventDefault();},{passive:false});
function setView(idx){document.querySelectorAll('.m-tab').forEach((t,i)=>t.classList.toggle('active',i===idx));document.querySelectorAll('.panel').forEach((p,i)=>p.classList.toggle('active',i===idx));}
const THEMES=[{bg:[255,140,0],fg:[20,10,0],hex:'#ff8c00'},{bg:[5,12,18],fg:[0,240,255],hex:'#00f0ff'},{bg:[4,16,7],fg:[0,255,102],hex:'#00ff66'},{bg:[11,11,11],fg:[255,255,255],hex:'#ffffff'}];
let curTheme=0;
function syncSystemPrefs(tIdx,padVis){if(tIdx!==curTheme){curTheme=tIdx;const t=THEMES[curTheme]||THEMES[0];const c=document.getElementById('oledTouch');c.style.background=t.hex;c.style.boxShadow=`0 0 18px ${t.hex}55`;}document.getElementById('hwPad').classList.toggle('hidden',!padVis);}
const oledCv=document.getElementById('oled');const oledCtx=oledCv.getContext('2d');const oledImg=oledCtx.createImageData(128,64);
let netBusy=false;let lastInputTime=0;
function renderBufferToCanvas(buf){const b=new Uint8Array(buf);if(b.length!==1024)return;const d=oledImg.data;const bg=THEMES[curTheme].bg;const fg=THEMES[curTheme].fg;for(let p=0;p<8;p++){for(let x=0;x<128;x++){const byte=b[p*128+x];for(let bit=0;bit<8;bit++){const y=p*8+bit;const on=(byte&(1<<bit))!==0;const i=(y*128+x)*4;d[i]=on?fg[0]:bg[0];d[i+1]=on?fg[1]:bg[1];d[i+2]=on?fg[2]:bg[2];d[i+3]=255;}}}oledCtx.putImageData(oledImg,0,0);}
function pullOled(){if(netBusy||btnInFlight)return;netBusy=true;fetch('/api/screen').then(r=>r.arrayBuffer()).then(buf=>{renderBufferToCanvas(buf);netBusy=false;}).catch(()=>{netBusy=false;});}
let pendingKey=null;let btnInFlight=false;
function trigBtn(k){if(navigator.vibrate)navigator.vibrate(12);lastInputTime=Date.now();if(btnInFlight){pendingKey=k;return;}btnInFlight=true;fetch('/api/btn?k='+k).then(r=>r.arrayBuffer()).then(buf=>{renderBufferToCanvas(buf);btnInFlight=false;if(pendingKey){const n=pendingKey;pendingKey=null;trigBtn(n);}}).catch(()=>{btnInFlight=false;});}
let tx=0,ty=0;const touchArea=document.getElementById('oledTouch');
touchArea.addEventListener('touchstart',e=>{tx=e.changedTouches[0].screenX;ty=e.changedTouches[0].screenY;},{passive:true});
touchArea.addEventListener('touchend',e=>{let dx=e.changedTouches[0].screenX-tx;let dy=e.changedTouches[0].screenY-ty;if(Math.abs(dx)<16&&Math.abs(dy)<16)trigBtn('OK');else if(Math.abs(dx)>Math.abs(dy))trigBtn(dx>0?'R':'L');else trigBtn(dy>0?'R':'L');},{passive:true});
document.addEventListener('keydown',e=>{if(document.activeElement.tagName==='INPUT')return;if(e.key==='ArrowLeft'||e.key==='ArrowUp'){e.preventDefault();trigBtn('L');}if(e.key==='ArrowRight'||e.key==='ArrowDown'){e.preventDefault();trigBtn('R');}if(e.key==='Enter'||e.key===' '){e.preventDefault();trigBtn('OK');}if(e.key==='Escape'||e.key==='Backspace'){e.preventDefault();trigBtn('BACK');}});
const cliOut=document.getElementById('cliOut');const cliIn=document.getElementById('cliIn');
function printCli(txt){cliOut.textContent+="\n"+txt;cliOut.scrollTop=cliOut.scrollHeight;}
function execTerm(cmd){cmd=cmd.trim();if(!cmd)return;printCli("root@dipper-os:~# "+cmd);if(cmd.toLowerCase()==='clear'){cliOut.textContent="[Terminal cleared]";return;}fetch('/api/cli?q='+encodeURIComponent(cmd)).then(r=>r.text()).then(res=>{printCli(res);pullOled();pullTele();});}
function sendCliInput(){execTerm(cliIn.value);cliIn.value='';}
cliIn.addEventListener('keydown',e=>{if(e.key==='Enter')sendCliInput();});

let pAuto=false, pSnd=true, lastJoySend=0;
function setAutoOff(){if(pAuto){pAuto=false;let b=document.getElementById('petAutoBtn');b.innerText='AUTO AI: OFF';b.classList.remove('on');}}
function petToggleAuto(){pAuto=!pAuto;let b=document.getElementById('petAutoBtn');b.innerText='AUTO AI: '+(pAuto?'ON':'OFF');b.classList.toggle('on',pAuto);fetch('/api/pet?auto='+(pAuto?1:0));}
function petToggleSnd(){pSnd=!pSnd;let b=document.getElementById('petSndBtn');b.innerText='AUDIO: '+(pSnd?'ON':'OFF');b.classList.toggle('on',pSnd);fetch('/api/pet?snd='+(pSnd?1:0));}
function petHome(){setAutoOff();document.getElementById('knobL').style.transform='translate(0px,0px)';document.getElementById('knobR').style.transform='translate(0px,0px)';document.getElementById('joyLVal').innerText='0°';document.getElementById('joyRVal').innerText='90°';fetch('/api/pet?home=1');}
function petMood(m){fetch('/api/pet?mood='+m);}
function petSay(){let t=document.getElementById('petSayIn').value.trim();if(t)fetch('/api/pet?say='+encodeURIComponent(t));}

function initJoystick(areaId, knobId, axis, onChange, onRelease) {
  const area = document.getElementById(areaId);
  const knob = document.getElementById(knobId);
  const maxR = 44;
  let dragging = false;

  function updatePos(clientX, clientY) {
    const rect = area.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    let dx = clientX - cx;
    let dy = clientY - cy;
    const dist = Math.hypot(dx, dy);
    if (dist > maxR) { dx = (dx / dist) * maxR; dy = (dy / dist) * maxR; }
    knob.style.transform = `translate(${dx}px, ${dy}px)`;
    const normX = dx / maxR;
    const normY = -dy / maxR;
    onChange(axis === 'x' ? normX : normY);
  }

  function releaseJoy() {
    if (!dragging) return;
    dragging = false;
    knob.style.transition = 'transform 0.14s ease-out';
    knob.style.transform = 'translate(0px, 0px)';
    onRelease();
  }

  area.addEventListener('pointerdown', e => {
    dragging = true;
    knob.style.transition = 'none';
    area.setPointerCapture(e.pointerId);
    updatePos(e.clientX, e.clientY);
  });
  area.addEventListener('pointermove', e => { if (dragging) updatePos(e.clientX, e.clientY); });
  area.addEventListener('pointerup', releaseJoy);
  area.addEventListener('pointercancel', releaseJoy);
}

initJoystick('joyL', 'knobL', 'x',
  v => {
    setAutoOff();
    const stp = Math.round(v * 60);
    document.getElementById('joyLVal').innerText = stp + '°';
    const now = Date.now();
    if (now - lastJoySend > 50) { lastJoySend = now; fetch('/api/pet?t=' + stp); }
  },
  () => {
    document.getElementById('joyLVal').innerText = '0°';
    fetch('/api/pet?t=0');
  }
);

initJoystick('joyR', 'knobR', 'y',
  v => {
    setAutoOff();
    const srv = Math.round(90 + v * 75);
    document.getElementById('joyRVal').innerText = srv + '°';
    const now = Date.now();
    if (now - lastJoySend > 50) { lastJoySend = now; fetch('/api/pet?p=' + srv); }
  },
  () => {
    document.getElementById('joyRVal').innerText = '90°';
    fetch('/api/pet?p=90');
  }
);

const wfHistory=[];
function drawMiniHudSpec(ppsArr,chArr){const cv=document.getElementById('miniSpec');const ctx=cv.getContext('2d');ctx.clearRect(0,0,cv.width,cv.height);for(let c=1;c<=13;c++){let h=chArr[c]||1;ctx.fillStyle='#ff8c00';ctx.fillRect((c-1)*5+2,cv.height-h,4,h);}ctx.strokeStyle='#00ff66';ctx.beginPath();for(let i=0;i<32;i++){let v=ppsArr[28+i]||0;let y=cv.height-2-Math.min(cv.height-4,(v/150)*(cv.height-4));let x=70+i*3;if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}ctx.stroke();}
function drawLabInstruments(d){const oCv=document.getElementById('oscCv');const oCtx=oCv.getContext('2d');oCtx.clearRect(0,0,oCv.width,oCv.height);oCtx.strokeStyle='#112218';oCtx.beginPath();for(let x=0;x<oCv.width;x+=32){oCtx.moveTo(x,0);oCtx.lineTo(x,oCv.height);}for(let y=0;y<oCv.height;y+=18){oCtx.moveTo(0,y);oCtx.lineTo(oCv.width,y);}oCtx.stroke();oCtx.strokeStyle='#00ff66';oCtx.lineWidth=2;oCtx.beginPath();for(let i=0;i<60;i++){let val=d.ppsHist[i]||0;let x=(i/59)*(oCv.width-8)+4;let y=oCv.height-5-Math.min(oCv.height-10,(val/180)*(oCv.height-10));if(i===0)oCtx.moveTo(x,y);else oCtx.lineTo(x,y);}oCtx.stroke();oCtx.lineWidth=1;
wfHistory.unshift(d.chLoad.slice(1,14));if(wfHistory.length>16)wfHistory.pop();
const sCv=document.getElementById('specCv');const sCtx=sCv.getContext('2d');sCtx.clearRect(0,0,sCv.width,sCv.height);
const colW=sCv.width/13;for(let c=1;c<=13;c++){let load=d.chLoad[c]||0;let bh=Math.min(36,(load/32)*36);sCtx.fillStyle=load>20?'#ff2a55':'#00f0ff';sCtx.fillRect((c-1)*colW+4,38-bh,colW-8,bh);sCtx.fillStyle='#789';sCtx.font='9px monospace';sCtx.fillText(c,(c-1)*colW+8,48);}
for(let row=0;row<wfHistory.length;row++){for(let c=0;c<13;c++){let v=wfHistory[row][c]||0;if(v>0){let intens=Math.min(255,v*8);sCtx.fillStyle=`rgb(${intens},${Math.floor(intens*0.5)},${255-intens})`;sCtx.fillRect(c*colW+2,51+row*2,colW-4,2);}}}
drawGim('gimL',d.sbus[3],d.sbus[2],'YAW / THR');drawGim('gimR',d.sbus[0],d.sbus[1],'ROLL / PITCH');}
function drawGim(id,xV,yV,lbl){const c=document.getElementById(id);const ctx=c.getContext('2d');ctx.clearRect(0,0,c.width,c.height);const cx=c.width/2,cy=c.height/2;ctx.strokeStyle='#1b2636';ctx.beginPath();ctx.moveTo(cx,0);ctx.lineTo(cx,c.height);ctx.moveTo(0,cy);ctx.lineTo(c.width,cy);ctx.stroke();let px=Math.max(8,Math.min(c.width-8,((xV-172)/(1811-172))*(c.width-16)+8));let py=Math.max(8,Math.min(c.height-8,c.height-8-((yV-172)/(1811-172))*(c.height-16)));ctx.fillStyle='#ff8c00';ctx.beginPath();ctx.arc(px,py,5,0,Math.PI*2);ctx.fill();ctx.fillStyle='#678';ctx.font='9px monospace';ctx.fillText(lbl,5,11);}
let prevDeauth=0;let teleBusy=false;
function pullTele(){if(teleBusy||btnInFlight||(Date.now()-lastInputTime<300))return;teleBusy=true;fetch('/api/telemetry').then(r=>r.json()).then(d=>{teleBusy=false;syncSystemPrefs(d.theme,d.pad);
document.getElementById('procBar').innerText=d.proc;
const pwrLabel=d.usb?'[PWR:USB 5.0V]':`[BAT:${d.bat}%]`;
document.getElementById('pwrMeta').innerText=pwrLabel;
document.getElementById('hPwr').innerText=d.usb?'USB 5.0V':`BAT ${d.bat}%`;
document.getElementById('topMeta').innerText=`RAM:${d.ram}%`;
document.getElementById('cliUp').innerText=`UP: ${d.up}s`;
document.getElementById('oledModeLbl').innerText=d.sniffer?`SNIFFER CH:${d.ch}`:`STEALTH AP (${d.clients})`;
document.getElementById('petRcStat').innerText=d.rcLive?'R81 LINK ACTIVE':'WEB / R81 READY';
document.getElementById('topApSsid').innerText=d.topSsid;
document.getElementById('topApCh').innerText=d.apCnt>0?`CH ${d.topCh}`:'CH --';
document.getElementById('topApAuth').innerText=d.topAuth;
document.getElementById('topApRssi').innerText=d.apCnt>0?`${d.topRssi} dBm`:'-- dBm';
let apPct=d.apCnt>0?Math.max(5,Math.min(100,((d.topRssi+95)/60)*100)):0;
document.getElementById('topApBar').style.width=apPct+'%';
document.getElementById('topBleCnt').innerText=`${d.bleCnt} FOUND`;
document.getElementById('topBleName').innerText=d.bleName;
document.getElementById('topBleMac').innerText=d.bleMac;
document.getElementById('topBleRssi').innerText=d.bleCnt>0?`${d.bleRssi} dBm`:'-- dBm';
let blePct=d.bleCnt>0?Math.max(5,Math.min(100,((d.bleRssi+95)/60)*100)):0;
document.getElementById('topBleBar').style.width=blePct+'%';
document.getElementById('kPps').innerText=d.pps;
document.getElementById('kBcn').innerText=d.bcn;
document.getElementById('kPrb').innerText=d.prb;
document.getElementById('hAp').innerText=d.apCnt;
document.getElementById('hCh').innerText=d.peak;
document.getElementById('hRc').innerText=d.rcLive?'SYNC':'STBY';
document.getElementById('oscPps').innerText=`${d.pps} PKT/S`;
document.getElementById('wfPeak').innerText=`PEAK: CH ${d.peak}`;
document.getElementById('rcSt').innerText=d.rcLive?'R81 ONLINE':'NO SIGNAL';
const idsEl=document.getElementById('kIds');
if(d.deauth>0){idsEl.innerText=`ALRT(${d.deauth})`;idsEl.style.color='#ff2a55';}else{idsEl.innerText='CLEAN';idsEl.style.color='#00ff66';}
if(d.deauth>prevDeauth){printCli(`[!] IDS ALERT: 802.11 Deauth! (Total: ${d.deauth})`);prevDeauth=d.deauth;}
if(d.logs){ document.getElementById('logOut').innerText = d.logs.join('\n'); }
drawMiniHudSpec(d.ppsHist,d.chLoad);drawLabInstruments(d);}).catch(()=>{teleBusy=false;});}
setInterval(()=>{if(!btnInFlight&&(Date.now()-lastInputTime>350))pullOled();},600);
setInterval(pullTele,1000);
pullOled();pullTele();
</script></body></html>)rawliteral";

// =====================================================================
//  FORWARD DECLS
// =====================================================================
const char* authToStr(wifi_auth_mode_t m);
void handleInput();
void renderDipperToWebBuffer();
void renderPhysicalOled();
void drawTopBar(const char* title);
void drawBottomBar(const char* leftHint, const char* rightHint);
void drawDipperMascot(int x, int y, bool frame2);
void drawSpeechBubble(const char* line1, const char* line2);
void drawMainMenu();
void drawListMenu(const char* title, const char** items, int count, int sel);
void drawApp();
void processWiFiScan();
void drawWiFiScan();
void drawWiFiAudit();
void drawWiFiSpectrum();
void drawWiFiRogue();
void drawWiFiProbe();
void drawWiFiDeauth();
void drawWiFiPPSGraph();
void runBLEScan(bool force);
void drawBLEScan();
void drawBLEProx();
void drawBLETracker();
void updateSimulatorHID();
void drawRC8ChMatrix();
void drawRC4ChPrecise();
void drawRCLink();
void drawRCSimulator();
void drawSettingsApp(int idx);
void setupWebServer();
void drawMazeArt();

// =====================================================================
//  WIFI scan & sniffer
// =====================================================================
void startSingleWiFiScan() {
  if (!wifiScanning && !snifferActive) {
    wifiScanning = true;
    WiFi.scanNetworks(true);
    addLog("[WIFI] Scan Started");
  }
}

void IRAM_ATTR snifferCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  const wifi_promiscuous_pkt_t* pkt = (const wifi_promiscuous_pkt_t*)buf;
  uint8_t subtype = pkt->payload[0];
  pktTotal++;
  if (subtype == 0x40) pktProbe++;
  else if (subtype == 0xC0 || subtype == 0xA0) pktDeauth++;
  else if (subtype == 0x80) pktBeacon++;
}

void setSnifferMode(bool enable) {
  if (enable && !snifferActive) {
    WiFi.scanDelete(); wifiScanning = false;
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&snifferCallback);
    snifferActive = true; lastPktTotal = pktTotal; lastPpsCalc = millis();
    addLog("[RF] Sniffer ACTIVE");
  } else if (!enable && snifferActive) {
    esp_wifi_set_promiscuous(false);
    snifferActive = false;
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    addLog("[RF] Sniffer STOPPED");
  }
}

// =====================================================================
//  Async BLE Scan
// =====================================================================
void runBLEScan(bool force) {
  if (bleScanningNow) return;
  bleScanningNow = true;
  bleDevCount = 0;
  addLog("[BLE] Radar Sweep Started");
  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(new ScanCallbacks());
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(99);
  pScan->start(2000, false);
}

// =====================================================================
//  RENDERING: WEB BUFFER vs PHYSICAL OLED
// =====================================================================
void renderDipperToWebBuffer() {
  display.clearDisplay();
  switch (screen) {
    case MAIN_MENU: drawMainMenu(); break;
    case WIFI_MENU: drawListMenu("WIFI AUDIT", wifiMenu, wifiCount, wifiIdx); break;
    case BLE_MENU:  drawListMenu("BLE RADAR",  bleMenu,  bleCount,  bleIdx);  break;
    case RC_MENU:   drawListMenu("RC & SIM",   rcMenu,   rcCount,   rcIdx);   break;
    case SET_MENU:  drawListMenu("SYSTEM",     setMenu,  setCount,  setIdx);  break;
    case APP:       drawApp(); break;
  }
  memcpy(webScreenBuf, display.getBuffer(), 1024);
}

void renderPhysicalOled() {
  unsigned long now = millis();

  // 1. Free Wi-Fi Alert
  if (now < freeWifiAlertUntil) {
    display.clearDisplay();
    if ((now / 250) % 2 == 0) {
      display.drawRect(0, 0, 128, 64, WHITE);
      display.drawRect(2, 2, 124, 60, WHITE);
    }
    display.setTextSize(1); display.setTextColor(WHITE);
    display.setCursor(18, 10); display.print("! FREE WI-FI !");
    display.setCursor(8, 28);
    String s = freeWifiAlertSSID;
    if (s.length() > 18) s = s.substring(0, 18);
    display.print(s);
    display.setCursor(8, 44);
    display.print("RSSI: "); display.print(freeWifiAlertRSSI); display.print(" dBm (OPEN)");
    display.display();
    return;
  }

  // 2. Mode Switch Banner (AUTO PET / MANUAL RC / SIMULATOR)
  if (now < modeBannerUntil) {
    display.clearDisplay();
    display.drawRoundRect(2, 6, 124, 52, 4, WHITE);
    display.drawRoundRect(4, 8, 120, 48, 3, WHITE);
    display.setTextSize(1); display.setTextColor(WHITE);
    int x1 = max(8, (128 - (int)modeBannerLine1.length() * 6) / 2);
    display.setCursor(x1, modeBannerLine2.length() ? 20 : 28);
    display.print(modeBannerLine1);
    if (modeBannerLine2.length() > 0) {
      int x2 = max(8, (128 - (int)modeBannerLine2.length() * 6) / 2);
      display.setCursor(x2, 36);
      display.print(modeBannerLine2);
    }
    display.display();
    return;
  }

  // 3. Active Scan / Sniffer Loading Bar
  if (wifiScanning || bleScanningNow || snifferActive) {
    display.clearDisplay();
    display.setTextSize(1); display.setTextColor(WHITE);
    display.drawRect(0, 0, 128, 64, WHITE);

    display.setCursor(10, 10);
    if (wifiScanning)        display.print("DIPPER: WIFI SCAN");
    else if (bleScanningNow) display.print("DIPPER: BLE RADAR");
    else {
      display.print("SNIFFER CH:"); display.print(snifferCh);
      display.print(" P:"); display.print(currentPPS);
    }

    display.drawRect(10, 28, 108, 12, WHITE);
    int barX = (now / 15) % 84;
    display.fillRect(12 + barX, 30, 22, 8, WHITE);

    display.setCursor(10, 48);
    display.print("AP:"); display.print(wifiNetCount);
    display.print(" BLE:"); display.print(bleDevCount);
    display.display();
    return;
  }

  // 4. RoboEyes
  if (pktDeauth > 0) roboEyes.setMood(ANGRY);
  else if (petManualMood == 1) roboEyes.setMood(HAPPY);
  else if (petManualMood == 2) roboEyes.setMood(TIRED);
  else if (petManualMood == 3) roboEyes.setMood(ANGRY);
  else if (petAutoMove && petState == PET_TIRED) roboEyes.setMood(TIRED);
  else roboEyes.setMood(DEFAULT);

  roboEyes.update();
}

void drawMazeArt() { drawScreen_1(); }

// =====================================================================
//  Web
// =====================================================================
void sendWebScreenBuffer() {
  renderDipperToWebBuffer();
  WiFiClient client = server.client();
  client.setNoDelay(true);
  server.sendHeader("Cache-Control", "no-store, no-cache");
  server.setContentLength(1024);
  server.send(200, "application/octet-stream", "");
  client.write(webScreenBuf, 1024);
}

void setupWebServer() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/api/screen", HTTP_GET, []() { sendWebScreenBuffer(); });

  server.on("/api/pet", HTTP_GET, []() {
    if (server.hasArg("auto")) {
      petAutoMove = (server.arg("auto").toInt() == 1);
      addLog(petAutoMove ? "[PET] MODE: AUTO PET" : "[PET] MODE: MANUAL RC");
      showModeBanner(petAutoMove ? "MODE: AUTO PET" : "MODE: MANUAL RC", petAutoMove ? "AI ACTIVE" : "WEB / TX12", 2000);
    }
    if (server.hasArg("snd")) {
      petSoundOn = (server.arg("snd").toInt() == 1);
      addLog(petSoundOn ? "[PET] Audio: ON" : "[PET] Audio: MUTED");
    }
    if (server.hasArg("home")) {
      petAutoMove = false;
      targetServoDeg = 90.0f;
      setTargetStepDeg(0.0f);
      petManualMood = 0;
      playWarmChirp();
      addLog("[PET] Position HOME");
    }
    if (server.hasArg("p")) {
      petAutoMove = false;
      targetServoDeg = constrain(server.arg("p").toFloat(), 15.0f, 165.0f);
    }
    if (server.hasArg("t")) {
      petAutoMove = false;
      setTargetStepDeg(server.arg("t").toFloat());
    }
    if (server.hasArg("mood")) {
      petManualMood = constrain(server.arg("mood").toInt(), 0, 3);
      playWarmChirp();
    }
    if (server.hasArg("chirp")) { playWarmChirp(); }
    if (server.hasArg("say")) {
      String s = server.arg("say");
      addLog("[PET] Say: " + s);
      speakLifelike(s);
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/api/btn", HTTP_GET, []() {
    String k = server.arg("k");
    if (k == "L") vBtnL = true;
    else if (k == "R") vBtnR = true;
    else if (k == "OK") vBtnOK = true;
    else if (k == "BACK") vBtnBack = true;
    handleInput();
    sendWebScreenBuffer();
  });

  server.on("/api/cli", HTTP_GET, []() {
    String q = server.arg("q"); q.trim();
    String low = q; low.toLowerCase();
    String out = "";
    if (low == "help") {
      out += "=== DipperOS v12.9 ===\n";
      out += "  neofetch  - System info\n";
      out += "  wifi      - AP table\n";
      out += "  scan wifi - Start WiFi scan\n";
      out += "  sniffer   - Toggle IDS Sniffer\n";
      out += "  ble       - BLE list\n";
      out += "  scan ble  - Start BLE scan\n";
      out += "  sbus      - R81 Channels\n";
      out += "  say <txt> - Droid voice\n";
      out += "  theme     - Cycle OLED theme\n";
      out += "  maze      - MAZE ART\n";
      out += "  clear     - Clear screen";
    }
    else if (low == "neofetch" || low == "sys") {
      char procName[32]; getActiveProcessName(procName, sizeof(procName));
      out += "   _.-~-._     root@dipper-os\n";
      out += "  (  o  _ )>   --------------\n";
      out += "  /  `-'  \\    OS     : DipperOS v12.9\n";
      out += " |  |---|  |   ACTIVE : " + String(procName) + "\n";
      out += "  \\_______/    MCU    : ESP32 @ " + String(ESP.getCpuFreqMHz()) + "MHz\n";
      out += "               RAM    : " + String(ESP.getFreeHeap() / 1024) + " KB\n";
      out += "               AP SSID: " + String(AP_SSID) + "\n";
    }
    else if (low == "maze") { parentMenu = SET_MENU; activeApp = 6; screen = APP; out = "[+] MAZE ART active."; }
    else if (low.startsWith("say ")) { String msg = q.substring(4); speakLifelike(msg); out = "[+] Speaking: " + msg; }
    else if (low == "wifi") {
      out += "ID  SSID               CH  RSSI  AUTH\n";
      out += "--------------------------------------\n";
      for (int i = 0; i < wifiNetCount; i++) {
        char line[48];
        snprintf(line, sizeof(line), "%02d  %-18s %-3d %4ddB %s\n", i + 1, wifiList[i].ssid.c_str(), wifiList[i].ch, (int)wifiList[i].rssi, authToStr(wifiList[i].auth));
        out += line;
      }
      if (wifiNetCount <= 0) out += "(Empty. Run: 'scan wifi')\n";
    }
    else if (low == "scan wifi") { setSnifferMode(false); alertedOpenSSIDs = ""; startSingleWiFiScan(); out = "[+] WiFi scan started."; }
    else if (low == "ble") {
      out += "NAME               RSSI    MAC\n";
      out += "-----------------------------------\n";
      for (int i = 0; i < bleDevCount; i++) {
        char line[60];
        snprintf(line, sizeof(line), "%-18s %4ddB  %s\n", bleDevs[i].name.c_str(), bleDevs[i].rssi, bleDevs[i].mac.c_str());
        out += line;
      }
      if (bleDevCount == 0) out += "(Empty. Run: 'scan ble')\n";
    }
    else if (low == "scan ble") { runBLEScan(true); out = "[+] BLE scan started."; }
    else if (low == "sniffer") { setSnifferMode(!snifferActive); out = String("[*] Sniffer: ") + (snifferActive ? "ACTIVE" : "STOPPED"); }
    else if (low == "theme") { currentTheme = (currentTheme + 1) % 4; setMenu[0] = themeLabels[currentTheme]; out = String("[*] Theme: ") + setMenu[0]; }
    else if (low == "sbus") {
      out += String("R81: ") + ((sbusValid && millis() - sbusLastRx < 500) ? "ONLINE" : "NO SIG") + " (Frames: " + String(sbusFrameCount) + ")\n";
      for (int i = 0; i < 8; i++) out += "CH" + String(i + 1) + ":" + String(sbusCh[i]) + (i == 3 ? "\n" : "  ");
    }
    else { out = "Unknown: '" + q + "'. Type 'help'."; }
    server.send(200, "text/plain", out);
  });

  server.on("/api/telemetry", HTTP_GET, []() {
    bool rcLive = sbusValid && (millis() - sbusLastRx < 500);
    int ramPct = (int)((ESP.getFreeHeap() * 100) / ESP.getHeapSize());
    char procStr[32]; getActiveProcessName(procStr, sizeof(procStr));

    String topSsid = "None"; int topRssi = -100; int topCh = 0; String topAuth = "---";
    if (wifiNetCount > 0) {
      int best = 0;
      for (int i = 1; i < wifiNetCount; i++) if (wifiList[i].rssi > wifiList[best].rssi) best = i;
      topSsid = wifiList[best].ssid.length() ? wifiList[best].ssid : "<Hidden>";
      topSsid.replace("\"", ""); topRssi = wifiList[best].rssi; topCh = wifiList[best].ch; topAuth = authToStr(wifiList[best].auth);
    }
    String bleName = "None"; String bleMac = "--:--:--"; int bleRssi = -100;
    if (bleDevCount > 0) {
      int bestB = 0;
      for (int i = 1; i < bleDevCount; i++) if (bleDevs[i].rssi > bleDevs[bestB].rssi) bestB = i;
      bleName = bleDevs[bestB].name; bleName.replace("\"", ""); bleMac = bleDevs[bestB].mac.substring(9); bleRssi = bleDevs[bestB].rssi;
    }

    String json; json.reserve(1500);
    json = "{";
    json += "\"proc\":\"" + String(procStr) + "\",";
    json += "\"ram\":" + String(ramPct) + ",";
    json += "\"up\":" + String(millis() / 1000) + ",";
    json += "\"usb\":" + String(isUsbPowered ? "true" : "false") + ",";
    json += "\"bat\":" + String(batteryPct) + ",";
    json += "\"theme\":" + String(currentTheme) + ",";
    json += "\"pad\":" + String(showHwPad ? "true" : "false") + ",";
    json += "\"sniffer\":" + String(snifferActive ? "true" : "false") + ",";
    json += "\"idle\":" + String(idlePocketAIActive ? "true" : "false") + ",";
    json += "\"ch\":" + String(snifferCh) + ",";
    json += "\"peak\":" + String(peakChannel) + ",";
    json += "\"pps\":" + String(currentPPS) + ",";
    json += "\"bcn\":" + String(pktBeacon) + ",";
    json += "\"prb\":" + String(pktProbe) + ",";
    json += "\"deauth\":" + String(pktDeauth) + ",";
    json += "\"apCnt\":" + String(wifiNetCount) + ",";
    json += "\"bleCnt\":" + String(bleDevCount) + ",";
    json += "\"clients\":" + String(WiFi.softAPgetStationNum()) + ",";
    json += "\"rcLive\":" + String(rcLive ? "true" : "false") + ",";
    json += "\"topSsid\":\"" + topSsid + "\",";
    json += "\"topRssi\":" + String(topRssi) + ",";
    json += "\"topCh\":" + String(topCh) + ",";
    json += "\"topAuth\":\"" + topAuth + "\",";
    json += "\"bleName\":\"" + bleName + "\",";
    json += "\"bleMac\":\"" + bleMac + "\",";
    json += "\"bleRssi\":" + String(bleRssi) + ",";
    json += "\"logs\":[";
    for(int i=0; i<6; i++) {
      if(sysLogs[i] != "") {
        json += "\"" + sysLogs[i] + "\"";
        if(i < 5 && sysLogs[i+1] != "") json += ",";
      }
    }
    json += "],\"sbus\":[";
    for (int i = 0; i < 8; i++) json += String(sbusCh[i]) + (i < 7 ? "," : "");
    json += "],\"ppsHist\":[";
    for (int i = 0; i < 60; i++) json += String(ppsHistory[i]) + (i < 59 ? "," : "");
    json += "],\"chLoad\":[";
    for (int i = 0; i < 15; i++) json += String(channelLoad[i]) + (i < 14 ? "," : "");
    json += "]}";
    WiFiClient client = server.client(); client.setNoDelay(true);
    server.send(200, "application/json", json);
  });

  server.begin();
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial2.setRxBufferSize(1024);
  Serial2.begin(SBUS_BAUD, SERIAL_8E2, SBUS_RX_PIN, -1, SBUS_INVERT);
  pinMode(BAT_ADC_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false);
  display.setTextWrap(false);
  display.clearDisplay();

  roboEyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 30);
  roboEyes.setAutoblinker(ON, 3, 2);
  roboEyes.setIdleMode(ON, 2, 2);
  roboEyes.setMood(DEFAULT);

  ESP32PWM::allocateTimer(0);
  headServo.setPeriodHertz(50);
  headServo.attach(SERVO_PIN, 500, 2400);
  headServo.writeMicroseconds(1450); // 90° középállás
  tiltStepper.setSpeed(14);
  delay(150);

  WiFi.persistent(false);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setTxPower(WIFI_POWER_15dBm);
  WiFi.setSleep(false);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);
  delay(250);

  updatePowerSupply();
  setupWebServer();
  initNativeBleGamepad();

  // 1. SBUS olvasó szál (Core 0, Priority 1 -> nem akadályozza a Wi-Fi-t)
  xTaskCreatePinnedToCore(sbusTask, "SbusTask", 2048, NULL, 1, NULL, 0);

  // 2. Motor vezérlő szál (Core 1, Priority 2 -> Stepper.h + finom szervó)
  xTaskCreatePinnedToCore(motorTask, "MotorTask", 2048, NULL, 2, NULL, 1);

  // Induláskori rövid stepper teszt (10° jobbra, majd vissza 0°-ra)
  setTargetStepDeg(10.0f);
  delay(650);
  setTargetStepDeg(0.0f);

  playWarmStartup();
  addLog("[SYS] DipperOS v12.9 Final Booted");
  showModeBanner("DIPPER OS v12.9", "MANUAL RC READY", 1800);
  lastUserInput = millis();
  petStateEnter = millis();
  screen = MAIN_MENU;
  renderPhysicalOled();
}

// =====================================================================
//  LOOP
// =====================================================================
void loop() {
  unsigned long now = millis();

  server.handleClient();
  updateSimulatorHID();
  updatePowerSupply();
  updateRobotPetLogic(now);

  if (triggerBleScan) {
    triggerBleScan = false;
    runBLEScan(true);
  }

  if (bleScanningNow) {
    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (!pScan->isScanning()) {
      bleScanningNow = false;
      pScan->clearResults();
      addLog("[BLE] Scan Complete: " + String(bleDevCount) + " devices");
    }
  }

  if (snifferActive) {
    if (now - lastChHop > 350) {
      lastChHop = now; snifferCh = (snifferCh % 13) + 1;
      esp_wifi_set_channel(snifferCh, WIFI_SECOND_CHAN_NONE);
    }
    if (now - lastPpsCalc >= 250) {
      uint32_t diff = pktTotal - lastPktTotal; lastPktTotal = pktTotal; currentPPS = diff * 4;
      for (int i = 0; i < 59; i++) ppsHistory[i] = ppsHistory[i + 1];
      ppsHistory[59] = currentPPS; lastPpsCalc = now;
    }
  } else { processWiFiScan(); }

  static unsigned long lastPhysDraw = 0;
  if (now - lastPhysDraw >= 33) {
    lastPhysDraw = now;
    renderPhysicalOled();
  }
}

void updateSimulatorHID() {
  unsigned long now = millis();
  if (!sbusValid || (now - sbusLastRx > 500)) return;
  if (now - lastSimReport < 14) return;
  lastSimReport = now;

  if (simModeEnabled && bleHidConnected && inputGamepad != nullptr) {
    uint8_t report[7]; uint8_t buttons = 0;
    for (int b = 0; b < 4; b++) if (sbusCh[4 + b] > 1050) buttons |= (1 << b);
    report[0] = buttons;
    for (int a = 0; a < 6; a++) {
      int v = constrain((int)sbusCh[a], 172, 1811);
      report[1 + a] = (uint8_t)map(v, 172, 1811, -127, 127);
    }
    inputGamepad->setValue(report, sizeof(report));
    inputGamepad->notify();
  }
  if (serialSimBridge) {
    Serial.print("$SBUS");
    for (int i = 0; i < 8; i++) { Serial.print(","); Serial.print(sbusCh[i]); }
    Serial.println();
  }
}

// =====================================================================
//  INPUT
// =====================================================================
void handleInput() {
  lastUserInput = millis();

  if (vBtnBack) {
    vBtnBack = false; setSnifferMode(false);
    if (screen == APP) screen = parentMenu;
    else if (screen != MAIN_MENU) screen = MAIN_MENU;
    return;
  }

  if (screen == MAIN_MENU) {
    if (vBtnL) { vBtnL = false; mainIdx = (mainIdx - 1 + mainCount) % mainCount; }
    if (vBtnR) { vBtnR = false; mainIdx = (mainIdx + 1) % mainCount; }
    if (vBtnOK) {
      vBtnOK = false;
      if (mainIdx == 0)      { screen = WIFI_MENU; wifiIdx = 0; }
      else if (mainIdx == 1) { screen = BLE_MENU;  bleIdx = 0; }
      else if (mainIdx == 2) { screen = RC_MENU;   rcIdx = 0; }
      else if (mainIdx == 3) { parentMenu = MAIN_MENU; activeApp = 3; screen = APP; }
      else if (mainIdx == 4) { screen = SET_MENU;  setIdx = 0; }
    }
    return;
  }

  if (screen == WIFI_MENU || screen == BLE_MENU || screen == RC_MENU || screen == SET_MENU) {
    int count   = (screen == WIFI_MENU) ? wifiCount : (screen == BLE_MENU) ? bleCount : (screen == RC_MENU) ? rcCount : setCount;
    int* idxPtr = (screen == WIFI_MENU) ? &wifiIdx  : (screen == BLE_MENU) ? &bleIdx  : (screen == RC_MENU) ? &rcIdx  : &setIdx;
    if (vBtnL) { vBtnL = false; *idxPtr = (*idxPtr - 1 + count) % count; }
    if (vBtnR) { vBtnR = false; *idxPtr = (*idxPtr + 1) % count; }
    if (vBtnOK) {
      vBtnOK = false;
      int sel = *idxPtr;
      if (sel == count - 1) { screen = MAIN_MENU; return; }

      if (screen == SET_MENU && sel == 0) { currentTheme = (currentTheme + 1) % 4; setMenu[0] = themeLabels[currentTheme]; return; }
      if (screen == SET_MENU && sel == 1) { showHwPad = !showHwPad; setMenu[1] = showHwPad ? "HW Pad: SHOW" : "HW Pad: HIDE"; return; }
      if (screen == SET_MENU && sel == 2) {
        robotPetEnabled = !robotPetEnabled;
        setMenu[2] = robotPetEnabled ? "Robot Pet: ON" : "Robot Pet: OFF";
        showModeBanner(robotPetEnabled ? "ROBOT PET: ON" : "ROBOT PET: OFF", "", 1800);
        return;
      }

      parentMenu = screen; activeApp = sel; screen = APP;
      if (parentMenu == WIFI_MENU && (activeApp >= 4 && activeApp <= 6)) setSnifferMode(true);
      if (parentMenu == RC_MENU && activeApp == 3) {
        showModeBanner("MODE: SIMULATOR", simModeEnabled ? "BLE HID: ON" : "BLE HID: OFF", 2000);
      }
    }
    return;
  }

  if (screen == APP) {
    if (vBtnOK) { vBtnOK = false; setSnifferMode(false); screen = parentMenu; return; }
    if (vBtnL) {
      vBtnL = false;
      if (parentMenu == WIFI_MENU && activeApp == 0 && wifiScroll > 0) wifiScroll--;
      if (parentMenu == BLE_MENU  && activeApp == 0 && bleScroll > 0)  bleScroll--;
      if (parentMenu == WIFI_MENU && (activeApp >= 4 && activeApp <= 6)) {
        pktProbe = 0; pktDeauth = 0; pktBeacon = 0; pktTotal = 0; lastPktTotal = 0;
        memset(ppsHistory, 0, sizeof(ppsHistory));
        addLog("[RF] Sniffer counters reset");
      }
      if (parentMenu == RC_MENU && activeApp == 3) {
        simModeEnabled = !simModeEnabled;
        addLog(simModeEnabled ? "[SIM] BLE HID: ON" : "[SIM] BLE HID: OFF");
        showModeBanner("MODE: SIMULATOR", simModeEnabled ? "BLE HID: ON" : "BLE HID: OFF", 2000);
      }
    }
    if (vBtnR) {
      vBtnR = false;
      if (parentMenu == WIFI_MENU && activeApp == 0) {
        if (wifiNetCount > 0 && wifiScroll < wifiNetCount - 1) wifiScroll++;
        else startSingleWiFiScan();
      }
      if (parentMenu == BLE_MENU && activeApp == 0) {
        if (bleDevCount > 0 && bleScroll < bleDevCount - 1) bleScroll++;
        else triggerBleScan = true;
      } else if (parentMenu == BLE_MENU) { triggerBleScan = true; }
      if (parentMenu == WIFI_MENU && activeApp == 3 && wifiNetCount > 0) {
        mySSID = wifiList[wifiScroll].ssid; myBSSID = wifiList[wifiScroll].bssid;
      }
      if (parentMenu == RC_MENU && activeApp == 3) {
        serialSimBridge = !serialSimBridge;
        addLog(serialSimBridge ? "[SIM] USB Bridge: ON" : "[SIM] USB Bridge: OFF");
        showModeBanner("MODE: SIMULATOR", serialSimBridge ? "USB BRIDGE: ON" : "USB BRIDGE: OFF", 2000);
      }
    }
  }
}

// =====================================================================
//  Draw Bars & Menus
// =====================================================================
void drawTopBar(const char* title) {
  display.fillRect(0, 0, 128, 11, WHITE);
  display.setTextColor(BLACK); display.setTextSize(1);
  char safeTitle[14]; strncpy(safeTitle, title, 13); safeTitle[13] = '\0';
  display.setCursor(2, 2); display.print(safeTitle);
  display.fillRect(80, 0, 48, 11, WHITE);
  display.drawFastVLine(80, 0, 11, BLACK);

  if (wifiScanning || bleScanningNow || snifferActive) {
    int f = (millis() / 150) % 4;
    display.drawPixel(84, 3, BLACK); display.drawPixel(85, 4, BLACK); display.drawPixel(84, 5, BLACK); display.drawPixel(83, 4, BLACK);
    if(f==0) display.fillRect(86, 2, 3, 3, BLACK);
    else if(f==1) display.fillRect(88, 4, 3, 3, BLACK);
    else if(f==2) display.fillRect(86, 6, 3, 3, BLACK);
    else display.fillRect(84, 4, 3, 3, BLACK);
  } else {
    display.setCursor(83, 2);
    if (isUsbPowered) display.print("USB");
    else { display.print("B"); display.print(constrain(batteryPct, 0, 99)); }
    int ramPct = constrain((int)((ESP.getFreeHeap() * 100) / ESP.getHeapSize()), 0, 99);
    display.drawFastVLine(103, 0, 11, BLACK);
    display.setCursor(106, 2);
    if (ramPct < 10) display.print(" ");
    display.print(ramPct); display.print("%");
  }
  display.setTextColor(WHITE);
}

void drawBottomBar(const char* leftHint, const char* rightHint) {
  display.fillRect(0, 54, 128, 10, BLACK);
  display.drawFastHLine(0, 54, 128, WHITE);
  display.setTextSize(1); display.setTextColor(WHITE);
  char safeLeft[12]; strncpy(safeLeft, leftHint, 11); safeLeft[11] = '\0';
  display.setCursor(2, 56); display.print(safeLeft);
  char safeRight[11]; strncpy(safeRight, rightHint, 10); safeRight[10] = '\0';
  int rLen = strlen(safeRight) * 6;
  display.setCursor(126 - rLen, 56); display.print(safeRight);
}

void drawDipperMascot(int x, int y, bool frame2) {
  int dy = frame2 ? y - 1 : y;
  display.fillRoundRect(x + 4, dy + 4, 20, 14, 5, WHITE);
  display.fillTriangle(x + 23, dy + 8, x + 31, dy + 11, x + 23, dy + 14, WHITE);
  display.fillTriangle(x + 8, dy + 4, x + 13, dy, x + 16, dy + 4, WHITE);
  display.fillTriangle(x, dy + 12, x + 5, dy + 9, x + 5, dy + 16, WHITE);
  display.fillRect(x + 15, dy + 7, 7, 5, BLACK);
  if (!frame2) display.fillRect(x + 17, dy + 8, 3, 3, WHITE);
  else display.drawFastHLine(x + 16, dy + 9, 5, WHITE);
  display.drawFastHLine(x + 19, dy + 14, 5, BLACK);
  if (frame2) display.drawCircle(x + 29, dy + 3, 2, WHITE);
  else display.drawPixel(x + 27, dy + 5, WHITE);
}

void drawSpeechBubble(const char* line1, const char* line2) {
  display.drawRoundRect(38, 13, 88, 23, 3, WHITE);
  display.fillTriangle(33, 25, 38, 22, 38, 28, WHITE);
  display.drawLine(38, 23, 38, 27, BLACK);
  char s1[14], s2[14];
  strncpy(s1, line1, 13); s1[13] = '\0';
  strncpy(s2, line2, 13); s2[13] = '\0';
  display.setTextColor(WHITE);
  display.setCursor(43, 16); display.print(s1);
  display.setCursor(43, 25); display.print(s2);
}

void drawMainMenu() {
  unsigned long now = millis();
  drawTopBar("DipperOS v12.9");
  bool animFrame = ((now / 450) % 2) == 1;
  drawDipperMascot(2, 15, animFrame);

  bool rcLive = sbusValid && (now - sbusLastRx < 500);
  if (pktDeauth > 0) drawSpeechBubble("! DEAUTH !", "Under Attack!");
  else if (petAutoMove) drawSpeechBubble("MODE:", "AUTO PET AI");
  else if (rcLive)   drawSpeechBubble("R81 TX12:", "Manual Live");
  else if (snifferActive) {
    char b2[14]; snprintf(b2, sizeof(b2), "CH:%d PPS:%d", snifferCh, currentPPS);
    drawSpeechBubble("IDS Sniffer:", b2);
  } else {
    int phase = (now / 3000) % 3;
    if (phase == 0) {
      char b2[14]; snprintf(b2, sizeof(b2), "%d APs Found", wifiNetCount);
      drawSpeechBubble("RF Spectrum:", b2);
    } else if (phase == 1) {
      drawSpeechBubble("Power State:", isUsbPowered ? "USB Cable 5V" : "Battery Mode");
    } else {
      char b2[14]; snprintf(b2, sizeof(b2), "BLE:%d Cli:%d", bleDevCount, WiFi.softAPgetStationNum());
      drawSpeechBubble("DipperOS OK!", b2);
    }
  }

  display.fillRoundRect(0, 40, 128, 24, 3, WHITE);
  display.setTextColor(BLACK);
  display.setCursor(3, 48); display.print("<");
  display.drawBitmap(12, 44, mainIcons[mainIdx], 16, 16, BLACK);
  display.setCursor(32, 44); display.print(mainMenu[mainIdx]);
  display.setCursor(98, 44); display.print(mainIdx + 1); display.print("/5");
  display.drawFastHLine(32, 53, 84, BLACK);
  display.setCursor(32, 55);
  if (mainIdx == 0)      display.print("802.11 & IDS");
  else if (mainIdx == 1) display.print("BLE & AirTag");
  else if (mainIdx == 2) display.print("SBUS & Sim");
  else if (mainIdx == 3) display.print("Pocket AI Robot");
  else                   display.print("Themes & Cfg");
  display.setCursor(120, 48); display.print(">");
  display.setTextColor(WHITE);
}

void drawListMenu(const char* title, const char** items, int count, int sel) {
  drawTopBar(title);
  int visibleRows = 4; int start = 0;
  if (sel >= visibleRows) start = sel - visibleRows + 1;
  if (start > count - visibleRows) start = count - visibleRows;
  if (start < 0) start = 0;
  for (int i = start; i < count && i < start + visibleRows; i++) {
    int y = 13 + (i - start) * 10;
    char safeItem[18]; strncpy(safeItem, items[i], 17); safeItem[17] = '\0';
    if (i == sel) {
      display.fillRoundRect(0, y, 122, 10, 2, WHITE);
      display.setTextColor(BLACK);
      display.setCursor(2, y + 1);  display.print(">");
      display.setCursor(10, y + 1); display.print(safeItem);
    } else {
      display.setTextColor(WHITE);
      display.drawPixel(4, y + 4, WHITE);
      display.setCursor(10, y + 1); display.print(safeItem);
    }
  }
  display.drawRect(124, 13, 4, 40, WHITE);
  int barH = max(6, 40 / count);
  int barY = 13 + ((40 - barH) * sel) / max(1, count - 1);
  display.fillRect(125, barY, 2, barH, WHITE);
  drawBottomBar("[<>]Select", "[OK]Open");
}

void drawApp() {
  if (parentMenu == MAIN_MENU) {
    drawTopBar("Pocket AI");
    display.setCursor(12, 24); display.print("Active on OLED");
    display.setCursor(12, 36); display.print("Use [4] PET / R81");
    drawBottomBar("[OK]Back", "");
    return;
  }

  const char* toolName = "";
  if (parentMenu == WIFI_MENU)      toolName = wifiMenu[activeApp];
  else if (parentMenu == BLE_MENU)  toolName = bleMenu[activeApp];
  else if (parentMenu == RC_MENU)   toolName = rcMenu[activeApp];
  else                              toolName = setMenu[activeApp];
  drawTopBar(toolName);

  if (parentMenu == WIFI_MENU) {
    switch (activeApp) {
      case 0: drawWiFiScan(); break;
      case 1: drawWiFiAudit(); break;
      case 2: drawWiFiSpectrum(); break;
      case 3: drawWiFiRogue(); break;
      case 4: drawWiFiProbe(); break;
      case 5: drawWiFiDeauth(); break;
      case 6: drawWiFiPPSGraph(); break;
    }
  } else if (parentMenu == BLE_MENU) {
    switch (activeApp) {
      case 0: drawBLEScan(); break;
      case 1: drawBLEProx(); break;
      case 2: drawBLETracker(); break;
    }
  } else if (parentMenu == RC_MENU) {
    switch (activeApp) {
      case 0: drawRC8ChMatrix(); break;
      case 1: drawRC4ChPrecise(); break;
      case 2: drawRCLink(); break;
      case 3: drawRCSimulator(); break;
    }
  } else if (parentMenu == SET_MENU) {
    if (activeApp == 6) drawMazeArt();
    else drawSettingsApp(activeApp);
  }
}

// =====================================================================
//  WiFi Apps + FREE WI-FI DETECTOR
// =====================================================================
const char* authToStr(wifi_auth_mode_t m) {
  switch (m) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP:  return "WEP ";
    case WIFI_AUTH_WPA_PSK: return "WPA ";
    case WIFI_AUTH_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA3_PSK: return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA3";
    default: return "ENT ";
  }
}

void processWiFiScan() {
  int n = WiFi.scanComplete();
  if (n >= 0) {
    wifiScanning = false;
    wifiNetCount = min(n, 25);
    for (int c = 1; c <= 14; c++) channelLoad[c] = 0;
    int maxLoad = -1;
    String foundFreeSSID = "";
    int foundFreeRSSI = -100;

    for (int i = 0; i < wifiNetCount; i++) {
      wifiList[i].ssid  = WiFi.SSID(i);
      wifiList[i].bssid = WiFi.BSSIDstr(i);
      wifiList[i].rssi  = WiFi.RSSI(i);
      wifiList[i].ch    = WiFi.channel(i);
      wifiList[i].auth  = WiFi.encryptionType(i);
      int ch = wifiList[i].ch;
      if (ch >= 1 && ch <= 14) {
        int rssiClamped = constrain(wifiList[i].rssi, -95, -30);
        int s = map(rssiClamped, -95, -30, 4, 32);
        if (s > channelLoad[ch]) channelLoad[ch] = s;
        if (channelLoad[ch] > maxLoad) { maxLoad = channelLoad[ch]; peakChannel = ch; }
      }
      if (wifiList[i].auth == WIFI_AUTH_OPEN && wifiList[i].ssid.length() > 0) {
        String key = "|" + wifiList[i].ssid + "|";
        if (alertedOpenSSIDs.indexOf(key) == -1) {
          alertedOpenSSIDs += key;
          foundFreeSSID = wifiList[i].ssid;
          foundFreeRSSI = wifiList[i].rssi;
        }
      }
    }
    if (wifiScroll >= wifiNetCount) wifiScroll = max(0, wifiNetCount - 1);
    WiFi.scanDelete();
    addLog("[WIFI] Scan Complete: " + String(wifiNetCount) + " APs");

    if (foundFreeSSID.length() > 0) {
      freeWifiAlertSSID  = foundFreeSSID;
      freeWifiAlertRSSI  = foundFreeRSSI;
      freeWifiAlertUntil = millis() + 5500;
      targetServoDeg = 115.0f;
      setTargetStepDeg(0.0f);
      addLog("[!] FREE WIFI: " + foundFreeSSID);
      playFreeWifiAlertSound();
    }
  }
}

void drawWiFiScan() {
  if (wifiScanning) { display.setCursor(18, 28); display.print("Scanning RF..."); drawBottomBar("Wait", "[OK]X"); return; }
  if (wifiNetCount == 0) { display.setCursor(12, 28); display.print("Empty ([>]Scan)"); drawBottomBar("[>]Scan", "[OK]X"); return; }
  for (int i = 0; i < 3; i++) {
    int idx = wifiScroll + i; if (idx >= wifiNetCount) break;
    int y = 13 + i * 9;
    if (i == 0) { display.fillRect(0, y, 128, 9, WHITE); display.setTextColor(BLACK); }
    else display.setTextColor(WHITE);
    String ssid = wifiList[idx].ssid; if (ssid.length() == 0) ssid = "<Hidden>";
    if (ssid.length() > 14) ssid = ssid.substring(0, 14);
    display.setCursor(2, y + 1); display.print(ssid);
    char buf[8]; snprintf(buf, sizeof(buf), "%3ddB", (int)wifiList[idx].rssi);
    display.setCursor(96, y + 1); display.print(buf);
  }
  display.setTextColor(WHITE);
  display.drawFastHLine(0, 41, 128, WHITE);
  WifiEntry &sel = wifiList[wifiScroll];
  display.setCursor(2, 43); display.print("C"); display.print(sel.ch);
  display.print(" "); display.print(authToStr(sel.auth));
  display.print(" "); display.print(sel.bssid.substring(9));
  char nav[11]; snprintf(nav, sizeof(nav), "%d/%d [OK]", wifiScroll + 1, wifiNetCount);
  drawBottomBar("[<>]Scroll", nav);
}

void drawWiFiAudit() {
  int openCnt = 0, wepCnt = 0, wpa2Cnt = 0, wpa3Cnt = 0;
  for (int i = 0; i < wifiNetCount; i++) {
    if (wifiList[i].auth == WIFI_AUTH_OPEN) openCnt++;
    else if (wifiList[i].auth == WIFI_AUTH_WEP) wepCnt++;
    else if (wifiList[i].auth == WIFI_AUTH_WPA3_PSK || wifiList[i].auth == WIFI_AUTH_WPA2_WPA3_PSK) wpa3Cnt++;
    else wpa2Cnt++;
  }
  display.drawFastHLine(0, 25, 128, WHITE);
  display.setCursor(2, 28); display.print("OPEN :"); display.print(openCnt);
  display.setCursor(2, 38); display.print("WEP  :"); display.print(wepCnt);
  display.setCursor(2, 48); display.print("WPA2 :"); display.print(wpa2Cnt);
  display.setCursor(64, 28); display.print("WPA3 :"); display.print(wpa3Cnt);
  display.setCursor(64, 38); display.print("Total:"); display.print(wifiNetCount);
  drawBottomBar("[OK]Back", "");
}

void drawWiFiSpectrum() {
  for (int c = 1; c <= 13; c++) {
    int x = 4 + (c - 1) * 9;
    int h = min(30, channelLoad[c]);
    display.fillRect(x, 40 - h, 7, h, WHITE);
    display.setCursor(x + 2, 42); display.print(c == 10 ? "0" : String(c).substring(0,1));
  }
  display.setCursor(2, 54); display.print("Peak CH:"); display.print(peakChannel);
  drawBottomBar("[OK]Back", "");
}

void drawWiFiRogue() {
  display.drawFastHLine(0, 25, 128, WHITE);
  display.setCursor(2, 30); display.print(wifiNetCount > 0 ? "Monitoring... OK" : "No APs");
  display.setCursor(2, 42); display.print("Deauth:"); display.print(pktDeauth);
  drawBottomBar("[OK]Back", "");
}

void drawWiFiProbe() {
  display.drawFastHLine(0, 25, 128, WHITE);
  display.setCursor(2, 30); display.print("Probes:"); display.print(pktProbe);
  display.setCursor(2, 42); display.print("Total :"); display.print(pktTotal);
  display.setCursor(2, 54); display.print("PPS   :"); display.print(currentPPS);
  drawBottomBar("[L]Reset", "[OK]Back");
}

void drawWiFiDeauth() {
  display.drawFastHLine(0, 25, 128, WHITE);
  if (pktDeauth > 0) {
    display.setCursor(2, 30); display.print("!! ATTACK !!");
    display.setCursor(2, 42); display.print("Count:"); display.print(pktDeauth);
  } else {
    display.setCursor(2, 30); display.print("Status: CLEAN");
    display.setCursor(2, 42); display.print("Beacons:"); display.print(pktBeacon);
  }
  drawBottomBar("[L]Reset", "[OK]Back");
}

void drawWiFiPPSGraph() {
  display.setCursor(2, 12); display.print("PPS:");
  display.setCursor(30, 12); display.print(currentPPS);
  int maxV = 1;
  for (int i = 0; i < 60; i++) if (ppsHistory[i] > maxV) maxV = ppsHistory[i];
  for (int i = 0; i < 60; i++) {
    int h = (ppsHistory[i] * 34) / maxV;
    if (h > 34) h = 34;
    int x = 2 + i * 2;
    display.drawFastVLine(x, 50 - h, h, WHITE);
  }
  drawBottomBar("[OK]Back", "");
}

// =====================================================================
//  BLE Apps
// =====================================================================
void drawBLEScan() {
  if (bleScanningNow) { display.setCursor(18, 28); display.print("BLE Scanning..."); drawBottomBar("Wait", "[OK]X"); return; }
  if (bleDevCount == 0) { display.setCursor(12, 28); display.print("Empty ([>]Scan)"); drawBottomBar("[>]Scan", "[OK]X"); return; }
  for (int i = 0; i < 3; i++) {
    int idx = bleScroll + i; if (idx >= bleDevCount) break;
    int y = 13 + i * 9;
    if (i == 0) { display.fillRect(0, y, 128, 9, WHITE); display.setTextColor(BLACK); }
    else display.setTextColor(WHITE);
    String nm = bleDevs[idx].name; if (nm.length() > 14) nm = nm.substring(0, 14);
    display.setCursor(2, y + 1); display.print(nm);
    char buf[8]; snprintf(buf, sizeof(buf), "%4ddB", bleDevs[idx].rssi);
    display.setCursor(96, y + 1); display.print(buf);
  }
  display.setTextColor(WHITE);
  display.drawFastHLine(0, 41, 128, WHITE);
  display.setCursor(2, 43); display.print(bleDevs[bleScroll].mac);
  drawBottomBar("[<>]Scroll", "[OK]X");
}

void drawBLEProx() {
  display.drawFastHLine(0, 25, 128, WHITE);
  if (bleDevCount > 0) {
    int best = 0;
    for (int i = 1; i < bleDevCount; i++) if (bleDevs[i].rssi > bleDevs[best].rssi) best = i;
    display.setCursor(2, 30); display.print(bleDevs[best].name.substring(0, 17));
    display.setCursor(2, 42); display.print("RSSI:"); display.print(bleDevs[best].rssi); display.print("dB");
    int b = constrain(map(bleDevs[best].rssi, -100, -30, 0, 100), 0, 100);
    display.drawRect(2, 54, 124, 4, WHITE);
    display.fillRect(2, 54, b * 124 / 100, 4, WHITE);
  } else { display.setCursor(2, 30); display.print("No BLE devices"); }
  drawBottomBar("[OK]Back", "");
}

void drawBLETracker() {
  display.drawFastHLine(0, 25, 128, WHITE);
  int tagCount = 0;
  for (int i = 0; i < bleDevCount; i++) if (bleDevs[i].name.indexOf("AirTag") >= 0 || bleDevs[i].name.indexOf("Tag") >= 0) tagCount++;
  display.setCursor(2, 30); display.print("Tag-like:"); display.print(tagCount);
  display.setCursor(2, 42); display.print("Total   :"); display.print(bleDevCount);
  drawBottomBar("[>]Scan", "[OK]Back");
}

// =====================================================================
//  RC Apps
// =====================================================================
void drawRC8ChMatrix() {
  for (int i = 0; i < 8; i++) {
    int col = i % 4; int row = i / 4;
    int x = 2 + col * 32; int y = 14 + row * 16;
    display.setCursor(x, y); display.print("CH"); display.print(i + 1);
    display.setCursor(x, y + 8); display.print(sbusCh[i]);
  }
  drawBottomBar("[OK]Back", "");
}

void drawRC4ChPrecise() {
  for (int i = 0; i < 4; i++) {
    display.setCursor(2, 24 + i * 8); display.print("CH"); display.print(i + 1);
    display.print(": "); display.print(sbusCh[i]);
  }
  drawBottomBar("[OK]Back", "");
}

void drawRCLink() {
  bool live = sbusValid && (millis() - sbusLastRx < 500);
  display.drawFastHLine(0, 25, 128, WHITE);
  display.setCursor(2, 30); display.print("Link :"); display.print(live ? "ONLINE" : "NO SIGNAL");
  display.setCursor(2, 42); display.print("Frame:"); display.print(sbusFrameCount);
  display.setCursor(2, 54); display.print("Last :"); display.print(millis() - sbusLastRx); display.print("ms");
  drawBottomBar("[OK]Back", "");
}

void drawRCSimulator() {
  display.drawFastHLine(0, 25, 128, WHITE);
  display.setCursor(2, 30); display.print("Mode:"); display.print(simModeEnabled ? "HID ON" : "OFF");
  display.setCursor(2, 42); display.print("BLE :"); display.print(bleHidConnected ? "CONNECTED" : "ADVERT");
  display.setCursor(2, 54); display.print("SBUS:"); display.print(serialSimBridge ? "BRIDGE" : "OFF");
  drawBottomBar("[L]HID", "[R]Bridge");
}

// =====================================================================
//  Settings App
// =====================================================================
void drawSettingsApp(int idx) {
  if (idx == 2) {
    display.setCursor(2, 14); display.print("Robot Pet Setting");
    display.drawFastHLine(0, 25, 128, WHITE);
    display.setCursor(2, 30); display.print("Status:");
    display.setCursor(2, 42); display.print(setMenu[2]);
    display.setCursor(2, 54); display.print("Press [OK] to toggle");
  } else if (idx == 3) {
    display.setCursor(2, 14); display.print("Power & SoC");
    display.drawFastHLine(0, 25, 128, WHITE);
    display.setCursor(2, 30); display.print(isUsbPowered ? "PWR : USB 5V" : "PWR : Battery");
    display.setCursor(2, 42); display.print("BAT :"); display.print(batteryPct); display.print("%");
    display.setCursor(2, 54); display.print("RAM :"); display.print(ESP.getFreeHeap() / 1024); display.print("KB");
  } else if (idx == 4) {
    display.setCursor(2, 14); display.print("Reset Sniffer");
    display.drawFastHLine(0, 25, 128, WHITE);
    display.setCursor(2, 30); display.print("Press [OK] to reset");
  } else if (idx == 5) {
    display.setCursor(2, 14); display.print("Clear Guard AP");
    display.drawFastHLine(0, 25, 128, WHITE);
    display.setCursor(2, 30); display.print("Press [OK] to clear");
  } else {
    display.setCursor(2, 14); display.print("System Config");
    display.drawFastHLine(0, 25, 128, WHITE);
    display.setCursor(2, 30); display.print("Selected option:");
    display.setCursor(2, 42); display.print(setMenu[idx]);
  }
  drawBottomBar("[OK]Back", "");
}