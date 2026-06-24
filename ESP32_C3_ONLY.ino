/*
 * ESP32-C3-MINI-1-N4 — LSM6DS3TR-C BLE Gyroscope
 * Reads gyro + accelerometer over I2C and sends to
 * phone over BLE using Nordic UART Service (NUS)
 *
 *   Match these pin numbers to your KiCad schematic:
 *   SDA → GPIO8  (change if different in your schematic)
 *   SCL → GPIO9  (change if different in your schematic)
 *
 * Phone app: nRF Toolbox (free, iOS/Android) → UART
 *            LightBlue   (free, iOS/Android)
 *            Serial Bluetooth Terminal (Android)
 *
 * Data format sent over BLE:
 *   gx,gy,gz,ax,ay,az
 *   e.g. -0.21,1.05,-0.08,0.0012,0.0034,0.9981
 */

#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ─── Pin config — change to match your schematic ───────────────────────────
#define SDA_PIN         8
#define SCL_PIN         9

// ─── LSM6DS3TR-C ────────────────────────────────────────────────────────────
// I2C address: SDO/SA0 tied to GND = 0x6A, tied to 3V3 = 0x6B
#define LSM6DS3_ADDR    0x6A

#define REG_WHO_AM_I    0x0F   // Should return 0x6A
#define REG_CTRL1_XL    0x10   // Accelerometer control
#define REG_CTRL2_G     0x11   // Gyroscope control
#define REG_CTRL3_C     0x12   // General control (auto-increment)
#define REG_OUTX_L_G    0x22   // Gyro  X low byte (first of 6)
#define REG_OUTX_L_XL   0x28   // Accel X low byte (first of 6)

#define GYRO_SCALE      0.00875f   // 250 dps → 8.75 mdps/LSB
#define ACCEL_SCALE     0.000061f  // 2g     → 0.061 mg/LSB

// ─── BLE Nordic UART Service ────────────────────────────────────────────────
#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // ESP32 → phone
#define NUS_RX_CHAR_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone → ESP32
#define DEVICE_NAME       "GyroSensor"

// ─── Timing ─────────────────────────────────────────────────────────────────
#define SEND_INTERVAL_MS  20  // 50Hz update rate

// ─── Globals ────────────────────────────────────────────────────────────────
BLEServer*         pServer       = nullptr;
BLECharacteristic* pTxChar       = nullptr;
bool               deviceConnected = false;
bool               oldConnected    = false;

// ─── BLE callbacks ──────────────────────────────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Phone connected!");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Phone disconnected.");
  }
};

// ─── LSM6DS3 helpers ────────────────────────────────────────────────────────
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(LSM6DS3_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(LSM6DS3_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DS3_ADDR, 1);
  return Wire.read();
}

void readBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(LSM6DS3_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DS3_ADDR, len);
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
}

// ─── Setup ──────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Booting...");

  // I2C init with custom pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // 400kHz fast mode
  delay(100);

  // Verify sensor
  uint8_t whoAmI = readReg(REG_WHO_AM_I);
  if (whoAmI != 0x6A) {
    Serial.print("ERROR: LSM6DS3 not found! WHO_AM_I=0x");
    Serial.println(whoAmI, HEX);
    Serial.println("Check SDA/SCL pins and I2C address.");
    while (1) delay(1000);
  }
  Serial.println("LSM6DS3TR-C found!");

  // Accelerometer: 104Hz ODR, 2g full scale
  writeReg(REG_CTRL1_XL, 0x40);

  // Gyroscope: 104Hz ODR, 250dps full scale
  writeReg(REG_CTRL2_G, 0x40);

  // Enable register auto-increment for burst reads
  writeReg(REG_CTRL3_C, 0x04);

  Serial.println("Sensor ready.");

  // BLE init
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(NUS_SERVICE_UUID);

  // TX characteristic — sends data to phone
  pTxChar = pService->createCharacteristic(
    NUS_TX_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxChar->addDescriptor(new BLE2902());

  // RX characteristic — receive commands from phone (optional)
  pService->createCharacteristic(
    NUS_RX_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );

  pService->start();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(NUS_SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);
  BLEDevice::startAdvertising();

  Serial.println("BLE advertising as \"" DEVICE_NAME "\"");
  Serial.println("Open nRF Toolbox or LightBlue on your phone.");
}

// ─── Loop ───────────────────────────────────────────────────────────────────
void loop() {
  static uint32_t lastSend = 0;

  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();

    uint8_t buf[6];

    // Read gyroscope
    readBytes(REG_OUTX_L_G, buf, 6);
    float gx = (int16_t)(buf[1] << 8 | buf[0]) * GYRO_SCALE;  // deg/s
    float gy = (int16_t)(buf[3] << 8 | buf[2]) * GYRO_SCALE;
    float gz = (int16_t)(buf[5] << 8 | buf[4]) * GYRO_SCALE;

    // Read accelerometer
    readBytes(REG_OUTX_L_XL, buf, 6);
    float ax = (int16_t)(buf[1] << 8 | buf[0]) * ACCEL_SCALE;  // g
    float ay = (int16_t)(buf[3] << 8 | buf[2]) * ACCEL_SCALE;
    float az = (int16_t)(buf[5] << 8 | buf[4]) * ACCEL_SCALE;

    // Build CSV string
    char payload[64];
    snprintf(payload, sizeof(payload),
      "%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\n",
      gx, gy, gz, ax, ay, az
    );

    // Send over BLE
    if (deviceConnected) {
      pTxChar->setValue((uint8_t*)payload, strlen(payload));
      pTxChar->notify();
      delay(10);
    }

    // Debug to USB serial
    Serial.print(payload);
  }

  // Restart advertising after disconnect
  if (!deviceConnected && oldConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Advertising restarted...");
    oldConnected = false;
  }

  if (deviceConnected && !oldConnected) {
    oldConnected = true;
  }
}
