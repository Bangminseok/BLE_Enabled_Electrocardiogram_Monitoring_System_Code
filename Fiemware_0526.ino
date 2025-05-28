#include <bluefruit.h>
#include "S1sBp6a.h"

#define PAYLOAD_SIZE    20

#define ECG_SERVICE_UUID  "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define ECG_CHAR_UUID     "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define BPM_CHAR_UUID     "6e400004-b5a3-f393-e0a9-e50e24dcca9e"
#define ECG_BATTERY_UUID  "6e400005-b5a3-f393-e0a9-e50e24dcca9e"

BLEService       ecgService(ECG_SERVICE_UUID);
BLECharacteristic ecgCharacteristic(ECG_CHAR_UUID);
BLECharacteristic bpmCharacteristic(BPM_CHAR_UUID);
BLECharacteristic batteryCharacteristic(ECG_BATTERY_UUID);

#define SUBSAMPLING        16
#define ECG_CHANNEL        0
#define MAX_SAMPLES        64
#define HALF_SAMPLES       (MAX_SAMPLES / 2)
#define BATCH_INTERVAL     250

uint32_t rawBuf[CHANNEL_MAX][SAMPLE_LENGTH_MAX];
int16_t  sampleBuf[MAX_SAMPLES];
int      sampleIdx = 0;
bool     isConnected = false;
unsigned long lastBatchTime = 0;
unsigned long lastBpmTime = 0;

// 🔋 배터리 측정 관련 변수
float vbatSum = 0;
int vbatCount = 0;
unsigned long lastVbatUpdate = 0;
const unsigned long VBAT_SEND_INTERVAL = 3000;

// 🔋 배터리 전압 측정 함수
float readVBAT() {
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);
  delay(1);
  float sum = 0;
  const int N = 5;
  for (int i = 0; i < N; i++) {
    sum += analogRead(A6);
    delay(1);
  }
  analogReference(AR_DEFAULT);
  analogReadResolution(10);
  float avg = sum / N;
  return avg * 1.446F;
}

// 🔋 전압(mV) → 5단계 배터리 구간 변환 함수
uint8_t batteryStepFromMV(float mv) {
  if (mv >= 4000) return 5;
  else if (mv >= 3850) return 4;
  else if (mv >= 3700) return 3;
  else if (mv >= 3550) return 2;
  else return 1;
}

void debugPrintBattery(uint8_t step, float mv) {
  if (Serial) {
    Serial.print("🔋 Battery: ");
    Serial.print(mv);
    Serial.print(" mV → Step ");
    Serial.println(step);
  }
}

void connect_callback(uint16_t conn_handle) {
  SERIAL_OUT.println("✅ BLE Connected");
  isConnected = true;
  Bluefruit.Periph.setConnInterval(6, 6);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  SERIAL_OUT.println("⚠️ BLE Disconnected");
  isConnected = false;
  sampleIdx = 0;
  Bluefruit.Advertising.start(0);
}

void setup() {
  SERIAL_OUT.begin(460800);

  bp6a.begin(OP_MODE_ECG0);
  bp6a.start();

  Bluefruit.begin();
  Bluefruit.setName("SPAL ECG Analyzer");
  Bluefruit.setTxPower(4);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  ecgService.begin();

  ecgCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  ecgCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  ecgCharacteristic.setMaxLen(PAYLOAD_SIZE);
  ecgCharacteristic.begin();

  bpmCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  bpmCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bpmCharacteristic.setFixedLen(sizeof(int16_t));
  bpmCharacteristic.begin();

  batteryCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  batteryCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  batteryCharacteristic.setFixedLen(1);
  batteryCharacteristic.begin();

  Bluefruit.Advertising.addService(ecgService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.start(0);

  lastBatchTime = millis();
  lastBpmTime = millis();
  lastVbatUpdate = millis();
}

void sendEcg(int count) {
  if (count <= 0) return;
  uint8_t* ptr = (uint8_t*)sampleBuf;
  int totalBytes = count * sizeof(int16_t);
  for (int off = 0; off < totalBytes; off += PAYLOAD_SIZE) {
    int len = min(PAYLOAD_SIZE, totalBytes - off);
    ecgCharacteristic.notify(ptr + off, len);
    delay(5);
  }
  SERIAL_OUT.print("📤 ECG samples sent: ");
  SERIAL_OUT.println(count);
}

void sendBpm() {
  BpAlgoResult& result = bp6a.readAlgoResult();
  if (result.ecg.available()) {
    int16_t bpm = (int16_t)result.ecg.heartRate;
    bpmCharacteristic.notify((uint8_t*)&bpm, sizeof(int16_t));
    SERIAL_OUT.print("❤️ BPM sent: ");
    SERIAL_OUT.println(bpm);
  }
}

void loop() {
  if (!isConnected) {
    delay(10);
    return;
  }

  int avail = bp6a.available(ECG_CHANNEL);
  if (avail > 0) {
    bp6a.readData(ECG_CHANNEL, rawBuf[ECG_CHANNEL], avail);
    for (int i = 0; i < avail && sampleIdx < MAX_SAMPLES; i += SUBSAMPLING) {
      uint32_t raw = rawBuf[ECG_CHANNEL][i];
      if (raw == 0 || raw > 1000000) continue;
      sampleBuf[sampleIdx++] = (int16_t)raw;
      if (sampleIdx >= HALF_SAMPLES) {
        sendEcg(sampleIdx);
        int rem = sampleIdx - HALF_SAMPLES;
        if (rem > 0) {
          memmove(sampleBuf, sampleBuf + HALF_SAMPLES, rem * sizeof(int16_t));
        }
        sampleIdx = rem;
        lastBatchTime = millis();
      }
    }
  }

  unsigned long now = millis();
  if (now - lastBatchTime >= BATCH_INTERVAL && sampleIdx > 0) {
    sendEcg(sampleIdx);
    sampleIdx = 0;
    lastBatchTime = now;
  }

  // ✅ BPM 전송 (1초 간격)
  if (now - lastBpmTime >= 1000) {
    sendBpm();
    lastBpmTime = now;
  }

  // 🔋 배터리 전송 (30초 간격, 평균 기반)
  float mv = readVBAT();
  vbatSum += mv;
  vbatCount++;
  if (now - lastVbatUpdate >= VBAT_SEND_INTERVAL && vbatCount > 0) {
    float avg = vbatSum / vbatCount;
    uint8_t step = batteryStepFromMV(avg);
    batteryCharacteristic.write8(step);
    batteryCharacteristic.notify8(step);
    debugPrintBattery(step, avg);
    vbatSum = 0;
    vbatCount = 0;
    lastVbatUpdate = now;
  }

  delay(1);
}
