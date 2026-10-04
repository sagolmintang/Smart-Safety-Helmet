#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MLX90614.h>
#include <ArduinoBLE.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

const char* HELMET_NAME = "HELMET_H1";
const char* HELMET_ID = "H1";
BLEService helmetService("19B10010-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic statusChar("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 80);
BLEStringCharacteristic alarmChar("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 80);
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MLX90614 mlx;
const uint8_t MLX_ADDR = 0x5A;
const int PIN_ECG = A0;
const int PIN_LOP = D10;
const int PIN_LON = D11;
const int PIN_BUZZ = D9;
const int BPM_ALARM_THRESHOLD = 200;
const int HYST = 5;
const uint16_t TONE1_HZ = 1500, TONE2_HZ = 2300;
const uint16_t TONE_MS = 180, GAP_MS = 80;
const uint8_t ALARM_REPS = 5;
struct {
  bool active = false;
  bool armed = true;
  uint8_t repsLeft = 0, phase = 0;
  unsigned long nextMs = 0;
} alarmSM;

const unsigned long FS_HZ = 250, DRAW_HZ = 5;
unsigned long drawIntervalMs, tNextDrawMs;
const float HPF_A = 0.98f;
const int MA_N = 5;
const unsigned long REFRACT_MS = 250;
const float ENV_BETA = 0.02f, TH_GAIN = 2.0f;
float x_prev = 0, y_prev = 0;
int maBuf[MA_N] = {0}, maSum = 0, maIdx = 0;
float env = 20.0f;
bool wasHigh = false;
unsigned long lastPeakMs = 0;
int bpm_current = 0, beatAvg = 0, avgBuf[4] = {0}, avgIdx = 0;
float objC = NAN, ambC = NAN;
bool leadOff = false;
#define COMMON_ANODE true
const int PIN_LED_R = D3, PIN_LED_G = D4, PIN_LED_B = D5;
bool oledReady = false, mlxReady = false, bleReady = false;
bool havePeak = false, previousLeadOff = true;
uint8_t avgCount = 0;
unsigned long leadOnMs = 0, nextTempMs = 0, nextStatusMs = 0;
const unsigned long PEAK_TIMEOUT_MS = 2500, SETTLE_MS = 1500;
struct ECGReading {
  int bpm;
  bool disconnected;
};
ECGReading publishedECG = {0, true};
ECGReading currentECG = {0, true};
portMUX_TYPE ecgMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t ecgTaskHandle = nullptr;

inline void rgbWriteRaw(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(PIN_LED_R, COMMON_ANODE ? 255-r : r);
  analogWrite(PIN_LED_G, COMMON_ANODE ? 255-g : g);
  analogWrite(PIN_LED_B, COMMON_ANODE ? 255-b : b);
}

void alarmTrigger() {
  if (alarmSM.active) return;
  alarmSM.active = true;
  alarmSM.repsLeft = ALARM_REPS;
  alarmSM.phase = 0;
  alarmSM.nextMs = millis();
}

void alarmService() {
  if (!alarmSM.active) return;
  unsigned long now = millis();
  if ((long)(now-alarmSM.nextMs) < 0) return;
  switch (alarmSM.phase) {
    case 0:
      tone(PIN_BUZZ, TONE1_HZ, TONE_MS);
      alarmSM.nextMs = now+TONE_MS; alarmSM.phase = 1; break;
    case 1:
      noTone(PIN_BUZZ);
      alarmSM.nextMs = now+GAP_MS; alarmSM.phase = 2; break;
    case 2:
      tone(PIN_BUZZ, TONE2_HZ, TONE_MS);
      alarmSM.nextMs = now+TONE_MS; alarmSM.phase = 3; break;
    case 3:
      noTone(PIN_BUZZ);
      if (alarmSM.repsLeft > 0) --alarmSM.repsLeft;
      if (alarmSM.repsLeft == 0) { alarmSM.active = false; break; }
      alarmSM.nextMs = now+GAP_MS; alarmSM.phase = 0; break;
  }
}

void resetECG(int raw) {
  x_prev = raw; y_prev = 0;
  maSum = 0; maIdx = 0;
  for (int i=0; i<MA_N; ++i) maBuf[i] = 0;
  env = 20.0f; wasHigh = false; havePeak = false;
  lastPeakMs = 0; bpm_current = 0; beatAvg = 0;
  avgIdx = 0; avgCount = 0;
  for (int i=0; i<4; ++i) avgBuf[i] = 0;
}

void sampleECG() {
  unsigned long now = millis();
  int raw = analogRead(PIN_ECG);
  leadOff = digitalRead(PIN_LOP) || digitalRead(PIN_LON);
  if (leadOff) {
    if (!previousLeadOff) resetECG(raw);
    previousLeadOff = true;
    return;
  }
  if (previousLeadOff) {
    resetECG(raw); leadOnMs = now; previousLeadOff = false;
  }
  float y = HPF_A * (y_prev + raw - x_prev);
  x_prev = raw; y_prev = y;
  maSum -= maBuf[maIdx];
  maBuf[maIdx] = (int)y; maSum += maBuf[maIdx];
  maIdx = (maIdx+1) % MA_N;
  float filtered = (float)maSum / MA_N;
  env += ENV_BETA * (fabsf(filtered)-env);
  float threshold = fmaxf(20.0f, TH_GAIN*env);
  bool high = filtered > threshold;
  if (now-leadOnMs < SETTLE_MS) { wasHigh = high; return; }
  if (havePeak && now-lastPeakMs > PEAK_TIMEOUT_MS) {
    resetECG(raw); leadOnMs = now;
    return;
  }
  if (high && !wasHigh && (!havePeak || now-lastPeakMs >= REFRACT_MS)) {
    if (havePeak) {
      unsigned long rr = now-lastPeakMs;
      int estimate = (int)(60000UL/rr);
      if (estimate >= 30 && estimate <= 240) {
        bpm_current = estimate;
        avgBuf[avgIdx] = estimate; avgIdx = (avgIdx+1)%4;
        if (avgCount < 4) ++avgCount;
        int total = 0;
        for (uint8_t i=0; i<avgCount; ++i) total += avgBuf[i];
        beatAvg = total/avgCount;
      } else {
        bpm_current = 0; beatAvg = 0; avgCount = 0; avgIdx = 0;
        for (int i=0; i<4; ++i) avgBuf[i] = 0;
      }
    }
    lastPeakMs = now; havePeak = true;
  }
  wasHigh = high;
}

void ecgTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(1000UL/FS_HZ);
  static_assert(configTICK_RATE_HZ >= FS_HZ, "FreeRTOS tick rate is too low");
  TickType_t previousWake = xTaskGetTickCount();
  unsigned long previousUs = micros();
  for (;;) {
    unsigned long nowUs = micros();
    if (nowUs-previousUs > 20000UL) {
      resetECG(analogRead(PIN_ECG));
      previousLeadOff = true;
      previousWake = xTaskGetTickCount();
    }
    previousUs = nowUs;
    sampleECG();
    portENTER_CRITICAL(&ecgMux);
    publishedECG.bpm = beatAvg;
    publishedECG.disconnected = leadOff;
    portEXIT_CRITICAL(&ecgMux);
    vTaskDelayUntil(&previousWake, period);
  }
}

String tempText(float value) {
  return isfinite(value) ? String(value, 1) : String("NA");
}

void sendAlarm(const char* event) {
  String msg = String("ALARM,")+HELMET_ID+","+event+","+String(currentECG.bpm)+","+String(millis());
  if (bleReady) alarmChar.writeValue(msg);
  Serial.println(msg);
}

void updateAlarmAndLED() {
  bool valid = !currentECG.disconnected && currentECG.bpm > 0;
  if (valid && currentECG.bpm >= BPM_ALARM_THRESHOLD && alarmSM.armed) {
    alarmSM.armed = false;
    alarmTrigger(); sendAlarm("HIGH_BPM");
  } else if (valid && currentECG.bpm <= BPM_ALARM_THRESHOLD-HYST && !alarmSM.armed) {
    alarmSM.armed = true;
    alarmSM.active = false;
    noTone(PIN_BUZZ);
    sendAlarm("CLEAR");
  }
  if (alarmSM.active || !alarmSM.armed) rgbWriteRaw(255,0,0);
  else if (!valid) rgbWriteRaw(0,0,255);
  else rgbWriteRaw(0,255,0);
}

void drawOLED() {
  if (!oledReady) return;
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,0);
  display.print(HELMET_NAME);
  display.setCursor(0,12); display.print("IR: "); display.print(tempText(objC)); display.print(" C");
  display.setCursor(0,22); display.print("Ambient: "); display.print(tempText(ambC)); display.print(" C");
  display.setCursor(0,34); display.setTextSize(2); display.print("BPM:");
  if (currentECG.disconnected || currentECG.bpm == 0) display.print("--"); else display.print(currentECG.bpm);
  display.setTextSize(1); display.setCursor(0,55);
  if (currentECG.disconnected) display.print("ECG LEADS OFF");
  else if (!alarmSM.armed) display.print("HIGH BPM ALARM");
  else if (currentECG.bpm == 0) display.print("ACQUIRING ECG");
  else display.print("MONITORING");
  display.display();
}

void sendStatus() {
  String msg = String("STATUS,")+HELMET_ID+","+tempText(objC)+","+tempText(ambC)+","+
    String(currentECG.bpm)+","+String(currentECG.disconnected ? 1 : 0)+","+String(alarmSM.armed ? 0 : 1)+","+String(millis());
  if (bleReady) statusChar.writeValue(msg);
  Serial.println(msg);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LOP, INPUT); pinMode(PIN_LON, INPUT);
  pinMode(PIN_BUZZ, OUTPUT); noTone(PIN_BUZZ);
  pinMode(PIN_LED_R, OUTPUT); pinMode(PIN_LED_G, OUTPUT); pinMode(PIN_LED_B, OUTPUT);
  rgbWriteRaw(0,0,0);
  analogReadResolution(12);
  Wire.begin();
  oledReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  mlxReady = mlx.begin(MLX_ADDR, &Wire);
  if (!oledReady) Serial.println("OLED initialization failed");
  if (!mlxReady) Serial.println("MLX90614 initialization failed");
  bleReady = BLE.begin();
  if (bleReady) {
    BLE.setLocalName(HELMET_NAME); BLE.setDeviceName(HELMET_NAME);
    BLE.setAdvertisedService(helmetService);
    helmetService.addCharacteristic(statusChar); helmetService.addCharacteristic(alarmChar);
    BLE.addService(helmetService);
    statusChar.writeValue(String("STATUS,")+HELMET_ID+",NA,NA,0,1,0,0");
    alarmChar.writeValue(String("ALARM,")+HELMET_ID+",NONE,0,0");
    BLE.advertise();
  } else Serial.println("BLE initialization failed; monitoring continues");
  resetECG(analogRead(PIN_ECG)); leadOff = true;
  drawIntervalMs = 1000UL/DRAW_HZ;
  tNextDrawMs = millis();
  nextTempMs = millis(); nextStatusMs = millis();
  if (xTaskCreatePinnedToCore(ecgTask, "ECG", 4096, nullptr, 2, &ecgTaskHandle, 1) != pdPASS) {
    Serial.println("ECG task initialization failed");
  }
}

void loop() {
  if (bleReady) BLE.poll();
  portENTER_CRITICAL(&ecgMux);
  currentECG = publishedECG;
  portEXIT_CRITICAL(&ecgMux);
  updateAlarmAndLED();
  alarmService();
  unsigned long now = millis();
  if ((long)(now-nextTempMs) >= 0) {
    nextTempMs = now+1000;
    if (mlxReady) { objC = mlx.readObjectTempC(); ambC = mlx.readAmbientTempC(); }
  }
  if ((long)(now-nextStatusMs) >= 0) { nextStatusMs = now+1000; sendStatus(); }
  if ((long)(now-tNextDrawMs) >= 0) { tNextDrawMs = now+drawIntervalMs; drawOLED(); }
  delay(1);
}
