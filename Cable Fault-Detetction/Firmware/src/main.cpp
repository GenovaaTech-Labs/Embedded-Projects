// ESP32 R-C cable fault detection (no sensor modules)
// D25 drives the line through R_s (1 kOhm); D34 reads node A.
// Cable model: 100 ohm per conductor per km, 100 uF across each km, R_L 10 kOhm at the end.
//
// Short:  steady V low; R = R_s * V / (3.3 - V), distance = R / (2 * R_PER_KM)
// Else:   discharge, then time the charge to 63% of 3.3 V.
//         tau = R_s * C = 100 ms per connected km.
//         All 4 km connected -> healthy; fewer -> open after that many km.
//
// Relay (SO4): D26 -> relay IN, COM/NC in series with the line at 3 km.
// Serial Monitor: send '1' to open SO4 (open fault after 3 km), '0' to restore.
//
// Alerts: green LED D32 = healthy, yellow LED D33 = open, red LED D14 = short.
//         Buzzer D27: short = 3 quick beeps, open = 1 long beep each cycle.
//         Serial 'm' mutes / unmutes the buzzer.
//
// OLED SSD1306 128x64 (I2C 0x3C): SDA = D21, SCL = D22, VCC = 3V3.
// TMP117 (I2C 0x48) on the same SDA/SCL: cable temperature.
//         Healthy cable at or above OVERHEAT_C -> OVERHEAT (red LED, 2 beeps).
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_TMP117.h>

const int OUT_PIN = 25;
const int ADC_PIN = 34;
const int RELAY_PIN = 26;
const int LED_GREEN = 32;
const int LED_YELLOW = 33;
const int LED_RED = 14;
const int BUZZER_PIN = 27;
const bool RELAY_ACTIVE_LOW = true; // most relay modules switch on when IN is LOW
const bool BUZZER_ACTIVE_LOW = false;
const float VCC = 3.3;
const float RS = 1000.0;            // series resistor R_s
const float R_PER_KM = 100.0;       // per conductor, per km
const float TAU_PER_KM_MS = 100.0;  // R_s * 100 uF
const int CABLE_KM = 4;
const float SHORT_V = 2.0;          // steady V below this: short
const int TAU_MV = 2080;            // 63% of 3.3 V
const unsigned long SETTLE_MS = 2500;
const unsigned long TAU_TIMEOUT_MS = 3000;
const float OVERHEAT_C = 40.0;

enum Status { HEALTHY, SHORT_FAULT, OPEN_FAULT, OVERHEAT };

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
Adafruit_TMP117 tmp117;
bool oledOk = false;
bool tmpOk = false;
float tempC = NAN;
bool relayOn = false;
bool muted = false;

// Status screen: header, big status, big distance, small V / tau line.
void drawScreen(const char* status, const char* distance, float v, float tau) {
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("CABLE");
  oled.setCursor(44, 0);
  if (isnan(tempC)) oled.print("--.-C");
  else oled.printf("%.1fC", tempC);
  if (muted) {
    oled.setCursor(104, 0);
    oled.print("MUTE");
  }
  oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(0, 15);
  oled.print(status);
  oled.setCursor(0, 34);
  oled.print(distance);
  oled.setTextSize(1);
  oled.setCursor(0, 56);
  if (tau >= 0) oled.printf("V=%.2fV  tau=%.0fms", v, tau);
  else oled.printf("V=%.2fV", v);
  oled.display();
}

void setRelay(bool on) {
  relayOn = on;
  digitalWrite(RELAY_PIN, (on != RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void buzzer(bool on) {
  digitalWrite(BUZZER_PIN, (on != BUZZER_ACTIVE_LOW) ? HIGH : LOW);
}

void beep(int count, int onMs, int offMs) {
  if (muted) return;
  for (int i = 0; i < count; i++) {
    buzzer(true);
    delay(onMs);
    buzzer(false);
    delay(offMs);
  }
}

void showStatus(Status st) {
  digitalWrite(LED_GREEN, st == HEALTHY);
  digitalWrite(LED_YELLOW, st == OPEN_FAULT);
  digitalWrite(LED_RED, st == SHORT_FAULT || st == OVERHEAT);
  if (st == SHORT_FAULT) beep(3, 100, 100);
  else if (st == OPEN_FAULT) beep(1, 600, 0);
  else if (st == OVERHEAT) beep(2, 300, 200);
}

void readTemp() {
  if (!tmpOk) return;
  sensors_event_t t;
  tmp117.getEvent(&t);
  tempC = t.temperature;
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '1') {
      setRelay(true);
      Serial.println(">> Relay ON: SO4 open (fault after 3 km)");
    } else if (c == '0') {
      setRelay(false);
      Serial.println(">> Relay OFF: SO4 closed (normal)");
    } else if (c == 'm') {
      muted = !muted;
      if (muted) buzzer(false);
      Serial.println(muted ? ">> Buzzer muted" : ">> Buzzer on");
    }
  }
}

float readVolts() {
  long sum = 0;
  for (int i = 0; i < 32; i++) sum += analogReadMilliVolts(ADC_PIN);
  return sum / 32.0 / 1000.0;
}

float measureTauMs() {
  digitalWrite(OUT_PIN, LOW);       // discharge the line
  unsigned long t = millis();
  while (analogReadMilliVolts(ADC_PIN) > 30 && millis() - t < 5000) delay(10);
  delay(300);
  unsigned long t0 = millis();
  digitalWrite(OUT_PIN, HIGH);
  while (analogReadMilliVolts(ADC_PIN) < TAU_MV && millis() - t0 < TAU_TIMEOUT_MS) {}
  return millis() - t0;
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  setRelay(false);                  // SO4 closed at power-up
  pinMode(BUZZER_PIN, OUTPUT);
  buzzer(false);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  Serial.begin(115200);
  Wire.begin(21, 22);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!oledOk) Serial.println("OLED not found - check SDA/SCL/3V3/GND");
  tmpOk = tmp117.begin(0x48, &Wire);
  if (!tmpOk) Serial.println("TMP117 not found - check SDA/SCL/3V3/GND");
  drawScreen("STARTING", "", 0, -1);
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, HIGH);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);
  Serial.println("Cable fault detection started");
  Serial.println("Send '1' = open SO4 via relay, '0' = restore, 'm' = mute buzzer");
  // lamp test: all LEDs + one short beep
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_YELLOW, HIGH);
  digitalWrite(LED_RED, HIGH);
  beep(1, 150, 0);
  delay(500);
  showStatus(HEALTHY);
}

void loop() {
  handleSerial();
  digitalWrite(OUT_PIN, HIGH);
  delay(SETTLE_MS);                 // let the capacitors charge
  float v = readVolts();
  readTemp();
  Serial.printf("[SO4 %s] T = %.1f C  V = %.2f V  ->  ", relayOn ? "OPEN" : "ok", tempC, v);

  if (v < SHORT_V) {
    float r = RS * v / (VCC - v);
    float km = r / (2 * R_PER_KM);
    char dist[16];
    snprintf(dist, sizeof(dist), "%.2f km", km);
    Serial.printf("SHORT at %.2f km (R = %.0f ohm)\n", km, r);
    drawScreen("SHORT", dist, v, -1);
    showStatus(SHORT_FAULT);
    return;
  }

  float tau = measureTauMs();
  float km = tau / TAU_PER_KM_MS;
  char dist[16];
  Serial.printf("tau = %.0f ms  ->  ", tau);
  if (km > CABLE_KM - 0.5 && tempC >= OVERHEAT_C) {
    snprintf(dist, sizeof(dist), "%.1f C", tempC);
    Serial.printf("OVERHEAT (%.1f C)\n", tempC);
    drawScreen("OVERHEAT", dist, v, tau);
    showStatus(OVERHEAT);
  } else if (km > CABLE_KM - 0.5) {
    Serial.println("HEALTHY");
    drawScreen("HEALTHY", "Cable OK", v, tau);
    showStatus(HEALTHY);
  } else if (km < 0.5) {
    Serial.println("OPEN at the start (before 1 km)");
    drawScreen("OPEN", "at start", v, tau);
    showStatus(OPEN_FAULT);
  } else {
    snprintf(dist, sizeof(dist), "~%.1f km", km);
    Serial.printf("OPEN after ~%.1f km\n", km);
    drawScreen("OPEN", dist, v, tau);
    showStatus(OPEN_FAULT);
  }
}
