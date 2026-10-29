// ESP32 R-C cable fault detection
// GPIO25 drives the line through R_s (1 kOhm); GPIO34 reads node A.
// Short: R = R_s * V / (3.3 - V), distance = R / (2 * R_PER_KM)
// Open:  time to 63% of 3.3 V gives tau = R_s * C, about 1 ms per km (1 uF/km)
#include <Arduino.h>

const int OUT_PIN = 25;
const int ADC_PIN = 34;
const float VCC = 3.3;
const float RS = 1000.0;        // series resistor R_s
const float R_PER_KM = 100.0;   // per conductor, per km
const float TAU_PER_KM = 1.0;   // ms per km (R_s * 1 uF)
const float SHORT_V = 2.0;      // below this: short
const float OPEN_V = 3.15;      // above this: open
const int TAU_MV = 2080;        // 63% of 3.3 V

float readVolts() {
  long sum = 0;
  for (int i = 0; i < 32; i++) sum += analogReadMilliVolts(ADC_PIN);
  return sum / 32.0 / 1000.0;
}

float measureTauMs() {
  digitalWrite(OUT_PIN, LOW);   // discharge the line
  delay(100);
  unsigned long t0 = micros();
  digitalWrite(OUT_PIN, HIGH);
  while (analogReadMilliVolts(ADC_PIN) < TAU_MV && micros() - t0 < 20000) {}
  return (micros() - t0) / 1000.0;
}

void setup() {
  Serial.begin(115200);
  pinMode(OUT_PIN, OUTPUT);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);
  Serial.println("Cable fault detection started");
}

void loop() {
  digitalWrite(OUT_PIN, HIGH);
  delay(100);
  float v = readVolts();
  Serial.printf("V = %.2f V  ->  ", v);

  if (v < SHORT_V) {
    float r = RS * v / (VCC - v);
    Serial.printf("SHORT at %.2f km (R = %.0f ohm)\n", r / (2 * R_PER_KM), r);
  } else if (v > OPEN_V) {
    float tau = measureTauMs();
    Serial.printf("OPEN after ~%.1f km (tau = %.2f ms)\n", tau / TAU_PER_KM, tau);
  } else {
    Serial.println("HEALTHY");
  }
  delay(1000);
}
