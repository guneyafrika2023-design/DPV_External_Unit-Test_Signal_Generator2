// Using Arduino IDE with STM32 official core
// Non-blocking PWM ramp using a millisecond clock (millis()).
// 1 kHz PWM, duty cycles from 0% -> 100% over 10s, then 100% -> 0% over 10s (triangle wave).
#include <Arduino.h>

 HardwareSerial Serial2(PA3, PA2); // RX, TX

// ===================== PWM / LED CONFIG =====================
const uint32_t PWM_FREQ   = 1000;   // 1 kHz
const uint32_t PWM_PIN    = PB6;    // Timer-capable pin (e.g., TIM4_CH1)
const uint32_t LED_PIN    = PC13;   // Onboard LED
const uint32_t RAMP_MS    = 100;  // x seconds for one ramp
const uint16_t MAX_DUTY   = 255;    // analogWrite() default resolution

// ============= Data over UART (Serial2) =============
// Frame constants
const uint8_t SYNC1 = 0x55;
const uint8_t SYNC2 = 0xAA;

// Labels (define your map once, keep it stable)
const uint8_t L_BATT = 0x01;
const uint8_t L_TEMP = 0x02;
const uint8_t L_RPM  = 0x03;

// Simple CRC-8 (poly 0x07, init 0x00, MSB-first)
uint8_t crc8_07(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x07;
      else            crc <<= 1;
    }
  }
  return crc;
}

void sendFrame(uint8_t label, uint8_t value) {
  uint8_t buf[5];
  buf[0] = SYNC1;
  buf[1] = SYNC2;
  buf[2] = label;
  buf[3] = value;
  buf[4] = crc8_07(&buf[2], 2); // CRC over LABEL, VALUE
  Serial2.write(buf, sizeof(buf));
}

uint16_t BattRand = 0;
uint16_t TempRand = 0;
uint16_t RpmRand = 0;

// ===================== CLOCK STATE =====================
uint32_t t0 = 0;            // Start time reference
uint16_t lastDuty = 0xFFFF; // Track last duty to avoid redundant writes

void setup() {
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);   // USB serial for debug (optional)
  Serial2.begin(115200);  // UART link to Board A
  delay(200);

  randomSeed(12345);
  Serial.println("Sender up");

  // Set global PWM frequency (applies to all PWM channels in STM32 official core)
  analogWriteFrequency(PWM_FREQ);

  t0 = millis();
  analogWrite(PWM_PIN, 0); // start at 0%
}

// Demo: send different labels at different cadences
unsigned long tBatt = 0, tTemp = 0, tRPM = 0;
uint8_t batt = 95;  // %
uint8_t temp = 27;  // C (offset for demo)
uint8_t rpm  = 10;  // arbitrary unit for demo

void loop() {
  unsigned long now = millis();

  // Compute time since start within a 20s full period (up 10s, down 10s)
  const uint32_t fullPeriod = 2 * RAMP_MS;
  uint32_t elapsed = millis() - t0;                        // unsigned math handles overflow
  uint32_t t = elapsed % fullPeriod;                       // phase time within the triangle

  // Calculate duty based on triangle waveform
  uint16_t duty;
  if (t < RAMP_MS) {
    // Ramp up: 0 -> MAX_DUTY over 10s
    duty = (uint16_t)((t * MAX_DUTY) / RAMP_MS);
  } else {
    // Ramp down: MAX_DUTY -> 0 over next 10s
    uint32_t t2 = t - RAMP_MS;                             // 0..RAMP_MS
    duty = (uint16_t)(((RAMP_MS - t2) * MAX_DUTY) / RAMP_MS);
  }

  // Update PWM/display only when duty changes; toggle LED at each change
  if (duty != lastDuty) {
    analogWrite(PWM_PIN, duty);
    lastDuty = duty;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));          // Toggle LED on each step change
  }

  // Battery every 1000 ms
  if (now - tBatt >= (6000+BattRand)) {
    sendFrame(L_BATT, batt);
    tBatt = now;
    Serial.print(millis());
    Serial.print(" -->  Batt: ");
    Serial.println(batt);
    // demo drift
    if (batt > 5) batt--;
    BattRand = random(0, 5000);
  }

  // Temperature every 600 ms
  if (now - tTemp >= (6000+TempRand)) {
    sendFrame(L_TEMP, temp);
    tTemp = now;
    Serial.print(millis());
    Serial.print(" -->  Temp: ");
    Serial.println(temp);
    temp++; // demo drift
    TempRand = random(0, 5000);
  }

  // RPM every 200 ms
  if (now - tRPM >= (6000+RpmRand)) {
    sendFrame(L_RPM, rpm);
    tRPM = now;
    Serial.print(millis());
    Serial.print(" -->  RPM: ");
    Serial.println(rpm);
    rpm += 3; // demo drift
    RpmRand = random(0, 5000);
  }

  // Tiny sleep to reduce CPU churn
  delay(1);
}