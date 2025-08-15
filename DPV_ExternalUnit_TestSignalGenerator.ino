// Using Arduino IDE with STM32 official core
// Non-blocking PWM ramp using a millisecond clock (millis()).
// 1 kHz PWM, duty cycles from 0% -> 100% over 10s, then 100% -> 0% over 10s (triangle wave).

// ===================== PWM / LED CONFIG =====================
const uint32_t PWM_FREQ   = 1000;   // 1 kHz
const uint32_t PWM_PIN    = PB6;    // Timer-capable pin (e.g., TIM4_CH1)
const uint32_t LED_PIN    = PC13;   // Onboard LED
const uint32_t RAMP_MS    = 100;  // x seconds for one ramp
const uint16_t MAX_DUTY   = 255;    // analogWrite() default resolution

// ===================== CLOCK STATE =====================
uint32_t t0 = 0;            // Start time reference
uint16_t lastDuty = 0xFFFF; // Track last duty to avoid redundant writes

void setup() {
  pinMode(LED_PIN, OUTPUT);

  // Set global PWM frequency (applies to all PWM channels in STM32 official core)
  analogWriteFrequency(PWM_FREQ);

  t0 = millis();
  analogWrite(PWM_PIN, 0); // start at 0%
}

void loop() {
  // Compute time since start within a 20s full period (up 10s, down 10s)
  const uint32_t fullPeriod = 2 * RAMP_MS;                 // 20,000 ms
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

  // Tiny sleep to reduce CPU churn
  delay(1);
}
