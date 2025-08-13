// For STM32 Blue Pill (STM32F103C8)
// Using Arduino IDE with STM32 official core
// Non-blocking PWM ramp using a millisecond clock (millis()).
// 1 kHz PWM, duty cycles from 0% -> 100% over 10s, then 100% -> 0% over 10s (triangle wave).
// Also shows the current duty on an OLED and animates a progress bar.

#include <U8g2lib.h>
#include <SPI.h>


// ===================== PWM / LED CONFIG =====================
const uint32_t PWM_FREQ   = 1000;   // 1 kHz
const uint32_t PWM_PIN    = PB6;    // Timer-capable pin (e.g., TIM4_CH1)
const uint32_t LED_PIN    = PC13;   // Onboard LED
const uint32_t RAMP_MS    = 1000;  // 10 seconds for one ramp
const uint16_t MAX_DUTY   = 255;    // analogWrite() default resolution

// ===================== OLED CONFIG (U8g2) =====================
// Hardware SPI pins on Blue Pill: SCK=PA5, MOSI=PA7 (MISO not used by OLED)
// Choose your control pins below (change if needed):
static const uint8_t OLED_CS   = PB12;  // Chip Select
static const uint8_t OLED_DC   = PB1;   // Data/Command
static const uint8_t OLED_RST  = PB0;   // Reset

// SSD1309 128x64 over 4-wire HW SPI (common for many 2.42"/2.7" OLEDs)
U8G2_SSD1309_128X64_NONAME2_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ OLED_CS, /* dc=*/ OLED_DC, /* reset=*/ OLED_RST);

// If you actually have SSD1306 instead of SSD1309, comment the line above and use:
// U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ OLED_CS, /* dc=*/ OLED_DC, /* reset=*/ OLED_RST);

// ===================== CLOCK STATE =====================
uint32_t t0 = 0;            // Start time reference
uint16_t lastDuty = 0xFFFF; // Track last duty to avoid redundant writes

void setup() {
  pinMode(LED_PIN, OUTPUT);

  // Set global PWM frequency (applies to all PWM channels in STM32 official core)
  analogWriteFrequency(PWM_FREQ);

  // OLED init
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x12_tf); // Compact readable font

  t0 = millis();
  analogWrite(PWM_PIN, 0); // start at 0%

  // Initial splash
  drawOLED(0);
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
    drawOLED(duty);
  }

  // Tiny sleep to reduce CPU churn
  delay(1);
}

// ===================== OLED RENDERING =====================
void drawOLED(uint16_t duty) {
  // Percent with rounding
  uint8_t percent = (uint8_t)(( (uint32_t)duty * 100 + (MAX_DUTY/2) ) / MAX_DUTY);

  // Progress bar geometry
  const int16_t W = 128;
  const int16_t H = 64;
  const int16_t marginX = 8;
  const int16_t barY = 40;
  const int16_t barH = 14;
  const int16_t barW = W - 2 * marginX;
  int16_t fillW = (int16_t)(( (uint32_t)barW * percent ) / 100);

  u8g2.firstPage();
  do {
    // Title
    u8g2.setCursor(8, 14);
    u8g2.print("PWM 1kHz");

    // Duty text
    u8g2.setCursor(8, 28);
    u8g2.print("Duty: ");
    u8g2.print(percent);
    u8g2.print("% (");
    u8g2.print(duty);
    u8g2.print("/255)");

    // Bar frame
    u8g2.drawFrame(marginX, barY, barW, barH);
    // Fill (stretch/shrink with duty)
    if (fillW > 0) {
      u8g2.drawBox(marginX + 1, barY + 1, fillW - (fillW>1?1:0), barH - 2);
    }

    // End caps as simple chevrons
    u8g2.drawTriangle(marginX-3, barY+barH/2, marginX, barY, marginX, barY+barH);
    u8g2.drawTriangle(marginX+barW+3, barY+barH/2, marginX+barW, barY, marginX+barW, barY+barH);
  } while (u8g2.nextPage());
}
