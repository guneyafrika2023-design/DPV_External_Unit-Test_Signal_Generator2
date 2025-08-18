// Using Arduino IDE with STM32 official core
// Non-blocking PWM ramp using a millisecond clock (millis()).
// 1 kHz PWM, duty cycles from 0% -> 100% over 10s, then 100% -> 0% over 10s (triangle wave).
#include <Arduino.h>

struct ParamState;
struct RxFrame;

HardwareSerial Serial2(PA3, PA2); // RX, TX

// ===================== PWM / LED CONFIG =====================
const uint32_t PWM_FREQ   = 1000;   // 1 kHz
const uint32_t PWM_PIN    = PB6;    // Timer-capable pin (e.g., TIM4_CH1)
const uint32_t LED_PIN    = PC13;   // Onboard LED
const uint32_t RAMP_MS    = 300;    // x ms for one ramp
const uint16_t MAX_DUTY   = 255;    // analogWrite() default resolution

// ============= Data over UART (Serial2) =============
// Frame constants
const uint8_t SYNC1 = 0x55;
const uint8_t SYNC2 = 0xAA;

// Labels (define your map once, keep it stable)
const uint8_t L_BATT = 0x01;
const uint8_t L_TEMP = 0x02;
const uint8_t L_RPM  = 0x03;
const uint8_t L_DCME = 0x04;


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

//---- ----
enum RxState : uint8_t { HUNT_SYNC1, HUNT_SYNC2, READ_LABEL, READ_VALUE, READ_CRC };
static RxState rx_state = HUNT_SYNC1;

//----Timeout helpers ----
// ---------- PARAM TIMEOUT TRACKING ----------
static const uint32_t PARAM_TIMEOUT_MS = 10000;

struct ParamState {
  bool     has = false;        // true = value is valid; false = "NULL"
  uint8_t  val = 0;            // last seen value (ignored if has == false)
  uint32_t last_ms = 0;        // millis() when we last updated it
  bool     announced_timeout = false; // to avoid spamming Serial on every loop
};

static ParamState g_batt, g_temp, g_rpm, g_dutycycle;

static const char* labelToName(uint8_t label) {
  switch (label) {
    case L_BATT: return "Battery";
    case L_TEMP: return "Temperature";
    case L_RPM:  return "RPM";
    case L_DCME: return "DutyCycle";
    default:     return "Unknown";
  }
}

static ParamState& stateForLabel(uint8_t label) {
  switch (label) {
    case L_BATT: return g_batt;
    case L_TEMP: return g_temp;
    case L_RPM:  return g_rpm;
    case L_DCME: return g_dutycycle;
    default:     return g_batt; // safe fallback, won't be used logically
  }
}

// Call this whenever we successfully parse a frame
static void noteParamSeen(uint8_t label, uint8_t value) {
  ParamState& ps = stateForLabel(label);
  ps.val = value;
  ps.has = true;
  ps.last_ms = millis();
  if (ps.announced_timeout) {
    // One-time "recovered" message after a timeout
    Serial.print(millis());
    Serial.print(" --> ");
    Serial.print(labelToName(label));
    Serial.println(" recovered");
    ps.announced_timeout = false;
  }
}

// Call this from loop() to invalidate stale values
static void paramTimeouts_step() {
  const uint32_t now = millis();

  auto check = [&](const char* name, ParamState& ps) {
    if (ps.has && (now - ps.last_ms >= PARAM_TIMEOUT_MS)) {
      // Invalidate -> interpret as "NULL"
      ps.has = false;
      ps.announced_timeout = true;
      Serial.print(millis());
      Serial.print(" --> ");
      Serial.print(name);
      Serial.println(" timeout -> NULL");
    }
  };

  check("DutyCycle", g_dutycycle);
}

//----Timeout helpers ----
// Define the frame TYPE *before* any function that uses it
struct RxFrame { uint8_t label; uint8_t value; };


// Manual prototype prevents Arduino from generating a wrong one
static bool uart_rx_step(RxFrame &out);

static uint8_t rx_label = 0, rx_value = 0;

// Consume at most ONE byte per call; return true only when a full, CRC-valid frame is ready.
static bool uart_rx_step(RxFrame &out) {
  if (!Serial2.available()) return false;   // <= 1 byte per loop iteration

  uint8_t byteIn = (uint8_t)Serial2.read();

  switch (rx_state) {
    case HUNT_SYNC1:
      if (byteIn == SYNC1) rx_state = HUNT_SYNC2;
      return false;

    case HUNT_SYNC2:
      if (byteIn == SYNC2) {
        rx_state = READ_LABEL;
      } else if (byteIn != SYNC1) {
        rx_state = HUNT_SYNC1;
      }
      return false;

    case READ_LABEL:
      rx_label = byteIn;
      rx_state = READ_VALUE;
      return false;

    case READ_VALUE:
      rx_value = byteIn;
      rx_state = READ_CRC;
      return false;

    case READ_CRC: {
      uint8_t calc = crc8_07((uint8_t[]){rx_label, rx_value}, 2);
      bool ok = (calc == byteIn);
      rx_state = HUNT_SYNC1;     // always return to hunt
      if (ok) { out = {rx_label, rx_value}; return true; }
      return false;
    }
  }
  // Safety fallback
  rx_state = HUNT_SYNC1;
  return false;
}
//---- ----

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

  RxFrame f;
  if (uart_rx_step(f)) {
    Serial.print(millis());
    Serial.print(" <-- ");
    Serial.print(labelToName(f.label));
    Serial.print(" = ");
    Serial.println(f.value);

    // Record that we saw this parameter now
    noteParamSeen(f.label, f.value);
  }
  paramTimeouts_step(); // invalidate stale values -> becomes "NULL"


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