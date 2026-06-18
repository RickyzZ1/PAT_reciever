#include <Arduino.h>
#include <avr/interrupt.h>

// ===================== Pins =====================
const int TX_PIN = 10;        // D10 = OC1B, Timer1 40 kHz output
const int PHASE_RX_PIN = 8;   // D8 = ICP1, Timer1 input capture

// Active peak detector amplitude pins
const int HOLD_PIN = A0;      // HOLD node from active peak detector
const int BIAS_PIN = A1;      // BIAS / virtual ground node

// ===================== Constants =====================
// Timer1 clock = 16 MHz, 1 tick = 62.5 ns = 0.0625 us
// 40 kHz period = 25 us = 400 ticks
const int PERIOD_TICKS = 400;

// Phase calibration
const int ZERO_OFFSET_TICKS = 0;

// ADC reference voltage
// Ideally measure Arduino 5V pin with multimeter and replace 5.0 with real value.
const float ADC_REF = 5.0;

// Amplitude conversion constants
// HC10T-40TR-P sensitivity: -75 dBV/ubar ≈ 1.78 mV/Pa
const float RX_SENSITIVITY_V_PER_PA = 0.00178;

// Your receiver amplifier gain:
// R6 = 22k, R5 = 10k => gain = 1 + 22k/10k = 3.2
const float AMP_GAIN = 3.2;

// Active peak detector zero offset.
// First set TX off / no ultrasound, read Vpeak.
// If Vpeak reads e.g. 0.015 V with no signal, set this to 0.015.
const float PEAK_ZERO_OFFSET = -0.07;

// ===================== Capture variables =====================
volatile uint16_t captureTicksISR = 0;
volatile bool newCaptureISR = false;

// ===================== Wrap helper =====================
int wrapToSignedPeriod(int ticks) {
  ticks %= PERIOD_TICKS;

  if (ticks >= PERIOD_TICKS / 2) {
    ticks -= PERIOD_TICKS;
  }

  if (ticks < -PERIOD_TICKS / 2) {
    ticks += PERIOD_TICKS;
  }

  return ticks;
}

// ===================== Timer1: 40 kHz output on D10 + input capture on D8 =====================
void setupTimer1_40kHz_and_InputCapture() {
  pinMode(TX_PIN, OUTPUT);
  pinMode(PHASE_RX_PIN, INPUT);

  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
  TCNT1 = 0;

  // Fast PWM mode 15:
  // TOP = OCR1A
  // frequency = 16 MHz / (1 * (1 + OCR1A))
  //
  // OCR1A = 399 -> period = 400 ticks -> 40 kHz
  //
  // OC1B on D10:
  // non-inverting PWM, OCR1B = 200 -> 50% duty
  OCR1A = 399;
  OCR1B = 200;

  // WGM13:0 = 1111, Fast PWM, TOP = OCR1A
  TCCR1A |= (1 << WGM10) | (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);

  // Enable OC1B output on D10, non-inverting
  TCCR1A |= (1 << COM1B1);

  // Input capture on rising edge of D8 / ICP1
  TCCR1B |= (1 << ICES1);

  // Optional noise canceller:
  // TCCR1B |= (1 << ICNC1);

  // Enable input capture interrupt
  TIMSK1 |= (1 << ICIE1);

  // Start Timer1, prescaler = 1
  TCCR1B |= (1 << CS10);

  interrupts();
}

// ===================== Input Capture ISR =====================
ISR(TIMER1_CAPT_vect) {
  captureTicksISR = ICR1;
  newCaptureISR = true;
}

// ===================== Analog averaging =====================
int readAverageAnalog(int pin) {
  const int N = 50;
  long sum = 0;

  // Dummy read after switching ADC channel
  analogRead(pin);
  delayMicroseconds(50);

  for (int i = 0; i < N; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }

  return sum / N;
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);

  pinMode(HOLD_PIN, INPUT);
  pinMode(BIAS_PIN, INPUT);

  setupTimer1_40kHz_and_InputCapture();

  Serial.println("Timer1 hardware input-capture phase + active peak amplitude");
  Serial.println("holdRaw,biasRaw,Vhold,Vbias,Vpeak,Vpp,Vrms,p_rms,rawTicks,correctedTicks,dt_us,phase_deg");
}

// ===================== Loop =====================
void loop() {
  // ===================== Amplitude measurement =====================
  int holdRaw = readAverageAnalog(HOLD_PIN);
  int biasRaw = readAverageAnalog(BIAS_PIN);

  float Vhold = holdRaw * ADC_REF / 1023.0;
  float Vbias = biasRaw * ADC_REF / 1023.0;

  // Active peak detector:
  // Vhold ≈ Vbias + Vpeak_pin1
  float Vpeak_pin1 = Vhold - Vbias - PEAK_ZERO_OFFSET;

  if (Vpeak_pin1 < 0) {
    Vpeak_pin1 = 0;
  }

  float Vpp_pin1 = 2.0 * Vpeak_pin1;
  float Vrms_pin1 = Vpeak_pin1 / 1.41421356;

  // Estimated acoustic pressure RMS
  // pin1 voltage already includes amplifier gain
  float p_rms = Vrms_pin1 / (RX_SENSITIVITY_V_PER_PA * AMP_GAIN);

  // ===================== Phase measurement: unchanged =====================
  uint16_t rawTicksLocal = 0;
  bool hasCapture = false;

  noInterrupts();
  if (newCaptureISR) {
    rawTicksLocal = captureTicksISR;
    newCaptureISR = false;
    hasCapture = true;
  }
  interrupts();

  // ===================== Serial output =====================
  Serial.print("holdRaw=");
  Serial.print(holdRaw);

  Serial.print(", biasRaw=");
  Serial.print(biasRaw);

  Serial.print(", Vhold=");
  Serial.print(Vhold, 3);
  Serial.print(" V");

  Serial.print(", Vbias=");
  Serial.print(Vbias, 3);
  Serial.print(" V");

  Serial.print(", Vpeak=");
  Serial.print(Vpeak_pin1, 4);
  Serial.print(" V");

  Serial.print(", Vpp=");
  Serial.print(Vpp_pin1, 4);
  Serial.print(" V");

  Serial.print(", Vrms=");
  Serial.print(Vrms_pin1, 4);
  Serial.print(" V");

  Serial.print(", p_rms=");
  Serial.print(p_rms, 2);
  Serial.print(" Pa");

  if (hasCapture) {
    int rawTicks = rawTicksLocal % PERIOD_TICKS;

    int correctedTicks = rawTicks - ZERO_OFFSET_TICKS;
    correctedTicks = wrapToSignedPeriod(correctedTicks);

    float dt_us = correctedTicks * 0.0625;
    float phase_deg = correctedTicks * 360.0 / PERIOD_TICKS;

    Serial.print(", rawTicks=");
    Serial.print(rawTicks);

    Serial.print(", correctedTicks=");
    Serial.print(correctedTicks);

    Serial.print(", dt=");
    Serial.print(dt_us, 3);
    Serial.print(" us");

    Serial.print(", phase=");
    Serial.print(phase_deg, 2);
    Serial.println(" deg");
  } else {
    Serial.println(", no capture");
  }

  delay(200);
}