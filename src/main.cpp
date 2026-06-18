#include <Arduino.h>
#include <avr/interrupt.h>

// ===================== Pins =====================
const int TX_PIN = 10;        // D10 = OC1B, Timer1 40 kHz output
const int PHASE_RX_PIN = 8;   // D8 = ICP1, Timer1 input capture
const int RX_ENV_PIN = A0;    // envelope voltage input

// ===================== Constants =====================
// Timer1 clock = 16 MHz, 1 tick = 62.5 ns = 0.0625 us
// 40 kHz period = 25 us = 400 ticks
const int PERIOD_TICKS = 400;

// Start with 0.
// For calibration: connect D10 directly to D8.
// If rawTicks is stable at e.g. 2, set ZERO_OFFSET_TICKS = 2.
const int ZERO_OFFSET_TICKS = 0;

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
  // Uncomment if D8 has glitches, but it adds a small fixed delay.
  // TCCR1B |= (1 << ICNC1);

  // Enable input capture interrupt
  TIMSK1 |= (1 << ICIE1);

  // Start Timer1, prescaler = 1
  TCCR1B |= (1 << CS10);

  interrupts();
}

// ===================== Input Capture ISR =====================
ISR(TIMER1_CAPT_vect) {
  // Hardware captures TCNT1 into ICR1 at the D8 edge.
  captureTicksISR = ICR1;
  newCaptureISR = true;
}

// ===================== Envelope averaging =====================
int readAverageA0() {
  const int N = 50;
  long sum = 0;

  for (int i = 0; i < N; i++) {
    sum += analogRead(RX_ENV_PIN);
    delayMicroseconds(200);
  }

  return sum / N;
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);

  pinMode(RX_ENV_PIN, INPUT);

  setupTimer1_40kHz_and_InputCapture();

  Serial.println("Timer1 hardware input-capture phase measurement");
  Serial.println("raw,avg,Venv,rawTicks,correctedTicks,dt_us,phase_deg");
}

// ===================== Loop =====================
void loop() {
  int raw = analogRead(RX_ENV_PIN);
  int avg = readAverageA0();
  float voltage = avg * 5.0 / 1023.0;

  uint16_t rawTicksLocal = 0;
  bool hasCapture = false;

  noInterrupts();
  if (newCaptureISR) {
    rawTicksLocal = captureTicksISR;
    newCaptureISR = false;
    hasCapture = true;
  }
  interrupts();

  Serial.print("raw=");
  Serial.print(raw);

  Serial.print(", avg=");
  Serial.print(avg);

  Serial.print(", Venv=");
  Serial.print(voltage, 3);
  Serial.print(" V");

  if (hasCapture) {
    // rawTicksLocal is the Timer1 count when RX comparator rising edge arrives.
    // Since Timer1 period is 400 ticks, this is already phase within one cycle.
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