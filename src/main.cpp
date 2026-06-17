#include <Arduino.h>

// ===================== Pins =====================
const int TX_PIN       = 11;   // D11 = OC2A, Timer2 40 kHz output
const int REF_PIN      = 3;    // D3 = INT1, reference input, jumper from D11
const int PHASE_RX_PIN = 2;    // D2 = INT0, comparator output
const int RX_ENV_PIN   = A0;   // envelope voltage input

// ===================== Constants =====================
// Timer1: 16 MHz, 1 tick = 62.5 ns = 0.0625 us
// 40 kHz period = 25 us = 400 ticks
const int PERIOD_TICKS = 400;

// IMPORTANT:
// This offset must be measured using THIS timestamp algorithm.
// Start with 0.
// If same signal on D2 and D3 gives rawPhaseTicks = k,
// set ZERO_OFFSET_TICKS = k.
const int ZERO_OFFSET_TICKS = 0;

// ===================== ISR shared variables =====================
volatile uint16_t refTickISR = 0;
volatile uint16_t rxTickISR  = 0;

volatile bool haveRef = false;
volatile bool haveRx  = false;
volatile bool newRxEdge = false;

// ===================== Generate 40 kHz on D11 =====================
void start40kHzOnD11() {
  pinMode(TX_PIN, OUTPUT);

  // Timer2 CTC mode, toggle OC2A on compare match.
  //
  // f = 16 MHz / (2 * prescaler * (1 + OCR2A))
  // prescaler = 8, OCR2A = 24
  // f = 16 MHz / (2 * 8 * 25) = 40 kHz

  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  TCCR2A |= (1 << WGM21);    // CTC mode
  TCCR2A |= (1 << COM2A0);   // Toggle OC2A on D11
  TCCR2B |= (1 << CS21);     // prescaler = 8

  OCR2A = 24;
}

// ===================== Timer1 free-running =====================
void startTimer1FreeRunning() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  TCCR1B |= (1 << CS10);     // no prescaler, 16 MHz
}

// ===================== Wrap helper =====================
int wrapToSignedPeriod(int ticks) {
  while (ticks >= PERIOD_TICKS / 2) {
    ticks -= PERIOD_TICKS;
  }

  while (ticks < -PERIOD_TICKS / 2) {
    ticks += PERIOD_TICKS;
  }

  return ticks;
}

// ===================== Interrupts =====================
void refISR() {
  refTickISR = TCNT1;
  haveRef = true;
}

void rxISR() {
  rxTickISR = TCNT1;
  haveRx = true;
  newRxEdge = true;
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
  pinMode(REF_PIN, INPUT);
  pinMode(PHASE_RX_PIN, INPUT);

  startTimer1FreeRunning();
  start40kHzOnD11();

  attachInterrupt(digitalPinToInterrupt(REF_PIN), refISR, RISING);
  attachInterrupt(digitalPinToInterrupt(PHASE_RX_PIN), rxISR, RISING);

  Serial.println("Timestamp phase measurement");
  Serial.println("raw, avg, Venv, refPhase, rxPhase, deltaTicks, rawPhaseTicks, correctedTicks, dt_us, phase_deg");
}

// ===================== Loop =====================
void loop() {
  // -------- Envelope amplitude --------
  int raw = analogRead(RX_ENV_PIN);
  int avg = readAverageA0();
  float voltage = avg * 5.0 / 1023.0;

  // -------- Copy phase timestamps atomically --------
  uint16_t refTickLocal;
  uint16_t rxTickLocal;
  bool localHaveRef;
  bool localHaveRx;
  bool localNewRx;

  noInterrupts();
  refTickLocal = refTickISR;
  rxTickLocal  = rxTickISR;

  localHaveRef = haveRef;
  localHaveRx  = haveRx;
  localNewRx   = newRxEdge;

  newRxEdge = false;
  interrupts();

  Serial.print("raw=");
  Serial.print(raw);

  Serial.print(", avg=");
  Serial.print(avg);

  Serial.print(", Venv=");
  Serial.print(voltage, 3);
  Serial.print(" V");

  if (localHaveRef && localHaveRx && localNewRx) {
    // Convert both timestamps to phase within one 40 kHz period.
    // These are only for debugging; do not subtract them directly because
    // Timer1 overflows every 65536 ticks, and 65536 is not a multiple of 400.
    int refPhaseTicks = refTickLocal % PERIOD_TICKS;
    int rxPhaseTicks  = rxTickLocal  % PERIOD_TICKS;

    // First subtract the raw Timer1 timestamps. uint16_t subtraction handles
    // one Timer1 overflow correctly as long as the two edges are less than
    // 65536 ticks apart.
    uint16_t deltaTicks = rxTickLocal - refTickLocal;

    // Then fold the elapsed time into one 40 kHz period.
    int rawPhaseTicks = deltaTicks % PERIOD_TICKS;
    rawPhaseTicks = wrapToSignedPeriod(rawPhaseTicks);

    // Apply calibration offset
    int correctedTicks = rawPhaseTicks - ZERO_OFFSET_TICKS;
    correctedTicks = wrapToSignedPeriod(correctedTicks);

    float dt_us = correctedTicks * 0.0625;
    float phase_deg = correctedTicks * 360.0 / PERIOD_TICKS;

    Serial.print(", refPhase=");
    Serial.print(refPhaseTicks);

    Serial.print(", rxPhase=");
    Serial.print(rxPhaseTicks);

    Serial.print(", deltaTicks=");
    Serial.print(deltaTicks);

    Serial.print(", rawPhaseTicks=");
    Serial.print(rawPhaseTicks);

    Serial.print(", correctedTicks=");
    Serial.print(correctedTicks);

    Serial.print(", dt=");
    Serial.print(dt_us, 3);
    Serial.print(" us");

    Serial.print(", phase=");
    Serial.print(phase_deg, 2);
    Serial.println(" deg");
  } else {
    Serial.println(", no phase");
  }

  delay(20);
}
