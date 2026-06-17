#include <Arduino.h>
#include <avr/interrupt.h>

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
volatile int16_t phaseDeltaTicksISR = 0;

volatile bool haveRef = false;
volatile bool haveRx  = false;
volatile bool newPhase = false;

volatile uint8_t lastPindISR = 0;

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

// ===================== Pin-change timestamping =====================
void startD2D3PinChangeInterrupts() {
  noInterrupts();

  lastPindISR = PIND;

  // D2 = PD2 = PCINT18, D3 = PD3 = PCINT19. They share the PCINT2 vector,
  // so simultaneous edges are timestamped once instead of being delayed by
  // external-interrupt priority between INT0 and INT1.
  PCIFR  |= (1 << PCIF2);                       // clear pending flag
  PCICR  |= (1 << PCIE2);                       // enable Port D pin changes
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);    // watch D2 and D3 only

  interrupts();
}

ISR(PCINT2_vect) {
  uint16_t now = TCNT1;
  uint8_t pindNow = PIND;
  uint8_t changed = pindNow ^ lastPindISR;
  uint8_t rising = changed & pindNow;

  lastPindISR = pindNow;

  bool refRise = rising & (1 << PD3);
  bool rxRise  = rising & (1 << PD2);

  if (refRise) {
    refTickISR = now;
    haveRef = true;
  }

  if (rxRise) {
    rxTickISR = now;
    haveRx = true;

    if (haveRef) {
      phaseDeltaTicksISR = (int16_t)(rxTickISR - refTickISR);
      newPhase = true;
    }
  }
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

  startD2D3PinChangeInterrupts();

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
  int16_t deltaTicksLocal;
  bool localHaveRef;
  bool localHaveRx;
  bool localNewPhase;

  noInterrupts();
  refTickLocal = refTickISR;
  rxTickLocal  = rxTickISR;
  deltaTicksLocal = phaseDeltaTicksISR;

  localHaveRef = haveRef;
  localHaveRx  = haveRx;
  localNewPhase = newPhase;

  newPhase = false;
  interrupts();

  //Serial.print("raw=");
  //Serial.print(raw);

  //Serial.print(", avg=");
  //Serial.print(avg);

  //Serial.print(", Venv=");
  //Serial.print(voltage, 3);
  //Serial.print(" V");

  if (localHaveRef && localHaveRx && localNewPhase) {
    // Convert both timestamps to phase within one 40 kHz period.
    // These are only for debugging; do not subtract them directly because
    // Timer1 overflows every 65536 ticks, and 65536 is not a multiple of 400.
    int refPhaseTicks = refTickLocal % PERIOD_TICKS;
    int rxPhaseTicks  = rxTickLocal  % PERIOD_TICKS;

    // Fold the signed elapsed time into one 40 kHz period.
    int rawPhaseTicks = wrapToSignedPeriod(deltaTicksLocal);
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
    Serial.print(deltaTicksLocal);

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

  delay(200);
}
