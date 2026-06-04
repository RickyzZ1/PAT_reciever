#include <Arduino.h>

const int TX_PIN = 8;
const int RX_PIN = A0;

// 40 kHz: period = 25 us
// digitalWrite has some overhead, so this is approximate.
// Good enough for first breadboard testing.
const int HIGH_US = 12;
const int LOW_US  = 13;

void setup() {
  Serial.begin(115200);

  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, LOW);

  Serial.println("Bare ultrasonic TX/RX envelope test");
  Serial.println("baseline,peak,diff");
}

void send40kBurst(int cycles) {
  for (int i = 0; i < cycles; i++) {
    digitalWrite(TX_PIN, HIGH);
    delayMicroseconds(HIGH_US);

    digitalWrite(TX_PIN, LOW);
    delayMicroseconds(LOW_US);
  }
}

int readAverageA0() {
  long sum = 0;
  const int N = 50;

  for (int i = 0; i < N; i++) {
    sum += analogRead(RX_PIN);
    delayMicroseconds(300);
  }

  return sum / N;
}

void loop() {
  // 1. Measure baseline with transmitter off
  digitalWrite(TX_PIN, LOW);
  delay(30);

  int baseline = readAverageA0();

  // 2. Send repeated 40 kHz bursts and measure envelope peak
  int peak = 0;

  unsigned long startTime = millis();

  while (millis() - startTime < 80) {
    send40kBurst(40);     // 40 cycles ≈ 1 ms burst

    delayMicroseconds(300);

    int value = analogRead(RX_PIN);

    if (value > peak) {
      peak = value;
    }
  }

  int diff = peak - baseline;

  Serial.print(baseline);
  Serial.print(",");
  Serial.print(peak);
  Serial.print(",");
  Serial.println(diff);

  delay(200);
}






