# PAT Receiver

Arduino Uno + PlatformIO project for measuring ultrasonic receiver phase and amplitude.

This project generates a 40 kHz transmit reference signal, captures the phase of the received comparator signal with hardware input capture, and estimates received amplitude from an active peak detector.

## What it does

- Outputs a 40 kHz square wave on Arduino `D10` using Timer1 hardware PWM.
- Measures received signal phase on Arduino `D8 / ICP1` using Timer1 input capture.
- Reads active peak detector output on `A0`.
- Reads analog bias / virtual ground reference on `A1`.
- Prints phase and amplitude data over serial at `115200` baud.

## Hardware overview

The expected analog front end is:

```text
40 kHz transmit output  -> transmitter / driver / transducer
receiver transducer     -> op-amp gain stages
amplified receiver      -> LM311P comparator for phase timing
amplified receiver      -> active peak detector for amplitude
bias / midpoint ref     -> analog reference readback
```

Typical parts used in the current setup:

- Arduino Uno
- Ultrasonic transducers, nominally 40 kHz
- MCP6022 dual op-amp for receiver amplification / envelope circuitry
- LM311P comparator for phase edge detection
- Active peak detector / HOLD node for amplitude measurement
- Virtual ground / bias reference for single-supply analog amplification

All analog circuitry and the Arduino must share the same ground.

## Pin map

| Signal | Arduino pin | Purpose |
|---|---:|---|
| 40 kHz TX reference | `D10` / `OC1B` | Timer1 PWM output, 50% duty |
| Phase input | `D8` / `ICP1` | Timer1 input capture from comparator output |
| Amplitude / HOLD | `A0` | Active peak detector hold voltage |
| Bias / midpoint | `A1` | Virtual ground / analog bias reference |
| Serial monitor | USB serial | Debug/data output at 115200 baud |

## Timer1 configuration

Timer1 is configured directly through AVR registers.

The Arduino Uno runs Timer1 from a 16 MHz clock with prescaler 1:

```text
Timer tick = 1 / 16 MHz = 62.5 ns
40 kHz period = 25 us = 400 ticks
```

The project uses:

```cpp
OCR1A = 399;   // TOP, 400 ticks per period
OCR1B = 200;   // 50% duty on D10 / OC1B
```

Input capture is enabled on `D8 / ICP1` using rising-edge capture. The capture ISR stores `ICR1` as the raw phase tick.

## Analog measurement

The code reads `A0` and `A1` with averaging:

- `A0` = `HOLD_PIN`, active peak detector output
- `A1` = `BIAS_PIN`, virtual ground / midpoint reference

The voltage conversion uses:

```cpp
Vhold = holdRaw * ADC_REF / 1023.0;
Vbias = biasRaw * ADC_REF / 1023.0;
```

The peak voltage is estimated as:

```cpp
Vpeak = Vhold - Vbias - PEAK_ZERO_OFFSET;
```

Then:

```cpp
Vpp  = 2.0 * Vpeak;
Vrms = Vpeak / sqrt(2);
p_rms = Vrms / (RX_SENSITIVITY_V_PER_PA * AMP_GAIN);
```

## Important calibration constants

These are near the top of `src/main.cpp`.

### `ADC_REF`

```cpp
const float ADC_REF = 5.0;
```

This should match the real Arduino 5V rail. For better amplitude accuracy, measure the Arduino `5V` pin with a multimeter and replace `5.0` with the measured value.

### `ZERO_OFFSET_TICKS`

```cpp
const int ZERO_OFFSET_TICKS = 0;
```

Use this to remove fixed phase delay from wiring, comparator propagation delay, transducer placement, and analog front-end delay.

Calibration method:

1. Connect a known reference setup.
2. Observe `rawTicks`.
3. Set `ZERO_OFFSET_TICKS` so the displayed phase is zero at the chosen reference condition.

### `PEAK_ZERO_OFFSET`

```cpp
const float PEAK_ZERO_OFFSET = -0.07;
```

This compensates the active peak detector zero offset. To calibrate:

1. Turn off the transmitter or block the ultrasonic signal.
2. Watch the printed `Vpeak` or compare `Vhold - Vbias`.
3. Adjust `PEAK_ZERO_OFFSET` until the no-signal peak value is close to zero.

### `RX_SENSITIVITY_V_PER_PA`

```cpp
const float RX_SENSITIVITY_V_PER_PA = 0.00178;
```

This is the receiver transducer sensitivity used for pressure estimation.

### `AMP_GAIN`

```cpp
const float AMP_GAIN = 3.2;
```

This is the receiver amplifier gain used for pressure estimation. The current code assumes:

```text
Gain = 1 + 22k / 10k = 3.2
```

Update this if the analog gain resistor values change.

## Serial output

At startup, the project prints a header, then repeatedly prints measurements such as:

```text
holdRaw=..., biasRaw=..., Vhold=... V, Vbias=... V, Vpeak=... V, Vpp=... V, Vrms=... V, p_rms=... Pa, rawTicks=..., correctedTicks=..., dt=... us, phase=... deg
```

If no new input-capture event has occurred, the line ends with:

```text
no capture
```

## Software dependencies

Recommended development setup:

- Visual Studio Code
- PlatformIO IDE extension
- Arduino Uno connected over USB

No extra project libraries are required beyond the Arduino framework and AVR support installed by PlatformIO.

The PlatformIO environment is:

```ini
[env:uno]
platform = atmelavr
board = uno
framework = arduino
monitor_speed = 115200
```

## Build, upload, and monitor

From the project root:

```bash
pio run
```

Upload to Arduino Uno:

```bash
pio run -t upload
```

Open serial monitor:

```bash
pio device monitor -b 115200
```

Or use the PlatformIO buttons in VS Code:

- Build
- Upload
- Monitor

## Quick test

1. Upload the firmware.
2. Open the serial monitor at `115200` baud.
3. Confirm that `D10` outputs a 40 kHz square wave.
4. Feed a comparator output into `D8 / ICP1`.
5. Confirm that `rawTicks`, `dt`, and `phase` appear in the serial output.
6. Check `A0` and `A1` values for amplitude and bias behavior.

## Troubleshooting

### Serial monitor shows nothing

- Confirm the monitor baud rate is `115200`.
- Check that the correct serial port is selected.
- Press the Arduino reset button after opening the monitor.

### `no capture` appears

- Check the LM311P output and pull-up resistor.
- Confirm the comparator output is connected to `D8`.
- Confirm the comparator produces a clean digital edge.
- Check that the receiver and Arduino grounds are connected.

### Phase is unstable

- Improve comparator signal quality.
- Add hysteresis around the comparator if needed.
- Keep analog wiring short.
- Check that the receiver signal is not clipping.
- Use shielding or grounding improvements around the transducer receiver path.

### Amplitude is wrong

- Measure the real Arduino 5V rail and update `ADC_REF`.
- Recalibrate `PEAK_ZERO_OFFSET`.
- Confirm the amplifier gain and update `AMP_GAIN` if resistor values changed.
- Verify that the active peak detector output is connected to `A0` and the bias reference to `A1`.
