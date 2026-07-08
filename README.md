# PAT Receiver STM32 Migration Notes

This project is the STM32 migration of the Arduino/PlatformIO `PAT_reciever` project.

The original Arduino project used:

- Arduino Uno / PlatformIO (`board = uno`, `framework = arduino`, serial monitor at 115200 baud).
- D10 / OC1B / Timer1 as the 40 kHz transmit PWM output.
- D8 / ICP1 / Timer1 input capture for phase measurement.
- A0 as the active peak detector `HOLD` / amplitude input.
- A1 as the `BIAS` / virtual ground input.
- Timer1 at 16 MHz, 40 kHz period = 400 ticks.

The STM32 port currently targets:

- Board: NUCLEO-G431KB / STM32G431KBTx
- Tool flow: STM32CubeMX -> Makefile -> VS Code -> `arm-none-eabi-gcc` -> OpenOCD/ST-LINK
- Timer clock: 170 MHz
- 40 kHz period: 4250 timer ticks

---

## 1. STM32 pin mapping

| Function | Arduino project | STM32 port | Notes |
|---|---:|---:|---|
| 40 kHz transmit PWM | D10 / OC1B | PA8 / TIM1_CH1 | 3.3 V logic PWM |
| Phase comparator input | D8 / ICP1 | PA6 / TIM3_CH1 | LM311 output must be pulled up to 3.3 V |
| Amplitude / HOLD ADC | A0 | PA0 / ADC1_IN1 | Use divider if signal is 0-5 V |
| Bias / midpoint ADC | A1 | PA1 / ADC1_IN2 | Use divider if signal is 0-5 V |
| Serial debug | USB serial | USART2 VCP PA2/PA3 | 115200 baud |

The first validated loopback test was:

```text
PA8 -> PA6
```

Expected serial behavior in loopback:

```text
delta ~= 40000 captures/s
rawTicks ~= 5
phase ~= 0.423 deg before calibration
```

---

## 2. Power and level shifting

The analog front-end was originally powered from the Arduino board 5 V pin. Keep that design if the analog chain depends on a 5 V midpoint and signal swing.

Recommended power structure:

```text
NUCLEO 5V  -> MCP6022 VDD
NUCLEO 5V  -> LM311P VCC+
NUCLEO GND -> analog circuit GND
STM32 GND  -> analog circuit GND
```

The STM32 MCU pins are still 3.3 V-domain pins. Do not feed 5 V analog signals directly into PA0/PA1.

### 2.1 MCP6022 analog outputs to STM32 ADC

If the MCP6022 outputs may reach 5 V, add a divider on each ADC input:

```text
MCP6022 HOLD / amplitude ---- 10k ----+---- PA0 / ADC1_IN1
                                      |
                                     18k
                                      |
                                     GND

MCP6022 BIAS / midpoint ------ 10k ----+---- PA1 / ADC1_IN2
                                       |
                                      18k
                                       |
                                      GND
```

This maps:

```text
5.00 V -> 3.21 V
2.50 V -> 1.61 V
```

The STM32 code converts the ADC reading back to the original pre-divider voltage using:

```c
original_mV = adc_raw * 3300 * 28 / (4095 * 18);
```

If you later power the analog chain from 3.3 V and remove the dividers, change:

```c
#define ADC_DIVIDER_NUM 28UL
#define ADC_DIVIDER_DEN 18UL
```

to:

```c
#define ADC_DIVIDER_NUM 1UL
#define ADC_DIVIDER_DEN 1UL
```

### 2.2 LM311P comparator output to PA6

LM311P output is open collector. It can keep 5 V supply, but the output pull-up must be to 3.3 V:

```text
LM311P collector/output ----+---- PA6 / TIM3_CH1
                            |
                           4.7k to 10k
                            |
                           STM32 3.3V

LM311P emitter ------------ GND
```

Do not pull LM311P output to 5 V when it is connected to PA6.

### 2.3 PA8 transmit PWM level

PA8 outputs a 3.3 V 40 kHz PWM. If the original Arduino D10 5 V PWM directly affected transmitter drive strength, add a 3.3 V-to-5 V driver stage or MOSFET/transistor driver. If PA8 only drives a high-impedance logic/driver input, 3.3 V may already be enough.

---

## 3. CubeMX configuration

### Clock

- SYSCLK / HCLK: 170 MHz
- APB1 timer clocks: 170 MHz
- APB2 timer clocks: 170 MHz

### TIM1: 40 kHz PWM output

- TIM1 clock source: Internal Clock
- Channel 1: PWM Generation CH1
- Pin: PA8 / TIM1_CH1
- Prescaler: 0
- Counter Period / ARR: 4249
- Pulse / CCR1: 2125
- TRGO / Master Output Trigger: Update Event

Frequency:

```text
170 MHz / (4249 + 1) = 40 kHz
```

### TIM3: input capture phase timer

- Clock Source: Internal Clock
- Slave Mode: Reset Mode
- Trigger Source: ITR0 / TIM1_TRGO
- CH1: Input Capture direct mode -> PA6
- CH2: Input Capture direct mode -> PA7, configured for future expansion
- CH3: Input Capture direct mode -> PB0, configured for future expansion
- CH4: Input Capture direct mode -> PB7, configured for future expansion
- Prescaler: 0
- Counter Period / ARR: 4249
- NVIC: TIM3 global interrupt enabled

Only CH1 is started in code for now.

### ADC1

- PA0 -> ADC1_IN1 Single-ended, Rank 1
- PA1 -> ADC1_IN2 Single-ended, Rank 2
- Number Of Conversion: 2
- Resolution: 12-bit
- External Trigger: Software Start
- Continuous Conversion: Disabled
- Sampling time: 47.5 cycles or higher

### USART2 / VCP

- Mode: Asynchronous
- Baud rate: 115200
- Word length: 8 bits
- Parity: None
- Stop bits: 1
- Hardware flow control: None
- Pins: PA2/PA3

---

## 4. VS Code and Linux dependencies

Install VS Code extensions:

```bash
code --install-extension ms-vscode.cpptools
code --install-extension ms-vscode.makefile-tools
code --install-extension marus25.cortex-debug
```

Install Linux packages:

```bash
sudo apt update
sudo apt install gcc-arm-none-eabi make gdb-multiarch openocd picocom
```

If serial permission is denied:

```bash
sudo usermod -aG dialout $USER
```

Then log out and log back in.

---

## 5. Build, flash, serial monitor

From the CubeMX-generated Makefile project directory:

```bash
make clean && make && openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
  -c "program build/PAT_RECEIVER_STM32.elf verify reset exit"
```

Use `&&` so OpenOCD runs only if the build succeeds.

Find the stable ST-LINK VCP path:

```bash
ls -l /dev/serial/by-id/
```

Open serial monitor:

```bash
picocom -b 115200 /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_0041001E3335510333383531-if02
```

Exit picocom:

```text
Ctrl + A, then Ctrl + X
```

---

## 6. Calibration constants

### Phase zero offset

In loopback PA8 -> PA6, rawTicks was about 5. For real hardware, calibrate with a known reference and then set:

```c
#define ZERO_OFFSET_TICKS 0L
```

For example, if your reference setup reads 5 ticks and you want that to be 0 degrees:

```c
#define ZERO_OFFSET_TICKS 5L
```

### Peak detector zero offset

The Arduino code used:

```c
const float PEAK_ZERO_OFFSET = -0.07;
```

The STM32 fixed-point version uses:

```c
#define PEAK_ZERO_OFFSET_MV (-70L)
```

Re-measure this with TX off / no ultrasound.

### ADC divider ratio

For 10k/18k divider:

```c
#define ADC_DIVIDER_NUM 28UL
#define ADC_DIVIDER_DEN 18UL
```

For direct 0-3.3 V input:

```c
#define ADC_DIVIDER_NUM 1UL
#define ADC_DIVIDER_DEN 1UL
```

### Receiver sensitivity and gain

Original project constants:

```c
RX sensitivity = 0.00178 V/Pa
AMP gain = 3.2
```

STM32 fixed-point constants:

```c
#define RX_SENS_UV_PER_PA 1780UL
#define AMP_GAIN_X1000    3200UL
```

---

## 7. Expected output

Example output after ADC and capture are working:

```text
holdRaw=...,biasRaw=...,Vhold_mV=...,Vbias_mV=...,Vpeak_mV=...,Vpp_mV=...,Vrms_mV=...,p_cPa=...,rawTicks=...,correctedTicks=...,dt_ns=...,phase_mdeg=...,capCount=...,adc_ok=1
```

Units:

- `Vhold_mV`, `Vbias_mV`, `Vpeak_mV`, `Vpp_mV`, `Vrms_mV`: millivolts in the original analog 5 V domain before divider.
- `p_cPa`: centi-Pascal. Divide by 100 to get Pa.
- `phase_mdeg`: milli-degrees. Divide by 1000 to get degrees.
- `dt_ns`: nanoseconds.

---

## 8. Quick validation checklist

1. PA8 -> PA6 loopback:

```text
capture delta around 40000 per second
rawTicks stable
```

2. ADC test:

```text
PA0 to GND  -> holdRaw near 0
PA0 to 3.3V -> holdRaw near 4095
PA1 to GND  -> biasRaw near 0
PA1 to 3.3V -> biasRaw near 4095
```

3. Real circuit:

```text
MCP6022/LM311P powered from 5V
GND common with STM32
MCP6022 outputs go through dividers to PA0/PA1
LM311 output pull-up goes to 3.3V, not 5V
PA8 drives transmitter input
PA6 receives comparator output
```
