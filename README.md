# 🤖 STM32F407 — 4WD Bluetooth Robot Car

> Bare-metal embedded C project on the **STM32F407VG** Discovery board.  
> Full peripheral control (GPIO, TIM, ADC, USART) coded from scratch — **no HAL, no CubeMX.**

## 📸 Gallery

| Top View | Wiring |
|----------|--------|
| ![top](images/top_view.jpg) | ![wiring](images/wiring.svg) |

### 🎬 In Action

https://github.com/user-attachments/assets/e5261168-0720-463b-8404-047694418e77


## 📋 Table of Contents

- [Overview](#overview)
- [Hardware](#hardware)
- [Wiring](#wiring)
- [Firmware Architecture](#firmware-architecture)
- [Peripheral Deep-Dive](#peripheral-deep-dive)
- [Command Protocol](#command-protocol)
- [ADC Telemetry](#adc-telemetry)
- [PWM & Motor Control](#pwm--motor-control)
- [How to Build & Flash](#how-to-build--flash)
- [Project Structure](#project-structure)
- [What I Learned](#what-i-learned)

---

## Overview

This project implements a **4-wheel drive robot car** controlled wirelessly via a **HC-06 Bluetooth module**.  
A smartphone app sends single-character commands (`F`, `B`, `L`, `R`, `STOP`) over Bluetooth → USART2.  
Three **potentiometers** are sampled by the ADC and their values streamed back over USART as telemetry.

**Everything is written in bare-metal C**, directly manipulating registers from the STM32F4 reference manual — no abstraction layers.

---

## Hardware

| Component | Description |
|-----------|-------------|
| **MCU** | STM32F407VG (Discovery board) — Cortex-M4, 168 MHz |
| **Motor Driver** | L298N H-Bridge — dual full-bridge |
| **Motors** | 4× DC gear motors |
| **Bluetooth** | HC-06 module — UART slave, 9600 baud |
| **Sensors** | 3× potentiometers (analog telemetry / future sensor inputs) |
| **Power** | 7.4 V LiPo → L298N Vin; 5 V regulated for HC-06 & MCU |

---

## Wiring

> 📷 *See `/images/` folder for full wiring photographs.*

### USART2 ↔ HC-06

| STM32 Pin | HC-06 Pin | Function |
|-----------|-----------|----------|
| PA2 (AF7) | RXD | MCU TX → BT RX |
| PA3 (AF7) | TXD | BT TX → MCU RX |
| 3.3 V | VCC | Power |
| GND | GND | Ground |

> ⚠️ The HC-06 TXD outputs 3.3 V logic — safe to connect directly to PA3.

### TIM3 PWM → L298N

| STM32 Pin | TIM3 Channel | L298N Input | Motor |
|-----------|-------------|-------------|-------|
| PA6 (AF2) | CH1 / CCR1 | IN1 | LEFT — forward |
| PA7 (AF2) | CH2 / CCR2 | IN2 | LEFT — reverse |
| PB0 (AF2) | CH3 / CCR3 | IN3 | RIGHT — forward |
| PB1 (AF2) | CH4 / CCR4 | IN4 | RIGHT — reverse |

### ADC1 — Potentiometers

| STM32 Pin | ADC Channel | Label |
|-----------|-------------|-------|
| PA4 | CH4 | V1 |
| PC4 | CH14 | V2 |
| PC5 | CH15 | V3 |

### EXTI0 — Push Button

| STM32 Pin | Function |
|-----------|----------|
| PA0 | Rising edge → starts TIM6 & ADC sampling |

---

## Firmware Architecture

```
main()
 ├── config_TIM3_PWM()       — 4-channel PWM @ 16 kHz, motors idle
 ├── config_USART2_TXRX()    — 9600 baud, TX+RX+RXNE+IDLE interrupts
 ├── config_TIM6_IT()        — 1 Hz timebase (not started yet)
 ├── config_ADC1_SCAN_DISCONT_IT() — scan 3 channels, EOC interrupt
 └── config_EXTI0()          — PA0 rising edge starts TIM6

ISR flow:
  EXTI0_IRQHandler   → starts TIM6
  TIM6_DAC_IRQHandler → every 1 s: triggers ADC SWSTART
  ADC_IRQHandler      → collects CH4→CH14→CH15, then sends USART frame
  USART2_IRQHandler   → RXNE: buffer char | IDLE: parse command → PWM
```

The main loop is intentionally **empty** — everything runs through interrupts.  
This demonstrates proper **interrupt-driven embedded design**.

---

## Peripheral Deep-Dive

### TIM6 — 1 Hz Timebase

```c
TIM6->PSC = 15999;   // 16 MHz / 16000 = 1 kHz
TIM6->ARR = 999;     // 1 kHz / 1000  = 1 Hz
```
TIM6 is not started at boot — it is gated by the PA0 button via EXTI0.  
This is a deliberate design choice: the user arms the sensor acquisition loop.

### ADC1 — Scan + Discontinuous Mode

3 channels are converted **one at a time**, each triggered by a software start (`SWSTART`).  
The `EOCS` bit (CR2 bit 10) makes EOC fire after **each individual conversion**, not the whole sequence.  
After all 3 values are collected, `prepare_and_send_buffer()` formats and transmits them.

```
SWSTART → CH4 → EOC ISR → store [0] → SWSTART
       → CH14 → EOC ISR → store [1] → SWSTART
       → CH15 → EOC ISR → store [2] → send "CHx=N CHy=N CHz=N\n"
```

### USART2 — Idle-Line Detection

Rather than using a terminator character, the firmware uses the **IDLE line interrupt** (CR1 bit 4).  
The UART fires IDLE when the bus goes silent after receiving data — perfect for variable-length commands.

```c
if (sr & (1U << 4))   // IDLE flag
{
    (void)USART2->SR;  // mandatory double-read to clear IDLE
    (void)USART2->DR;
    // parse rx_buffer here
}
```

### TIM3 — PWM @ 16 kHz

```c
TIM3->PSC = 0;        // no prescaler
TIM3->ARR = 999;      // 16 MHz / 1000 = 16 kHz
```

PWM mode 1 with output preload enabled on all 4 channels.  
The **UG event** (`TIM3->EGR = 1`) is forced after configuration to load shadow registers before `CEN` — a critical step often missed that causes glitchy first cycles.

---

## Command Protocol

Sent from a Bluetooth terminal app (e.g. **Serial Bluetooth Terminal**) at 9600 baud.

| Command | Action |
|---------|--------|
| `F` | Forward — both motors forward @ PWM_HIGH (80%) |
| `B` | Reverse — both motors reverse @ PWM_HIGH (80%) |
| `L` | Turn left — left motor reverse, right forward |
| `R` | Turn right — right motor reverse, left forward |
| `STOP` | All PWM channels → 0 |

The parser uses `strstr()` so commands can be embedded in longer strings (e.g. from joystick apps that send `"MOVE:F\n"`).

---

## ADC Telemetry

Every second (once armed), the MCU transmits over USART:

```
CHx=1847 CHy=2301 CHz=892
```

Values are 12-bit (0–4095), proportional to 0–3.3 V on the potentiometer wiper.  
This channel can be repurposed for **IR sensors, ultrasonic distance, or line following** — the ADC infrastructure is already in place.

---

## PWM & Motor Control

The L298N truth table implemented:

| Mode | IN_A (CCRx) | IN_B (CCRy) | Result |
|------|-------------|-------------|--------|
| Forward | PWM_HIGH (800) | 0 | Motor spins forward |
| Reverse | 0 | PWM_HIGH (800) | Motor spins reverse |
| Stop | 0 | 0 | Motor coasts |

`PWM_HIGH = 800` out of `ARR = 999` → **80% duty cycle**.  
`PWM_LOW = 300` is defined for future speed control (can be extended to variable speed via Bluetooth command).

---

## How to Build & Flash

### Requirements

- **IDE**: Keil µVision 5
- **Device Pack**: Keil.STM32F4xx_DFP
- **Programmer**: ST-Link (on-board on STM32F407 Discovery)

### Build & Flash

1. Open the `.uvprojx` project file in Keil µVision
2. Build: `F7`
3. Flash: `F8` (via on-board ST-Link)

### Serial Monitor

```bash
# Linux/macOS
screen /dev/ttyUSB0 9600

# Windows
# Use PuTTY or Tera Term @ 9600 8N1
```

---

## Project Structure

```
stm32-robot-car/
├── main.c              ← All firmware (single-file, self-contained)
├── images/
│   ├── top_view.jpg
│   ├── wiring.jpg
│   └── demo.mp4
└── README.md
```

## What I Learned

This project was built to demonstrate **low-level embedded programming skills**:

- **Register-level peripheral configuration** — no HAL, every bit set manually with the reference manual open
- **Interrupt-driven architecture** — zero polling in the main loop; all logic lives in ISRs
- **ADC scan + discontinuous mode** — understanding EOCS vs EOC, software-triggered multi-channel conversion
- **UART IDLE detection** — a robust, protocol-agnostic way to frame variable-length messages
- **PWM shadow register pitfall** — the `EGR=UG` requirement before `CEN` is a classic beginner trap; this project handles it correctly
- **H-Bridge direction control** — translating L298N truth tables into complementary PWM channel pairs
- **Bluetooth UART integration** — transparent serial bridge, no custom protocol needed

---

## Future Improvements

- [ ] Variable speed control via Bluetooth (e.g. `F:60` for 60% duty)
- [ ] Ultrasonic obstacle avoidance (HC-SR04 on TIM capture input)
- [ ] Replace potentiometers with IR line sensors for line-following mode
- [ ] DMA for USART TX instead of blocking `while(TXE==0)`
- [ ] PID speed control using motor encoder feedback

---

## 🙏 Acknowledgements

I would like to express my sincere gratitude to **Mr. Khalil ABID**, my Embedded Systems professor at the Higher National Engineering School of Tunis (ENSIT). His expert guidance, insightful teaching, and dedication throughout the lab sessions provided me with the essential skills and foundation necessary to make this project possible.

---

## License

MIT — feel free to use, modify, and learn from this code.

---

*Documentation and implementation based on the STM32F4 Reference Manual (RM0090)*
