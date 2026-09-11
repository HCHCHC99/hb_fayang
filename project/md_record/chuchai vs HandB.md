# BOARD_VERSION: Hardware Board Variant Differences

## Overview

`BOARD_VERSION` is defined in `template/source/main.h:30` and controls hardware-specific conditional compilation across 5 source files. It selects between two board hardware revisions:

| BOARD_VERSION | Board Name | Description |
|---|---|---|
| `0` | **chu_chai** (original HB_chuchai board) | Legacy board with Hall current sensor, GPIO motor control |
| `1` | **HandB** (integrated board) | New integrated board with differential amplifier current sensor, PWM motor control |

---

## 1. ADC Voltage Sensing Pin

**File:** `App/App_Motor_Project.h:96-107`

Controls which ADC pin and channel is used to sample the DC bus voltage.

```c
#if BOARD_VERSION == 0
    #define PIN_ADC_VOLTAGE_PORT    GPIO_PORT_A
    #define PIN_ADC_VOLTAGE_PIN     GPIO_PIN_04    // PA04
    #define PIN_ADC_VOLTAGE_CH      (4)            // ADC channel 4
#else
    #define PIN_ADC_VOLTAGE_PORT    GPIO_PORT_A
    #define PIN_ADC_VOLTAGE_PIN     GPIO_PIN_06    // PA06
    #define PIN_ADC_VOLTAGE_CH      (6)            // ADC channel 6
#endif
```

| Item | chu_chai (`0`) | HandB (`1`) |
|---|---|---|
| GPIO Port | GPIO_PORT_A | GPIO_PORT_A |
| GPIO Pin | PA04 | PA06 |
| ADC Channel | CH4 | CH6 |

> **Impact:** The MCU reads the bus voltage from a different physical pin. The ADC channel number is used to configure the ADC peripheral for the correct analog input.

---

## 2. Motor Control Mode (GPIO vs PWM)

**File:** `Dev/dev_motor.h:29-38`

Defines `MOTOR_CONTROL_MODE`, which determines how the motor is physically driven.

```c
#include "main.h"
#if BOARD_VERSION == 0
    #define MOTOR_CONTROL_MODE  0   // GPIO mode
#else
    #define MOTOR_CONTROL_MODE  1   // PWM mode
#endif
```

### 2.1 GPIO Mode (`MOTOR_CONTROL_MODE == 0`) — chu_chai board

**File:** `Dev/dev_motor.c:334-400`

Motor control uses two GPIO pins (PB8, PB9) defined in `Adp/Adapter.h:9-13`:

```c
#define PHU_PORT    GPIO_PORT_B
#define PHU_PIN     GPIO_PIN_08     // PB8
#define PHV_PORT    GPIO_PORT_B
#define PHV_PIN     GPIO_PIN_09     // PB9
```

Pin initialization in `Adp/Hardware.c:267-268` — both initialized LOW.

**Control Logic (H-bridge pattern):**

| Action | Normal Direction (`motor_dir=0`) | Inverted Direction (`motor_dir!=0`) |
|---|---|---|
| **Stop** | PHU=0, PHV=0 | PHU=0, PHV=0 |
| **Forward (Fwd)** | PHU=1, PHV=0 | PHU=0, PHV=1 |
| **Reverse (Rev)** | PHU=0, PHV=1 | PHU=1, PHV=0 |

- A 100ms delay (`tickTimer_DelayMs(100)`) is inserted before direction changes as dead-time protection.
- The `g_AppParam.motor_dir` flag can globally invert the motor direction mapping.
- **No soft-start/soft-stop** — the motor switches on/off immediately via GPIO level change.

### 2.2 PWM Mode (`MOTOR_CONTROL_MODE == 1`) — HandB board

**File:** `Dev/dev_motor.c:14-19, 341-404`

Motor control uses 4-channel PWM via TMRA module:

```c
#include "Pwm.h"
extern pwm_t g_motor_pwm_ch1;   // PB6
extern pwm_t g_motor_pwm_ch2;   // PB7
extern pwm_t g_motor_pwm_ch3;
extern pwm_t g_motor_pwm_ch4;
```

PWM updates happen in the main loop (`template/source/main.c:123-129`):

```c
#if MOTOR_CONTROL_MODE == 1
    PWM_Update(&g_motor_pwm_ch1);
    PWM_Update(&g_motor_pwm_ch2);
    PWM_Update(&g_motor_pwm_ch3);
    PWM_Update(&g_motor_pwm_ch4);
#endif
```

**Control Logic:**
- Uses **duty cycle ramp** for soft-start and soft-stop (`Motor_RampForward()`, `Motor_RampReverse()`, `Motor_SetStopDuty()`).
- Direction changes go through a polarity switch mechanism (`s_bPolaritySwitchPending`).
- The motor speed is proportional to the PWM duty cycle (0–100%), allowing smooth acceleration and deceleration.

| Action | Method |
|---|---|
| Stop | `Motor_SetStopDuty()` — ramps duty to 0 |
| Forward | `Motor_RampForward(duty_percent)` — ramps to target duty |
| Reverse | `Motor_RampReverse(duty_percent)` — ramps to target duty |

---

## 3. Bus Voltage Divider Resistor

**File:** `Dev/dev_voltage.h:36-45`

Controls the high-side resistor value of the voltage divider used for bus voltage measurement. The low-side resistor is fixed at 10kΩ.

```c
#include "main.h"
#if BOARD_VERSION == 0
    #define VOLTAGE_DIVIDER_TOP_OHM        (110000UL)   // 110kΩ
#else
    #define VOLTAGE_DIVIDER_TOP_OHM        (150000UL)   // 150kΩ
#endif
#define VOLTAGE_DIVIDER_BOTTOM_OHM     (10000UL)        // 10kΩ (fixed)
#define VOLTAGE_DIVIDER_RATIO          ((VOLTAGE_DIVIDER_TOP_OHM + VOLTAGE_DIVIDER_BOTTOM_OHM) / VOLTAGE_DIVIDER_BOTTOM_OHM)
```

| Item | chu_chai (`0`) | HandB (`1`) |
|---|---|---|
| High-side resistor | **110kΩ** | **150kΩ** |
| Low-side resistor | 10kΩ | 10kΩ |
| Divider ratio | (110+10)/10 = **12** | (150+10)/10 = **16** |
| Compensation | +400mV | +400mV |

**Bus voltage calculation:**
```
BusVoltage(mV) = ADC_Reading(mV) × DividerRatio + Compensation(400mV)
```

> **Impact:** With the same actual bus voltage, the HandB board produces a lower ADC reading (1/16 vs 1/12). The different ratio ensures the firmware correctly scales the ADC value back to the actual bus voltage.

---

## 4. Current Sensor Type and Parameters

**File:** `Dev/dev_sensor.h:24-83` and `Dev/dev_sensor.c`

This is a two-level conditional compilation. First, `BOARD_VERSION` selects the sensor type enum; then `SENSOR_TYPE_DIFF_AMP_ENABLE` selects the corresponding calibration parameters.

### 4.1 Sensor Type Selection

**File:** `Dev/dev_sensor.h:24-33`

```c
#include "main.h"
#if BOARD_VERSION == 0
    #define SENSOR_TYPE_DIFF_AMP_ENABLE    0   // Hall sensor
#else
    #define SENSOR_TYPE_DIFF_AMP_ENABLE    1   // Differential amplifier
#endif
```

### 4.2 Sensor Calibration Parameters

**File:** `Dev/dev_sensor.h:74-83`

```c
#if SENSOR_TYPE_DIFF_AMP_ENABLE
    // HandB: Differential amplifier (LM358 IBUS circuit)
    #define SENSOR_RAW_ZERO_MV             (0)
    #define SENSOR_RAW_SENSITIVITY_MV_PER_A (100)
#else
    // chu_chai: Hall current sensor
    #define SENSOR_RAW_ZERO_MV             (1650)
    #define SENSOR_RAW_SENSITIVITY_MV_PER_A (264)
#endif
```

### 4.3 chu_chai (`0`) — Hall Current Sensor

| Parameter | Value | Description |
|---|---|---|
| Sensor type | Hall effect sensor | Bidirectional current sensing |
| Zero-current output | **1650mV** | Vcc/2 (mid-supply), supports ± measuring |
| Sensitivity | **264mV/A** | Output voltage change per ampere |
| Range | ±2.5A | (990mV to 2310mV within 0-3.3V ADC range) |

**Transfer function:**
```
Vout(mV) = 1650 + I(A) × 264
I(A)     = (Vout - 1650) / 264
```

### 4.4 HandB (`1`) — Differential Amplifier (LM358 IBUS)

| Parameter | Value | Description |
|---|---|---|
| Sensor type | Differential amplifier | LM358 op-amp with shunt resistor |
| Shunt resistor | 0.005Ω | Two 0.01Ω resistors in parallel |
| Op-amp gain | ×20 | 1kΩ input, 20kΩ feedback |
| Overall sensitivity | **100mV/A** | 0.005Ω × 20 = 0.1V/A |
| Zero-current output | **0mV** | Output near 0V at zero current (biased by diode network to ~2.5V from +5V_CAN) |
| Bias circuit | Diode + resistor to +5V_CAN | Provides DC offset so bidirectional current can be measured |

**Signal chain:**
```
I(A) → Shunt(0.005Ω) → Vsense = I × 0.005 (V)
      → Op-Amp(×20)   → Vsig  = I × 0.1   (V)
      → + Bias         → Vout  = Vbias + I × 0.1 (V)
```

**Transfer function (code uses theoretical zero=0mV):**
```
Vout(mV) = 0 + I(A) × 100
I(A)     = Vout / 100
```

> **Important:** The actual bias voltage (`Vbias`, approximately 2.5V) from the diode+resistor network connected to +5V_CAN cannot be precisely calculated from the schematic. The firmware uses **automatic zero-point calibration** (`Sensor_CalibrateZeroInternal()` in `dev_sensor.c`) which samples the ADC voltage at startup (assuming 0A motor current) and computes an offset. This offset is stored in `stcCalibration.s32ZeroOffsetMv` and applied in all subsequent current calculations. If the motor is drawing current at calibration time (first 5 seconds after init), the calibration will be incorrect.

### 4.5 Shared Zero-Point Calibration System

Both sensor types share the same calibration infrastructure in `dev_sensor.c`:

**Automatic calibration** (in `Sensor_Device_Update()`):
1. On first valid ADC reading (100mV < V < 3300mV), measures the actual voltage.
2. Calculates `ZeroOffset = V_measured - V_theory`.
3. Stores with magic number `0x5A5A5A5A` as validity marker.
4. Timeout: if no valid reading within 5000ms, sets `ZeroOffset = 0`.

**Manual calibration** (`Sensor_Device_CalibrateZero()`):
- Callable externally (e.g., via Modbus) to re-trigger zero-point calibration at any time.

**Current calculation formula** (in `Sensor_CalcCurrentInternal()`):
```c
I(mA) = (Vadc - SENSOR_VOUT_ZERO_MA_INT - ZeroOffset) × 1000 / SENSOR_SENSITIVITY_INT
```

---

## 5. RS485 Direction Control Pin

**File:** `Adp/rs485.c:21-32`

Controls which GPIO pin is used to toggle the RS485 transceiver between transmit and receive mode.

```c
#include "main.h"
#if BOARD_VERSION == 0
    #define RS485_DIR_PORT      (GPIO_PORT_B)
    #define RS485_DIR_PIN       (GPIO_PIN_14)   // PB14
#else
    #define RS485_DIR_PORT      (GPIO_PORT_A)
    #define RS485_DIR_PIN       (GPIO_PIN_03)   // PA3
#endif
```

| Item | chu_chai (`0`) | HandB (`1`) |
|---|---|---|
| DIR Port | GPIO_PORT_B | GPIO_PORT_A |
| DIR Pin | **PB14** | **PA3** |
| DIR Mode | RS485_DIR_MODE_1 (TX=HIGH, RX=LOW) | RS485_DIR_MODE_1 (TX=HIGH, RX=LOW) |

> **Impact:** The RS485 transceiver direction pin is on a different GPIO port/pin. The control logic (TX mode = set pin HIGH, RX mode = set pin LOW via `RS485_DIR_MODE_1`) is identical for both boards.

---

## Summary Table

| Dimension | chu_chai (`BOARD_VERSION=0`) | HandB (`BOARD_VERSION=1`) | File |
|---|---|---|---|
| **Bus voltage ADC pin** | PA04 / CH4 | PA06 / CH6 | `App_Motor_Project.h` |
| **Motor control** | GPIO (PB8/PB9) direct on/off | 4-ch PWM (PB6/PB7) with soft ramp | `dev_motor.h` / `dev_motor.c` / `main.c` |
| **Bus voltage divider** | 110kΩ + 10kΩ (ratio=12) | 150kΩ + 10kΩ (ratio=16) | `dev_voltage.h` |
| **Current sensor type** | Hall sensor (1650mV zero, 264mV/A) | Diff-amp LM358 (0mV zero, 100mV/A) | `dev_sensor.h` / `dev_sensor.c` |
| **RS485 DIR pin** | PB14 | PA3 | `rs485.c` |

## Files Involved

| # | File | What changes |
|---|---|---|
| 1 | `template/source/main.h:30` | `BOARD_VERSION` definition |
| 2 | `App/App_Motor_Project.h:97-107` | ADC voltage pin/channel |
| 3 | `Dev/dev_motor.h:32-38` | `MOTOR_CONTROL_MODE` macro |
| 4 | `Dev/dev_motor.c:14-19, 334-404, 1203, 1315` | Motor control implementation (GPIO vs PWM) |
| 5 | `template/source/main.c:123-129` | PWM update in main loop |
| 6 | `Dev/dev_voltage.h:37-43` | Voltage divider resistor |
| 7 | `Dev/dev_sensor.h:27-33` | `SENSOR_TYPE_DIFF_AMP_ENABLE` macro |
| 8 | `Dev/dev_sensor.h:74-83` | Sensor zero & sensitivity parameters |
| 9 | `Dev/dev_sensor.c` | Current calculation & calibration logic |
| 10 | `Adp/rs485.c:24-32` | RS485 direction control pin |
