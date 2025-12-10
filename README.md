# raPID_car

This project utilizes a **Raspberry Pi Pico 2** to act as a "man-in-the-middle" controller for an RC car. It intercepts IBUS signals from the receiver, processes them alongside **BNO055 IMU** data, and outputs corrected servo signals and lighting controls.

## 🏎️ Features

* **IBUS Communication:** Reads channels from the Receiver (Rx) and sends telemetry data back to the Transmitter (Tx).
* **Heading Hold (Hdg):** Uses the BNO055 absolute orientation sensor to maintain the car's heading when a specific switch is flipped on the Tx.
* **Dual Servo Modes (Pin 18):**
    * **Mode A (Mirror):** Direct pass-through of steering input.
    * **Mode B (Mirror + Heading):** Mixes steering input with Gyro correction for drift stabilization.
* **Telemetry Feedback:** Sends the current BNO055 heading value back to the Tx screen via IBUS sensors.
* **Smart Lighting:**
    * **Side Lights:** Automatically on/off based on steering input thresholds.
    * **Front Strip:** Toggled via a specific value sent from the Tx (Aux channel).

## 🛠️ Hardware Requirements

* **Microcontroller:** Raspberry Pi Pico 2 (RP2350)
* **IMU:** BNO055 (Connects via I2C)
* **RC System:** FlySky or compatible system supporting IBUS (Tx + Rx)
* **Servo:** Connected to GPIO 18
* **Lighting:** LEDs for turn signals and front strip



## 🔌 Pin Configuration

| Component | Pico 2 Pin | Function |
| :--- | :--- | :--- |
| **BNO055** | GPIO 26 (SDA) / GPIO 27 (SCL) | I2C Communication |
| **RC Receiver** | GPIO 5 (UART RX) | IBUS Input |
| **RC Receiver** | GPIO 17 (UART TX) / GPIO 16 (UART TX) connected with 1N4148 diode toghether | IBUS Telemetry Output |
| **Steering Servo** | GPIO 18 | PWM Output |
| **Front Light Strip** | GPIO 21 | PWM/Logic High |
| **Right Turn Signal** | GPIO 20 | Logic High (Blink) |
| **Left Turn Signal** | GPIO 19 | Logic High (Blink) |

## 🕹️ Control Logic

### 1. Heading Hold & Telemetry
The Pico reads the **Euler Heading** from the BNO055.
* **Telemetry:** This heading data is injected into the IBUS stream and sent back to the Transmitter to be displayed on the screen.
* **Activation:** When **Switch D** (assigned to channel 4) is active, the PID loop engages to correct the steering servo to maintain the locked heading.

### 2. Servo Modes (Pin 18)
Controlled by **Switch B** on the Transmitter:
* **State 0 (Mirror Only):** `Servo_Out = Stick_Input`
    * The servo strictly follows your hand movements.
* **State 1 (Mirror + Heading):** `Servo_Out = Stick_Input + Hdg_Correction`
    * The servo moves based on your hand, but the Gyro adds/subtracts value to fight external rotation (drifting assistance).

### 3. Lighting System
* **Turn Signals:** The code monitors the raw Steering Channel. If the value exceeds a defined threshold (e.g., > 1600 or < 1400), the corresponding side LED turn on.
* **Front Strip:** Monitors an Aux channel (Channel 3). There are 4 cases:
  * `Rx_Value < 1000`: the front strip turns ON;
  * `1000 < Rx_Value < 1500`: the front strip makes the kitt light effect (from the series Knight Rider);
  * `1500 < Rx_Value < 2000`: the front strip makes the police light effect (2 blue 2 red then 2 red 2 blue);
  * `2000 < Rx_Value`: the front strip turns OFF.

## 🚀 Installation

1. Clone the repo.
2. Copy `.py` files to the Pico 2 using thonny IDE.
