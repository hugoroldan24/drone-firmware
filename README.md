# ✈️ Drone Controller Embedded Software ✈️  

This project is focused on developing **embedded software in Embedded C** for a complete drone system: the **flight controller**, the **RF transmitter**, and the **RF receiver**.

Both the transmitter and the receiver are implemented on an ATmega328P MCU, while the flight controller is implemented on an STM32F4.

The project is still under development, I will continue fixing bugs, refining the code, and adding new features.

---

## 📑 Table of Contents

## 🌟 Motivation

## 💡 Learning

## 📁 File Structure

## 🧩 System Architecture

This project implements a distributed real-time embedded system composed of three primary modules: a transmitter unit, a receiver unit, and a flight controller. Each module has a well-defined functional responsibility within the overall closed-loop control architecture, enabling reliable wireless command transmission, real-time flight stabilization, and bidirectional telemetry exchange.

At a high level, the system operates as follows:

**1. User input is captured by the transmitter.**
The ATmega328 reads the joystick positions through its ADC peripherals and encodes the control inputs into a structured data packet.

**2. Control commands are transmitted wirelessly.**
The encoded packet is sent via the nRF24L01 transceiver over the 2.4 GHz ISM band to the drone’s RF receiver.

**3. The RF receiver acts as a communication bridge.**
The receiver module, also based on the ATmega328 and nRF24L01, receives the RF payload and forwards the decoded control data to the flight controller via a UART interface. It does not perform control computations, but serves as a transparent communication layer between the wireless link and the flight controller.

**4. The flight controller executes the real-time control loop.**
The STM32F411CEU6, running FreeRTOS, performs:
   - Sensor acquisition from the IMU
   - State estimation (attitude computation)
   - PID control computation
   - Motor mixing and transformation
   - PWM signal generation to drive the ESCs

**5. Telemetry data is acquired and prepared.**
In parallel with the control loop, the flight controller:
   - Measures battery voltage via ADC
   - Reads barometric pressure
   - Converts pressure measurements into altitude estimates
   - Packages telemetry data for transmission

**6. Telemetry is returned through the RF link.**
The telemetry data is sent back to the transmitter using the nRF24L01 ACK payload mechanism, enabling bidirectional communication within the same RF transaction.

**7. Telemetry is forwarded to the user interface.**
The transmitter receives the telemetry data and relays it through a Bluetooth module to a mobile device, where it is displayed to the user in real time.

### 📡 Transmitter & Receiver (ATmega38p)

The transmitter and receiver are built around the ATmega328 microcontroller and communicate wirelessly using the nRF24L01 RF transceiver operating in the 2.4 GHz ISM band. Both modules execute a lightweight cooperative scheduler to manage input acquisition, RF communication, and data handling tasks deterministically. Together, they form the remote control subsystem responsible for capturing user inputs, transmitting flight commands to the flight controller (FC), and returning telemetry data to the user.

The following diagram illustrates the operational architecture of the transmitter–receiver subsystem.



### 🧠 Flight Controller (STM32 + FreeRTOS)

The flight controller is the central brain of the drone, responsible for reading sensors, computing control signals, and driving the motors. It is implemented on an **STM32F4** microcontroller and runs **FreeRTOS**, providing a multitasking environment where fast and predictable responses to external events are critical. This allows the system to update motor signals every few milliseconds and react immediately to important events such as low battery or loss of RF communication.

#### Key Features

- **Multitasking with FreeRTOS** → tasks are synchronized efficiently to ensure timely sensor reading, control computation, and telemetry updates.  
- **Failsafe and low-power modes** → when the system detects low battery or lost communication, motors are safely controlled and some peripherals enter low-power states.  
- **Sensor interfacing**  
  - **I2C** with interrupt-driven state machine for the MPU6050 IMU.  
  - **SPI** communication with the MS5611 barometric pressure sensor.  
- **Telemetry system** → continuously computes relative altitude and battery voltage and sends them via UART to the receiver MCU.  
- **Efficient UART with DMA** → reception (double-buffered) for RF packets and transmission for telemetry, minimizing CPU load.  
- **PID-based flight control** → stabilizes the drone in roll, pitch, and yaw axes and allows joystick-based control from the RF transmitter.  
- **Motor mixing** → converts PID outputs and throttle inputs into individual motor commands.  
- **Safety algorithms** → automatically perform hover and safe landing when battery is low or RF signal is lost.

The ESCs are controlled using the OneShot125 protocol, generating pulses immediately after each PID cycle for minimal latency and precise motor response.

#### Choice of OneShot over Standard PWM

In this project, OneShot125 is used instead of traditional servo-style PWM. The main reason is that OneShot drastically reduces latency between the PID calculation and motor signal update, generating each pulse immediately after the control loop rather than waiting for a fixed PWM period. This improves drone stability and precision, avoiding timing jitter or phase lag that can occur with continuous fixed-period PWM, especially at high-frequency control loops.


## 🛠 Software

## 📦 Hardware

## 🔌 Wiring Diagram

## 📚 References

---
