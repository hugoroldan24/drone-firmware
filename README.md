# ✈️ Drone Controller Embedded Software ✈️  

This project is focused on developing **embedded software in Embedded C** for a complete drone system: the **flight controller**, the **RF transmitter**, and the **RF receiver**.  

The project is still under development, I will continue fixing bugs, refining the code, and adding new features.

---

## 📑 Table of Contents

## 🌟 Motivation

## 💡 Learning

## 📁 File Structure

## 🧩 System Architecture

## 🛠 Software

## 📦 Hardware

## 🔌 Wiring Diagram

## 📚 References

## 🧠 Flight Controller (STM32 + FreeRTOS)

The flight controller is the central brain of the drone, responsible for reading sensors, computing control signals, and driving the motors. It is implemented on an **STM32F4** microcontroller and runs **FreeRTOS**, providing a multitasking environment where fast and predictable responses to external events are critical. This allows the system to update motor signals every few milliseconds and react immediately to important events such as low battery or loss of RF communication.

### Key Features

- **Multitasking with FreeRTOS** → tasks are synchronized efficiently to ensure timely sensor reading, control computation, and telemetry updates.  
- **Failsafe and low-power modes** → when the system detects low battery or lost communication, motors are safely controlled and some peripherals enter low-power states.  
- **Sensor interfacing**  
  - **I2C** with interrupt-driven state machine for the MPU6050 IMU.  
  - **SPI** communication with the MS5611 barometric pressure sensor.  
- **Telemetry system** → continuously computes relative altitude and battery voltage and sends them via UART.  
- **Efficient UART with DMA** → reception (double-buffered) for RF packets and transmission for telemetry, minimizing CPU load.  
- **PID-based flight control** → stabilizes the drone in roll, pitch, and yaw axes and allows joystick-based control from the RF transmitter.  
- **Motor mixing** → converts PID outputs and throttle inputs into individual motor commands.  
- **Safety algorithms** → automatically perform hover and safe landing when battery is low or RF signal is lost.

---
