# NXP-Cup-Autonomous-Car-From-A-to-Z 🏎️🤖

Welcome to the ultimate, ground-up guide to building and programming a high-performance autonomous race car for the NXP Cup!

Whether you are a complete beginner holding a microcontroller for the first time, or an intermediate student looking to push your car to its absolute limits, this repository is built for you. We don't just give you the final code; we explain **why** and **how** everything works, from the physics of the hardware to bare-metal C++ programming.

> **💡 The Philosophy of this Course:**
> We believe in learning by understanding the roots. Instead of running into frustrating hardware conflicts or burning out servos and fixing them later, this guide is designed proactively. We will build the system step-by-step, ensuring you understand exactly what every register, pin, and algorithm does before moving forward.

---

## 🛠️ The Hardware Stack

Before we write any code, we need to know our machine. This project utilizes the official NXP Cup hardware ecosystem:

* **The Brains:** NXP Kinetis K64F Microcontroller (MK64FN1M0VLL12) featuring a 120 MHz ARM Cortex-M4 core.
* **The Shield:** RDDRONE-CUPK64 expansion board used to interface with motors and sensors.
* **The Muscle:** Standard NXP Cup chassis, rear DC motors, and an internal **MC33887 H-Bridge** driver to control speed and direction.
* **The Steering:** Recommended standard steering servo for precise directional control.
* **The Power:** 7.4V LiPo Battery providing high current for motors and regulated power for logic.
* **The Eyes:** **Pixy2 Line Scan Camera** communicating via SPI to detect track vectors.
* **The Environment Sensors:** Ultrasonic sensors for obstacle detection, plus the onboard IMU (Accelerometer, Magnetometer, Gyroscope) for inertia and orientation.

---

## 📚 Course Roadmap

This repository is structured as a chronological course. We highly recommend following the modules in order:

### **Module 1: Hardware Deep-Dive & The Physical Build**
Get to know the chassis, the motors, and the silicon. We break down the difference between a physical pin, an electrical signal, and an internal MCU interface (GPIO, SPI, I2C, PWM). Includes schematics and wiring photos!

### **Module 2: The Pin Muxing Architecture (Getting it Right)**
Learn how to configure the K64F's internal multiplexer. We'll dive into bare-metal coding to assign `ALT` modes and fix notorious hardware conflicts—specifically preventing the default FRDM-K64F UART and Ethernet settings from hijacking our motor control pins.

### **Module 3: Low-Level Control (ADC & PWM)**
Time to make things move. We cover:
* **Analog-to-Digital Conversion (ADC):** Reading potentiometers to test input systems.
* **FlexTimer (FTM):** Configuring the timer modules (FTM0 & FTM2) to generate precise PWM signals (20kHz for motors, 50-100Hz for servos).

### **Module 4: Machine Vision & Memory Management (Pixy2)**
Interfacing with the Pixy2 camera via SPI. We tackle C++ memory management, explaining why we use dynamic `std::vector` containers instead of fixed arrays to prevent crashes, and how to filter out visual noise (like sun glare) using Euclidean norm calculations.

### **Module 5: Control Theory (Building the Custom PID)**
From basic line-following to advanced stability. We build a custom PID controller and solve the dreaded "Waddling" (oscillation) problem using an advanced mathematical concept called **Integral Decay**, which naturally bleeds off error accumulation in straightaways.

### **Module 6: F1-Style Racing Algorithms**
Taking the training wheels off. We implement predictive algorithms:
* **Lookahead:** Dissociating the "Head" and "Tail" of a vector for better reaction times.
* **Asymmetry:** Calculating the geometry of the track to predict curves before entering them.
* **Apex Targeting:** Dynamically shifting the track's center point with an artificial offset to cut corners like a real race car.

### **Module 7: Hardware Security & Robustness**
How to make your car bulletproof. We implement a software **"Servo Saver"** incorporating:
* **Deadband:** Ignoring micro-jitters to prevent overheating.
* **Low-Pass Filter:** Soft-starting servo movements to protect gears.
* **Watchdog:** An emergency timeout to cut motor power if the car loses sight of the track.

---

## 🚀 How to Use This Repository

Each module contains detailed `README` explanations, schematic images to trace connections, and the heavily commented C++ code required for that step.

Grab your chassis, charge your LiPo, and let's start building!
