# NXP-Cup-Autonomous-Car-From-A-to-Z 🏎️🤖

Welcome to the ultimate, ground-up guide to building and programming a high-performance autonomous race car for the NXP Cup!

Whether you are a complete beginner holding a microcontroller for the first time, or an intermediate student looking to push your car to its absolute limits, this repository is built for you. We don't just give you the final code; we explain **why** and **how** everything works, from the physics of the hardware to bare-metal C++ programming.

> **💡 The Philosophy of this Course:**
> We believe in learning by understanding the roots. Instead of running into frustrating hardware conflicts or burning out servos and fixing them later, this guide is designed proactively. We will build the system step-by-step, ensuring you understand exactly what every register, pin, and algorithm does before moving forward.

---

## 🛠️ The Hardware Stack

Before we write any code, we need to know our machine. This project utilizes the official NXP Cup hardware ecosystem:

* [cite_start]**The Brains:** NXP Kinetis K64F Microcontroller (MK64FN1M0VLL12) [cite: 13, 1189] [cite_start]featuring a 120 MHz ARM Cortex-M4 core[cite: 1190].
* [cite_start]**The Shield:** RDDRONE-CUPK64 expansion board used to interface with motors and sensors[cite: 5, 1194].
* [cite_start]**The Muscle:** Standard NXP Cup chassis, rear DC motors, and an internal **MC33887 H-Bridge** driver to control speed and direction[cite: 6, 242].
* **The Steering:** Recommended standard steering servo for precise directional control.
* **The Power:** 7.4V LiPo Battery providing high current for motors and regulated power for logic.
* [cite_start]**The Eyes:** **Pixy2 Line Scan Camera** communicating via SPI to detect track vectors[cite: 64, 175, 269].
* [cite_start]**The Environment Sensors:** Ultrasonic sensors for obstacle detection, plus the onboard IMU (Accelerometer and Magnetometer on PTB2/PTB3, and Gyroscope) for inertia and orientation[cite: 458, 461, 462, 463].

---

## 📚 Course Roadmap

This repository is structured as a chronological course. We highly recommend following the modules in order:

### **Module 1: Hardware Deep-Dive & The Physical Build**
Get to know the chassis, the motors, and the silicon. [cite_start]We break down the difference between a physical pin, an electrical signal, and an internal MCU interface (GPIO, SPI, I2C, PWM)[cite: 125, 128, 137]. Includes schematics and wiring photos!

### **Module 2: The Pin Muxing Architecture (Getting it Right)**
Learn how to configure the K64F's internal multiplexer. [cite_start]We'll dive into bare-metal coding to assign `ALT` modes and fix notorious hardware conflicts—specifically preventing the default FRDM-K64F UART and Ethernet settings from hijacking our motor control pins[cite: 286, 287, 1118, 1123].

### **Module 3: Low-Level Control (ADC & PWM)**
Time to make things move. We cover:
* [cite_start]**Analog-to-Digital Conversion (ADC):** Reading potentiometers to test input systems[cite: 1184, 1195].
* [cite_start]**FlexTimer (FTM):** Configuring the timer modules (FTM0 & FTM2) to generate precise PWM signals (20kHz for motors, 50-100Hz for servos)[cite: 7, 31, 354, 355].

### **Module 4: Machine Vision & Memory Management (Pixy2)**
Interfacing with the Pixy2 camera via SPI. [cite_start]We tackle C++ memory management, explaining why we use dynamic `std::vector` containers instead of fixed arrays to prevent crashes [cite: 89, 90][cite_start], and how to filter out visual noise (like sun glare) using Euclidean norm calculations[cite: 511, 514].

### **Module 5: Control Theory (Building the Custom PID)**
From basic line-following to advanced stability. [cite_start]We build a custom PID controller and solve the dreaded "Waddling" (oscillation) problem using an advanced mathematical concept called **Integral Decay**, which naturally bleeds off error accumulation in straightaways[cite: 626, 819, 820].

### **Module 6: F1-Style Racing Algorithms**
Taking the training wheels off. We implement predictive algorithms:
* [cite_start]**Lookahead:** Dissociating the "Head" and "Tail" of a vector for better reaction times[cite: 597, 598, 599].
* [cite_start]**Asymmetry:** Calculating the geometry of the track to predict curves before entering them[cite: 607, 726].
* [cite_start]**Apex Targeting:** Dynamically shifting the track's center point with an artificial offset to cut corners like a real race car[cite: 641, 642, 659].

### **Module 7: Hardware Security & Robustness**
How to make your car bulletproof. [cite_start]We implement a software **"Servo Saver"** [cite: 501, 827] incorporating:
* [cite_start]**Deadband:** Ignoring micro-jitters to prevent overheating[cite: 524, 527].
* [cite_start]**Low-Pass Filter:** Soft-starting servo movements to protect gears[cite: 543, 840].
* [cite_start]**Watchdog:** An emergency timeout to cut motor power if the car loses sight of the track[cite: 671, 841].

---

## 🚀 How to Use This Repository

Each module contains detailed `README` explanations, schematic images to trace connections, and the heavily commented C++ code required for that step.

Grab your chassis, charge your LiPo, and let's start building!
