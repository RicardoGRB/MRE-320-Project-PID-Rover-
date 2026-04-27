# MRE-320-Project-PID-Rover-
# Autonomous Wall-Following Rover

## Overview

This project involves the design and implementation of an **autonomous mobile robot** capable of following a wall at a fixed distance using **feedback control**. The system uses ultrasonic sensors to measure distance and a **PID (Proportional–Integral–Derivative) controller** to continuously adjust the robot’s motion.

The project emphasizes the practical application of:

- Control systems (PID control)
- Sensor integration
- Embedded system design
- Mechanical design (custom chassis)

The rover was designed, built, and programmed to operate **fully autonomously**, maintaining a consistent distance from a wall while navigating alongside it.

---

## Project Goals

The main objectives of this project were:

- Design and build a **mobile robotic platform**
- Implement a **closed-loop control system** using a PID controller
- Maintain a **constant distance from a wall**
- Use **ultrasonic sensors** for real-time distance measurement
- Integrate hardware and software into a **fully autonomous system**
- Design and fabricate a **custom chassis** for the rover

---

## System Architecture

The rover system is composed of three main subsystems:

### 1. Sensing System

- Two **ultrasonic sensors** are mounted on the rover
- Sensors measure the **distance to the wall**
- Multiple sensors improve **stability and accuracy** of distance estimation

---

### 2. Control System

- A **PID controller** is implemented in the microcontroller
- The controller computes the error between:

  ```
  Error = Desired Distance − Measured Distance
  ```

- Based on this error, the controller adjusts motor behavior using:

  - **Proportional term (P):** reacts to current error  
  - **Integral term (I):** corrects accumulated error  
  - **Derivative term (D):** anticipates future error  

- The output of the PID controller is used to **adjust motor speeds** for steering correction

---

### 3. Actuation System

- The rover uses **two driven wheels** for propulsion
- A **passive omniwheel** is mounted at the front of the chassis
- The omniwheel allows **free lateral movement**, reducing friction during turns
- Steering is achieved through **differential drive**:
  - The robot turns by varying the speeds of the left and right motors
- This configuration enables **smooth directional corrections** while maintaining forward motion

---

### 4. Mechanical System

- A **custom chassis** was designed and built for the rover
- The chassis supports:
  - Sensor placement
  - Motor mounting
  - Electronics integration
  - Omniwheel placement for stability and maneuverability
- The design ensures:
  - Stability
  - Proper alignment with the wall
  - Reliable sensor readings

---

## Implementation Overview

### Sensor Integration

- Two ultrasonic sensors continuously measure the distance to the wall
- Sensor readings are processed in real time
- Data is used as input for the control algorithm

---

### Control Algorithm

1. Read distance from ultrasonic sensors  
2. Compute error relative to desired distance  
3. Apply PID control equation  
4. Generate correction signal  
5. Adjust motor speeds accordingly  

This loop runs continuously to ensure real-time response.

---

### Motor Control

- Motor speeds are adjusted based on PID output
- Differential drive is used to steer the rover
- The front omniwheel allows smooth turning without resistance
- Small speed adjustments result in precise wall-following behavior

---

### Chassis Design

- The rover chassis was **custom designed and built**
- Key considerations included:
  - Sensor positioning for accurate readings
  - Weight distribution
  - Structural stability
  - Integration of the omniwheel for improved maneuverability

---

## System Behavior

During operation, the rover:

1. Moves forward along a wall  
2. Continuously measures distance using ultrasonic sensors  
3. Computes error relative to the desired distance  
4. Adjusts its trajectory using PID control  
5. Maintains a stable and consistent path along the wall  

---

## Repository Structure

```
Autonomous-Wall-Following-Rover
│
├── README.md
│
├── code
│   └── rover_control
│
├── hardware
│   ├── chassis_design
│   └── wiring_diagrams
│
├── data
│   └── test_logs
│
└── Conclusion
    └── project_report.pdf
```

---

## Project Context

This project was completed as part of a **sensors adm actuators assignment**, focusing on the integration of:

- Sensors  
- Control algorithms  
- Embedded programming  
- Mechanical design  

The project demonstrates a complete **mechatronic system**, combining hardware and software to achieve autonomous behavior.

---


---
