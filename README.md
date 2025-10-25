# Quadruped Robot Controller

## Overview

This project implements a full control system for a quadruped robot developed by a team of robotics engineering students.  
The system is designed to run on a **Raspberry Pi Zero 2 W**, controlling 12 servo motors via a **PCA9685 PWM driver** and obtaining chassis orientation from a **BNO055 IMU**.  
The robot can walk omnidirectionally, rotate, and adjust its body posture in real time through a **web-based control interface** served by the onboard software.

The C++ core handles servo coordination, inverse kinematics, and gait control, while a Python server hosts a control webpage (`controller.html`) that communicates with the robot via WebSockets.

<p align="center">
  <img src="demo.gif" alt="Demo" style="width: 50%;"/>
</p>

---

## Hardware Architecture

| Component | Description |
|------------|-------------|
| **Raspberry Pi Zero 2 W** | Main controller running the C++ code and web server |
| **PCA9685 (I²C)** | 16-channel PWM driver controlling the 12 servo motors (3 per leg) |
| **BNO055 (I²C)** | 9-axis IMU providing real-time orientation and stabilization feedback |
| **Servomotors (x12)** | Three per leg: Coxa (hip yaw), Femur (hip pitch), Tibia (knee pitch) |

---


### Main Classes
- **`Joint`** – Encapsulates one servo actuator, handling angle commands and limits.  
- **`Leg`** – Controls a leg with three joints; computes and applies servo angles.  
- **`Kinematics`** – Provides forward and inverse kinematics computations.  
- **`Robot`** – Coordinates all legs, gait sequencing, movement, and body posture.  
- **`Stabilizer`** – Uses IMU data to maintain body balance and pitch correction.  
- **`Server`** – Implements a WebSocket server to receive remote commands.  

---

## Gait Algorithm and Trajectory Interpolation

The **gait algorithm** defines the coordinated sequence of leg movements that enable omnidirectional walking and rotation.  
At each control cycle, legs are divided into two groups:
- **Lifted legs** – move toward their next support position,
- **Flat legs** – remain on the ground, supporting the body.

Each leg’s trajectory between its current and target position is interpolated over several steps:

- **Linear interpolation** is used for translational motion:  
  The `(x, y, z)` position evolves linearly between two points, with a smooth vertical offset (`z`) added using a hyperbolic cosine function for the lifting phase.  

- **Circular interpolation** is used for rotations:  
  Legs follow an arc path around the robot’s center to simulate turning in place.  

The inverse kinematics solver (`Kinematics::computeIK`) converts the desired leg tip coordinates into joint angles for the PCA9685 driver.

---

## Web-Based Control Interface

The onboard WebSocket server (implemented with **websocketpp**) allows real-time control from any device connected to the same network.

The `controller.html` page provides:
- Directional control (forward/backward/sideways movement)
- Rotation (left/right)
- Adjustable parameters:
  - Body height  
  - Step size  
  - Turning step angle  
  - Pitch adjustment  
- Emergency stop button  

Commands are sent as WebSocket messages (e.g., `run_vector:x,y`, `turn_left_start`, `set_height:val`), parsed by the server, and executed in the robot’s control loop.

---

## System Requirements & Dependencies

### Hardware
- Raspberry Pi Zero 2 W (or equivalent Pi model with I²C)
- PCA9685 servo driver
- BNO055 IMU
- 12× servo motors (tested with MG996R)

### Software
| Component | Version / Note |
|------------|----------------|
| **Raspberry Pi OS (Bullseye or later)** | Recommended |
| **CMake** | ≥ 3.10 |
| **g++** | C++17 or newer |
| **websocketpp** | Header-only library |
| **Boost.Asio** | Used by websocketpp for networking |
| **i2c-dev** | Provides I²C access on Linux |
| **pigpio** or **wiringPi** | For low-level GPIO/I²C control |

---

## Build Instructions

```bash
# Clone the repository
git clone https://github.com/Cyclemnt/quadruped-controller.git
cd quadruped-controller

# Create build directory
mkdir build && cd build

# Configure and compile
cmake ..
make

# Start the server and run the main executable
cd ../webapp python3 -m http.server 8080 & cd ../build ./main
```

The server will start and listen on the configured port.
Once running, open the http://\<raspberrypi-ip\>:8080/controller.html page in a web browser connected to the same network.

## Future Work

- Implementation of dynamic gait adjustment based on IMU feedback
- Integration of force sensors for adaptive terrain response

## Authors
This project was developed as part of a Robotics Engineering Master's program at Polytech Dijon by a student team.
- Controller author: Clément Lamouller
- Trot gait research: Luan Parizot
