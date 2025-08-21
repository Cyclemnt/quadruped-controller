# Quadruped Robot Controller

A C++ controller for a 4-legged quadruped robot with 3 degrees of freedom per leg. This project includes inverse kinematics, smooth interpolation for movement, and will soon support real-time control using a PCA9685 PWM driver.

## Project Structure

```
.
├── CMakeLists.txt
├── include
│   ├── joint.hpp
│   ├── kinematics.hpp
│   ├── leg.hpp
│   ├── pca9685.hpp
│   └── robot.hpp
├── README.md
└── src
    ├── joint.cpp
    ├── kinematics.cpp
    ├── leg.cpp
    ├── main.cpp
    ├── pca9685.cpp
    └── robot.cpp
```

## Features

* Individual control of all four legs (FL, FR, RR, RL).
* Smooth motion via linear interpolation and lift trajectory for leg clearance.
* Inverse and forward kinematics implementations.
* Mechanical offset compensation for servo alignment.
* Core movement primitives:

  * `resetLegs()` — Set all legs to a default neutral position.
  * `rest()` — Set robot to a predefined resting position.
  * `sit(bool)` — Transition between low and high standing height.
  * `walk()` — Perform a basic walking gait.
  * `turn()` — Perform a basic turning gait.

## Dependencies

* **C++17** or later.
* **PCA9685 driver** to control servo motors via I2C.
* **Raspberry Pi** (or compatible) to run the software and interface with the PCA9685 board.
* **BNO055 imu** to measure orientation.
* **[CMake](https://cmake.org/)** to configure and build the project.

## Building the Project

Compile using `CMake`:

```bash
cd QuadrupedController
mkdir build && cd build
cmake .. && make
./main
```  

## Example Usage

```cpp
#include "../include/pca9685.hpp"
#include "../include/robot.hpp"

int main() {
    PCA9685 driver;
    Robot robot(&driver);

    robot.rest();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    robot.sit(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot.sit(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Main loop
    while (true) {
        robot.walk();
    }

    driver.disableAllPWM();
    return 0;
}
```

## Technical Highlights

* **Forward Kinematics (FK):** Calculates 3D foot position (x, y, z) from joint angles.
* **Inverse Kinematics (IK):** Calculates joint angles from 3D foot position (x, y, z).
* **Smooth Trajectories:** Custom interpolation curve for lifting motion based on hyperbolic cosine.
* **Mechanical Calibration:** Angle offsets are handled inside the Leg class to compensate for non-zero default servo positions.

## Future Work

* Implement trot gait cycle.
* Add directional turning (turn left/right).
* Include IMU.
* Implement real-time control.

## Contributions

**Author:** Clément Lamouller  