#include "../include/pca9685.hpp"
#include "../include/bno055.hpp"
#include "../include/robot.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    PCA9685 driver;
    BNO055 imu;
    Robot steve(&driver, &imu);
    
    steve.rest();

    for (int i = 0; i < 100000; i++) {
        auto orientation = steve.getOrientation();
        std::cout << "Heading: " << orientation[0]
                << " Roll: " << orientation[1]
                << " Pitch: " << orientation[2] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(30000));

    driver.disableAllPWM();

    return 0;
}
