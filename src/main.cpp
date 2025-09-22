#include "../include/pca9685.hpp"
#include "../include/bno055.hpp"
#include "../include/robot.hpp"
#include "../include/stabilizer.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    PCA9685 driver;
    BNO055 imu;
    Stabilizer stabilizer;
    Robot steve(&driver, &imu, &stabilizer);
    
    steve.rest();

    std::this_thread::sleep_for(std::chrono::milliseconds(8000));
    for (int i = 0; i < 100000; i++) {
        steve.level();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(30000));

    driver.disableAllPWM();

    return 0;
}
