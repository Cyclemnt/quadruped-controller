#include "../include/pca9685.hpp"
#include "../include/robot.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    PCA9685 driver;
    Robot steve(&driver);
    
    steve.rest();

    std::this_thread::sleep_for(std::chrono::milliseconds(6000));

    steve.walk();
    steve.rest();
    std::this_thread::sleep_for(std::chrono::milliseconds(30000));

    driver.disableAllPWM();

    return 0;
    
    return 0;
}
