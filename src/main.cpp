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

    steve.sit(true);

    std::cout << "walking..." << std::endl;
    steve.sit(false);
    steve.walk();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    steve.rest();

    std::cout << "running frontwards (4 it)..." << std::endl;
    steve.sit(true);
    for (int i = 0; i < 4; i++)
        steve.run(true);
    steve.stopRunning();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::cout << "running backwards (4 it)..." << std::endl;
    for (int i = 0; i < 2; i++)
        steve.run(false);
    steve.stopRunning();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::cout << "turning left (4 it)..." << std::endl;
    for (int i = 0; i < 4; i++)
        steve.turn(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::cout << "turning right (4 it)..." << std::endl;
    for (int i = 0; i < 4; i++)
        steve.turn(false);

    std::cout << "keeping level..." << std::endl;
    for (long i = 0; i < 999999999999; i++)
        steve.level();

    std::this_thread::sleep_for(std::chrono::milliseconds(30000));

    driver.disableAllPWM();

    return 0;
}
