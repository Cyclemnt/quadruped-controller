#include "../include/pca9685.hpp"
#include "../include/bno055.hpp"
#include "../include/robot.hpp"
#include "../include/stabilizer.hpp"
#include "../include/control_flags.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <algorithm>
#include "../include/network.hpp"

// Flags directionnels définis ici
std::atomic<bool> flag_walk_forward(false);
std::atomic<bool> flag_walk_backwards(false);
std::atomic<bool> flag_walk_right(false);
std::atomic<bool> flag_turn_left(false);
std::atomic<bool> stopRequested(false);

int main() {
    PCA9685 driver;
    BNO055 imu;
    Stabilizer stabilizer;
    Robot steve(&driver, &imu, &stabilizer);

    steve.rest();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Lancer le serveur réseau dans un thread
    std::thread netThread(network_thread_func, 12345);

    std::cout << "Robot prêt, attente des commandes réseau..." << std::endl;

    while (true) {
        if (flag_walk_forward.load()) {
            steve.walk();
        } else if (flag_walk_backwards.load()) {
            steve.run(false);
        } else if (flag_walk_right.load()) {
            steve.turn(false);
        } else if (flag_turn_left.load()) {
            steve.turn(true);
        } else {
            steve.level(); // stabilisation si aucune commande
        }

        if (stopRequested.load()) {
            steve.stopRunning();
            stopRequested.store(false);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    driver.disableAllPWM();
    netThread.join();

    return 0;
}
