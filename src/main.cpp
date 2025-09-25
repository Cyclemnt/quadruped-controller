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
    try {
        PCA9685 driver;
        BNO055 imu;
        Stabilizer stabilizer;
        Robot steve(&driver, &imu, &stabilizer);

        std::thread netThread(network_thread_func, 12345);

        // Pose initiale (optionnelle)
        steve.rest();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        std::cout << "[main] Entering control loop (directional flags only)\n";

        while (true) {
            // Marche avant
            if (flag_walk_forward.load()) {
                try {
                    steve.run();
                } catch (const std::exception& e) {
                    std::cerr << "walkForward() failed: " << e.what() << std::endl;
                }
            }

            // // Marche arrière
            // if (flag_walk_backwards.load()) {
            //     try {
            //         steve.walkBackwards();
            //     } catch (const std::exception& e) {
            //         std::cerr << "walkBackwards() failed: " << e.what() << std::endl;
            //     }
            // }

            // // Marche à droite
            // if (flag_walk_right.load()) {
            //     try {
            //         steve.walkRight();
            //     } catch (const std::exception& e) {
            //         std::cerr << "walkRight() failed: " << e.what() << std::endl;
            //     }
            // }

            // Marche à gauche
            if (flag_turn_left.load()) {
                try {
                    steve.turn();
                } catch (const std::exception& e) {
                    std::cerr << "walkLeft() failed: " << e.what() << std::endl;
                }
            }

            // Stabilisation continue (optionnel)
            try {
                steve.level();
            } catch (...) {
                // Ignore ou log
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Note : on n’atteint jamais ce code, boucle infinie
        if (netThread.joinable()) netThread.join();
    } catch (const std::exception& ex) {
        std::cerr << "Fatal exception: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
