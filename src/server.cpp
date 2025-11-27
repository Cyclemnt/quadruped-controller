#include "../include/server.hpp"
#include <iostream>
#include <chrono>

RobotServer::RobotServer() : steve(&driver, &imu, &stabilizer),
                             current_mode(IDLE),
                             next_mode(IDLE),
                             pending_action(PA_NONE)
{
    s.init_asio();
    s.set_message_handler(std::bind(&RobotServer::on_message, this, _1, _2));

    // Position de repos au démarrage
    steve.startup();

    // Lancer la boucle continue dans un thread
    loop_thread = std::thread(&RobotServer::loop, this);
}

void RobotServer::run(uint16_t port) {
    s.listen(port);
    s.start_accept();
    s.run();
}

void RobotServer::on_message(connection_hdl hdl, server::message_ptr msg) {
    std::string cmd = msg->get_payload();
    std::cout << "Commande reçue: " << cmd << std::endl;

    // run joystick: "run_vector:x,y"  (x,y in [-1,1])
    if (cmd.rfind("run_vector:", 0) == 0) {
        std::string data = cmd.substr(11);
        float x=0,y=0;
        sscanf(data.c_str(), "%f,%f", &x, &y);
        {
            std::lock_guard<std::mutex> lock(mtx);
            last_vector = {x,y};
        }
        next_mode.store(RUN);
        return;
    }
    if (cmd == "run_stop") {
        next_mode.store(IDLE);
        return;
    }

    // look joystick: "look_vector:x,y" and "look_stop"
    if (cmd.rfind("look_vector:", 0) == 0) {
        std::string data = cmd.substr(12);
        float jx=0,jy=0;
        sscanf(data.c_str(), "%f,%f", &jx, &jy);
        {
            std::lock_guard<std::mutex> lock(mtx);
            // on clamp just in case
            if (jx > 1.0f) jx = 1.0f;
            if (jx < -1.0f) jx = -1.0f;
            if (jy > 1.0f) jy = 1.0f;
            if (jy < -1.0f) jy = -1.0f;
            last_look = {jx,jy};
        }
        // Before first entering LOOK, memorize current chassis pose to be able to restore later
        // We only set prev_* the first time we transition into LOOK (handled in loop when nm != cm)
        next_mode.store(LOOK);
        return;
    }
    if (cmd == "look_stop") {
        // request restore to previous pose once we leave LOOK
        {
            std::lock_guard<std::mutex> lock(mtx);
            look_restore_pending = true;
        }
        next_mode.store(IDLE);
        return;
    }

    // turn joystick: "turn_angle:deg" and "turn_stop"
    if (cmd.rfind("turn_angle:", 0) == 0) {
        std::string data = cmd.substr(11);
        float deg = 0;
        sscanf(data.c_str(), "%f", &deg);
        {
            std::lock_guard<std::mutex> lock(mtx);
            last_turn_angle = deg;
        }
        // if deg is near zero you may want to go to IDLE, but better keep TURN_ANGLE so small adjustments apply
        next_mode.store(TURN_ANGLE);
        return;
    }
    if (cmd == "turn_stop") {
        next_mode.store(IDLE);
        return;
    }

    // stabilize button "="
    if (cmd == "stabilize_start") {
        next_mode.store(STABILIZE);
        return;
    }
    if (cmd == "stabilize_stop") {
        next_mode.store(IDLE);
        return;
    }

    // --- sliders ---
    if (cmd.rfind("set_height:", 0) == 0) {
        float value = std::stof(cmd.substr(11)); // 120..220 from UI
        // schedule a blocking height change (will call steve.setBodyHeight and rest() from loop)
        pending_action.store(PA_SET_HEIGHT);
        pending_value = value;
        // ensure we are/return to IDLE so the change can be applied safely
        next_mode.store(IDLE);
        return;
    }
    if (cmd.rfind("set_pitch:", 0) == 0) {
        float value = std::stof(cmd.substr(10)); // -10..10
        pending_action.store(PA_SET_PITCH);
        pending_value = value;
        next_mode.store(IDLE);
        return;
    }

    // emergency / hi
    if (cmd == "emergency_stop") {
        pending_action.store(PA_EMERGENCY);
        next_mode.store(IDLE);
        return;
    }
    if (cmd == "hi") {
        pending_action.store(PA_HI);
        next_mode.store(IDLE);
        return;
    }
}

void RobotServer::loop() {
    using namespace std::chrono_literals;
    while (true) {
        RobotMode nm = next_mode.load();
        RobotMode cm = current_mode.load();

        // Handle mode transitions (stop things when leaving a mode, handle look restore scheduling)
        if (nm != cm) {
            // If we are leaving RUN, always call stopRunning() (unless staying in RUN)
            if (cm == RUN && nm != RUN) {
                std::cout << "Arrêt du run -> stopRunning()" << std::endl;
                try { steve.stopRunning(); }
                catch (const std::exception &e) { std::cerr << "Error stopRunning: " << e.what() << std::endl; }
            }

            // If we are entering LOOK for the first time, save current chassis estimated pose so we can restore later
            if (nm == LOOK && cm != LOOK) {
                // store prev chassis pose for restoring later
                std::lock_guard<std::mutex> lock(mtx);
                prev_chassis_yaw = chassisYaw;
                prev_chassis_pitch = chassisPitch;
                prev_chassis_roll = chassisRoll;
                look_restore_pending = false; // ensure clear
            }

            // If we are leaving LOOK and restore was requested, perform restore now (blocking)
            if (cm == LOOK && nm != LOOK) {
                bool doRestore = false;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    doRestore = look_restore_pending;
                    look_restore_pending = false;
                }
                if (doRestore) {
                    std::cout << "Restoring chassis pose after LOOK" << std::endl;
                    // Use orientChassisTo to restore smoothly
                    try {
                        steve.orientChassisTo(prev_chassis_yaw, prev_chassis_pitch, prev_chassis_roll,  /*targetX*/0.0f, /*targetY*/0.0f, std::numeric_limits<float>::quiet_NaN(), /*steps*/20);
                    } catch (const std::exception &e) {
                        std::cerr << "Error during look restore: " << e.what() << std::endl;
                    }
                    // update our estimate
                    chassisYaw = prev_chassis_yaw;
                    chassisPitch = prev_chassis_pitch;
                    chassisRoll = prev_chassis_roll;
                }
            }

            current_mode.store(nm);
        }

        // If IDLE and there's a pending blocking action, execute it here (ensures exclusivity)
        if (current_mode.load() == IDLE && pending_action.load() != PA_NONE) {
            int act = pending_action.load();
            // Execute and clear action under try/catch
            try {
                if (act == PA_SET_HEIGHT) {
                    float v;
                    {
                        std::lock_guard<std::mutex> lock(mtx);
                        v = pending_value;
                    }
                    std::cout << "Applying setHeight: " << v << std::endl;
                    // the robot expects negative heights here as before
                    steve.setBodyHeight(-v);
                    steve.rest(); // apply immediately
                } else if (act == PA_SET_PITCH) {
                    float v;
                    {
                        std::lock_guard<std::mutex> lock(mtx);
                        v = pending_value;
                    }
                    std::cout << "Applying setPitch: " << v << std::endl;
                    steve.setPitch(v);
                    steve.rest();
                } else if (act == PA_HI) {
                    std::cout << "Executing hi emote" << std::endl;
                    steve.hi();
                } else if (act == PA_EMERGENCY) {
                    std::cout << "Executing emergency stop (tidy)" << std::endl;
                    steve.tidy();
                } else if (act == PA_RESTORE_LOOK) {
                    // handled above in transition logic; kept for completeness
                }
            } catch (const std::exception &e) {
                std::cerr << "Error performing pending action: " << e.what() << std::endl;
            }
            pending_action.store(PA_NONE);
        }

        // Execute current mode
        RobotMode execMode = current_mode.load();
        switch (execMode) {
            case RUN: {
                std::pair<float,float> v;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    v = last_vector;
                }
                try {
                    steve.run(v.first, v.second);
                } catch (const std::exception &e) {
                    std::cerr << "Error in run(): " << e.what() << std::endl;
                }
                break;
            }

            case TURN_ANGLE: {
                float angleDeg;
                float pitchLocal, rollLocal;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    angleDeg = last_turn_angle;
                    // read our current estimate of pitch/roll
                    pitchLocal = chassisPitch;
                    rollLocal = chassisRoll;
                }
                // Map: angleDeg already absolute in degrees; orient chassis yaw only (keep pitch/roll)
                try {
                    steve.turn(angleDeg);
                } catch (const std::exception &e) {
                    std::cerr << "Error in TURN_ANGLE orient: " << e.what() << std::endl;
                }
                break;
            }

            case LOOK: {
                float jx, jy;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    jx = last_look.first;
                    jy = last_look.second;
                }
                // Map joystick [-1,1] to yaw/pitch degrees
                const float maxYaw = 45.0f;   // deg
                const float maxPitch = 12.0f; // deg
                float targetYaw = jx * maxYaw;
                float targetPitch = jy * maxPitch;
                try {
                    // use small steps to remain responsive
                    steve.orientChassisTo(targetYaw, targetPitch, 0.0f, 0.0f, 0.0f, std::numeric_limits<float>::quiet_NaN(), 3);
                    // update estimate
                    chassisYaw = targetYaw;
                    chassisPitch = targetPitch;
                    chassisRoll = 0.0f;
                } catch (const std::exception &e) {
                    std::cerr << "Error in LOOK orient: " << e.what() << std::endl;
                }
                break;
            }

            case STABILIZE: {
                try {
                    steve.level(); // blocking stabilizer call
                } catch (const std::exception &e) {
                    std::cerr << "Error in level(): " << e.what() << std::endl;
                }
                // after stabilizing, return to IDLE (stabilize is single-shot)
                next_mode.store(IDLE);
                break;
            }

            case IDLE:
            default:
                // nothing to do
                break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
