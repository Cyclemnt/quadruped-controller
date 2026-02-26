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
    std::cout << "new cmd: " << cmd << std::endl;

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
    if (cmd == "stickbug") {
        pending_action.store(PA_STICKUG);
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
                std::cout << "STOP RUN" << std::endl;
                try { steve.stopRunning(); }
                catch (const std::exception &e) { std::cerr << "ERROR in stopRunning(): " << e.what() << std::endl; }
            }

            // If we are leaving STABILIZE, perform restore now
            if (cm == STABILIZE && nm != STABILIZE) {
                std::cout << "STOP STABILIZE" << std::endl;
                try { steve.rest(); }
                catch (const std::exception &e) { std::cerr << "ERROR in rest(): " << e.what() << std::endl; }
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
                    std::cout << "setting body height: " << v << std::endl;
                    // the robot expects negative heights here as before
                    steve.setBodyHeight(-v);
                    steve.rest(); // apply immediately
                } else if (act == PA_HI) {
                    std::cout << "executing hi emote" << std::endl;
                    steve.hi();
                } else if (act == PA_STICKUG) {
                    std::cout << "executing stickbug emote" << std::endl;
                    steve.stickBug();
                } else if (act == PA_EMERGENCY) {
                    std::cout << "executing emergency stop" << std::endl;
                    steve.tidy();
                }
            } catch (const std::exception &e) {
                std::cerr << "ERROR performing pending action: " << e.what() << std::endl;
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
                    std::cerr << "ERROR in run(): " << e.what() << std::endl;
                }
                break;
            }

            case TURN_ANGLE: {
                float angleDeg;
                float pitchLocal, rollLocal;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    angleDeg = last_turn_angle;
                }
                // Map: angleDeg already absolute in degrees; orient chassis yaw only (keep pitch/roll)
                try {
                    steve.turn(angleDeg);
                } catch (const std::exception &e) {
                    std::cerr << "ERROR in turn(): " << e.what() << std::endl;
                }
                break;
            }

            case STABILIZE: {
                try {
                    steve.level(); // blocking stabilizer call
                } catch (const std::exception &e) {
                    std::cerr << "ERROR in level(): " << e.what() << std::endl;
                }
                break;
            }

            case IDLE:
            default:
                // nothing to do
                break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
