#include "../include/server.hpp"
#include <iostream>

RobotServer::RobotServer() : steve(&driver, &imu, &stabilizer), current_mode(IDLE) {
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

    if (cmd.rfind("run_vector:", 0) == 0) {
        std::string data = cmd.substr(11);
        float x = 0, y = 0;
        sscanf(data.c_str(), "%f,%f", &x, &y);
        last_vector = {x, y};
        next_mode = RUN;
    }
    else if (cmd == "run_stop") {
        next_mode = IDLE;
    }

    else if (cmd == "turn_left_start") next_mode = TURN_LEFT;
    else if (cmd == "turn_left_stop") next_mode = IDLE;

    else if (cmd == "turn_right_start") next_mode = TURN_RIGHT;
    else if (cmd == "turn_right_stop") next_mode = IDLE;

    else if (cmd == "stabilize_start") next_mode = STABILIZE;
    else if (cmd == "stabilize_stop") next_mode = IDLE;

    // --- sliders ---
    else if (cmd.rfind("set_height:", 0) == 0) {
        int value = std::stoi(cmd.substr(11));
        std::cout << "Setting body height: " << value << std::endl;
        steve.setBodyHeight(-value);
        steve.rest(); // appliquer immédiatement la nouvelle hauteur
    }
    else if (cmd.rfind("set_step_angle:", 0) == 0) {
        int value = std::stoi(cmd.substr(15));
        std::cout << "Setting turning step angle: " << value << std::endl;
        steve.setTurningStepAngle(value);
    }
    else if (cmd.rfind("set_pitch:", 0) == 0) {
        int value = std::stoi(cmd.substr(10));
        std::cout << "Setting pitch: " << value << std::endl;
        steve.setPitch(value);
        steve.rest(); // appliquer immédiatement la nouvelle orientation
    }
    else if (cmd == "hi") {
        steve.hi();
    }
    else if (cmd == "emergency_stop") {
        std::cout << "Emergency stop triggered!" << std::endl;
        steve.tidy();
        //exit(0);
    }
}

void RobotServer::loop() {
    while (true) {
        RobotMode nm = next_mode.load();
        RobotMode cm = current_mode.load();

        if (nm != cm) {
            // Si on quitte un mode "run", on arrête la marche
            if (cm == RUN && nm == IDLE) {
                std::cout << "Arrêt du run -> stopRunning()" << std::endl;
                steve.stopRunning();
            }

            current_mode.store(nm);
        }

        // Exécution du mode courant
        switch (current_mode.load()) {
            case RUN: steve.run(last_vector.first, last_vector.second); break;
            case TURN_LEFT: steve.turn(true); break;
            case TURN_RIGHT: steve.turn(false); break;
            case STABILIZE: steve.level(); break;
            case IDLE: break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}