#include "../include/server.hpp"
#include <iostream>

RobotServer::RobotServer() : steve(&driver, &imu, &stabilizer), current_mode(IDLE) {
    s.init_asio();
    s.set_message_handler(std::bind(&RobotServer::on_message, this, _1, _2));

    // Position de repos au démarrage
    steve.rest();

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

    if (cmd == "run_forward_start") next_mode = RUN_FWD;
    else if (cmd == "run_forward_stop") next_mode = IDLE;

    else if (cmd == "run_backward_start") next_mode = RUN_BWD;
    else if (cmd == "run_backward_stop") next_mode = IDLE;

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
    else if (cmd.rfind("set_step_size:", 0) == 0) {
        int value = std::stoi(cmd.substr(14));
        std::cout << "Setting running step size: " << value << std::endl;
        steve.setRunningStepSize(value);
    }
    else if (cmd.rfind("set_step_angle:", 0) == 0) {
        int value = std::stoi(cmd.substr(15));
        std::cout << "Setting turning step angle: " << value << std::endl;
        steve.setTurningStepAngle(value);
    }
}


void RobotServer::loop() {
    while (true) {
        RobotMode nm = next_mode.load();   // lire la valeur
        RobotMode cm = current_mode.load();

        // Vérifie si un changement de mode est demandé
        if (nm != cm) {
            // Cas particulier : sortie de run -> stopRunning()
            if ((cm == RUN_FWD || cm == RUN_BWD) && nm == IDLE) {
                std::cout << "Arrêt du run -> stopRunning()" << std::endl;
                steve.stopRunning();
            }

            current_mode.store(nm);   // écrire la nouvelle valeur
        }

        // Exécute le mode courant
        switch (current_mode.load()) {
            case RUN_FWD: steve.run(true); break;
            case RUN_BWD: steve.run(false); break;
            case TURN_LEFT: steve.turn(true); break;
            case TURN_RIGHT: steve.turn(false); break;
            case STABILIZE: steve.level(); break;
            case IDLE: /* ne fait rien */ break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}