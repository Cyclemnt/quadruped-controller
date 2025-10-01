#include "../include/server.hpp"
#include <iostream>

RobotServer::RobotServer() : steve(&driver, &imu, &stabilizer), current_mode(IDLE) {
    s.init_asio();
    s.set_message_handler(std::bind(&RobotServer::on_message, this, _1, _2));

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

    if (cmd == "run_forward_start") current_mode = RUN_FWD;
    else if (cmd == "run_forward_stop") { steve.stopRunning(); current_mode = IDLE; }

    else if (cmd == "run_backward_start") current_mode = RUN_BWD;
    else if (cmd == "run_backward_stop") { steve.stopRunning(); current_mode = IDLE; }

    else if (cmd == "turn_left_start") current_mode = TURN_LEFT;
    else if (cmd == "turn_left_stop") current_mode = IDLE;

    else if (cmd == "turn_right_start") current_mode = TURN_RIGHT;
    else if (cmd == "turn_right_stop") current_mode = IDLE;

    else if (cmd == "stabilize_start") current_mode = STABILIZE;
    else if (cmd == "stabilize_stop") current_mode = IDLE;
}

void RobotServer::loop() {
    while (true) {
        switch (current_mode.load()) {
            case RUN_FWD: steve.run(true); break;
            case RUN_BWD: steve.run(false); break;
            case TURN_LEFT: steve.turn(true); break;
            case TURN_RIGHT: steve.turn(false); break;
            case STABILIZE: steve.level(); break;
            default: break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}