#include "../include/server.hpp"

RobotServer::RobotServer() : steve(&driver, &imu, &stabilizer) {
    s.init_asio();
    s.set_message_handler(std::bind(&RobotServer::on_message, this, _1, _2));
}

void RobotServer::run(uint16_t port) {
    s.listen(port);
    s.start_accept();
    s.run();
}

void RobotServer::on_message(connection_hdl hdl, server::message_ptr msg) {
    std::string cmd = msg->get_payload();

    if (cmd == "walk") steve.walk();
    else if (cmd == "run_forward") steve.run(true);
    else if (cmd == "run_backward") steve.run(false);
    else if (cmd == "turn_left") steve.turn(true);
    else if (cmd == "turn_right") steve.turn(false);
    else if (cmd == "sit") steve.sit(true);
    else if (cmd == "rest") steve.rest();
    else if (cmd == "stop") steve.stopRunning();
}
