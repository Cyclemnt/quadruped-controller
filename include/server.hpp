#ifndef ROBOT_SERVER_HPP
#define ROBOT_SERVER_HPP

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include "robot.hpp"
#include <functional>
#include <thread>
#include <atomic>

using websocketpp::connection_hdl;
typedef websocketpp::server<websocketpp::config::asio> server;

using std::placeholders::_1;
using std::placeholders::_2;

enum RobotMode { IDLE, RUN_FWD, RUN_BWD, TURN_LEFT, TURN_RIGHT, STABILIZE };

class RobotServer {
public:
    RobotServer();
    void run(uint16_t port);

private:
    void on_message(connection_hdl hdl, server::message_ptr msg);
    void loop();

    PCA9685 driver;
    BNO055 imu;
    Stabilizer stabilizer;
    Robot steve;

    server s;
    std::atomic<RobotMode> current_mode;
    std::atomic<RobotMode> next_mode;
    std::thread loop_thread;
};

#endif // ROBOT_SERVER_HPP