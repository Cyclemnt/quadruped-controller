#ifndef ROBOT_SERVER_HPP
#define ROBOT_SERVER_HPP

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include "robot.hpp"
#include <functional>

using websocketpp::connection_hdl;
typedef websocketpp::server<websocketpp::config::asio> server;

using std::placeholders::_1;
using std::placeholders::_2;

class RobotServer {
public:
    RobotServer();

    void run(uint16_t port);

private:
    void on_message(connection_hdl hdl, server::message_ptr msg);

    PCA9685 driver;
    BNO055 imu;
    Stabilizer stabilizer;
    Robot steve;

    server s;
};

#endif // ROBOT_SERVER_HPP