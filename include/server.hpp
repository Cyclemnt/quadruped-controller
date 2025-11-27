#ifndef ROBOT_SERVER_HPP
#define ROBOT_SERVER_HPP

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include "robot.hpp"
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>

using websocketpp::connection_hdl;
typedef websocketpp::server<websocketpp::config::asio> server;

using std::placeholders::_1;
using std::placeholders::_2;

enum RobotMode { IDLE, RUN, TURN_ANGLE, LOOK, STABILIZE };
enum PendingAction { PA_NONE = 0, PA_SET_HEIGHT, PA_SET_PITCH, PA_HI, PA_EMERGENCY, PA_RESTORE_LOOK };

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

    // Shared state protected by mtx
    std::mutex mtx;
    std::pair<float, float> last_vector{0.0f, 0.0f}; // run joystick x,y
    float last_turn_angle{0.0f};                     // degrees, + left, - right
    std::pair<float, float> last_look{0.0f, 0.0f};   // look joystick jx,jy in [-1,1]

    // pending actions (executed in loop when safe)
    std::atomic<int> pending_action;
    float pending_value{0.0f}; // for set_height / set_pitch

    // look restore helper
    bool look_restore_pending{false};

    std::thread loop_thread;
};

#endif // ROBOT_SERVER_HPP
