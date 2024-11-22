#include <chrono>
#include <iostream>
#include <ostream>
#include <thread>
#include "json_udp_client.hpp"
#include "haptic_device.hpp"
#include "json_utils.hpp"
#include "math_utils.hpp"


using namespace std;


int main() {
    auto ip_config_json = get_json_from_file("../../para.json");

    string IP_ = ip_config_json["ip"];
    int port_ = ip_config_json["port"];
    bool moveAbs = ip_config_json["moveAbs"];
    // Touch to NE30_Base
    Eigen::Matrix4d rotation_left_offset = get_trans_matrix_from_json(ip_config_json["rotation_left_offset"]);
    // NE30_End to Tool
    Eigen::Matrix4d rotation_right_offset = get_trans_matrix_from_json(ip_config_json["rotation_right_offset"]);

    HapticDevice device;
    JsonUdpClient haptic_client(IP_, port_);
    int cnt = 0;


    cout << rotation_left_offset << endl;
    cout << rotation_right_offset << endl;

    if (!device.initialize()) {
        cerr << "Failed to initialize the haptic device. Exiting..." << endl;
        return -1;
    }
    // 绝对位置控制
    device.setMoveAbs(moveAbs);

    cout << "Press q to quit." << endl;
    while (device.isRunningStatus()) {
        if (_kbhit()) {
            // 按下 q 退出程序
            auto c_ = _getch();
            if (c_ == 'q' || c_ == 'Q') { break; }
        }
        // 手柄的位置信息在 msg 中
        auto msg = device.update();
        // 消息为空 跳过
        if (msg.empty()) {
            this_thread::sleep_for(chrono::milliseconds(10));
            continue;
        }
        // 命令位为空 跳过
        if (msg.contains("cmd") && msg["cmd"] == CMD_UNKNOWN) {
            cnt++;
            if (cnt > 100) {
                cnt = 0;
                // 让机械臂记录当前位置为起点
                haptic_client.send_request(build_json_from_cmd(DEV_TOUCH, CMD_SET_ABS));
            }
            this_thread::sleep_for(chrono::milliseconds(20));
            continue;
        }

        if (msg.contains("cmd") && (msg["cmd"] == CMD_MOVE || msg["cmd"] == CMD_MOVE_ABS)) {
            // nlohmann::json{{"device", device}, {"cmd", cmd}, {"data", pos_and_rot}};
            // std::cout << msg.dump(4) << std::endl;
            if (msg.contains("data_trans")) {
                // x, y, z, yaw(z), pitch(y), roll(x)
                auto trans = get_trans_matrix_from_json(msg);
                auto vec = get_vec_from_trans_matrix(rotation_left_offset * trans * rotation_right_offset);
                haptic_client.send_request(build_json_from_pos_and_rot(msg["device"], msg["cmd"], vec));
            }
        }

        // 命令位为 reset
        if (msg.contains("cmd") && msg["cmd"] == CMD_RESET) {
            haptic_client.send_request(msg);
        }

        if (msg.contains("cmd") && msg["cmd"] == CMD_SET_ABS) {
            // 连续发送3条防止丢包
            for (int i = 0; i < 3; i++) {
                haptic_client.send_request(msg);
                this_thread::sleep_for(chrono::milliseconds(20));
            }
        }


        this_thread::sleep_for(chrono::milliseconds(20));
    }

    cout << "Exiting program." << endl;
    return 0;
}
