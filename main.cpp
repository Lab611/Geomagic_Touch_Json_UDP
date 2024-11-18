#include <chrono>
#include <iostream>
#include <ostream>
#include <thread>
#include "json_udp_client.hpp"
#include "haptic_device.hpp"


using namespace std;

const string IP_ = "192.168.2.38";
constexpr int port_ = 8080;
constexpr bool moveAbs = true;

int main() {
    HapticDevice device;
    JsonUdpClient haptic_client(IP_, port_);

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
            this_thread::sleep_for(chrono::milliseconds(10));
            continue;
        }

        if (msg.contains("cmd") && (msg["cmd"] == CMD_MOVE || msg["cmd"] == CMD_MOVE_ABS)) {
            // nlohmann::json{{"device", device}, {"cmd", cmd}, {"data", pos_and_rot}};
            // std::cout << msg.dump(4) << std::endl;
            haptic_client.send_request(msg);
        }

        // 命令位为 reset
        if (msg.contains("cmd") && msg["cmd"] == CMD_RESET) {
            haptic_client.send_request(msg);
        }

        // 这样子会发送手柄当前的绝对位置 这样做也行 但是不安全
        // 建议是新建一个变量 记录上一次的手柄位置
        // 然后传输手柄位置的变化量，欧拉角就直接传输
        // 需要注意坐标变化
        // NE 30 需要发送 mm 为单位 rad 为单位
        // auto msg_send = build_json_from_pos_and_rot(DEV_TOUCH, CMD_MOVE, 位移量 + 欧拉角的长度为 6 的 vector);
        // haptic_client.send_request(msg);

        this_thread::sleep_for(chrono::milliseconds(10));
    }

    cout << "Exiting program." << endl;
    return 0;
}
