#include <chrono>
#include <iostream>
#include <ostream>
#include <thread>
#include "json_udp_client.hpp"
#include "haptic_device.hpp"


using namespace std;

const string IP_ = "192.168.2.38";
constexpr int port_ = 8080;

int main() {
    HapticDevice device;
    JsonUdpClient haptic_client(IP_, port_);

    if (!device.initialize()) {
        cerr << "Failed to initialize the haptic device. Exiting..." << endl;
        return -1;
    }

    cout << "Press ESC to exit." << endl;
    while (device.isRunningStatus()) {
        if (_kbhit() && _getch() == 27) {
            break;
        }
        // 手柄的位置信息在 msg 中
        auto msg = device.update();

        if (msg.empty()) {
            this_thread::sleep_for(chrono::milliseconds(10));
            continue;
        }
        if (msg.contains("cmd") && msg["cmd"] == CMD_UNKNOWN) {
            this_thread::sleep_for(chrono::milliseconds(10));
            continue;
        }

        // 这样子会发送手柄当前的绝对位置 这样做也行 但是不安全
        // 建议是新建一个变量 记录上一次的手柄位置
        // 然后传输手柄位置的变化量，欧拉角就直接传输
        // haptic_client.send_request(msg);

        this_thread::sleep_for(chrono::milliseconds(20));
    }

    cout << "Exiting program." << endl;
    return 0;
}