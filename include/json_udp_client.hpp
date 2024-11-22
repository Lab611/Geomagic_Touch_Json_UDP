#ifndef JSON_UDP_CLIENT_HPP
#define JSON_UDP_CLIENT_HPP

#include <iostream>
#include <winsock2.h>
#include <string>
#include <thread>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class JsonUdpClient {
public:
    JsonUdpClient(const std::string& server_ip, int port);
    ~JsonUdpClient();
    void send_request(const json& request);

private:
    SOCKET sockfd_;
    int port_;
    struct sockaddr_in server_addr_;
    WSADATA wsaData_;
};

JsonUdpClient::JsonUdpClient(const std::string& server_ip, int port) : port_(port) {
    // 初始化 Winsock
    if (WSAStartup(MAKEWORD(2, 2), &wsaData_) != 0) {
        std::cerr << "Winsock initialization failed" << std::endl;
        exit(1);
    }

    // 创建 UDP 套接字
    sockfd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd_ == INVALID_SOCKET) {
        std::cerr << "Socket creation failed" << std::endl;
        WSACleanup();
        exit(1);
    }

    // 设置服务器地址
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port_);
    server_addr_.sin_addr.s_addr = inet_addr(server_ip.c_str());  // 设置服务器 IP

    // UDP 套接字不需要 bind，一般只有在需要接收数据时才需要 bind
}

JsonUdpClient::~JsonUdpClient() {
    closesocket(sockfd_);
    WSACleanup();
}

void JsonUdpClient::send_request(const json& request) {
    // 将 JSON 对象转换为字符串
    std::string request_str = request.dump();

    // 通过 sendto 发送数据
    int send_result = sendto(sockfd_, request_str.c_str(), request_str.length(), 0,
                             (struct sockaddr*)&server_addr_, sizeof(server_addr_));

    if (send_result == SOCKET_ERROR) {
        std::cerr << "Send failed with error code: " << WSAGetLastError() << std::endl;
    } else {
        std::cout << "Request sent successfully!" << std::endl;
    }
}

#endif // JSON_UDP_CLIENT_HPP
