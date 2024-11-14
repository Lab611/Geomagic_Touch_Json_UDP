//
// Created by Lab611-Y7KP on 24-11-14.
//

#include "json_tcp_client.hpp"

JsonTcpClient::JsonTcpClient(const std::string& server_ip, int port)
    : server_ip_(server_ip), port_(port) {
    // 初始化 Winsock
    if (WSAStartup(MAKEWORD(2, 2), &wsaData_) != 0) {
        std::cerr << "Winsock initialization failed" << std::endl;
        exit(1);
    }

    // 创建 TCP 套接字
    sockfd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd_ == INVALID_SOCKET) {
        std::cerr << "Socket creation failed" << std::endl;
        WSACleanup();
        exit(1);
    }

    // 设置服务器地址
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port_);
    if (InetPton(AF_INET, server_ip_.c_str(), &server_addr_.sin_addr) <= 0) {
        std::cerr << "Invalid server address" << std::endl;
        WSACleanup();
        exit(1);
    }

    // 连接到服务器
    if (connect(sockfd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) == SOCKET_ERROR) {
        std::cerr << "Connection failed" << std::endl;
        closesocket(sockfd_);
        WSACleanup();
        exit(1);
    }
}

JsonTcpClient::~JsonTcpClient() {
    closesocket(sockfd_);
    WSACleanup();
}

void JsonTcpClient::send_request(const json& request_json) {
    // 将请求 JSON 转换为字符串
    std::string request_str = request_json.dump();

    // 发送请求到服务器
    send(sockfd_, request_str.c_str(), request_str.length(), 0);

    // 接收服务器响应
    char buffer[1024];
    int recv_size = recv(sockfd_, buffer, sizeof(buffer), 0);
    if (recv_size > 0) {
        std::string response_str(buffer, recv_size);
        try {
            json response_json = json::parse(response_str);
            std::cout << "Response from server: " << response_json.dump() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error parsing server response: " << e.what() << std::endl;
        }
    } else {
        std::cerr << "Failed to receive response from server" << std::endl;
    }
}
