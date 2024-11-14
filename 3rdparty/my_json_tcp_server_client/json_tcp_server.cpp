#include "json_tcp_server.hpp"

JsonTcpServer::JsonTcpServer(int port) : port_(port) {
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
    server_addr_.sin_addr.s_addr = INADDR_ANY;
    server_addr_.sin_port = htons(port_);

    // 绑定端口
    if (bind(sockfd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) == SOCKET_ERROR) {
        std::cerr << "Bind failed" << std::endl;
        closesocket(sockfd_);
        WSACleanup();
        exit(1);
    }

    // 开始监听
    if (listen(sockfd_, SOMAXCONN) == SOCKET_ERROR) {
        std::cerr << "Listen failed" << std::endl;
        closesocket(sockfd_);
        WSACleanup();
        exit(1);
    }
}

JsonTcpServer::~JsonTcpServer() {
    closesocket(sockfd_);
    WSACleanup();
}

void JsonTcpServer::start_listening() {
    std::cout << "Server listening on port " << port_ << "..." << std::endl;

    while (true) {
        // 接受连接
        SOCKET client_sock = accept(sockfd_, nullptr, nullptr);
        if (client_sock == INVALID_SOCKET) {
            std::cerr << "Accept failed" << std::endl;
            continue;
        }

        // 处理客户端请求
        std::thread client_thread(&JsonTcpServer::handle_client, this, client_sock);
        client_thread.detach();  // 启动一个新线程来处理每个客户端
    }
}

void JsonTcpServer::handle_client(SOCKET client_sock) {
    // 接收和处理客户端请求
    receive_data(client_sock);
}

void JsonTcpServer::receive_data(SOCKET client_sock) {
    char buffer[1024];
    int recv_size;

    while ((recv_size = recv(client_sock, buffer, sizeof(buffer), 0)) > 0) {
        // 接收到的数据处理
        std::string received_data(buffer, recv_size);
        try {
            recv_json_ = json::parse(received_data);  // 解析 JSON 数据
            std::cout << "Received JSON: " << recv_json_.dump() << std::endl;

//            // 返回响应给客户端
//            json response = {{"status", "success"}, {"message", "Received JSON successfully"}};
//            std::string response_str = response.dump();
//            send(client_sock, response_str.c_str(), response_str.length(), 0);
        } catch (const std::exception& e) {
            std::cerr << "Error parsing JSON: " << e.what() << std::endl;
            // 返回错误响应
            json error_response = {{"status", "error"}, {"message", "Invalid JSON"}};
            std::string error_response_str = error_response.dump();
            send(client_sock, error_response_str.c_str(), error_response_str.length(), 0);
        }
    }

    // 关闭客户端连接
    closesocket(client_sock);
}
