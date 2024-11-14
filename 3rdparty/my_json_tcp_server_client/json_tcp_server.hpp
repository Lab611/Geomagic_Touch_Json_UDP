#ifndef JSON_TCP_SERVER_HPP
#define JSON_TCP_SERVER_HPP

#include <iostream>
#include <string>
#include <thread>
#include <winsock2.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class JsonTcpServer {
public:
    JsonTcpServer(int port);
    ~JsonTcpServer();
    void start_listening();
    nlohmann::json get_json(){return recv_json_;};

private:
    void handle_client(SOCKET client_sock);
    void receive_data(SOCKET client_sock);

    SOCKET sockfd_;
    int port_;
    struct sockaddr_in server_addr_;
    WSADATA wsaData_;
    nlohmann::json recv_json_;
};

#endif // JSON_TCP_SERVER_HPP
