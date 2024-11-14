#ifndef JSON_TCP_CLIENT_HPP
#define JSON_TCP_CLIENT_HPP

#include <iostream>
#include <string>
#include <winsock2.h>
#include <nlohmann/json.hpp>
#include <Ws2tcpip.h>

using json = nlohmann::json;

class JsonTcpClient {
public:
    JsonTcpClient(const std::string& server_ip, int port);
    ~JsonTcpClient();
    void send_request(const json& request_json);

private:
    SOCKET sockfd_;
    std::string server_ip_;
    int port_;
    struct sockaddr_in server_addr_;
    WSADATA wsaData_;
};

#endif //JSON_TCP_CLIENT_HPP
