// socket_client.cpp
#include "SocketClient.h"
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>

SocketClient::SocketClient(const std::string &ip, int port)
    : server_ip(ip), server_port(port) {}

SocketClient::~SocketClient()
{
    close(sock);
}

bool SocketClient::connect_to_server()
{
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        perror("Socket creation failed");
        return false;
    }

    sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(server_port);

    if (inet_pton(AF_INET, server_ip.c_str(), &serv_addr.sin_addr) <= 0)
    {
        perror("Invalid address");
        return false;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        perror("Connection Failed");
        return false;
    }

    std::cout << "Connected to server." << std::endl;
    receive_init(); // 读取初始信息
    return true;
}

void SocketClient::send_message(const std::string &msg)
{
    send(sock, msg.c_str(), msg.size(), 0);
}
void SocketClient::receive_init()
{
    char buffer[1024];
    memset(buffer, 0, sizeof(buffer));
    ssize_t n = recv(sock, buffer, sizeof(buffer) - 1, 0);
    if (n > 0)
    {
        std::cout << "Server init : " << buffer;
    }
}
void SocketClient::receive_response()
{
    char buffer[1024];
    memset(buffer, 0, sizeof(buffer));
    ssize_t n = recv(sock, buffer, sizeof(buffer) - 1, 0);
    if (n > 0)
    {
        std::cout << "Server: " << buffer;
    }
}

void SocketClient::close_connection()
{
    close(sock);
}
