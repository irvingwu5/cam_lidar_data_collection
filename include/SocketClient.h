// socket_client.h
#ifndef SOCKET_CLIENT_H
#define SOCKET_CLIENT_H

#include <string>

class SocketClient
{
public:
    SocketClient(const std::string &ip, int port);
    ~SocketClient();

    bool connect_to_server();
    void send_message(const std::string &msg);
    void receive_response();
    void receive_init();
    void close_connection();

private:
    int sock;
    std::string server_ip;
    int server_port;
};

#endif
