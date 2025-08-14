// socket_server.h
#ifndef SOCKET_SERVER_H
#define SOCKET_SERVER_H

#include <iostream>
#include <string>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <cstring>
#include "FileManager.h"
#include <ctime>
#include <unordered_map>
#include "BenewakeLidarManager.h"
#include "TanwayLidarManager.h"
#include "CentralCamManager.h"
#include "SideCamManager.h"
class BenewakeLidarManager;
class TanwayLidarManager;
class FileManager;

class SocketServer
{
public:
    SocketServer(int port);
    ~SocketServer();

    void start();

private:
    int server_fd, client_fd;
    int port;
    struct sockaddr_in server_addr, client_addr;
    void handle_client(int client_socket,  FileManager &fileManager,BenewakeLidarManager &benewakeLidarManager,TanwayLidarManager &tanwayLidarManager,
        CentralCamManager &central_cam_manager, SideCamManager &side_cam_manager);
    std::string dealBeneWakeLidar(BenewakeLidarManager &benewakeLidarManager,std::string save_path, bool isStart);
    std::string dealTanwayLidar(TanwayLidarManager &tanwayLidarManager,std::string save_path, bool isStart);
    std::string dealCentralCam(CentralCamManager &central_cam_manager, std::string save_path, bool isStart); //中间摄像头
    std::string dealSideCam(SideCamManager &side_cam_manager, std::string save_path, bool isStart); //侧方摄像头
    std::string dealAllLidar(BenewakeLidarManager &benewakeLidarManager,TanwayLidarManager &tanwayLidarManager,FileManager &fileManager, bool isStart);
    std::string dealAllCam(CentralCamManager &central_cam_manager, SideCamManager &side_cam_manager, FileManager &fileManager, bool isStart);
    std::string dealAllDev(BenewakeLidarManager &benewakeLidarManager,TanwayLidarManager &tanwayLidarManager,
    CentralCamManager &central_cam_manager, SideCamManager &side_cam_manager,FileManager &fileManager, bool isStart);
    std::string get_init_info( FileManager &fileManager,BenewakeLidarManager &benewakeLidarManager,TanwayLidarManager &tanwayLidarManager,
        CentralCamManager &central_cam_manager, SideCamManager &side_cam_manager);
    std::string process_command(const std::string &command,
                                FileManager &fileManager,BenewakeLidarManager &benewakeLidarManager,TanwayLidarManager &tanwayLidarManager,
                                CentralCamManager &central_cam_manager, SideCamManager &side_cam_manager);
};

#endif
