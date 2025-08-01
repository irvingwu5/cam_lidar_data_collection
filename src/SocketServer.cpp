// socket_server.cpp
#include "SocketServer.h"

SocketServer::SocketServer(int port) : port(port)
{
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0)
    {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, 1) < 0)
    {
        perror("Listen failed");
        exit(EXIT_FAILURE);
    }

    std::cout << "SocketServer listening on port " << port << std::endl;
}

SocketServer::~SocketServer()
{
    close(server_fd);
}
std::string SocketServer::get_init_info( FileManager &fileManager,BenewakeLidarManager &benewakeLidarManager,TanwayLidarManager &tanwayLidarManager,
    CentralCamManager &central_cam_manager, SideCamManager &side_cam_manager)
{
    std::string link_status = "OK";
    std::string is_has_UP = (fileManager.is_usb_inserted() ? "True" : "False");
    std::string up_path = (fileManager.is_usb_inserted() ? fileManager.get_usb_session_folder() : "");
    bool has_lidar_64 = tanwayLidarManager.hasLidar();
    bool has_lidar_248 = benewakeLidarManager.hasLidar();
    bool has_central_cam = central_cam_manager.hasCentralCamera();
    bool has_side_cam = side_cam_manager.hasSideCamera();
    std::string root_path = fileManager.getRootPath();
    std::vector<std::string> current_files = fileManager.getFileChildPaths();

    // 拼接 current_file 数组
    std::string current_file_str = "[";
    for (size_t i = 0; i < current_files.size(); ++i)
    {
        current_file_str += "\"" + current_files[i] + "\"";
        if (i != current_files.size() - 1)
            current_file_str += ", ";
    }
    current_file_str += "]";

    // 构造完整字符串
    std::string init_info = "{link_status:" + link_status +
                            ",is_has_UP:" + is_has_UP +
                            ",UP_path:" + up_path +
                            ",64_line_ladar:" + (has_lidar_64 ? "True" : "False") +
                            ",248_line_ladar:" + (has_lidar_248 ? "True" : "False") +
                            ",central_cam:" + (has_central_cam ? "True" : "False") +
                            ",side_cam:" + (has_side_cam ? "True" : "False") +
                            ",root_path:" + root_path +
                            ",current_file:" + current_file_str +
                            "}";
    return init_info;                        
}

void SocketServer::start()
{
    socklen_t len = sizeof(client_addr);
    client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &len);
    if (client_fd < 0)
    {
        perror("Accept failed");
        return;
    }

    std::cout << "\nClient connected!\n" << std::endl;
    FileManager fileManager = FileManager();
    BenewakeLidarManager benewakeLidarManager(client_fd, fileManager);
    benewakeLidarManager.initialize();
    TanwayLidarManager tanwayLidarManager(client_fd, fileManager);
    tanwayLidarManager.initialize();

    CentralCamManager central_cam_manager("/dev/video0", 1920, 1080, fileManager); //1240,370
    central_cam_manager.init();
    SideCamManager side_cam_manager("/dev/video2", 1920, 1080, fileManager); //1920，1080
    side_cam_manager.init();
    std::string init_info = get_init_info(fileManager, benewakeLidarManager,tanwayLidarManager, central_cam_manager, side_cam_manager);
    send(client_fd, init_info.c_str(), init_info.size(), 0);
    handle_client(client_fd, fileManager, benewakeLidarManager,tanwayLidarManager, central_cam_manager, side_cam_manager);
    close(client_fd);
}

void SocketServer::handle_client(int client_socket, FileManager &fileManager,BenewakeLidarManager &benewakeLidarManager,TanwayLidarManager &tanwayLidarManager,
    CentralCamManager &central_cam_manager, SideCamManager &side_cam_manager)
{
    char buffer[1024];
    while (true)
    {
        memset(buffer, 0, sizeof(buffer));
        ssize_t n = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
        if (n <= 0)
        {
            std::cout << "\nClient disconnected.\n" << std::endl;
            break;
        }

        std::string received(buffer);
        std::string response = process_command(received, fileManager, benewakeLidarManager,tanwayLidarManager, central_cam_manager, side_cam_manager);
        send(client_socket, response.c_str(), response.size(), 0);
    }
}

std::string SocketServer::dealTanwayLidar(TanwayLidarManager &tanwayLidarManager,  std::string save_path, bool isStart)
{
    std::string status = "1", error = "",info="";

    if (tanwayLidarManager.hasLidar())
    {
        if (isStart)
        {
            std::thread task_thread([&tanwayLidarManager]()
                                    { tanwayLidarManager.start(); });
            task_thread.detach(); // 后台运行
            info ="开始采集";
        }
        else
        {
            info = "结束采集";
            tanwayLidarManager.stop();
        }
    }
    else
    {
        status = "0";
        error = "Tanway Lidar 不存在";
    }

    std::string return_info = "{status: " + status +
                              ", path: " + save_path +
                              ", log: " + info +
                              ", error: " + "\"" + error + "\"" +
                              "}";

    return return_info;
}


std::string SocketServer::dealBeneWakeLidar(BenewakeLidarManager &benewakeLidarManager, std::string save_path, bool isStart)
{
    std::string status = "1", error = "",info="";

    if (benewakeLidarManager.hasLidar())
    {
        if (isStart)
        {
            std::thread task_thread([&benewakeLidarManager]()
                                    { benewakeLidarManager.start(); });
            task_thread.detach(); // 后台运行
            info ="开始采集";
        }
        else
        {
            info = "结束采集";
            benewakeLidarManager.stop();
        }
    }
    else
    {
        status = "0";
        error = "benewake lidar 不存在";
    }
    std::string return_info = "{status: " + status +
                              ", path: " + save_path +
                              ", log: " + info +
                              ", error: " + error +
                              "}";
    return return_info;
}

std::string SocketServer::dealCentralCam(CentralCamManager &central_camera_manager, std::string save_path, bool isStart) {
    std::string status = "1", error = "",info="";
    if (central_camera_manager.hasCentralCamera()) {
        if (isStart) {
            std::thread task_thread([&central_camera_manager](){central_camera_manager.startCapture();});
            task_thread.detach();//后台运行
            info = "\nThe central camera starts to collect data\n";
        }else {
            info = "\nThe central camera has stopped collecting data\n";
            central_camera_manager.stopCapture();
        }
    }else {
        status = "0";
        error = "central cam doesn't exit";
    }
    std::string return_info = "{status: " + status +
                              ", path: " + save_path +
                              ", log: " + info +
                              ", error: " + error +
                              "}";
    return return_info;
}

std::string SocketServer::dealSideCam(SideCamManager &side_cam_manager, std::string save_path, bool isStart) {
    std::string status = "1", error = "",info="";
    if (side_cam_manager.hasSideCamera()) {
        if (isStart) {
            std::thread task_thread([&side_cam_manager](){side_cam_manager.startCapture();});
            task_thread.detach();//后台运行
            info = "\nThe side camera starts to collect data\n";
        }else {
            info = "\nThe side camera has stopped collecting data\n";
            side_cam_manager.stopCapture();
        }
    }else {
        status = "0";
        error = "side cam doesn't exit";
    }
    std::string return_info = "{status: " + status +
                              ", path: " + save_path +
                              ", log: " + info +
                              ", error: " + error +
                              "}";
    return return_info;
}


std::string SocketServer::process_command(const std::string &command,
                                         FileManager &fileManager,BenewakeLidarManager &benewakeLidarManager,TanwayLidarManager &tanwayLidarManager,
                                         CentralCamManager &central_cam_manager, SideCamManager &side_cam_manager)
{
    // 拆分命令名与参数
    auto pos = command.find(':');
    std::string cmd = (pos != std::string::npos) ? command.substr(0, pos) : command;
    std::string arg = (pos != std::string::npos) ? command.substr(pos + 1) : "";

    if (cmd == "64_line_ladar_start")
    {
        return dealTanwayLidar(tanwayLidarManager,fileManager.get_64_lidar_save_path(), true);
    }
    else if (cmd == "64_line_ladar_end")
    {
        return dealTanwayLidar(tanwayLidarManager,fileManager.get_64_lidar_save_path(), false);
    }
    else if (cmd == "256_line_ladar_start")
    {
        return dealBeneWakeLidar(benewakeLidarManager,fileManager.get_256_lidar_save_path(), true);
    }
    else if (cmd == "256_line_ladar_end")
    {
        return dealBeneWakeLidar(benewakeLidarManager,fileManager.get_256_lidar_save_path(), false);
    }
    // 同时采集
    else if (cmd == "all_lidar_start")
    {
        dealTanwayLidar(tanwayLidarManager,fileManager.get_64_lidar_save_path(), true);
        dealBeneWakeLidar(benewakeLidarManager,fileManager.get_256_lidar_save_path(), true);
        return "Hello Client!\n";
    }
    else if (cmd == "all_lidar_end")
    {
        dealTanwayLidar(tanwayLidarManager,fileManager.get_64_lidar_save_path(), false);
        dealBeneWakeLidar(benewakeLidarManager,fileManager.get_256_lidar_save_path(), false);
        return "Goodbye Client!\n";
    }
    else if (cmd == "central_cam_start")
    {
        return dealCentralCam(central_cam_manager, fileManager.get_central_cam_path(),true);
    }
    else if (cmd == "central_cam_end") {
        return dealCentralCam(central_cam_manager, fileManager.get_central_cam_path(),false);
    }
    else if (cmd == "side_cam_start") {
        return dealSideCam(side_cam_manager, fileManager.get_side_cam_path(),true);
    }
    else if (cmd == "side_cam_end") {
        return dealSideCam(side_cam_manager, fileManager.get_side_cam_path(),false);
    }
    else if (cmd == "all_cam_start"){
        dealCentralCam(central_cam_manager, fileManager.get_central_cam_path(),true);
        dealSideCam(side_cam_manager, fileManager.get_side_cam_path(),true);
        return "Two camera start to take data!";
    }
    else if (cmd == "all_cam_end"){
        dealCentralCam(central_cam_manager, fileManager.get_central_cam_path(),false);
        dealSideCam(side_cam_manager, fileManager.get_side_cam_path(),false);
        return "Two camera stop to take data!";
    }
    // 创建保存目录
    else if (cmd == "create_path")
    {
        bool success = fileManager.createDirectory(arg);
        std::string return_info = "{status: " + std::string(success ? "1" : "0") +
                                  ", error: " + "" + "}";
        return return_info;
    }
    // 选择保存目录
    else if (cmd == "select_path")
    {
        fileManager.setSavePath(arg);
        std::string return_info = "{status: 1, error: }";
        return return_info;
    }
    // 删除指定目录
    else if (cmd == "det_path")
    {
        fileManager.deleteFile(arg);
        std::string init_info = get_init_info(fileManager, benewakeLidarManager,tanwayLidarManager, central_cam_manager, side_cam_manager);
        return init_info;
    }
    // 导出数据到u盘
    else if (cmd == "export_data_to_usb"){
        std::string return_info = "{status: 1 , error: }";
        if (fileManager.is_usb_inserted()){
            std::string src_path = fileManager.getSavePath();
            std::string dst_path = fileManager.get_usb_session_folder();
            std::string ret_info = fileManager.move_folder_contents(src_path, dst_path);
            if (ret_info!=""){
               return_info = "{status: 0 , error:"+ret_info+" }";
            }    
        }else{
               return_info = "{status: 0 , error: U盘没有挂载 }";
        }
        return return_info;
    }
    // 结束服务
    else if (cmd == "close")
    {
        Config::stopRun = false;
        return "The program has been closed and the service needs to be restarted";
    }
    else
    {
        return "Unknown command.\n";
    }
}
