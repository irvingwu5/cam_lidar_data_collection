// vs2017.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// #include "pch.h"
#include <iostream>
#include <string>

#include <fstream>
#include <sstream>
#include <iomanip>
#include <queue>
#include <thread>
#include <mutex>
#include "SocketServer.h"
#include "SocketClient.h"
#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
	// 启动服务器线程
	std::thread server_thread([]()
	{
		//防止端口被占用，先进行端口释放
		//system("sudo fuser -k 8082/tcp");  // Linux 下释放端口
		std::this_thread::sleep_for(std::chrono::seconds(1));
        SocketServer server(8082);
		while(Config::stopRun){
        server.start();
        }
    });

	// 稍等服务器启动
	std::this_thread::sleep_for(std::chrono::seconds(2));

	// 客户端交互

	//SocketClient client("127.0.0.1", 8080);
	//if (client.connect_to_server())
	//{
		//client.send_message("central_cam_start");
		//client.receive_response();
		// client.send_message("create_path:m6");
		// client.receive_response();
  //
		// client.send_message("select_path:m6");
		// client.receive_response();
  //
		//
		// client.send_message("together_start");
		// client.receive_response();
  //
		// std::this_thread::sleep_for(std::chrono::seconds(16));
		// client.send_message("together_end");
		// client.receive_response();
  //
  //       client.send_message("export_data_to_usb");
		// client.receive_response();
		// client.send_message("det_path:ram");
		// client.receive_response();

		//client.close_connection();
	//}

	server_thread.join();
	return 0;
}
