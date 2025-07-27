/**
* @file       benewake_udp.h

* @brief      Header of Benewake lidar's driver.

* @details 	  For those who want to develop their own program based on Benewake Horn X2 driver, this header
			  should be included in the codes. Corresponding Dynamic Link Library(.dll) and Object File
			  Library(.lib) also should be contained in the project.

* @author     Tan Wenquan

* @date       02/10/2023

* @version    v3.0.0

* @par Copyright (c):

*      Benewake (Beijing) Co., Ltd
*/

#ifndef INCLUDE_BENEWAKE_UDP_H__
#define INCLUDE_BENEWAKE_UDP_H__

#include <stdio.h>
#include <iostream>
#include <chrono>

#include "benewake_common.h"

#ifdef _WIN32
#include <direct.h>
#include <io.h>
#include <WinSock2.h>
#include <WS2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <dirent.h>
#include <errno.h>
#include <netdb.h>
#include <string.h>
#include <memory>
#endif // WIN32

namespace benewake
{
	enum class UDPType
	{
		NORMAL = 0,
		MULTICAST,
		BOARDCAST,
		LOCAL_BIND
	};

	class Udp
	{
	public:
		Udp(std::string _local_ip, int _local_port, std::string _remote_ip, int _remote_port);
		~Udp();

		int connect(UDPType _udp_type = UDPType::NORMAL);

		int disconnect();

		int sendData(const unsigned char *_buffer, int _nsize);

		void setLocalIP(std::string _local_ip) { local_ip_ = _local_ip; }

		void setMutlicatIP(std::string _multicast_ip, int _multicast_port);

		int recvData(unsigned char *_buffer, int _nsize, int _timeout_s = 2, int _timeout_us = 0);

		int recvCMDResponse(unsigned char* _buffer, int _nsize, uint8_t _cmd, int _timeout_s = 2, int _timeout_us = 0);

		int recvSomeData(unsigned char *_buffer, int _nsize, int _timeout_s = 2, int _timeout_us = 0);

		int recvUncertainCMDResponse(unsigned char* _buffer, int _nsize, uint8_t _cmd, int _timeout_s = 2, int _timeout_us = 0);

		int recvUncertainCMDResponseP4(unsigned char* _buffer, int _nsize, uint8_t _cmd, uint32_t _cmd_count, int _timeout_s = 2, int _timeout_us = 0);

		sockaddr_in getRecvInfo() { return remote_addr_; }

	private:
		std::string local_ip_, remote_ip_, multicast_ip_;
		int local_port_, remote_port_, multicast_port_;
		sockaddr_in recv_addr_, send_addr_, remote_addr_;
		bool connect_ = false;

#ifdef  _WIN32
		SOCKET socket_;
#else
		int socket_;
#endif //  WINDOWS

	};
}

#endif
