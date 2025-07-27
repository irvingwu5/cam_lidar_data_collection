#include "benewake_udp.h"
using namespace benewake;


Udp::Udp(std::string _local_ip, int _local_port, std::string _remote_ip, int _remote_port) :
	local_ip_(_local_ip), local_port_(_local_port), remote_ip_(_remote_ip), remote_port_(_remote_port)
{
#ifdef  _WIN32
	int err;
	WORD wVersionRequested;
	WSADATA wsaData;
	wVersionRequested = MAKEWORD(2, 2);
	err = WSAStartup(wVersionRequested, &wsaData);
	if (err != 0)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cerr << "WSA startup error!" << std::endl;
	}
#endif //  _WIN32
}

benewake::Udp::~Udp()
{
	this->disconnect();
#ifdef _WIN32
	WSACleanup();
#endif // _WIN32
}

int Udp::connect(UDPType _udp_type)
{
	int err;
	// init socket
	socket_ = socket(AF_INET, SOCK_DGRAM, 0);
	if (socket_ < 0)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Failed to initialize socket!" << std::endl;
#ifdef _WIN32
		err = WSAGetLastError();
		std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
		return STATUS_FAIL;
	}

	//set send addr
	send_addr_.sin_family = AF_INET;
	send_addr_.sin_addr.s_addr = inet_addr(local_ip_.data());
	send_addr_.sin_port = htons((u_short)local_port_);

	// set recv addr
	recv_addr_.sin_family = AF_INET;
	recv_addr_.sin_addr.s_addr = inet_addr(remote_ip_.data());
	recv_addr_.sin_port = htons((u_short)remote_port_);

	if (_udp_type == UDPType::LOCAL_BIND)
	{
		// bind ip&port
		sockaddr_in server_addr;
		server_addr.sin_family = AF_INET;
		server_addr.sin_addr.s_addr = inet_addr(local_ip_.data());
		server_addr.sin_port = htons((u_short)local_port_);
		std::cout << "bind to local ip " << local_ip_ << " port " << local_port_ << std::endl;
		if (bind(socket_, (sockaddr*)&server_addr, sizeof(server_addr)) != 0)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Failed to bind receive port!" << std::endl;
#ifdef _WIN32
			err = WSAGetLastError();
			std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
			return STATUS_FAIL;
		}
	}

	// set multicast connection
	if (_udp_type == UDPType::MULTICAST)
	{
		int opt = 1;
		if (setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt)) < 0)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Failed to set REUSEADDR!" << std::endl;
#ifdef _WIN32
			err = WSAGetLastError();
			std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
			return STATUS_FAIL;
		}

		// bind ip&port
		sockaddr_in local_addr;
		local_addr.sin_family = AF_INET;
#ifdef _WIN32
		local_addr.sin_addr.s_addr = inet_addr(local_ip_.data());
#else
		local_addr.sin_addr.s_addr = inet_addr(multicast_ip_.data());
#endif

		local_addr.sin_port = htons((u_short)multicast_port_);
		if (bind(socket_, (sockaddr*)&local_addr, sizeof(local_addr)) != 0)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Failed to bind receive port!" << std::endl;
#ifdef _WIN32
			err = WSAGetLastError();
			std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
			return STATUS_FAIL;
		}

		struct ip_mreq mreq;
		mreq.imr_multiaddr.s_addr = inet_addr(multicast_ip_.data());
		mreq.imr_interface.s_addr = inet_addr(local_ip_.data());
#ifdef _WIN32
		if (setsockopt(socket_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char*)&mreq, sizeof(mreq)) < 0)
#else
		if (setsockopt(socket_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)
#endif
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Failed to set multicast!" << std::endl;
#ifdef _WIN32
			err = WSAGetLastError();
			std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
			return STATUS_FAIL;
		}
	}
	
	// set boardcast connection
	if (_udp_type == UDPType::BOARDCAST)
	{
#ifdef _WIN32
		bool optval = true;
		if (setsockopt(socket_, SOL_SOCKET, SO_BROADCAST, (char*)&optval, sizeof(optval)) < 0)
#else
		int optval = 1;
		if (setsockopt(socket_, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval)) < 0)
#endif
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Failed to set boardcast!" << std::endl;
#ifdef _WIN32
			err = WSAGetLastError();
			std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
			return STATUS_FAIL;
		}

		int opt = 1;
		if (setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt, sizeof(opt)) < 0)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Failed to set REUSEADDR!" << std::endl;
#ifdef _WIN32
			err = WSAGetLastError();
			std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
			return STATUS_FAIL;
		}

		// bind ip&port
		sockaddr_in local_addr;
		local_addr.sin_family = AF_INET;
		local_addr.sin_addr.s_addr = inet_addr(local_ip_.data());
		local_addr.sin_port = htons((u_short)local_port_);
		if (bind(socket_, (sockaddr*)&local_addr, sizeof(local_addr)) != 0)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Failed to bind receive port!" << std::endl;
#ifdef _WIN32
			err = WSAGetLastError();
			std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
			return STATUS_FAIL;
		}
	}

	int nRecvBuf = 10 * 1024 * 1024;
	setsockopt(socket_, SOL_SOCKET, SO_RCVBUF, (const char*)&nRecvBuf, sizeof(int));
	
	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout));

	connect_ = true;
	return STATUS_OK;
}

int Udp::disconnect()
{
	if (!connect_)
		return STATUS_OK;
	
#ifdef _WIN32
	int ret = closesocket(socket_);
#else
	int ret = close(socket_);
#endif // _WIN32
	if (ret != 0)
		return STATUS_FAIL;

	memset(&recv_addr_, 0, sizeof(recv_addr_));
	memset(&send_addr_, 0, sizeof(send_addr_));
	connect_ = false;
	return STATUS_OK;
}

int Udp::sendData(const unsigned char *_buffer, int _nsize)
{
	if (!connect_)
	{
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Connection is not established!" << std::endl;
#endif // DEBUG_INFO
		return STATUS_FAIL;
	}
	try
	{
		int num = sendto(socket_, (const char*)_buffer, _nsize, 0, (sockaddr*)&recv_addr_, sizeof(recv_addr_));
		if (num != _nsize)
		{
#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Data is sent less than request!" << std::endl;
#endif // DEBUG_INFO
			return STATUS_FAIL;
		}
 	}
	catch (const std::exception& e)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << e.what() << std::endl;
		return STATUS_FAIL;
	}
	return STATUS_OK;
}

void benewake::Udp::setMutlicatIP(std::string _multicast_ip, int _multicast_port)
{
	multicast_ip_ = _multicast_ip;
	multicast_port_ = _multicast_port;
}

int Udp::recvData(unsigned char *_buffer, int _nsize, int _timeout_s, int _timeout_us)
{
	if (!connect_)
	{
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Connection is not established!" << std::endl;
#endif // DEBUG_INFO
		return STATUS_FAIL;
	}
	int err;
	try
	{
		std::string source_ip;
		struct timeval timeout;
		timeout.tv_sec = _timeout_s;
		timeout.tv_usec = _timeout_us;
		fd_set fd_read;
		std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
		std::chrono::system_clock::time_point stop;
		while (timeout.tv_sec != 0 || timeout.tv_usec != 0)
		{
			stop = std::chrono::system_clock::now();
			auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
			if (dur.count() > (int)timeout.tv_sec * 1000000 + (int)timeout.tv_usec)
				break;
			FD_ZERO(&fd_read);
			FD_SET(socket_, &fd_read);
#ifdef _WIN32
			int ret = select(0, &fd_read, NULL, NULL, &timeout);
#else
			int ret = select(socket_ + 1, &fd_read, NULL, NULL, &timeout);
#endif // _WIN32
			if (ret < 0) // socket error
			{
#ifdef DEBUG_INFO
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Socket error occurs!" << std::endl;
#ifdef _WIN32
				err = WSAGetLastError();
				std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
#endif // DEBUG_INFO
				return STATUS_FAIL;
			}
			else if (ret == 0) // timeout
			{
#ifdef DEBUG_INFO
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Timeout occurs!" << std::endl;
#endif // DEBUG_INFO
				return STATUS_TIME_OUT;
			}
			else if (FD_ISSET(socket_, &fd_read))
			{
#ifdef _WIN32
				int len = sizeof(remote_addr_);
#else
				socklen_t len = sizeof(remote_addr_);
#endif // _WIN32
				int num = recvfrom(socket_, (char*)_buffer, _nsize, 0, (sockaddr*)&remote_addr_, &len);
				if (remote_ip_.size() != 0)
				{
					source_ip = inet_ntoa(remote_addr_.sin_addr);
					if (strcmp(remote_ip_.data(), source_ip.data()) != 0)
						continue;
				}
				if (num < 0)
				{
#ifdef DEBUG_INFO
					std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
					std::cout << "Failed to receive data: " << num << " returned!" << std::endl;
#ifdef _WIN32
					err = WSAGetLastError();
					std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
#endif // DEBUG_INFO
					return STATUS_FAIL;
				}
				else if (num != _nsize)
				{
#ifdef DEBUG_INFO
					std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
					std::cout << "Received data is not equal to request. "
						<< _nsize << " bytes required but " << num << " bytes received." << std::endl;
#endif // DEBUG_INFO
					return STATUS_FAIL;
				}
				return STATUS_OK;
			}
		}
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Timeout occurs! " << std::endl;
#endif // DEBUG_INFO
		return STATUS_TIME_OUT;
	}
	catch (const std::exception& e)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << e.what() << std::endl;
		return STATUS_FAIL;
	}
}

int benewake::Udp::recvCMDResponse(unsigned char* _buffer, int _nsize, uint8_t _cmd, int _timeout_s, int _timeout_us)
{
	if (!connect_)
	{
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Connection is not established!" << std::endl;
#endif // DEBUG_INFO
		return STATUS_FAIL;
	}
	if (_nsize <= 15)
	{
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Response size should not be smaller than 16 bytes!" << std::endl;
#endif // DEBUG_INFO
		return STATUS_FAIL;
	}
	int err;
	try
	{
		std::string source_ip;
		struct timeval timeout;
		timeout.tv_sec = _timeout_s;
		timeout.tv_usec = _timeout_us;
		fd_set fd_read;
		std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
		std::chrono::system_clock::time_point stop;
		while (timeout.tv_sec != 0 || timeout.tv_usec != 0)
		{
			stop = std::chrono::system_clock::now();
			auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
			if (dur.count() > (int)timeout.tv_sec * 1000000 + (int)timeout.tv_usec)
				break;
			FD_ZERO(&fd_read);
			FD_SET(socket_, &fd_read);
#ifdef _WIN32
			int ret = select(0, &fd_read, NULL, NULL, &timeout);
#else
			int ret = select(socket_ + 1, &fd_read, NULL, NULL, &timeout);
#endif // _WIN32
			if (ret < 0) // socket error
			{
#ifdef DEBUG_INFO
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Socket error occurs!" << std::endl;
#ifdef _WIN32
				err = WSAGetLastError();
				std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
#endif // DEBUG_INFO
				return STATUS_FAIL;
			}
			else if (ret == 0) // timeout
			{
#ifdef DEBUG_INFO
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Timeout occurs!" << std::endl;
#endif // DEBUG_INFO
				return STATUS_TIME_OUT;
			}
			else if (FD_ISSET(socket_, &fd_read))
			{
#ifdef _WIN32
				int len = sizeof(remote_addr_);
#else
				socklen_t len = sizeof(remote_addr_);
#endif // _WIN32
				int num = recvfrom(socket_, (char*)_buffer, _nsize, 0, (sockaddr*)&remote_addr_, &len);
				if (remote_ip_.size() != 0)
				{
					source_ip = inet_ntoa(remote_addr_.sin_addr);
					if (strcmp(remote_ip_.data(), source_ip.data()) != 0)
						continue;
				}
				if (_buffer[6] != _cmd)
					continue;
				if (num < 0)
				{
#ifdef DEBUG_INFO
					std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
					std::cout << "Failed to receive data: " << num << " returned!" << std::endl;
#ifdef _WIN32
					err = WSAGetLastError();
					std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
#endif // DEBUG_INFO
					return STATUS_FAIL;
				}
				else if (num != _nsize)
				{
#ifdef DEBUG_INFO
					std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
					std::cout << "Received data is not equal to request. "
						<< _nsize << " bytes required but " << num << " bytes received." << std::endl;
#endif // DEBUG_INFO
					return STATUS_FAIL;
				}
				return STATUS_OK;
			}
		}
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Timeout occurs! " << std::endl;
#endif // DEBUG_INFO
		return STATUS_TIME_OUT;
	}
	catch (const std::exception& e)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << e.what() << std::endl;
		return STATUS_FAIL;
	}
}

int benewake::Udp::recvSomeData(unsigned char * _buffer, int _nsize, int _timeout_s, int _timeout_us)
{
	if (!connect_)
	{
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Connection is not established!" << std::endl;
#endif // DEBUG_INFO
		return STATUS_FAIL;
	}
	int err, receive_size;

	std::string source_ip;
	struct timeval timeout;
	timeout.tv_sec = _timeout_s;
	timeout.tv_usec = _timeout_us;
	fd_set fd_read;
	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
	std::chrono::system_clock::time_point stop;
	while (timeout.tv_sec != 0 || timeout.tv_usec != 0)
	{
		stop = std::chrono::system_clock::now();
		auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
		if (dur.count() > (int)timeout.tv_sec * 1000000 + (int)timeout.tv_usec)
			break;
		FD_ZERO(&fd_read);
		FD_SET(socket_, &fd_read);
#ifdef _WIN32
		int ret = select(0, &fd_read, NULL, NULL, &timeout);
#else
		int ret = select(socket_ + 1, &fd_read, NULL, NULL, &timeout);
#endif // _WIN32
		if (ret < 0) // socket error
		{
#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Socket error occurs!" << std::endl;
#ifdef _WIN32
			err = WSAGetLastError();
			std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
#endif // DEBUG_INFO
			return STATUS_FAIL;
		}
		else if (ret == 0) // timeout
		{
#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Timeout occurs!" << std::endl;
#endif // DEBUG_INFO
			return STATUS_TIME_OUT;
		}
		else if (FD_ISSET(socket_, &fd_read))
		{
#ifdef _WIN32
			int len = sizeof(remote_addr_);
#else
			socklen_t len = sizeof(remote_addr_);
#endif // _WIN32
			receive_size = recvfrom(socket_, (char*)_buffer, _nsize, 0, (sockaddr*)&remote_addr_, &len);
			if (remote_ip_.size() != 0)
			{
				source_ip = inet_ntoa(remote_addr_.sin_addr);
				if (strcmp(remote_ip_.data(), source_ip.data()) != 0)
					continue;
			}
			if (receive_size < 0)
			{
#ifdef DEBUG_INFO
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Failed to receive data: " << receive_size << " returned!" << std::endl;
#ifdef _WIN32
				err = WSAGetLastError();
				std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
#endif // DEBUG_INFO
				return STATUS_FAIL;
			}
			return receive_size;
		}
	}
#ifdef DEBUG_INFO
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	std::cout << "Timeout occurs!" << std::endl;
#endif // DEBUG_INFO
	return STATUS_TIME_OUT;
}

int benewake::Udp::recvUncertainCMDResponse(unsigned char* _buffer, int _nsize, uint8_t _cmd, int _timeout_s, int _timeout_us)
{
	if (!connect_)
	{
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Connection is not established!" << std::endl;
#endif // DEBUG_INFO
		return STATUS_FAIL;
	}
	if (_nsize <= 15)
	{
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Response size should not be smaller than 16 bytes!" << std::endl;
#endif // DEBUG_INFO
		return STATUS_FAIL;
	}
	int err, receive_size;
	try
	{
		std::string source_ip;
		struct timeval timeout;
		timeout.tv_sec = _timeout_s;
		timeout.tv_usec = _timeout_us;
		fd_set fd_read;
		std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
		std::chrono::system_clock::time_point stop;
		while (timeout.tv_sec != 0 || timeout.tv_usec != 0)
		{
			stop = std::chrono::system_clock::now();
			auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
			if (dur.count() > (int)timeout.tv_sec * 1000000 + (int)timeout.tv_usec)
				break;
			FD_ZERO(&fd_read);
			FD_SET(socket_, &fd_read);
#ifdef _WIN32
			int ret = select(0, &fd_read, NULL, NULL, &timeout);
#else
			int ret = select(socket_ + 1, &fd_read, NULL, NULL, &timeout);
#endif // _WIN32
			if (ret < 0) // socket error
			{
#ifdef DEBUG_INFO
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Socket error occurs!" << std::endl;
#ifdef _WIN32
				err = WSAGetLastError();
				std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
#endif // DEBUG_INFO
				return STATUS_FAIL;
			}
			else if (ret == 0) // timeout
			{
#ifdef DEBUG_INFO
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Timeout occurs!" << std::endl;
#endif // DEBUG_INFO
				return STATUS_TIME_OUT;
			}
			else if (FD_ISSET(socket_, &fd_read))
			{
#ifdef _WIN32
				int len = sizeof(remote_addr_);
#else
				socklen_t len = sizeof(remote_addr_);
#endif // _WIN32
				receive_size = recvfrom(socket_, (char*)_buffer, _nsize, 0, (sockaddr*)&remote_addr_, &len);
				if (remote_ip_.size() != 0)
				{
					source_ip = inet_ntoa(remote_addr_.sin_addr);
					if (strcmp(remote_ip_.data(), source_ip.data()) != 0)
						continue;
				}
				if (_buffer[6] != _cmd)
					continue;
				if (receive_size < 0)
				{
#ifdef DEBUG_INFO
					std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
					std::cout << "Failed to receive data: " << receive_size << " returned!" << std::endl;
#ifdef _WIN32
					err = WSAGetLastError();
					std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
#endif // DEBUG_INFO
					return STATUS_FAIL;
				}
				return receive_size;
			}
		}
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Timeout occurs!" << std::endl;
#endif // DEBUG_INFO
		return STATUS_TIME_OUT;
	}
	catch (const std::exception& e)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << e.what() << std::endl;
		return STATUS_FAIL;
	}
}

int benewake::Udp::recvUncertainCMDResponseP4(unsigned char* _buffer, int _nsize, uint8_t _cmd, uint32_t _cmd_count, int _timeout_s, int _timeout_us)
{
	if (!connect_)
	{
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Connection is not established!" << std::endl;
#endif // DEBUG_INFO
		return STATUS_FAIL;
	}
	if (_nsize <= 15)
	{
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Response size should not be smaller than 16 bytes!" << std::endl;
#endif // DEBUG_INFO
		return STATUS_FAIL;
	}
	int err, receive_size;
	try
	{
		std::string source_ip;
		struct timeval timeout;
		timeout.tv_sec = _timeout_s;
		timeout.tv_usec = _timeout_us;
		fd_set fd_read;
		std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
		std::chrono::system_clock::time_point stop;
		while (timeout.tv_sec != 0 || timeout.tv_usec != 0)
		{
			stop = std::chrono::system_clock::now();
			auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
			if (dur.count() > (int)timeout.tv_sec * 1000000 + (int)timeout.tv_usec)
				break;
			FD_ZERO(&fd_read);
			FD_SET(socket_, &fd_read);
#ifdef _WIN32
			int ret = select(0, &fd_read, NULL, NULL, &timeout);
#else
			int ret = select(socket_ + 1, &fd_read, NULL, NULL, &timeout);
#endif // _WIN32
			if (ret < 0) // socket error
			{
#ifdef DEBUG_INFO
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Socket error occurs!" << std::endl;
#ifdef _WIN32
				err = WSAGetLastError();
				std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
#endif // DEBUG_INFO
				return STATUS_FAIL;
			}
			else if (ret == 0) // timeout
			{
#ifdef DEBUG_INFO
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Timeout occurs!" << std::endl;
#endif // DEBUG_INFO
				return STATUS_TIME_OUT;
			}
			else if (FD_ISSET(socket_, &fd_read))
			{
#ifdef _WIN32
				int len = sizeof(remote_addr_);
#else
				socklen_t len = sizeof(remote_addr_);
#endif // _WIN32
				receive_size = recvfrom(socket_, (char*)_buffer, _nsize, 0, (sockaddr*)&remote_addr_, &len);
				if (remote_ip_.size() != 0)
				{
					source_ip = inet_ntoa(remote_addr_.sin_addr);
					if (strcmp(remote_ip_.data(), source_ip.data()) != 0)
						continue;
				}
				uint32_t header_count = *(int32_t*)&_buffer[6];
				if (header_count != _cmd_count)
					continue;
				if (_buffer[10] != _cmd)
					continue;
				if (receive_size < 0)
				{
#ifdef DEBUG_INFO
					std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
					std::cout << "Failed to receive data: " << receive_size << " returned!" << std::endl;
#ifdef _WIN32
					err = WSAGetLastError();
					std::cout << "WIN Socket Error Code: " << err << std::endl;
#endif // _WIN32
#endif // DEBUG_INFO
					return STATUS_FAIL;
				}
				return receive_size;
			}
		}
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Timeout occurs!" << std::endl;
#endif // DEBUG_INFO
		return STATUS_TIME_OUT;
	}
	catch (const std::exception& e)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << e.what() << std::endl;
		return STATUS_FAIL;
	}
}





