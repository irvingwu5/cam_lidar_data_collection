/**
* @file       benewake_dcsp.h

* @brief      Header of Benewake lidar's driver.

* @details 	  For those who want to develop their own program based on Benewake Horn X2 driver, this header
			  should be included in the codes. Corresponding Dynamic Link Library(.dll) and Object File
			  Library(.lib) also should be contained in the project.

* @author     TAN Wenquan

* @date       02/10/2023

* @version    v3.0.0

* @par Copyright (c):

*      Benewake (Beijing) Co., Ltd
*/
#ifndef INCLUDE_BENEWAKE_DSOP_H__
#define INCLUDE_BENEWAKE_DSOP_H__
#include <string>
#include <thread>
#include <atomic>
#include <memory>
#include "benewake_protocol.h"

#ifdef _WIN32

#ifdef BENEWAKE_DLL_EXPORT
#define BENEWAKE_API __declspec(dllexport)
#else
#define BENEWAKE_API __declspec(dllimport)
#endif // BENEWAKE_DLL_EXPORT

#else
#define BENEWAKE_API
#endif // _WIN32

namespace benewake
{
	typedef void(*SetProtocolVersionCallbcakFunc)(uint16_t _version, std::shared_ptr<void> _pDCSP);

	//class BENEWAKE_API DSOPProtocol
	class BENEWAKE_API DSOPProtocol : public Protocol
	{
	public:
		/** \brief DCSPProtocol constructor
		* \param[in] _local_ip: target device ip.
		* \param[in] _local_port: remote port where send data to.
		* \param[in] _remote_ip: data source ip which will be verified. When set to "", means any data received by _recv_port will be returned.
		* \param[in] _remote_port: local port to receive data.
		* \param[in] _udp_type: connection mode. NORMAL, MULTICAST or BOARDCAST.
		* \return none
		* \note
		*/
		DSOPProtocol(std::string _local_ip = "255.255.255.255", int _local_port = 65000,
			std::string _remote_ip = "", int _remote_port = 65000, UDPType _udp_type = UDPType::BOARDCAST);

		~DSOPProtocol();

		void dsop_cancel_heart_beat_callback();

		/** \brief Disable heart beat receiver and set fix key status
		* \param[in] _ip: device IP address.
		* \param[in] _dcsp_port: device's DCSP port.
		* \param[in] _device_type: device type.
		* @verbatim
		*			0x00	- PRODUCT_ID_X2
		*			0x01	- PRODUCT_ID_P4
		* @verbatim
		* \param[in] _protocol_version: protocol version for data checking.
		* @verbatim
		*			0x00	- check sum
		*			0x01	- check CRC32
		*			0x02	- check CRC32_SB8_MODE_BEGIN
		* @verbatim
		* \return none
		* \note
		*/
		void dsop_disable_heart_beat_receiver(std::string _ip, int _dcsp_port, uint8_t _device_type, uint16_t _protocol_version);

		SYS_INFO dsop_get_system_information();

		SYS_INFO dsop_get_system_information_no_wait();

		void dsop_regist_heart_beat_callback(HeartBeatCallbcakFunc _func, void *_pData);

		void dsop_search_device(std::vector<SYS_INFO> & _device_list, int _search_milliseconds = 7000);

		int reset_connection(std::string _local_ip = "255.255.255.255", int _local_port = 65000,
			std::string _remote_ip = "", int _remote_port = 65000, UDPType _udp_type = UDPType::BOARDCAST);

		/** \brief Set Pointers to synchronize information from different ports
		* \note
		*/
		void setDeviceInfoPtr(SYS_INFO* _info);

	private:
		SYS_INFO sys_info_;
		uint8_t device_type_ = PRODUCT_ID_X2;

		std::thread *listen_thread_;
		bool thread_run_ = false;
#ifdef _WIN32
		HANDLE occup_;
		HANDLE callback_occup_;
#else
		sem_t occup_;
		sem_t callback_occup_;
#endif // _WIN32

		// search device
		std::vector<SYS_INFO> device_list_;
		std::atomic_bool device_searching_;
		int device_search_time_ = 0;
		std::chrono::system_clock::time_point device_search_start_, device_search_elapsed_;
		bool new_pkg_ = false;

		std::atomic_bool callback_enable_;
		HeartBeatCallbcakFunc callback_func_;
		void * callback_pointer_;

		SYS_INFO* dsop_info_ = nullptr;

		// for disable heart beat receiver
		bool heart_beat_disabled_ = false;

		void listenHeartBeat();

		void processPayloadX(SYS_INFO& _info, unsigned char* _data);

		void processPayloadP4(SYS_INFO& _info, unsigned char* _data);
	};
}

#endif