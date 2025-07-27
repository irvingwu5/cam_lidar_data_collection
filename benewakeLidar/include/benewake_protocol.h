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
#ifndef INCLUDE_BENEWAKE_PROTOCOL_H__
#define INCLUDE_BENEWAKE_PROTOCOL_H__
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include "benewake_udp.h"
#include "benewake_tables.h"

#ifdef _WIN32
#ifdef BENEWAKE_DLL_EXPORT
#define BENEWAKE_API __declspec(dllexport)
#else
#define BENEWAKE_API __declspec(dllimport)
#endif // BENEWAKE_DLL_EXPORT
#else
#define BENEWAKE_API
#endif

namespace benewake
{
	class BENEWAKE_API Protocol
	{
	public:
		/** \brief DCSPProtocol constructor
		* \param[in] _send_ip: target device ip.
		* \param[in] _send_port: remote port where send data to.
		* \param[in] _recv_ip: data source ip which will be verified. When set to "", means any data received by _recv_port will be returned.
		* \param[in] _recv_port: local port to receive data.
		* \param[in] _udp_type: connection mode. NORMAL, MULTICAST or BOARDCAST.
		* \return none
		* \note
		*/
		Protocol(std::string _local_ip, int _local_port, std::string _remote_ip, int _remote_port, UDPType _udp_type);

		virtual ~Protocol();

		int close();

		int open();

		int open(UDPType _udp_type);

		virtual int reset_connection(std::string _local_ip, int _local_port, std::string _remote_ip, int _remote_port, UDPType _udp_type);
		
	protected:

		std::unique_ptr<Udp> pudp_;
		std::string local_ip_, remote_ip_;
		int local_port_, remote_port_;
		benewake::UDPType connection_type_;

		uint32_t check_sum_with_protocol_version(uint16_t _version, uint8_t* _buffer, size_t _size);

		uint32_t checksum(const uint8_t *_buffer, size_t _size);

		uint32_t check_crc32(const uint8_t* _buffer, size_t _size);

		uint32_t check_crc32_sb8_mode_begin(const uint8_t* _buffer, size_t _size);
	};

	bool mkDir(const std::string& strPath);

#define FOURYEARDAY (365+365+365+366)  //Total number of days in a 4-year cycle (there is no such year as 2100 from 1970 to 2038, so it is not optimized for now)

	typedef struct utc_time_struct
	{
		uint16_t year;       // 1970~2038
		uint8_t month;       // 1~12
		uint8_t day;		 // 1~31
		uint8_t hour;        // 0~23
		uint8_t minute;      // 0~59
		uint8_t second;      // 0~59

	}utc_time_t;

	static uint8_t monthDay[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }; //ƽ�� 
	static uint8_t leapMonthDay[12] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }; //���� 

	bool isLeapYear(uint16_t year);

	/** \brief Convert UTC to Unix time
	* \param[in] _UTCTime: target device ip.
	* \return Unix timestamp (number of seconds from 1970/1/1 00:00:00 to present)
	* \note The correctness of input parameters is not evaluated
	*/
	uint32_t covUTC2UnixTimestamp(utc_time_t* _UTCTime);
}
#endif