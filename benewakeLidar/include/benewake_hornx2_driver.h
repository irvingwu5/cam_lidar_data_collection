/**
* @file       benewake_hornx2_driver.h

* @brief      Header of Benewake Horn X2 lidar's driver.

* @details 	  For those who want to develop their own program based on Benewake Horn X2 driver, this header
			  should be included in the codes. Corresponding Dynamic Link Library(.dll) and Object File
			  Library(.lib) also should be contained in the project.

* @author     TAN Wenquan

* @date       11/19/2020

* @version    v2.0.0

* @par Copyright (c):

*      Benewake (Beijing) Co., Ltd
*/
#ifndef BENEWAKE_HORNX2_DRIVER_H__
#define BENEWAKE_HORNX2_DRIVER_H__

#include "benewake_lidar_driver.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
	namespace benewake
	{
		class BENEWAKE_API BenewakeHornX2 : public BenewakeLidar
		{
		public:

			/**
			* @brief Constructor.
			* @param _ip: Lidar's IP address, default is 192.168.0.2.
			* @param _port: Control commands port.
			* @param _type: Type to receive point cloud data, NORMAL or MULTICAST.
			* @param _local_ip: local ip. Must be set when use driver on Linux with MULTICAST mode.
			*/
			BenewakeHornX2(std::string _ip = "192.168.0.2", uint32_t _port = 2469, UDPType _type = benewake::UDPType::NORMAL, std::string _local_ip = "0.0.0.0");

			/**
			* @brief Constructor.
			* @param _info: Lidar's heart beat information.
			* @param _local_ip: local ip. Must be set when use driver on Linux with MULTICAST mode.
			*/
			BenewakeHornX2(benewake::SYS_INFO _info, std::string _local_ip = "0.0.0.0");

			~BenewakeHornX2();
		};
	}

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // BENEWAKE_HORNX2_H
