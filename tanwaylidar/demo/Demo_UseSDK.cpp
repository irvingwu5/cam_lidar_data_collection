/*
* Software License Agreement (BSD License)
*   
*  Copyright (c) Tanway science and technology co., LTD.
*
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without modification, 
*  are permitted provided  that the following conditions are met:
*
*   1.Redistributions of source code must retain the above copyright notice, 
*     this list of conditions and the following disclaimer.
*
*   2.Redistributions in binary form must reproduce the above copyright notice, 
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*     
*   3.Neither the name of the copyright holder(s) nor the names of its  contributors
*     may be used to endorse or promote products derived from this software without 
*     specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
*  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
*  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "../include/ILidarDevice.h"
#include "../include/ILidarAlgo.h"
#include "iostream"
#include <thread>
#include <iomanip>

using namespace tanway;
class LidarObserver : public ILidarObserver{
public:
  virtual void OnPointCloud(const LidarInfo &lidarInfo, const UserPointCloud &pointCloud){
	/*
	*The point cloud struct uses a const reference. 
	*Please copy the point cloud data to another thread for use.
	*Avoid directly operating the UI in the callback function.
	*/
	std::cout << "width:" << pointCloud.width 
			  << " height:" << pointCloud.height 
			  << " point cloud size: " << pointCloud.points.size() << std::endl;
  }
  virtual void OnIMU(const LidarInfo &lidarInfo, const IMUData &imu){
// std::cout << "IMU data callback:" << std::endl
//       << "  Angular velocity [rad/s]: "
//       << "X: " << std::fixed << std::setprecision(6)
//       << imu.angular_velocity[0] << ", Y: " << std::fixed
//       << std::setprecision(6) << imu.angular_velocity[1]
//       << ", Z: " << std::fixed << std::setprecision(6)
//       << imu.angular_velocity[2] << std::endl
//       << "  Linear acceleration [g]: "
//       << "X: " << std::fixed << std::setprecision(6)
//       << imu.linear_acceleration[0] << ", Y: " << std::fixed
//       << std::setprecision(6) << imu.linear_acceleration[1]
//       << ", Z: " << std::fixed << std::setprecision(6)
//       << imu.linear_acceleration[2] << std::endl;

  }
  virtual void OnException(const LidarInfo &lidarInfo, const Exception &e){
	/* 
	*This callback function is called when the SDK sends a tip or raises an exception problem.
	*Use another thread to respond to the exception to avoid time-consuming operations.
	*/
	if (e.GetErrorCode() > 0)
		std::cout << "[Error Code]: " << e.GetErrorCode() << " -> " << e.ToString() << std::endl;
	if (e.GetTipsCode() > 0)
		std::cout << "[Tips Code]: " << e.GetTipsCode() << " -> " << e.ToString() << std::endl;	

  }
  virtual void OnDeviceInfoFrame(const LidarInfo &lidarInfo, const DeviceInfoFrame &deviceInfoFrame){}
  virtual void OnParsePcapProcess(const LidarInfo &lidarInfo, int process, uint64_t frame, uint64_t stamp){}
};

int main()
{
	LidarObserver lidarObserver;
	//example:Duetto
	//online:
	// 接收端口不要改
	auto lidar = ILidarDevice::Create("192.168.111.51", "192.168.111.204", 5600, 5700, &lidarObserver, LT_FocusB2_B3_MP);  
	//if lidar has separate IMU port 5800:
	// auto lidar = ILidarDevice::Create("192.168.111.51", "192.168.111.204", 5600, 5700, &lidarObserver, LT_TW360,false,0,5800);  
	
	//offline:using pcap file to replay.
	//auto lidar = ILidarDevice::Create("test.pcap", "192.168.111.51", 5600, 5700, &lidarObserver, LT_Duetto, true);  //pcap包路径使用绝对路径
	
	//use algo:
	auto algo = ILidarAlgo::Create(LT_FocusB2_B3_MP, "/root/workspace/CollectionControlServer/tanwaylidar/config/algo_table.json");  //json文件路径使用绝对路径
	lidar->SetLidarAlgo(algo.get());
	
	//start lidar
	lidar->Start();

	//quit
	// 以下代码演示了每三秒进行一次雷达的开启和关闭操作。
    // 正常情况下，用户无需频繁执行此操作。调用 lidar->Start() 启动雷达后，根据需要调用 lidar->Stop() 停止雷达即可。
	bool run_t = false;
	while (true)
	{
		std::this_thread::sleep_for(std::chrono::seconds(3));
		if (run_t)
		{
			lidar->Start();
			std::cout << "===========: start()" << std::endl;
			run_t = false;
		}
		else
		{
			lidar->Stop();
			std::cout << "===========: stop()" << std::endl;
			run_t = true;
		}
	}

    return 0;
}

