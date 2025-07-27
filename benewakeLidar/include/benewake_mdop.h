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
#ifndef INCLUDE_BENEWAKE_MDOP_H__
#define INCLUDE_BENEWAKE_MDOP_H__
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <memory>
#include <ctime>
#include "benewake_protocol.h"
#include "msg_queue.h"

#ifdef _WIN32
#ifdef BENEWAKE_DLL_EXPORT
#define BENEWAKE_API __declspec(dllexport)
#else
#define BENEWAKE_API __declspec(dllimport)
#endif // BENEWAKE_DLL_EXPORT
#else
#include <semaphore.h>
#define BENEWAKE_API
#endif

namespace benewake
{
	struct RingBufferData
	{
		uint16_t data_length = 0;
		unsigned char* data = NULL;
	};

	struct RingBufferInfo
	{
		uint32_t head = 0;
		uint32_t tail = 0;
	};

	//class BENEWAKE_API MDOPProtocol
	class BENEWAKE_API MDOPProtocol : public Protocol
	{
	public:
		/** \brief DCSPProtocol constructor
		* \param[in] _local_ip: local device ip.
		* \param[in] _local_port: local port to receive data.
		* \param[in] _remote_ip: data source ip which will be verified. When set to "", means any data received by _recv_port will be returned.
		* \param[in] _remote_port: remote port that the data from.
		* \param[in] _udp_type: connection mode. NORMAL, MULTICAST or BOARDCAST.
		* \return none
		* \note
		*/
		MDOPProtocol(std::string _local_ip = "0.0.0.0", int _local_port = 2468,
			std::string _remote_ip = "192.168.0.2", int _remote_port = 2468, UDPType _udp_type = UDPType::NORMAL);

		~MDOPProtocol();

		void mdop_cancel_point_cloud_callback();

		void mdop_enable_AB_Frame(bool _enable, int _frame_rows);

		virtual int mdop_enable_decode(bool _enable);

		int mdop_enable_gray(bool _enable);

		int mdop_enable_rgb(bool _enable);

		benewake::BwPointCloud::Ptr mdop_front_time_window_data();

		void mdop_front_view_filter(BwPointCloud::Ptr& _cloud);

		void mdop_get_data_transmission_state(uint32_t& _delay_time, float& _pkg_loss_rate, float& _frame_loss_rate);

		int mdop_get_frame(BwPointCloud::Ptr& _ptr, int & _frame_id, int _wait_ms = 1000);

		void mdop_get_front_view_filter_parameters(int& _rows, int& _cols, int& _bind, int& _n_scan, float& _h_resolution, float& _v_resolution);

		void mdop_get_signal_message(char* _msg, uint16_t& _time_offset);

		int mdop_get_single_point_data(float* _data, int _wait_ms = 1000);

		int mdop_get_sphere_coord_data(std::vector<BwSphereCoord>& _sphere_data, int _wait_ms = 1000);

		int mdop_get_test_efficient_percent(float& _percent, int _wait_ms = 1000);

		int mdop_log(bool _enable);

		benewake::BwPointCloud::Ptr mdop_pop_front_time_window_data();

		void mdop_pop_time_window_data();

		void mdop_regist_point_cloud_callback(PointCloudCallbackFunc _func, void *_pData);

		int mdop_save_data(std::string _save_path = "OrigData", std::string _save_file_name = "", int _frame_num = 1);

		int mdop_save_data(bool _is_dominant_wave_mode, std::string _save_path = "OrigData", std::string _save_file_name = "", int _frame_num = 1);

		virtual int mdop_save_data_p4(std::string _save_path = "OrigData", std::string _save_file_name = "", uint32_t _save_times = 1) { return 0; }

		virtual int mdop_save_data_g66(std::string _save_path = "OrigData", std::string _save_file_name = "", uint32_t _save_times = 1) { return 0; }

		void mdop_set_current_work_mode(uint8_t _mode) { work_mode_ = _mode; }

		void mdop_set_front_view_filter_parameters(int _rows, int _cols, int _bind, int _n_scan, float _h_resolution, float _v_resolution);

		void mdop_set_local_transform(bool _enable, float _roll, float _pitch, float _yaw, float _x, float _y, float _z);

		void mdop_set_multicast_ip(std::string _multicast_ip, int _multicast_port);

		bool mdop_set_time_window(bool _enable_time_window, uint32_t _interval = 144); // _interval in millisecond

		// if FPGA version is long range version, set TRUE
		void mdop_set_X2_long_range_version(bool _flag);

		int mdop_time_window_data_queue_size();

		int mdop_switch_protocol(BwProtocolType _protocol);

		int open();

		int open(UDPType _udp_type);

		virtual void mdop_set_angle_calibration(bool _enable) {}

		virtual void mdop_get_angle_calibration(bool& _enable) {}

		int mdop_enable_data_forwarding(bool _enabel, std::string _forward_ip, int _forward_port);

	protected:
		BwProtocolType protocol_type_ = BwProtocolType::CLIENT;
		bool new_frame_ = false;
		int frame_id_ = 0;
		bool log_enable_ = false;
		std::string log_path_;

		bool AB_frame_flag_ = false;
		int frame_rows_half_ = 0;

		float single_point_data_[8];
		bool new_data_ = false;

		bool gray_enable_ = false;
		bool rgb_enable_ = false;

		unsigned char mdop[4] = { 'M', 'D', 'O', 'P' };
		BwPointCloud::Ptr point_cloud_buffer_;

		std::thread *decode_thread_;
		bool thread_run_ = false;
		std::string save_path_ = "";
		std::string save_file_name_ = "";

		std::unique_ptr<Udp> dataforward_udp_;

#ifdef _WIN32
		HANDLE occup_;
#else
		sem_t occup_;
#endif // _WIN32
		// callback func
		PointCloudCallbackFunc callback_func_;
		void *callback_pointer_;
		bool callback_enable_ = false;

		// sephere data
		std::vector<BwSphereCoord> sphere_coord_out_buffer_;
		std::vector<BwSphereCoord> sphere_coord_buffer_;
		std::vector<BwSphereCoordAll> sphere_coord_all_buffer_;
		bool save_flag_ = false;
		bool is_dominant_wave_mode_ = false;
		int save_num_ = 0;

		// test data
		float eff_percent_ = 0;

		// data statics
		std::atomic<long long> delay_time_; // us
		std::atomic<float> pkg_loss_rate_;
		std::atomic<float> frame_loss_rate_		;

		// algorithm
		std::atomic_bool enable_angle_calib_;
		uint8_t work_mode_ = 0;

		std::atomic_bool enable_flag_dataforward_;

		// time window
		std::shared_ptr<benewake::MsgQueue<benewake::BwPointCloud::Ptr>> data_queue_;
		bool enable_time_windows_ = false;
		uint32_t frame_interval_ = 0; // in ms

		// X2 long range version flag
		bool X2_long_range_version_ = true;

		// front view filter parameters
		int FVF_rows_ = 1100;
		int FVF_cols_ = 2000;
		int FVF_bind_ = 10;
		int FVF_n_scan_ = 4;
		float FVF_h_resolution_ = 0.045;
		float FVF_v_resolution_ = 0.026;
		int FVF_r_bind_ = 110;
		int FVF_c_bind_ = 200;

		// signal message
		char* signal_message_ = nullptr;
		uint16_t signal_time_offset_ = 0;

		// local transform
		bool enable_local_transform_ = false;
		double transform_matrix_[16] = { 0 };

		virtual void clientDataDecode();

		virtual void sphereDataDecode();

		virtual void testDataDecode();

	};

	class BENEWAKE_API MDOPProtocolP4 : public MDOPProtocol
	{
	public:
		/** \brief DCSPProtocol constructor
		* \param[in] _local_ip: local device ip.
		* \param[in] _local_port: local port to receive data.
		* \param[in] _remote_ip: data source ip which will be verified. When set to "", means any data received by _recv_port will be returned.
		* \param[in] _remote_port: remote port that the data from.
		* \param[in] _udp_type: connection mode. NORMAL, MULTICAST or BOARDCAST.
		* \return none
		* \note
		*/
		MDOPProtocolP4(std::string _local_ip = "0.0.0.0", int _local_port = 2468,
			std::string _remote_ip = "192.168.0.2", int _remote_port = 2468, UDPType _udp_type = UDPType::NORMAL);

		~MDOPProtocolP4();

		int mdop_enable_decode(bool _enable);

		benewake::BwTestDataP4* getSrcDataPtr();

		int mdop_save_data_p4(std::string _save_path = "OrigData", std::string _save_file_name = "", uint32_t _save_times = 1);

		void mdop_set_angle_calibration(bool _enable);

		void mdop_get_angle_calibration(bool &_enable);

	protected:
		void recvPackageHandler();

		void clientDataDecode();

		void sphereDataDecode();

		void testDataDecode();

		void treatEchoMode(uint8_t _echo_flag, int& _echo_mode, uint8_t& _echo_mark_1, uint8_t& _echo_mark_2);

		void getTestDataFromBuffer(benewake::BwTestDataP4* _test_data, unsigned char* _buffer, uint16_t _data_offset);

		void saveTestData(benewake::BwTestDataP4& _data);

		void saveTestDataOnceFrame(BwTestDataP4Frame::Ptr _test_data);

		void saveSphereCoordData(BwSphereCoordP4Frame::Ptr _data);

		void saveDataThread();

		void openDataOutputStream();

		void closeDataOutputStream();

		/** \brief Process entire frame for data output
		* \param[in/out] _frame_buf: buffer of entire frame.
		* \param[in/out] _next_frame_buf: backup buffer of next frame's data.
		* \param[in/out] _backup_sphere_buf: buffer of frame in sphere coordinate for storage.
		* \param[in/out] _frame_id: frame ID.
		* \param[in/out] _lost_pkg_count: packages lost from last frame.
		* \param[in/out] _diff_frame: frame ID difference from last frame.
		* \return none
		*/
		void processFrameBufferSphereCoord(
			benewake::BwPointCloud::Ptr& _frame_buf,
			benewake::BwPointCloud::Ptr& _next_frame_buf,
			benewake::BwSphereCoordP4Frame::Ptr& _backup_sphere_buf,
			uint16_t& _frame_id,
			uint32_t& _lost_pkg_count,
			uint32_t& _diff_frame);

		/** \brief Process entire frame for test data output
		* \param[in/out] _frame_buf: buffer of entire frame.
		* \param[in/out] _frame_id: frame ID.
		* \return none
		*/
		void processFrameBufferTestData(
			benewake::BwTestDataP4Frame::Ptr& _frame_buf,
			uint16_t& _frame_id);

		/** \brief Angle calibration function
		* \param[in] _in_rad_h: horizontal angle of current 16 channel points, in radian.
		* \param[in] _in_rad_v_ch0: vertical angle of current channel 0 point, in radian.
		* \param[in] _in_ch: which channel of current point, from 0 to 15.
		* \param[out] _out_rad_h: calibrated horizontal angle of current point, in radian.
		* \param[out] _out_rad_v: calibrated vertical angle of current point, in radian.
		* \return none
		*/
		void angleCalibration_AD2_B(float _in_rad_h, float _in_rad_v_ch0, int _in_ch, float& _out_rad_h, float& _out_rad_v);

		/** \brief Angle compensate function
		* \param[in] _in_rad_h: azimuth of current 16 channel points, in radian.
		* \param[in] _in_rad_v_ch0: elevation of current channel 0 point, in radian.
		* \param[in] _in_ch: which channel of current point, from 0 to 15.
		* \param[out] _out_rad_h: channel compensate azimuth of current point, in radian.
		* \param[out] _out_rad_v: channel compensate elevation of current point, in radian.
		* \return none
		*/
		void angleCalibration_AD2_C(float _in_azimuth, float _in_elevation, int _in_ch, float& _out_azimuth, float& _out_elevation);

		bool ringBufferIsFull(const RingBufferInfo& _ringbuffer);

		bool ringBufferIsEmpty(const RingBufferInfo& _ringbuffer);

		BwTestDataP4 src_data_;

		std::ofstream data_out_2_file_;

		uint32_t save_times_ = 0;
		int save_name_flag_ = 0;

		bool save_data_thread_run_ = false;

		std::thread *save_data_thread_;

		benewake::BwSphereCoordP4Frame::Ptr sphere_coord_frame_data_;

		std::shared_ptr<benewake::MsgQueue<benewake::BwSphereCoordP4Frame::Ptr>> sphere_coord_frame_data_queue_ = NULL;

		benewake::BwTestDataP4Frame::Ptr test_frame_data_;

		std::shared_ptr<benewake::MsgQueue<benewake::BwTestDataP4Frame::Ptr>> test_frame_data_queue_ = NULL;

		std::thread* recv_thread_;
		RingBufferData* recv_buffer_queue_ = NULL;
		RingBufferInfo recv_buffer_info_;

		benewake::SYS_INFO sys_info_;
	};

	class BENEWAKE_API MDOPProtocolG66 : public MDOPProtocolP4
	{
	public:
		/** \brief DCSPProtocol constructor
		* \param[in] _local_ip: local device ip.
		* \param[in] _local_port: local port to receive data.
		* \param[in] _remote_ip: data source ip which will be verified. When set to "", means any data received by _recv_port will be returned.
		* \param[in] _remote_port: remote port that the data from.
		* \param[in] _udp_type: connection mode. NORMAL, MULTICAST or BOARDCAST.
		* \return none
		* \note
		*/
		MDOPProtocolG66(std::string _local_ip = "0.0.0.0", int _local_port = 2468,
			std::string _remote_ip = "192.168.0.2", int _remote_port = 2468, UDPType _udp_type = UDPType::NORMAL);

		~MDOPProtocolG66();

		int mdop_enable_decode(bool _enable);

		benewake::BwTestDataG66* getSrcDataPtr();

		int mdop_save_data_g66(std::string _save_path = "OrigData", std::string _save_file_name = "", uint32_t _save_times = 1);

	protected:
		void sphereDataDecode();

		//void testDataDecode();

		/** \brief Process entire frame for data output
		* \param[in/out] _frame_buf: buffer of entire frame.
		* \param[in/out] _backup_sphere_buf: buffer of frame in sphere coordinate for storage.
		* \param[in/out] _frame_id: frame ID.
		* \param[in/out] _lost_pkg_count: packages lost from last frame.
		* \param[in/out] _diff_frame: frame ID difference from last frame.
		* \return none
		*/
		void processFrameBufferSphereCoord(
			benewake::BwPointCloud::Ptr& _frame_buf,
			benewake::BwSphereCoordG66Frame::Ptr& _backup_sphere_buf,
			uint16_t& _frame_id,
			uint32_t& _lost_pkg_count,
			uint32_t& _diff_frame);

		/** \brief Process entire frame for test data output
		* \param[in/out] _frame_buf: buffer of entire frame.
		* \return none
		*/
		//void processFrameBufferTestData(benewake::BwTestDataG66Frame::Ptr& _frame_buf);

		//void decodeTestDataFromBuffer(benewake::BwTestDataG66* _test_data, unsigned char* _buffer, int _data_offset);

		//void saveDataThread();

		//void openDataOutputStream();

		//void saveSphereCoordData(BwSphereCoordG66Frame::Ptr _data);

		//void saveTestDataOnceFrame(BwTestDataG66Frame::Ptr _test_data);

		BwTestDataG66 src_data_;

		std::shared_ptr<benewake::MsgQueue<benewake::BwSphereCoordG66Frame::Ptr>> sphere_coord_frame_data_queue_ = NULL;
		std::shared_ptr<benewake::MsgQueue<benewake::BwTestDataG66Frame::Ptr>> test_frame_data_queue_ = NULL;
	};
}

#endif