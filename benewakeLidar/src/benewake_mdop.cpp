#include "benewake_mdop.h"
#include <float.h>

static float kAngRes_AD2 = 0.05; // angle resoluton of channel along vertical
static float kAngRes_G66 = 0.22; // angle resoluton of channel along vertical

static float ang2Rad(float ang)
{
	return ang * benewake::kPI / 180.0;
}
benewake::MDOPProtocol::MDOPProtocol(std::string _local_ip, int _local_port, std::string _remote_ip, int _remote_port, UDPType _udp_type) : 
	Protocol(_local_ip, _local_port, _remote_ip, _remote_port, _udp_type)
{
	point_cloud_buffer_ = std::make_shared<BwPointCloud>();
	delay_time_ = 0;
	pkg_loss_rate_ = 0;
	frame_loss_rate_ = 0;
	enable_angle_calib_ = false;
	enable_flag_dataforward_ = false;
	signal_message_ = new char[5];
	memset(signal_message_, 0, 5 * sizeof(char));

#ifdef _WIN32
	occup_ = CreateSemaphore(NULL, 1, 1, NULL);
#else
	sem_init(&occup_, 0, 1);
#endif // _WIN32
	data_queue_ = std::make_shared<benewake::MsgQueue<benewake::BwPointCloud::Ptr>>("pointcloud", 5);
}

benewake::MDOPProtocol::~MDOPProtocol()
{
	thread_run_ = false;
	delete[] signal_message_;
	signal_message_ = nullptr;
}

void benewake::MDOPProtocol::mdop_cancel_point_cloud_callback()
{
	if (callback_func_ != NULL)
		callback_func_ = NULL;
	if (callback_pointer_ != NULL)
		callback_pointer_ = NULL;
	callback_enable_ = false;
}

void benewake::MDOPProtocol::mdop_enable_AB_Frame(bool _enable, int _frame_rows)
{
	AB_frame_flag_ = _enable;
	if (AB_frame_flag_)
		frame_rows_half_ = _frame_rows / 2;
	else
		frame_rows_half_ = 0;
}

int benewake::MDOPProtocol::mdop_enable_decode(bool _enable)
{
	if (_enable)
	{
		if (thread_run_)
		{
			return STATUS_OK;
		}
		thread_run_ = true;

		if (protocol_type_ == benewake::BwProtocolType::CLIENT)
		{
			decode_thread_ = new std::thread(&MDOPProtocol::clientDataDecode, this);
		}
		else if (protocol_type_ == benewake::BwProtocolType::SPHERE)
		{
			decode_thread_ = new std::thread(&MDOPProtocol::sphereDataDecode, this);
		}
		else if (protocol_type_ == benewake::BwProtocolType::TEST)
		{
			decode_thread_ = new std::thread(&MDOPProtocol::testDataDecode, this);
		}
		return STATUS_OK;
	}
	else
	{
		if (thread_run_)
		{
			thread_run_ = false;
			new_frame_ = false;
			if (protocol_type_ != benewake::BwProtocolType::TEST)
				decode_thread_->join();
		}
		return STATUS_OK;
	}
}

int benewake::MDOPProtocol::mdop_enable_gray(bool _enable)
{
	gray_enable_ = _enable;
	return STATUS_OK;
}

int benewake::MDOPProtocol::mdop_enable_rgb(bool _enable)
{
	rgb_enable_ = _enable;
	return STATUS_OK;
}

benewake::BwPointCloud::Ptr benewake::MDOPProtocol::mdop_front_time_window_data()
{
	return data_queue_->front();
}

void benewake::MDOPProtocol::mdop_front_view_filter(BwPointCloud::Ptr& _cloud)
{
	BwPointCloud::Ptr* front_view = new BwPointCloud::Ptr[FVF_r_bind_ * FVF_c_bind_];
	for (int i = 0; i < FVF_r_bind_ * FVF_c_bind_; ++i)
	{
		front_view[i].reset(new BwPointCloud());
	}

	for (size_t i = 0; i < _cloud->points.size(); ++i)
	{
		BwPoint pt = _cloud->points[i];
		float h_angle = atan2(pt.y, pt.x) * 180.0 / kPI;
		int h_idx = FVF_c_bind_ - (static_cast<int>(h_angle / FVF_h_resolution_ / FVF_bind_) + FVF_c_bind_ / 2);
		float v_angle = atan2(pt.z, sqrt(pt.x * pt.x + pt.y * pt.y)) * 180.0 / kPI;
		int v_idx = FVF_r_bind_ - (static_cast<int>(v_angle / FVF_v_resolution_ / FVF_bind_) + FVF_r_bind_ / 2);
		if (h_idx >= 0 && h_idx < FVF_c_bind_ && v_idx >= 0 && v_idx < FVF_r_bind_)
		{
			front_view[v_idx * FVF_c_bind_ + h_idx]->points.push_back(pt);
		}
	}

	int lidar_range = 5000;
	_cloud->points.clear();
	for (size_t i = 0; i < FVF_r_bind_ * FVF_c_bind_; ++i)
	{
		int num_points = static_cast<int>(front_view[i]->points.size());
		if (num_points == 0)
		{
			continue;
		}

		std::vector<float> distances(num_points);
		std::vector<int> histogram(lidar_range);
		for (int j = 0; j < num_points; ++j)
		{
			BwPoint pt = front_view[i]->points[j];
			float d = static_cast<float>(sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) * 10);
			distances[j] = d;
			int idx = (std::min)(static_cast<int>(d + 0.5), lidar_range - 1);
			histogram[idx]++;
		}

		std::vector<int> maxima;
		auto max_iter = std::max_element(histogram.begin(), histogram.end());
		int max_idx = static_cast<int>(std::distance(histogram.begin(), max_iter));
		maxima.push_back(max_idx);

		for (auto max_idx : maxima)
		{
			int tolerance = 0;
			if (max_idx < 300)
				tolerance = static_cast<int>(max_idx * 0.03);
			else if (max_idx < 500)
				tolerance = static_cast<int>(max_idx * 0.10);
			else
				tolerance = 200;

			for (int j = 0; j < num_points; ++j)
			{
				BwPoint pt = front_view[i]->points[j];
				float d = distances[j];
				if (d < 300 && pt.intensity == 0)
					continue;
				int idx = static_cast<int>(d + 0.5);
				if (idx > (max_idx - tolerance) && idx < (max_idx + tolerance))
				{
					_cloud->points.push_back(pt);
				}
			}
		}
	}

	for (int i = 0; i < FVF_r_bind_ * FVF_c_bind_; ++i)
	{
		front_view[i].reset();
	}
	delete[] front_view;
}

void benewake::MDOPProtocol::mdop_get_data_transmission_state(uint32_t& _delay_time, float& _pkg_loss_rate, float& _frame_loss_rate)
{
	if (delay_time_ >= UINT32_MAX)
		_delay_time = UINT32_MAX;
	else
		_delay_time = delay_time_;
	_pkg_loss_rate = pkg_loss_rate_;
	_frame_loss_rate = frame_loss_rate_;
}

int benewake::MDOPProtocol::mdop_get_frame(BwPointCloud::Ptr& _ptr, int & _frame_id, int _wait_ms)
{
	_ptr.reset();
	if (!thread_run_) // thread is not running
	{
		if (rgb_enable_)
			_ptr = std::make_shared<BwPointCloudRGB>();
		else if (gray_enable_)
			_ptr = std::make_shared<BwPointCloudGray>();
		else
			_ptr = std::make_shared<BwPointCloud>();
		return STATUS_FAIL;
	}
	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
	std::chrono::system_clock::time_point stop;
	while (!new_frame_)
	{
		if (!thread_run_) // thread stop when waiting
		{
			if (rgb_enable_)
				_ptr = std::make_shared<BwPointCloudRGB>();
			else if (gray_enable_)
				_ptr = std::make_shared<BwPointCloudGray>();
			else
				_ptr = std::make_shared<BwPointCloud>();
			return STATUS_FAIL;
		}
#ifdef _WIN32
		Sleep(1);
#else
		usleep(1000);
#endif // _WIN32
		stop = std::chrono::system_clock::now();
		auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
		if (dur.count() > _wait_ms)
		{
#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Wait for point cloud time out!" << std::endl;
#endif // DEBUG_INFO
			if (rgb_enable_)
				_ptr = std::make_shared<BwPointCloudRGB>();
			else if (gray_enable_)
				_ptr = std::make_shared<BwPointCloudGray>();
			else
				_ptr = std::make_shared<BwPointCloud>();
			return STATUS_TIME_OUT;
		}
	}

#ifdef _WIN32
	WaitForSingleObject(occup_, INFINITE);
#else
	sem_wait(&occup_);
#endif // _WIN32
	_ptr = point_cloud_buffer_;
	_frame_id = frame_id_;
	new_frame_ = false;
#ifdef _WIN32
	ReleaseSemaphore(occup_, 1, NULL);
#else
	sem_post(&occup_);
#endif // _WIN32

	if (enable_local_transform_)
	{
		for (int i = 0; i < _ptr->points.size(); ++i)
		{
			benewake::BwPoint pt = _ptr->points[i];
			_ptr->points[i].x = transform_matrix_[0] * pt.x + transform_matrix_[1] * pt.y + transform_matrix_[2] * pt.z + transform_matrix_[3];
			_ptr->points[i].y = transform_matrix_[4] * pt.x + transform_matrix_[5] * pt.y + transform_matrix_[6] * pt.z + transform_matrix_[7];
			_ptr->points[i].z = transform_matrix_[8] * pt.x + transform_matrix_[9] * pt.y + transform_matrix_[10] * pt.z + transform_matrix_[11];
		}
	}

	if (_ptr->points.size() < 1000)
	{
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Points amount is anomalous!" << std::endl;
#endif // DEBUG_INFO
		return STATUS_ANOMALOUS;
	}

	return STATUS_OK;
}

void benewake::MDOPProtocol::mdop_get_front_view_filter_parameters(int& _rows, int& _cols, int& _bind, int& _n_scan, float& _h_resolution, float& _v_resolution)
{
	_rows = FVF_rows_;
	_cols = FVF_cols_;
	_bind = FVF_bind_;
	_n_scan = FVF_n_scan_;
	_h_resolution = FVF_h_resolution_;
	_v_resolution = FVF_v_resolution_;
}

void benewake::MDOPProtocol::mdop_get_signal_message(char* _msg, uint16_t& _time_offset)
{
#ifdef _WIN32
	WaitForSingleObject(occup_, INFINITE);
#else
	sem_wait(&occup_);
#endif // _WIN32
	if (signal_message_ != nullptr)
		memcpy(_msg, signal_message_, 5 * sizeof(char));
	_time_offset = signal_time_offset_;
#ifdef _WIN32
	ReleaseSemaphore(occup_, 1, NULL);
#else
	sem_post(&occup_);
#endif // _WIN32
}

int benewake::MDOPProtocol::mdop_get_single_point_data(float* _data, int _wait_ms)
{
	if (!new_data_)
	{
		if (!thread_run_)
			return STATUS_FAIL;
		std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
		std::chrono::system_clock::time_point stop;
		while (!new_frame_ && thread_run_)
		{
#ifdef _WIN32
			Sleep(50);
#else
			usleep(50000);
#endif // _WIN32
			stop = std::chrono::system_clock::now();
			auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
			if (dur.count() > _wait_ms)
			{
#ifdef DEBUG_INFO
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Wait for signal data time out!" << std::endl;
#endif // DEBUG_INFO
				return STATUS_TIME_OUT;
			}
	}
}
#ifdef _WIN32
	WaitForSingleObject(occup_, INFINITE);
#else
	sem_wait(&occup_);
#endif // _WIN32
	memcpy(_data, single_point_data_, 8 * sizeof(float));
	new_data_ = false;
#ifdef _WIN32
	ReleaseSemaphore(occup_, 1, NULL);
#else
	sem_post(&occup_);
#endif // _WIN32
	return STATUS_OK;
}

int benewake::MDOPProtocol::mdop_get_sphere_coord_data(std::vector<BwSphereCoord>& _sphere_data, int _wait_ms)
{
	_sphere_data.clear();
	if (!new_frame_)
	{
		if (!thread_run_)
		{
			return STATUS_FAIL;
		}
		std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
		std::chrono::system_clock::time_point stop;
		while (!new_frame_ && thread_run_)
		{
#ifdef _WIN32
			Sleep(50);
#else
			usleep(50000);
#endif // _WIN32
			stop = std::chrono::system_clock::now();
			auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
			if (dur.count() > _wait_ms)
			{
#ifdef DEBUG_INFO
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Wait for spherical coordinate data time out!" << std::endl;
#endif // DEBUG_INFO
				return STATUS_TIME_OUT;
			}
		}
	}

#ifdef _WIN32
	WaitForSingleObject(occup_, INFINITE);
#else
	sem_wait(&occup_);
#endif // _WIN32
	_sphere_data = sphere_coord_out_buffer_;
	new_frame_ = false;
#ifdef _WIN32
	ReleaseSemaphore(occup_, 1, NULL);
#else
	sem_post(&occup_);
#endif // _WIN32

	return STATUS_OK;
}

int benewake::MDOPProtocol::mdop_get_test_efficient_percent(float& _percent, int _wait_ms)
{
	if (!new_frame_)
	{
		if (!thread_run_)
		{
			_percent = 0;
			return STATUS_FAIL;
		}
		std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
		std::chrono::system_clock::time_point stop;
		while (!new_frame_ && thread_run_)
		{
#ifdef _WIN32
			Sleep(50);
#else
			usleep(50000);
#endif // _WIN32
			stop = std::chrono::system_clock::now();
			auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
			if (dur.count() > _wait_ms)
			{
#ifdef DEBUG_INFO
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::cout << "Wait for test data time out!" << std::endl;
#endif // DEBUG_INFO
				_percent = 0;
				return STATUS_TIME_OUT;
			}
		}
	}

#ifdef _WIN32
	WaitForSingleObject(occup_, INFINITE);
#else
	sem_wait(&occup_);
#endif // _WIN32
	_percent = eff_percent_;
	new_frame_ = false;
#ifdef _WIN32
	ReleaseSemaphore(occup_, 1, NULL);
#else
	sem_post(&occup_);
#endif // _WIN32

	return STATUS_OK;
}

int benewake::MDOPProtocol::mdop_log(bool _enable)
{
	log_enable_ = _enable;
	return STATUS_OK;
}

benewake::BwPointCloud::Ptr benewake::MDOPProtocol::mdop_pop_front_time_window_data()
{
	BwPointCloud::Ptr cloud = data_queue_->popMsg();
	if (cloud == NULL)
		cloud.reset(new BwPointCloud());
	return cloud;
}

void benewake::MDOPProtocol::mdop_pop_time_window_data()
{
	data_queue_->pop();
}

void benewake::MDOPProtocol::mdop_regist_point_cloud_callback(PointCloudCallbackFunc _func, void * _pData)
{
	callback_func_ = _func;
	callback_pointer_ = _pData;
	callback_enable_ = true;
}

int benewake::MDOPProtocol::mdop_save_data(std::string _save_path, std::string _save_file_name, int _frame_num)
{
	save_flag_ = true;
	save_path_ = _save_path;
	save_file_name_ = _save_file_name;
	save_num_ = _frame_num;
	is_dominant_wave_mode_ = false;
	return STATUS_OK;
}

int benewake::MDOPProtocol::mdop_save_data(bool _is_dominant_wave_mode, std::string _save_path, std::string _save_file_name, int _frame_num)
{
	save_flag_ = true;
	save_path_ = _save_path;
	save_file_name_ = _save_file_name;
	save_num_ = _frame_num;
	is_dominant_wave_mode_ = _is_dominant_wave_mode;
	return STATUS_OK;
}

void benewake::MDOPProtocol::mdop_set_front_view_filter_parameters(int _rows, int _cols, int _bind, int _n_scan, float _h_resolution, float _v_resolution)
{
	FVF_rows_ = _rows;
	FVF_cols_ = _cols;
	FVF_bind_ = _bind;
	FVF_n_scan_ = _n_scan;
	FVF_h_resolution_ = _h_resolution;
	FVF_v_resolution_ = _v_resolution;
	FVF_r_bind_ = FVF_rows_ / FVF_bind_;
	FVF_c_bind_ = FVF_cols_ / FVF_bind_;
}

void benewake::MDOPProtocol::mdop_set_local_transform(bool _enable, float _roll, float _pitch, float _yaw, float _x, float _y, float _z)
{
	enable_local_transform_ = _enable;
	if (enable_local_transform_)
	{
		double cr = cos(_roll);
		double sr = sin(_roll);
		double cp = cos(_pitch);
		double sp = sin(_pitch);
		double cy = cos(_yaw);
		double sy = sin(_yaw);

		transform_matrix_[0] = cy * cp;
		transform_matrix_[1] = cy * sp * sr - sy * cr;
		transform_matrix_[2] = cy * sp * cr + sy * sr;
		transform_matrix_[3] = _x;
		transform_matrix_[4] = sy * cp;
		transform_matrix_[5] = sy * sp * sr + cy * cr;
		transform_matrix_[6] = sy * sp * cr - cy * sr;
		transform_matrix_[7] = _y;
		transform_matrix_[8] = -sp;
		transform_matrix_[9] = cp * sr;
		transform_matrix_[10] = cp * cr;
		transform_matrix_[11] = _z;
		transform_matrix_[15] = 1.0;
	}
}

void benewake::MDOPProtocol::mdop_set_multicast_ip(std::string _multicast_ip, int _multicast_port)
{
	pudp_->setMutlicatIP(_multicast_ip, _multicast_port);
}

bool benewake::MDOPProtocol::mdop_set_time_window(bool _enable_time_window, uint32_t _interval)
{
	if (_interval == 0)
		return false;
	enable_time_windows_ = _enable_time_window;
	frame_interval_ = _interval;
	data_queue_->setPopWaitTime(_interval);
	return true;
}

void benewake::MDOPProtocol::mdop_set_X2_long_range_version(bool _flag)
{
	X2_long_range_version_ = _flag;
}

int benewake::MDOPProtocol::mdop_time_window_data_queue_size()
{
	return data_queue_->size();
}

int benewake::MDOPProtocol::mdop_switch_protocol(BwProtocolType _protocol)
{
	protocol_type_ = _protocol;
	return STATUS_OK;
}

int benewake::MDOPProtocol::open()
{
	return MDOPProtocol::open(connection_type_);
}

int benewake::MDOPProtocol::open(UDPType _udp_type)
{
	sphere_coord_buffer_.clear();
	connection_type_ = _udp_type;
	int stat = Protocol::open(connection_type_);
	if (stat != STATUS_OK)
		return stat;
	if (connection_type_ == benewake::UDPType::NORMAL || connection_type_ == benewake::UDPType::LOCAL_BIND)
	{
		stat = pudp_->sendData(mdop, 4);
		if (stat != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Send MDOP string failed!" << std::endl;
			return stat;
		}
		//unsigned char recv[4];
		//stat = pudp_->recvData(recv, 4);
		//if (stat != STATUS_OK)
		//{
		//	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		//	std::cout << "Receive MDOP string failed!" << std::endl;
		//	return stat;
		//}
		//if (recv[0] != 0x4d || recv[1] != 0x44 || recv[2] != 0x4f || recv[3] != 0x50)
		//{
		//	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		//	std::cout << "MDOP string checking failed!" << std::endl;
		//	return STATUS_FAIL;
		//}
	}
	return STATUS_OK;
}

int benewake::MDOPProtocol::mdop_enable_data_forwarding(bool _enabel, std::string _forward_ip, int _forward_port)
{
    if (_enabel)
    {
        dataforward_udp_ = std::unique_ptr<Udp>(new Udp(local_ip_, local_port_, _forward_ip, _forward_port));

        dataforward_udp_->disconnect();
        int status = dataforward_udp_->connect();

		if (status == STATUS_OK)
		{
			enable_flag_dataforward_ = true;
		}
        
        return status;
    }
    else
    {
        enable_flag_dataforward_ = false;

        dataforward_udp_->disconnect();

        return STATUS_OK; 
    }
}


void benewake::MDOPProtocol::clientDataDecode()
{
	BwPointCloud::Ptr point_cloud_tmp;
	BwPoint pt;
	BwRGB rgb;
	std::vector<uint8_t> gray_vector;
	std::vector<BwRGB> rgb_vector;

	BwPointCloud::Ptr point_cloud_backup;
	std::vector<uint8_t> gray_vector_backup;
	std::vector<BwRGB> rgb_vector_backup;
	long long dt_time = 0;

	int point_bytes = 0;
	if (rgb_enable_)
	{
		point_cloud_buffer_.reset();
		point_cloud_buffer_ = std::make_shared<BwPointCloudRGB>();
		point_cloud_tmp = std::make_shared<BwPointCloudRGB>();
		point_cloud_backup = std::make_shared<BwPointCloudRGB>();
		point_bytes = 15;
	}
	else if (gray_enable_)
	{
		point_cloud_buffer_.reset();
		point_cloud_buffer_ = std::make_shared<BwPointCloudGray>();
		point_cloud_tmp = std::make_shared<BwPointCloudGray>();
		point_cloud_backup = std::make_shared<BwPointCloudGray>();
		point_bytes = 13;
	}
	else
	{
		point_cloud_buffer_.reset();
		point_cloud_buffer_ = std::make_shared<BwPointCloud>();
		point_cloud_tmp = std::make_shared<BwPointCloud>();
		point_cloud_backup = std::make_shared<BwPointCloud>();
		point_bytes = 12;
	}
	benewake::BwPoint time_pt;
	point_cloud_tmp->points.push_back(time_pt);
	bool first_frame_point0_is_set = false;

	std::chrono::system_clock::time_point static_start, static_stop;
	uint32_t lost_pkgs = 0, static_start_pkg = 0, lost_frames = 0, static_start_frame = 0;

	unsigned char *data = new unsigned char[PROTOCOL_DATA_PACKAGE_MAX_LENGTH];
	char *signal_msg = new char[4];
	int recv_size = 0, total_bytes = 0;
	uint32_t pkg_count = 0, pkg_count_last = 0, time_s = 0, time_ns = 0, signal_msg_time_offset = 0;
	int time_offset = 0;
	bool use_gps_src = false;
	uint32_t diff = 0, total_lost_package = 0, check_sum = 0;
	uint16_t nFrame = 0, nFrame_last = 0, nLine = 0, nLine_last = 0, nPoint = 0, uval16 = 0, protocol_version = 0;
	int16_t dist16 = 0;
	uint8_t uval8 = 0;
	int year = 0, month = 0, day = 0, hour = 0, minute = 0;
	bool new_frame_received = false, first_pkg = true;
	long long frame_win_num = 0;
	long long last_frame_win_num = 0;
	int current_pkg_num = 0;

	pkg_loss_rate_ = 0;
	frame_loss_rate_ = 0;
	static_start = std::chrono::system_clock::now();

	int dataforward_status = 0; 
	
	while (thread_run_)
	{
		recv_size = pudp_->recvSomeData(data, PROTOCOL_DATA_PACKAGE_MAX_LENGTH, 3);
		if (recv_size < 0)
		{
#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Receive point cloud data failed!" << std::endl;
#endif // DEBUG_INFO
		}
		else if (recv_size < 44)
		{
#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Point cloud data package size error! The package should be larger than 44 bytes but only " 
				<< recv_size << " bytes received." << std::endl;
#endif // DEBUG_INFO
		}
		else
		{
			if (data[0] == 0x42 && data[1] == 0x57 && data[2] == PRODUCT_ID_X2 && data[3] == PROTOCOL_ID_MDOP 
				&& data[recv_size - 2] == 0x00 && data[recv_size - 1] == 0xff)
			{
				if (enable_flag_dataforward_)
				{
					dataforward_status = dataforward_udp_->sendData(data, recv_size);
					if (dataforward_status != STATUS_OK)
					{
						std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
						std::cout << "Point cloud data forwarding failed!" << std::endl;
					}
				}

				memcpy(&protocol_version, &data[PROTOCOL_DATA_PACKAGE_VERSION_OFFSET], sizeof(uint16_t));
				memcpy(&pkg_count, &data[PROTOCOL_DATA_PACKAGE_COUNT_OFFSET], sizeof(uint32_t));
				memcpy(&nFrame, &data[PROTOCOL_DATA_FRAME_COUNT_OFFSET], sizeof(uint16_t));
				memcpy(&nLine, &data[PROTOCOL_DATA_LINE_NUM_OFFSET], sizeof(uint16_t));
				memcpy(&nPoint, &data[PROTOCOL_DATA_POINT_COUNT_OFFSET], sizeof(uint16_t));
				memcpy(&time_s, &data[PROTOCOL_X_DATA_TIME_S_OFFSET], sizeof(uint32_t));
				memcpy(&time_ns, &data[PROTOCOL_X_DATA_TIME_NS_OFFSET], sizeof(uint32_t));
				memcpy(signal_msg, &data[PROTOCOL_X_DATA_SIGNAL_MESSAGE], 4 * sizeof(char));
				memcpy(&signal_msg_time_offset, &data[PROTOCOL_X_DATA_SIGNAL_TIME_OFFSET], sizeof(uint16_t));
				
				use_gps_src = false;
				if ((time_s & 0x80000000) == 0x80000000)
				{
					year = (time_s & 0x3f000000) >> 24;
					month = (time_s & 0x00fc0000) >> 18;
					day = (time_s & 0x0003f000) >> 12;
					hour = (time_s & 0x00000fc0) >> 6;
					minute = (time_s & 0x0000003f);
					
					utc_time_t utcTime;
					utcTime.year = year + 2000;
					utcTime.month = month;
					utcTime.day = day;
					utcTime.hour = hour;
					utcTime.minute = minute;
					utcTime.second = time_ns / 1000000 % 100;
					time_s = covUTC2UnixTimestamp(&utcTime);

					time_ns = time_ns % 1000000 * 1000;
					use_gps_src = true;
				}

				total_bytes = nPoint * point_bytes + 44;
				if (total_bytes != recv_size)
				{
#ifdef DEBUG_INFO
					//std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
					//std::cout << "Point cloud data package size error! The package should be " << total_bytes << " bytes but "
					//	<< recv_size << " bytes received." << std::endl;
					std::cout << "INFO: package size " << total_bytes << " / " << recv_size << std::endl;
#endif // DEBUG_INFO
					continue;
				}
				
				check_sum = check_sum_with_protocol_version(protocol_version, data, recv_size - 6);
				if (memcmp(&check_sum, &data[recv_size - 6], 4) != 0)
				{
#ifdef DEBUG_INFO
					uint32_t recv_check_sum;
					memcpy(&recv_check_sum, &data[recv_size - 6], sizeof(uint32_t));
					//std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
					//std::printf("Point cloud data package check sum error! Check sum should be  0x%08x but receive 0x%08x!\n", 
					//	check_sum, recv_check_sum);
					std::printf("INFO: point cloud data package check sum 0x%08x<0x%08x>\n", check_sum, recv_check_sum);
#endif // DEBUG_INFO
					continue;
				}

				if (nFrame == 0 && !first_frame_point0_is_set)
				{
					if (enable_time_windows_)
					{
						point_cloud_tmp->points[0].timestamp_s = 0;
						point_cloud_tmp->points[0].timestamp_ns = 0;
						first_frame_point0_is_set = true;
					}
					else
					{
						point_cloud_tmp->points[0].timestamp_s = time_s;
						point_cloud_tmp->points[0].timestamp_ns = time_ns;
						first_frame_point0_is_set = true;
					}
				}

				if (first_pkg)
				{
					static_start_pkg = pkg_count;
					static_start_frame = nFrame;
					pkg_count_last = pkg_count;
					nFrame = nFrame_last;
					first_pkg = false;
				}

				diff = pkg_count - pkg_count_last;
				if (diff > 1)
				{
					total_lost_package += (diff - 1);
					lost_pkgs += (diff - 1);
				}
				pkg_count_last = pkg_count;

				new_frame_received = false;
				diff = uint16_t(nFrame - nFrame_last);
				if (!enable_time_windows_ && 
					(diff >= 1 || (AB_frame_flag_ && nLine_last < frame_rows_half_ && nLine >= frame_rows_half_) || (AB_frame_flag_ && nLine < nLine_last)))
					new_frame_received = true;
				else if (enable_time_windows_)
				{
					if (recv_size > 44) {
						memcpy(&time_offset, &data[38 + 8], sizeof(int));
						if (use_gps_src)
							time_offset *= 1000;
						dt_time = (long long)time_s * 1000.0 + time_ns * 0.000001 + time_offset * 0.000001;
					}
					else
						dt_time = (long long)time_s * 1000.0 + time_ns * 0.000001;
					frame_win_num = dt_time / frame_interval_;
					current_pkg_num++;
					if (frame_win_num > last_frame_win_num) // when package is empty, dt may be located at last frame and {frame_win_num < last_frame_win_num} may occur in this situation
					{
						new_frame_received = true;
						last_frame_win_num = frame_win_num;
						current_pkg_num = 0;
					}
					else if (current_pkg_num > MDOP_DATA_MAX_PKG_NUM)
					{
						new_frame_received = true;
						last_frame_win_num = frame_win_num;
						current_pkg_num = 0;
#ifdef DEBUG_INFO
						std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
						std::cout << "Abnormal: timestamp exception detected!" << std::endl;
#endif // DEBUG_INFO
					}
				}
				if (new_frame_received)
				{
					if (rgb_enable_)
					{
						auto ptr = std::dynamic_pointer_cast<BwPointCloudGray>(point_cloud_tmp);
						ptr->gray = gray_vector;
						gray_vector.clear();
					}
					else if (gray_enable_)
					{
						auto ptr = std::dynamic_pointer_cast<BwPointCloudRGB>(point_cloud_tmp);
						ptr->rgb = rgb_vector;
						rgb_vector.clear();
					}
#ifdef _WIN32
					WaitForSingleObject(occup_, INFINITE);
#else
					sem_wait(&occup_);
#endif // _WIN32
					point_cloud_buffer_.reset();
					point_cloud_buffer_ = point_cloud_tmp;
					frame_id_ = nFrame_last;
					new_frame_ = true;
					if (signal_message_ != nullptr)
						memcpy(signal_message_, signal_msg, 4 * sizeof(char));
					signal_time_offset_ = signal_msg_time_offset;
#ifdef _WIN32
					ReleaseSemaphore(occup_, 1, NULL);
#else
					sem_post(&occup_);
#endif // _WIN32
					// push to time window data queue
					data_queue_->pushMsg(point_cloud_tmp);

					if (rgb_enable_)
						point_cloud_tmp.reset(new BwPointCloudRGB());
					else if (gray_enable_)
						point_cloud_tmp.reset(new BwPointCloudGray());
					else
						point_cloud_tmp.reset(new BwPointCloud());
					point_cloud_tmp->points.reserve(1100000);
					if (enable_time_windows_) 
					{
						time_pt.timestamp_s = frame_win_num * frame_interval_ / 1000;
						time_pt.timestamp_ns = (frame_win_num * frame_interval_ % 1000) * 1000000;
						point_cloud_tmp->points.push_back(time_pt);

						point_cloud_tmp->points.insert(point_cloud_tmp->points.end(), point_cloud_backup->points.begin(), point_cloud_backup->points.end());
						point_cloud_backup->points.clear();
						if (gray_enable_)
						{
							gray_vector = gray_vector_backup;
							gray_vector_backup.clear();
						}
						else if (rgb_enable_)
						{
							rgb_vector = rgb_vector_backup;
							rgb_vector_backup.clear();
						}
					}
					else
					{
						benewake::BwPoint time_pt;
						time_pt.timestamp_s = time_s;
						time_pt.timestamp_ns = time_ns;
						point_cloud_tmp->points.push_back(time_pt);
					}

					if (callback_enable_)
					{
						callback_func_(point_cloud_buffer_, frame_id_, callback_pointer_);
					}


					if (total_lost_package > 0)
					{
						//std::cout << "Package lost detected! " << total_lost_package << " package(s) lost in last frame." << std::endl;
						std::cout << "INFO: PL in last frame " << total_lost_package << std::endl;
						total_lost_package = 0;
					}
					if (diff > 1)
					{
						std::cout << "INFO: FL " << (diff - 1) << std::endl;
						lost_frames += (diff - 1);
					}
				}
				nFrame_last = nFrame;
				nLine_last = nLine;

				for (int i = 38; i < (recv_size - 6); i += point_bytes)
				{
					memcpy(&uval8, &data[i + 6], sizeof(uint8_t));
					if (X2_long_range_version_)
					{
						memcpy(&uval16, &data[i], sizeof(uint16_t));
						pt.x = ((float)uval16) * 0.01;
						memcpy(&uval16, &data[i + 2], sizeof(uint16_t));
						if ((uval8 & 0x08) == 0x08)
							pt.y = -((float)(0xffff - uval16)) * 0.01;
						else
							pt.y = ((float)uval16) * 0.01;
					}
					else
					{
						memcpy(&dist16, &data[i], sizeof(int16_t));
						pt.x = dist16 * 0.01;
						memcpy(&dist16, &data[i + 2], sizeof(int16_t));
						pt.y = dist16 * 0.01;
					}
					memcpy(&dist16, &data[i + 4], sizeof(int16_t));
					pt.z = dist16 * 0.01;
					// memcpy(&uval8, &data[i + 6], sizeof(uint8_t)); // move to code above for efficiency
					pt.roi = uval8 & 0x03;
					uint8_t ch = uval8 & 0x04;
					if (ch == 0x00)
					{
						pt.channel = 1;
					}
					else if (ch == 0x04)
					{
						pt.channel = 2;
					}
					else
					{
						pt.channel = 0;
					}

					if ((uval8 & 0x10) == 0x10)
						pt.y = pt.y + 0.0025;
					if ((uval8 & 0x20) == 0x20)
						pt.y = pt.y + 0.005;
					if ((uval8 & 0x40) == 0x40)
						pt.z = pt.z + 0.0025;
					if ((uval8 & 0x80) == 0x80)
						pt.z = pt.z + 0.005;
					
					memcpy(&pt.intensity, &data[i + 7], sizeof(uint8_t));
					memcpy(&time_offset, &data[i + 8], sizeof(int));
					if (use_gps_src)
						time_offset *= 1000;
					pt.timestamp_s = time_s;

					if (time_offset < 0 && abs(time_offset) > time_ns)
					{
						pt.timestamp_ns = 1000000000 + (int)time_ns + time_offset;
						pt.timestamp_s--;
					}
					else
					{
						pt.timestamp_ns = time_ns + time_offset;
						if (pt.timestamp_ns >= 1000000000)
						{
							pt.timestamp_s += pt.timestamp_ns / 1000000000;
							pt.timestamp_ns %= 1000000000;
						}
					}

					if (new_frame_received && i == 38)
					{
						std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
						std::chrono::nanoseconds ns = tp.time_since_epoch();
						delay_time_ = ns.count() / 1000 - (long long)pt.timestamp_s * 1000000 - pt.timestamp_ns * 0.001;
					}
					
					pt.row = nLine;

					if (enable_time_windows_ &&
						((long long)pt.timestamp_s * 1000.0 + pt.timestamp_ns * 0.000001) >= (frame_win_num + 1) * frame_interval_)
					{
						point_cloud_backup->points.push_back(pt);
						if (gray_enable_)
						{
							memcpy(&uval8, &data[i + 12], sizeof(uint8_t));
							gray_vector_backup.push_back(uval8);
						}
						else if (rgb_enable_)
						{
							memcpy(&uval8, &data[i + 12], sizeof(uint8_t));
							rgb.r = uval8;
							memcpy(&uval8, &data[i + 13], sizeof(uint8_t));
							rgb.g = uval8;
							memcpy(&uval8, &data[i + 14], sizeof(uint8_t));
							rgb.b = uval8;
							rgb_vector_backup.push_back(rgb);
						}
					}
					else
					{
						point_cloud_tmp->points.push_back(pt);
						if (gray_enable_)
						{
							memcpy(&uval8, &data[i + 12], sizeof(uint8_t));
							gray_vector.push_back(uval8);
						}
						else if (rgb_enable_)
						{
							memcpy(&uval8, &data[i + 12], sizeof(uint8_t));
							rgb.r = uval8;
							memcpy(&uval8, &data[i + 13], sizeof(uint8_t));
							rgb.g = uval8;
							memcpy(&uval8, &data[i + 14], sizeof(uint8_t));
							rgb.b = uval8;
							rgb_vector.push_back(rgb);
						}
					}
				}
			}
			else
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::printf("Point cloud data package:%u header/tail error: 0x%02x 0x%02x 0x%02x 0x%02x ... 0x%02x 0x%02x!\n", 
					pkg_count, data[0], data[1], data[2], data[3], data[recv_size - 2], data[recv_size - 1]);
			}
		}

		static_stop = std::chrono::system_clock::now();
		auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(static_stop - static_start);
		if (dur.count() > 10000)
		{
			// update data statics
			if (pkg_count < static_start_pkg)
				pkg_loss_rate_ = (float)lost_pkgs / float((UINT32_MAX - static_start_pkg) + pkg_count) * 100.0;
			else if (pkg_count == static_start_pkg)
				pkg_loss_rate_ = 100.0;
			else
				pkg_loss_rate_ = (float)lost_pkgs / float(pkg_count - static_start_pkg) * 100.0;
			lost_pkgs = 0;
			static_start_pkg = pkg_count;


			if (nFrame < static_start_frame)
				frame_loss_rate_ = (float)lost_frames / float((UINT16_MAX - static_start_frame) + nFrame) * 100.0;
			else if (nFrame == static_start_frame)
				frame_loss_rate_ = 100.0;
			else
				frame_loss_rate_ = (float)lost_frames / float(nFrame - static_start_frame) * 100.0;
			lost_frames = 0;
			static_start_frame = nFrame;

			static_start = static_stop;
		}
	}
	delete[] data;
}

void benewake::MDOPProtocol::sphereDataDecode()
{
	sphere_coord_buffer_.clear();
	sphere_coord_buffer_.reserve(1100000);
	sphere_coord_all_buffer_.clear();
	sphere_coord_all_buffer_.reserve(3300000);
	BwSphereCoord pt;
	BwSphereCoordAll pt_all;
	unsigned char *data = new unsigned char[PROTOCOL_DATA_PACKAGE_MAX_LENGTH];
	int recv_size = 0, total_bytes = 0, point_bytes = 60;
	uint32_t pkg_count = 0, pkg_count_last = 0, time_s = 0, time_ns = 0, time_offset = 0;
	uint32_t diff = 0, total_lost_package = 0, check_sum = 0, valu32 = 0;
	uint16_t nFrame = 0, nFrame_last = 0, nLine = 0, nPoint = 0, valu16 = 0;
	while (thread_run_)
	{
		recv_size = pudp_->recvSomeData(data, PROTOCOL_DATA_PACKAGE_MAX_LENGTH, 10);
		if (recv_size < 0)
		{
#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Receive calibration data failed!" << std::endl;
#endif // DEBUG_INFO
		}
		else if (recv_size < 44)
		{
#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Calibration data package size error! The package should be larger than 44 bytes but only "
				<< recv_size << " bytes received." << std::endl;
#endif // DEBUG_INFO
		}
		else
		{
			if (data[0] == 0x42 && data[1] == 0x57 && data[2] == PRODUCT_ID_X2 && data[3] == PROTOCOL_ID_MDOP
				&& data[recv_size - 2] == 0x00 && data[recv_size - 1] == 0xff)
			{
				memcpy(&pkg_count, &data[PROTOCOL_DATA_PACKAGE_COUNT_OFFSET], sizeof(uint32_t));
				memcpy(&nFrame, &data[PROTOCOL_DATA_FRAME_COUNT_OFFSET], sizeof(uint16_t));
				memcpy(&nLine, &data[PROTOCOL_DATA_LINE_NUM_OFFSET], sizeof(uint16_t));
				memcpy(&nPoint, &data[PROTOCOL_DATA_POINT_COUNT_OFFSET], sizeof(uint16_t));
				memcpy(&time_s, &data[PROTOCOL_X_DATA_TIME_S_OFFSET], sizeof(uint32_t));
				memcpy(&time_ns, &data[PROTOCOL_X_DATA_TIME_NS_OFFSET], sizeof(uint32_t));

				total_bytes = nPoint * point_bytes + 44;
				if (total_bytes != recv_size)
				{
#ifdef DEBUG_INFO
					std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
					std::cout << "Calibration data package size error! The package should be " << total_bytes << " bytes but "
						<< recv_size << " bytes received." << std::endl;
#endif // DEBUG_INFO
					continue;
				}

				diff = pkg_count - pkg_count_last;
				if (diff > 1)
				{
					total_lost_package += (diff - 1);
				}
				pkg_count_last = pkg_count;

				diff = uint16_t(nFrame - nFrame_last);
				if (diff >= 1)
				{
#ifdef _WIN32
					WaitForSingleObject(occup_, INFINITE);
#else
					sem_wait(&occup_);
#endif // _WIN32
					sphere_coord_out_buffer_.clear();
					sphere_coord_out_buffer_ = sphere_coord_buffer_;
					new_frame_ = true;
#ifdef _WIN32
					ReleaseSemaphore(occup_, 1, NULL);
#else
					sem_post(&occup_);
#endif // _WIN32
					if (save_flag_)
					{
						std::string file_name;
						mkDir(save_path_);
						if (save_file_name_ == "")
						{
							time_t timep(time(NULL));
							char time_char[256];
							struct tm* nowTime = localtime(&timep);
							strftime(time_char, sizeof(time_char), "/%Y-%m-%d_%H_%M_%S.txt", nowTime);
							std::string time_str(time_char);
							file_name = save_path_ + time_str;
						}
						else
						{
							file_name = save_path_ + "/" + save_file_name_;
						}
						//���Ĵ���0x1308  0��������ģʽ   1������ģʽ

						std::ofstream out(file_name);
						if (out.is_open()) {
							std::cout << "save original data path:" << file_name;
							if (is_dominant_wave_mode_)
							{
								out << "line X Y time_s time_ns "
									<< "CH1_L_main_rise_ture CH1_L_main_width_ture CH1_H_main_rise_ture CH1_H_main_width_ture "
									<< "CH2_L_main_rise_ture CH2_L_main_width_ture CH2_H_main_rise_ture CH2_H_main_width_ture "
									<< "CH1_L_echo_rise_ture CH1_L_echo_width_ture CH1_H_echo_rise_ture CH1_H_echo_width_ture "
									<< "CH2_L_echo_rise_ture CH2_L_echo_width_ture CH2_H_echo_rise_ture CH2_H_echo_width_ture "
									<< "undefined_0 undefined_1 undefined_2 undefined_3 "
									<< "undefined_4 undefined_5 undefined_6 undefined_7 "
									<< std::endl;
								for (BwSphereCoordAll point : sphere_coord_all_buffer_)
								{
									out << point.line << " "
										<< point.x << " "
										<< point.y << " "
										<< point.timestamp_s << " "
										<< point.timestamp_ns << " "

										<< point.ch1_l_dist[0] << " "
										<< point.ch1_l_dist[1] << " "
										<< point.ch1_l_dist[2] << " "
										<< point.ch1_l_width[0] << " "

										<< point.ch1_l_width[1] << " "
										<< point.ch1_l_width[2] << " "
										<< point.ch1_h_dist[0] << " "
										<< point.ch1_h_dist[1] << " "

										<< point.ch1_h_dist[2] << " "
										<< point.ch1_h_width[0] << " "
										<< point.ch1_h_width[1] << " "
										<< point.ch1_h_width[2] << " "

										<< point.ch2_l_dist[0] << " "
										<< point.ch2_l_dist[1] << " "
										<< point.ch2_l_dist[2] << " "
										<< point.ch2_l_width[0] << " "

										//undefine...
										<< point.ch2_l_width[1] << " "
										<< point.ch2_l_width[2] << " "

										<< point.ch2_h_dist[0] << " "
										<< point.ch2_h_dist[1] << " "
										<< point.ch2_h_dist[2] << " "

										<< point.ch2_h_width[0] << " "
										<< point.ch2_h_width[1] << " "
										<< point.ch2_h_width[2] << " "
										<< std::endl;
								}
									
							}
							else
							{
								out << "line X Y time_s time_ns dist_ch1_lg dist_ch1_hg dist_ch2_lg dist_ch2_hg " <<
								"pulse_ch1_lg pulse_ch1_hg pulse_ch2_lg pulse_ch2_hg" << std::endl;
							for (BwSphereCoord point : sphere_coord_buffer_)
								out << point.line << " " << point.x << " " << point.y << " " << point.timestamp_s << " " << point.timestamp_ns
								<< " " << point.dist[0] << " " << point.dist[1] << " " << point.dist[2] << " " << point.dist[3]
								<< " " << point.pulse_width[0] << " " << point.pulse_width[1] << " " << point.pulse_width[2] << " "
								<< point.pulse_width[3] << std::endl;
							}
							
						}
						else
						{
							std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
							std::cout << "Create file failed when save calibration data!" << std::endl;
						}

						save_flag_ = false;
					}
					sphere_coord_buffer_.clear();
					sphere_coord_all_buffer_.clear();
				}
				nFrame_last = nFrame;
				for (int i = 38; i < (recv_size - 6); i += point_bytes)
				{
					if (!is_dominant_wave_mode_)
					{
						memcpy(&pt.x, &data[i], sizeof(uint16_t));
						memcpy(&pt.y, &data[i + 2], sizeof(uint16_t));
						memcpy(&pt.timestamp_s, &data[i + 4], sizeof(uint32_t));
						memcpy(&pt.timestamp_ns, &data[i + 8], sizeof(uint32_t));
						memcpy(&pt.dist[0], &data[i + 12], sizeof(uint16_t));
						memcpy(&pt.pulse_width[0], &data[i + 18], sizeof(uint16_t));
						memcpy(&pt.dist[1], &data[i + 24], sizeof(uint16_t));
						memcpy(&pt.pulse_width[1], &data[i + 30], sizeof(uint16_t));
						memcpy(&pt.dist[2], &data[i + 36], sizeof(uint16_t));
						memcpy(&pt.pulse_width[2], &data[i + 42], sizeof(uint16_t));
						memcpy(&pt.dist[3], &data[i + 48], sizeof(uint16_t));
						memcpy(&pt.pulse_width[3], &data[i + 54], sizeof(uint16_t));
						pt.line = nLine;
						sphere_coord_buffer_.push_back(pt);
					}
					else
					{
						memcpy(&pt_all.x, &data[i], sizeof(uint16_t));
						memcpy(&pt_all.y, &data[i + 2], sizeof(uint16_t));
						memcpy(&pt_all.timestamp_s, &data[i + 4], sizeof(uint32_t));
						memcpy(&pt_all.timestamp_ns, &data[i + 8], sizeof(uint32_t));

						memcpy(&pt_all.ch1_l_dist[0], &data[i + 12], sizeof(uint16_t));
						memcpy(&pt_all.ch1_l_dist[1], &data[i + 14], sizeof(uint16_t));
						memcpy(&pt_all.ch1_l_dist[2], &data[i + 16], sizeof(uint16_t));

						memcpy(&pt_all.ch1_l_width[0], &data[i + 18], sizeof(uint16_t));
						memcpy(&pt_all.ch1_l_width[1], &data[i + 20], sizeof(uint16_t));
						memcpy(&pt_all.ch1_l_width[2], &data[i + 22], sizeof(uint16_t));

						memcpy(&pt_all.ch1_h_dist[0], &data[i + 24], sizeof(uint16_t));
						memcpy(&pt_all.ch1_h_dist[1], &data[i + 26], sizeof(uint16_t));
						memcpy(&pt_all.ch1_h_dist[2], &data[i + 28], sizeof(uint16_t));

						memcpy(&pt_all.ch1_h_width[0], &data[i + 30], sizeof(uint16_t));
						memcpy(&pt_all.ch1_h_width[1], &data[i + 32], sizeof(uint16_t));
						memcpy(&pt_all.ch1_h_width[2], &data[i + 34], sizeof(uint16_t));

						memcpy(&pt_all.ch2_l_dist[0], &data[i + 36], sizeof(uint16_t));
						memcpy(&pt_all.ch2_l_dist[1], &data[i + 38], sizeof(uint16_t));
						memcpy(&pt_all.ch2_l_dist[2], &data[i + 40], sizeof(uint16_t));

						memcpy(&pt_all.ch2_l_width[0], &data[i + 42], sizeof(uint16_t));
						memcpy(&pt_all.ch2_l_width[1], &data[i + 44], sizeof(uint16_t));
						memcpy(&pt_all.ch2_l_width[2], &data[i + 46], sizeof(uint16_t));

						memcpy(&pt_all.ch2_h_dist[0], &data[i + 48], sizeof(uint16_t));
						memcpy(&pt_all.ch2_h_dist[1], &data[i + 50], sizeof(uint16_t));
						memcpy(&pt_all.ch2_h_dist[2], &data[i + 52], sizeof(uint16_t));

						memcpy(&pt_all.ch2_h_width[0], &data[i + 54], sizeof(uint16_t));
						memcpy(&pt_all.ch2_h_width[1], &data[i + 56], sizeof(uint16_t));
						memcpy(&pt_all.ch2_h_width[2], &data[i + 58], sizeof(uint16_t));

						pt_all.line = nLine;
						sphere_coord_all_buffer_.push_back(pt_all);
					}
				}
			}
		}
	}
	delete[] data;
}

void benewake::MDOPProtocol::testDataDecode()
{
	unsigned char *data = new unsigned char[PROTOCOL_DATA_PACKAGE_MAX_LENGTH];
	float signal_data[8] = { 0 };
	std::ofstream out;
	//float data[8];
	uint16_t ch1_l_r[4];
	uint16_t ch1_l_f[4];
	uint16_t ch1_h_r[4];
	uint16_t ch1_h_f[4];
	uint16_t ch2_l_r[4];
	uint16_t ch2_l_f[4];
	uint16_t ch2_h_r[4];
	uint16_t ch2_h_f[4];
	int rise_cnt = 0, fail_cnt = 0;
	int recv_size = 0, total_bytes = 0, point_bytes = 72;
	uint32_t pkg_count = 0, time_s = 0, time_ns = 0;
	uint32_t check_sum = 0, valu32 = 0;
	uint16_t nFrame = 0, nLine = 0, nPoint = 0, valu16 = 0;
	int total_num = 0, eff_num = 0;
	while (thread_run_)
	{
		recv_size = pudp_->recvSomeData(data, PROTOCOL_DATA_PACKAGE_MAX_LENGTH, 0, 200000);
		if (recv_size < 0)
		{
#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Receive test data failed!" << std::endl;
#endif // DEBUG_INFO
		}
		else if (recv_size < 44)
		{
#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Test data package size error! The package should be larger than 44 bytes but only "
				<< recv_size << " bytes received." << std::endl;
#endif // DEBUG_INFO
		}
		else
		{
			if (data[0] == 0x42 && data[1] == 0x57 && data[2] == PRODUCT_ID_X2 && data[3] == PROTOCOL_ID_MDOP
				&& data[recv_size - 2] == 0x00 && data[recv_size - 1] == 0xff)
			{
				memcpy(&pkg_count, &data[PROTOCOL_DATA_PACKAGE_COUNT_OFFSET], sizeof(uint32_t));
				memcpy(&nFrame, &data[PROTOCOL_DATA_FRAME_COUNT_OFFSET], sizeof(uint16_t));
				memcpy(&nLine, &data[PROTOCOL_DATA_LINE_NUM_OFFSET], sizeof(uint16_t));
				memcpy(&nPoint, &data[PROTOCOL_DATA_POINT_COUNT_OFFSET], sizeof(uint16_t));
				memcpy(&time_s, &data[PROTOCOL_X_DATA_TIME_S_OFFSET], sizeof(uint32_t));
				memcpy(&time_ns, &data[PROTOCOL_X_DATA_TIME_NS_OFFSET], sizeof(uint32_t));

				total_bytes = nPoint * point_bytes + 44;
				if (total_bytes != recv_size)
				{
#ifdef DEBUG_INFO
					std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
					std::cout << "Test data package size error! The package should be " << total_bytes << " bytes but "
						<< recv_size << " bytes received." << std::endl;
#endif // DEBUG_INFO
					continue;
				}

				if (total_num >= 10000)
				{
#ifdef _WIN32
					WaitForSingleObject(occup_, INFINITE);
#else
					sem_wait(&occup_);
#endif // _WIN32
					eff_percent_ = (float)eff_num / (float)total_num;
					new_frame_ = true;
#ifdef _WIN32
					ReleaseSemaphore(occup_, 1, NULL);
#else
					sem_post(&occup_);
#endif // _WIN32
					eff_num = 0;
					total_num = 0;
				}

				if (save_flag_ && !out.is_open())
				{
					std::string file_name;
					mkDir(save_path_);
					if (save_file_name_ == "")
					{
						time_t timep(time(NULL));
						char time_char[256];
						struct tm* nowTime = localtime(&timep);
						strftime(time_char, sizeof(time_char), "/%Y-%m-%d_%H_%M_%S.txt", nowTime);
						std::string time_str(time_char);
						file_name = save_path_ + time_str;
					}
					else
					{
						file_name = save_path_ + "/" + save_file_name_;
					}
					out.open(file_name);
					out << "ch1_lg_r0 " << "ch1_lg_r1 " << "ch1_lg_r2 " << "ch1_lg_r3 "
						<< "ch1_lg_f0 " << "ch1_lg_f1 " << "ch1_lg_f2 " << "ch1_lg_f3 "
						<< "ch1_hg_r0 " << "ch1_hg_r1 " << "ch1_hg_r2 " << "ch1_hg_r3 "
						<< "ch1_hg_f0 " << "ch1_hg_f1 " << "ch1_hg_f2 " << "ch1_hg_f3 "
						<< "ch2_lg_r0 " << "ch2_lg_r1 " << "ch2_lg_r2 " << "ch2_lg_r3 "
						<< "ch2_lg_f0 " << "ch2_lg_f1 " << "ch2_lg_f2 " << "ch2_lg_f3 "
						<< "ch2_hg_r0 " << "ch2_hg_r1 " << "ch2_hg_r2 " << "ch2_hg_r3 "
						<< "ch2_hg_f0 " << "ch2_hg_f1 " << "ch2_hg_f2 " << "ch2_hg_f3 " << std::endl;
				}
				for (int i = 38; i < (recv_size - 6); i += point_bytes)
				{
					memcpy(ch1_l_r, &data[i], 4 * sizeof(uint16_t));
					memcpy(ch1_l_f, &data[i + 8], 4 * sizeof(uint16_t));
					memcpy(ch1_h_r, &data[i + 16], 4 * sizeof(uint16_t));
					memcpy(ch1_h_f, &data[i + 24], 4 * sizeof(uint16_t));
					memcpy(ch2_l_r, &data[i + 32], 4 * sizeof(uint16_t));
					memcpy(ch2_l_f, &data[i + 40], 4 * sizeof(uint16_t));
					memcpy(ch2_h_r, &data[i + 48], 4 * sizeof(uint16_t));
					memcpy(ch2_h_f, &data[i + 56], 4 * sizeof(uint16_t));

					if (save_flag_ && out.is_open())
					{
						out << ch1_l_r[0] << " " << ch1_l_r[1] << " " << ch1_l_r[2] << " " << ch1_l_r[3] << " "
							<< ch1_l_f[0] << " " << ch1_l_f[1] << " " << ch1_l_f[2] << " " << ch1_l_f[3] << " "
							<< ch1_h_r[0] << " " << ch1_h_r[1] << " " << ch1_h_r[2] << " " << ch1_h_r[3] << " "
							<< ch1_h_f[0] << " " << ch1_h_f[1] << " " << ch1_h_f[2] << " " << ch1_h_f[3] << " "
							<< ch2_l_r[0] << " " << ch2_l_r[1] << " " << ch2_l_r[2] << " " << ch2_l_r[3] << " "
							<< ch2_l_f[0] << " " << ch2_l_f[1] << " " << ch2_l_f[2] << " " << ch2_l_f[3] << " "
							<< ch2_h_r[0] << " " << ch2_h_r[1] << " " << ch2_h_r[2] << " " << ch2_h_r[3] << " "
							<< ch2_h_f[0] << " " << ch2_h_f[1] << " " << ch2_h_f[2] << " " << ch2_h_f[3] << std::endl;
						save_num_--;
					}

					rise_cnt = 0;
					fail_cnt = 0;
					for (int i = 0; i < 4; i++)
					{
						if (ch1_l_r[i] > 0)
							rise_cnt++;
						if (ch1_l_f[i] > 0)
							fail_cnt++;
					}
					if (rise_cnt == fail_cnt && rise_cnt >= 1 && 1200 < ch1_l_r[0] && ch1_l_r[0] < ch1_l_f[0])
					{
						signal_data[0] = (double)(ch1_l_r[0] - 1200.0) * 64.0 * 3.0 * 0.0001 / 2.0;
						signal_data[1] = ch1_l_f[0] - ch1_l_r[0];
						eff_num++;
					}
					else
					{
						signal_data[0] = 0.0;
						signal_data[1] = 0.0;
					}
					total_num++;

					rise_cnt = 0;
					fail_cnt = 0;
					for (int i = 0; i < 4; i++)
					{
						if (ch1_h_r[i] > 0)
							rise_cnt++;
						if (ch1_h_f[i] > 0)
							fail_cnt++;
					}
					if (rise_cnt == fail_cnt && rise_cnt >= 2 && ch1_h_r[0] < ch1_h_f[0] && ch1_h_f[0] < ch1_h_r[1] && ch1_h_r[1] < ch1_h_f[1])
					{
						signal_data[2] = (double)(ch1_h_r[1] - 1200.0) * 64.0 * 3.0 * 0.0001 / 2.0;
						signal_data[3] = ch1_h_f[1] - ch1_h_r[1];
						eff_num++;
					}
					else
					{
						signal_data[2] = 0.0;
						signal_data[3] = 0.0;
					}
					total_num++;

					rise_cnt = 0;
					fail_cnt = 0;
					for (int i = 0; i < 4; i++)
					{
						if (ch2_l_r[i] > 0)
							rise_cnt++;
						if (ch2_l_f[i] > 0)
							fail_cnt++;
					}
					if (rise_cnt == fail_cnt && rise_cnt >= 1 && 1200 < ch2_l_r[0] && ch2_l_r[0] < ch2_l_f[0])
					{
						signal_data[4] = (double)(ch2_l_r[0] - 1200.0) * 64.0 * 3.0 * 0.0001 / 2.0;
						signal_data[5] = ch2_l_f[0] - ch2_l_r[0];
						eff_num++;
					}
					else
					{
						signal_data[4] = 0.0;
						signal_data[5] = 0.0;
					}
					total_num++;

					rise_cnt = 0;
					fail_cnt = 0;
					for (int i = 0; i < 4; i++)
					{
						if (ch2_h_r[i] > 0)
							rise_cnt++;
						if (ch2_h_f[i] > 0)
							fail_cnt++;
					}
					if (rise_cnt == fail_cnt && rise_cnt >= 2 && ch2_h_r[0] < ch2_h_f[0] && ch2_h_f[0] < ch2_h_r[1] && ch2_h_r[1] < ch2_h_f[1])
					{
						signal_data[6] = (double)(ch2_h_r[1] - 1200.0) * 64.0 * 3.0 * 0.0001 / 2.0;
						signal_data[7] = ch2_h_f[1] - ch2_h_r[1];
						eff_num++;
					}
					else
					{
						signal_data[6] = 0.0;
						signal_data[7] = 0.0;
					}
					total_num++;
				}
				if (save_flag_ && save_num_ <= 0)
				{
					out.close();
					save_flag_ = false;
				}
#ifdef _WIN32
				WaitForSingleObject(occup_, INFINITE);
#else
				sem_wait(&occup_);
#endif // _WIN32
				memcpy(single_point_data_, signal_data, 8 * sizeof(float));
				new_data_ = true;
#ifdef _WIN32
				ReleaseSemaphore(occup_, 1, NULL);
#else
				sem_post(&occup_);
#endif // _WIN32
			}
		}
	}
	delete[] data;
}

benewake::MDOPProtocolP4::MDOPProtocolP4(std::string _local_ip, int _local_port, std::string _remote_ip, int _remote_port, UDPType _udp_type) :
	MDOPProtocol(_local_ip, _local_port, _remote_ip, _remote_port, _udp_type)
{
	protocol_type_ = benewake::BwProtocolType::SPHERE;
	enable_angle_calib_ = true;

	recv_buffer_queue_ = new RingBufferData[MDOP_DATA_RING_BUFFER_MAX_LENGTH]();
	for (int i = 0; i < MDOP_DATA_RING_BUFFER_MAX_LENGTH; ++i)
		recv_buffer_queue_[i].data = new unsigned char[PROTOCOL_DATA_PACKAGE_MAX_LENGTH];
}

benewake::MDOPProtocolP4::~MDOPProtocolP4()
{
	for (int i = 0; i < MDOP_DATA_RING_BUFFER_MAX_LENGTH; ++i)
		delete[] recv_buffer_queue_[i].data;

	delete[] recv_buffer_queue_;
}

int benewake::MDOPProtocolP4::mdop_enable_decode(bool _enable)
{
	if (_enable)
	{
		if (thread_run_)
		{
			return STATUS_OK;
		}
		thread_run_ = true;
		if (protocol_type_ == benewake::BwProtocolType::CLIENT)
		{
			decode_thread_ = new std::thread(&MDOPProtocolP4::clientDataDecode, this);
		}
		else if (protocol_type_ == benewake::BwProtocolType::SPHERE)
		{
			recv_buffer_info_.head = 0;
			recv_buffer_info_.tail = 0;
			recv_thread_ = new std::thread(&MDOPProtocolP4::recvPackageHandler, this);
			decode_thread_ = new std::thread(&MDOPProtocolP4::sphereDataDecode, this);
		}
		else if (protocol_type_ == benewake::BwProtocolType::TEST)
		{
			decode_thread_ = new std::thread(&MDOPProtocolP4::testDataDecode, this);
		}
		return STATUS_OK;
	}
	else
	{
		if (thread_run_)
		{
			thread_run_ = false;
			new_frame_ = false;
			if (protocol_type_ == benewake::BwProtocolType::SPHERE)
				recv_thread_->join();
			if (protocol_type_ != benewake::BwProtocolType::TEST)
				decode_thread_->join();
		}
		return STATUS_OK;
	}
}

benewake::BwTestDataP4* benewake::MDOPProtocolP4::getSrcDataPtr()
{
	return &src_data_;
}

int benewake::MDOPProtocolP4::mdop_save_data_p4(std::string _save_path, std::string _save_file_name, uint32_t _save_times)
{
	if (_save_times > 0)
	{
		save_path_ = _save_path;
		save_file_name_ = _save_file_name;
		save_name_flag_ = 0;
		save_times_ = _save_times;//开始保存
		return STATUS_OK;
	}
	else
	{
		return STATUS_FAIL;
	}
}

void benewake::MDOPProtocolP4::mdop_set_angle_calibration(bool _enable)
{
	enable_angle_calib_ = _enable;
}

void benewake::MDOPProtocolP4::mdop_get_angle_calibration(bool &_enable)
{
	_enable = enable_angle_calib_;
}

void benewake::MDOPProtocolP4::recvPackageHandler()
{
	unsigned char* data = new unsigned char[PROTOCOL_DATA_PACKAGE_MAX_LENGTH];
	int recv_size = 0;
	while (thread_run_)
	{
		recv_size = pudp_->recvSomeData(data, PROTOCOL_DATA_PACKAGE_MAX_LENGTH, 3);
		if (recv_size < 0)
		{
#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Receive point cloud data failed!" << std::endl;
#endif // DEBUG_INFO
		}
		else if (recv_size < 44)
		{
#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Point cloud data package size error! The package should be larger than 44 bytes but only "
				<< recv_size << " bytes received." << std::endl;
#endif // DEBUG_INFO
		}
		else
		{
			if (ringBufferIsFull(recv_buffer_info_))
			{
			}
			else
			{
				memcpy(recv_buffer_queue_[recv_buffer_info_.head].data, data, recv_size);
				recv_buffer_queue_[recv_buffer_info_.head].data_length = recv_size;
				recv_buffer_info_.head = (recv_buffer_info_.head + 1) % MDOP_DATA_RING_BUFFER_MAX_LENGTH;
			}
		}
	}
	delete[] data;
}

void benewake::MDOPProtocolP4::clientDataDecode()
{
	BwPointCloud::Ptr point_cloud_tmp;
	BwPoint pt;

	BwPointCloud::Ptr point_cloud_backup;
	long long dt_time = 0;

	int point_bytes = 10;
	point_cloud_buffer_.reset();
	point_cloud_buffer_ = std::make_shared<BwPointCloud>();
	point_cloud_tmp = std::make_shared<BwPointCloud>();
	point_cloud_backup = std::make_shared<BwPointCloud>();

	std::chrono::system_clock::time_point static_start, static_stop;
	uint32_t lost_pkgs = 0, static_start_pkg = 0, lost_frames = 0, static_start_frame = 0;

	unsigned char* data = new unsigned char[PROTOCOL_DATA_PACKAGE_MAX_LENGTH];
	int recv_size = 0, total_bytes = 4 * 164 + 44;
	int echo_mode = 1, data_block = 0;
	uint32_t pkg_count = 0, pkg_count_last = 0, time_s = 0, time_ns = 0;
	int time_offset = 0;
	bool use_gps_src = false;
	uint32_t diff = 0, total_lost_package = 0, check_sum = 0;
	uint16_t nFrame = 0, nFrame_last = 0, nLine = 0, nLine_last = 0, nPoint = 0, uval16 = 0, protocol_version = 0;
	int32_t dist32 = 0;
	uint8_t uval8 = 0;
	int year = 0, month = 0, day = 0, hour = 0, minute = 0;
	bool new_frame_received = false, first_pkg = true;
	long long frame_win_num = 0;
	long long last_frame_win_num = 0;
	int current_pkg_num = 0;

	pkg_loss_rate_ = 0;
	frame_loss_rate_ = 0;
	static_start = std::chrono::system_clock::now();
	while (thread_run_)
	{
		recv_size = pudp_->recvSomeData(data, PROTOCOL_DATA_PACKAGE_MAX_LENGTH, 3);
		if (recv_size < 0)
		{
#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Receive point cloud data failed!" << std::endl;
#endif // DEBUG_INFO
		}
		else if (recv_size < 44)
		{
#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Point cloud data package size error! The package should be larger than 44 bytes but only "
				<< recv_size << " bytes received." << std::endl;
#endif // DEBUG_INFO
		}
		else
		{
			if (data[0] == 0x42 && data[1] == 0x57 && data[2] == PRODUCT_ID_P4 && data[3] == PROTOCOL_ID_MDOP
				&& data[recv_size - 2] == 0x00 && data[recv_size - 1] == 0xff)
			{
				protocol_version = *(uint16_t*)&data[PROTOCOL_DATA_PACKAGE_VERSION_OFFSET];
				pkg_count = *(uint32_t*)&data[PROTOCOL_DATA_PACKAGE_COUNT_OFFSET];
				nFrame = *(uint16_t*)&data[PROTOCOL_DATA_FRAME_COUNT_OFFSET];
				nLine = *(uint16_t*)&data[PROTOCOL_DATA_LINE_NUM_OFFSET];
				nPoint = *(uint16_t*)&data[PROTOCOL_DATA_POINT_COUNT_OFFSET];
				time_s = *(uint32_t*)&data[PROTOCOL_AD2_DATA_TIME_S_OFFSET];
				time_ns = *(uint32_t*)&data[PROTOCOL_AD2_DATA_TIME_NS_OFFSET];
				uval8 = *(uint8_t*)&data[PROTOCOL_AD2_DATA_ECHO_MODE];
				if (uval8 <= 0x02)
					echo_mode = 1;
				else
					echo_mode = 2;

				use_gps_src = false;
				if ((time_s & 0x80000000) == 0x80000000)
				{
					year = (time_s & 0x3f000000) >> 24;
					month = (time_s & 0x00fc0000) >> 18;
					day = (time_s & 0x0003f000) >> 12;
					hour = (time_s & 0x00000fc0) >> 6;
					minute = (time_s & 0x0000003f);

					utc_time_t utcTime;
					utcTime.year = year + 2000;
					utcTime.month = month;
					utcTime.day = day;
					utcTime.hour = hour;
					utcTime.minute = minute;
					utcTime.second = time_ns / 1000000 % 100;
					time_s = covUTC2UnixTimestamp(&utcTime);

					time_ns = time_ns % 1000000 * 1000;
					use_gps_src = true;
				}

				if (total_bytes != recv_size)
				{
#ifdef DEBUG_INFO
					std::cout << "INFO: package size " << total_bytes << " / " << recv_size << std::endl;
#endif // DEBUG_INFO
					continue;
				}

				check_sum = check_sum_with_protocol_version(protocol_version, data, recv_size - 6);
				if (memcmp(&check_sum, &data[recv_size - 6], 4) != 0)
				{
#ifdef DEBUG_INFO
					uint32_t recv_check_sum = *(uint32_t*)&data[recv_size - 6];
					std::printf("INFO: point cloud data package check sum 0x%08x<0x%08x>\n", check_sum, recv_check_sum);
#endif // DEBUG_INFO
					continue;
				}

				if (first_pkg)
				{
					static_start_pkg = pkg_count;
					static_start_frame = nFrame;
					pkg_count_last = pkg_count;
					nFrame = nFrame_last;
					first_pkg = false;
				}

				diff = pkg_count - pkg_count_last;
				if (diff > 1)
				{
					total_lost_package += (diff - 1);
					lost_pkgs += (diff - 1);
				}
				pkg_count_last = pkg_count;

				new_frame_received = false;
				diff = uint16_t(nFrame - nFrame_last);
				if (!enable_time_windows_ && diff >= 1)
					new_frame_received = true;
				else if (enable_time_windows_)
				{
					if (recv_size > 44) {
						time_offset = *(int*)&data[38];
						if (use_gps_src)
							time_offset *= 1000;
						dt_time = (long long)time_s * 1000.0 + time_ns * 0.000001 + time_offset * 0.000001;
					}
					else
						dt_time = (long long)time_s * 1000.0 + time_ns * 0.000001;
					frame_win_num = dt_time / frame_interval_;
					current_pkg_num++;
					if (frame_win_num > last_frame_win_num) // when package is empty, dt may be located at last frame and {frame_win_num < last_frame_win_num} may occur in this situation
					{
						new_frame_received = true;
						last_frame_win_num = frame_win_num;
						current_pkg_num = 0;
					}
					else if (current_pkg_num > MDOP_DATA_MAX_PKG_NUM)
					{
						new_frame_received = true;
						last_frame_win_num = frame_win_num;
						current_pkg_num = 0;
#ifdef DEBUG_INFO
						std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
						std::cout << "Abnormal: timestamp exception detected!" << std::endl;
#endif // DEBUG_INFO
					}
				}
				if (new_frame_received)
				{
#ifdef _WIN32
					WaitForSingleObject(occup_, INFINITE);
#else
					sem_wait(&occup_);
#endif // _WIN32
					point_cloud_buffer_.reset();
					point_cloud_buffer_ = point_cloud_tmp;
					frame_id_ = nFrame_last;
					new_frame_ = true;
#ifdef _WIN32
					ReleaseSemaphore(occup_, 1, NULL);
#else
					sem_post(&occup_);
#endif // _WIN32
					// push to time window data queue
					if (enable_time_windows_)
						data_queue_->pushMsg(point_cloud_tmp);

					point_cloud_tmp.reset(new BwPointCloud());
					point_cloud_tmp->points.reserve(1100000);
					if (enable_time_windows_)
					{
						point_cloud_tmp->points.insert(point_cloud_tmp->points.end(), point_cloud_backup->points.begin(), point_cloud_backup->points.end());
						point_cloud_backup->points.clear();
					}

					if (callback_enable_)
					{
						callback_func_(point_cloud_buffer_, frame_id_, callback_pointer_);
					}

					if (total_lost_package > 0)
					{
						//std::cout << "Package lost detected! " << total_lost_package << " package(s) lost in last frame." << std::endl;
						std::cout << "INFO: PL in last frame " << total_lost_package << std::endl;
						total_lost_package = 0;
					}
					if (diff > 1)
					{
						std::cout << "INFO: FL " << (diff - 1) << std::endl;
						lost_frames += (diff - 1);
					}
				}
				nFrame_last = nFrame;
				nLine_last = nLine;

				data_block = nPoint * echo_mode;
				for (int i = 0; i < data_block; i++)
				{
					// timestamp
					time_offset = *(int*)&data[38 + i * 164];
					if (use_gps_src)
						time_offset *= 1000;
					pt.timestamp_s = time_s;

					if (time_offset < 0 && abs(time_offset) > time_ns)
					{
						pt.timestamp_ns = 1000000000 + (int)time_ns + time_offset;
						pt.timestamp_s--;
					}
					else
					{
						pt.timestamp_ns = time_ns + time_offset;
						if (pt.timestamp_ns >= 1000000000)
						{
							pt.timestamp_s += pt.timestamp_ns / 1000000000;
							pt.timestamp_ns %= 1000000000;
						}
					}
					if (new_frame_received && i == 0)
					{
						std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
						std::chrono::nanoseconds ns = tp.time_since_epoch();
						delay_time_ = ns.count() / 1000 - (long long)pt.timestamp_s * 1000000 - pt.timestamp_ns * 0.001;
					}

					// x y z intensity
					for (int j = 0; j < 16; j++)
					{
						memcpy(&dist32, &data[38 + i * 164 + 4 + j * point_bytes + 0], 3);
						pt.x = (float)dist32 * 0.004;
						memcpy(&dist32, &data[38 + i * 164 + 4 + j * point_bytes + 3], 3);
						pt.y = (float)dist32 * 0.004;
						memcpy(&dist32, &data[38 + i * 164 + 4 + j * point_bytes + 6], 3);
						pt.z = (float)dist32 * 0.004;
						pt.intensity = *(uint8_t*)&data[38 + i * 164 + 4 + j * point_bytes + 9];
						
						pt.channel = (uint8_t)j;

						pt.row = nLine;

						if (enable_time_windows_ &&
							((long long)pt.timestamp_s * 1000.0 + pt.timestamp_ns * 0.000001) >= (frame_win_num + 1) * frame_interval_)
						{
							point_cloud_backup->points.push_back(pt);
						}
						else
						{
							point_cloud_tmp->points.push_back(pt);
						}
					}
				}
			}
			else
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::printf("Point cloud data package header/tail error: 0x%02x 0x%02x 0x%02x 0x%02x ... 0x%02x 0x%02x!\n",
					data[0], data[1], data[2], data[3], data[recv_size - 2], data[recv_size - 1]);
			}
		}

		static_stop = std::chrono::system_clock::now();
		auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(static_stop - static_start);
		if (dur.count() > 10000)
		{
			// update data statics
			if (pkg_count < static_start_pkg)
				pkg_loss_rate_ = (float)lost_pkgs / float((UINT32_MAX - static_start_pkg) + pkg_count) * 100.0;
			else if (pkg_count == static_start_pkg)
				pkg_loss_rate_ = 100.0;
			else
				pkg_loss_rate_ = (float)lost_pkgs / float(pkg_count - static_start_pkg) * 100.0;
			lost_pkgs = 0;
			static_start_pkg = pkg_count;


			if (nFrame < static_start_frame)
				frame_loss_rate_ = (float)lost_frames / float((UINT16_MAX - static_start_frame) + nFrame) * 100.0;
			else if (nFrame == static_start_frame)
				frame_loss_rate_ = 100.0;
			else
				frame_loss_rate_ = (float)lost_frames / float(nFrame - static_start_frame) * 100.0;
			lost_frames = 0;
			static_start_frame = nFrame;

			static_start = static_stop;
		}
	}
	delete[] data;
}

void benewake::MDOPProtocolP4::sphereDataDecode()
{
	BwPointCloud::Ptr point_cloud_tmp;
	BwPoint pt;

	BwPointCloud::Ptr point_cloud_backup;
	uint64_t dt_time = 0;

	benewake::BwSphereCoordP4Frame::Ptr sphere_coord_frame_data_buffer;
	BwSphereCoordP4 sphere_coord_package_data;
	sphere_coord_frame_data_queue_ = std::make_shared<benewake::MsgQueue<benewake::BwSphereCoordP4Frame::Ptr>>();
	int point_bytes = 4;
	point_cloud_buffer_.reset();
	point_cloud_buffer_ = std::make_shared<BwPointCloud>();
	point_cloud_tmp = std::make_shared<BwPointCloud>();
	point_cloud_backup = std::make_shared<BwPointCloud>();
	sphere_coord_frame_data_buffer = std::make_shared<BwSphereCoordP4Frame>();

	std::chrono::system_clock::time_point statistics_start, statistics_stop;
	uint32_t lost_pkgs = 0, statistics_start_pkg = 0, lost_frames = 0, statistics_start_frame = 0;

	unsigned char* data;
	int data_block_size = 72, header_length = 38;
	int recv_size = 0, total_bytes = 12 * data_block_size + 44;
	int echo_mode = 1, data_block = 0;
	uint32_t pkg_count = 0, pkg_count_last = 0;
	uint64_t time_s = 0;
	uint32_t time_ns = 0;
	int time_offset = 0;
	bool use_gps_src = false;
	uint32_t diff = 0, total_lost_package = 0, check_sum = 0;
	uint16_t nFrame = 0, nFrame_last = 0, nLine = 0, nLine_last = 0, nPoint = 0, uval16 = 0, protocol_version = 0, last_pkg_mark = 0;
	int16_t roi_center_x_flag = 0, roi_center_y_flag = 0;
	float roi_center_x = 0, roi_center_y = 0;
	float roi_half_x = kPI * 15.0 / 180.0, roi_half_y = kPI * 6.4 / 180.0;
	int16_t val16 = 0, val16_ang_h = 0, val16_ang_v = 0;
	int32_t dist32 = 0;
	uint8_t uval8 = 0, echo_mark_1 = 0, echo_mark_2 = 0;
	int year = 0, month = 0, day = 0, hour = 0, minute = 0;
	bool new_frame_received = false, last_pkg_received = false, first_pkg = true;
	long long frame_win_num = 0;
	long long last_frame_win_num = 0;
	float azimuth_h, azimuth_v, dist_f;
	int current_pkg_num = 0;
	bool belongs_to_next_frame = false;

	pkg_loss_rate_ = 0;
	frame_loss_rate_ = 0;
	statistics_start = std::chrono::system_clock::now();

	while (thread_run_)
	{
		if (ringBufferIsEmpty(recv_buffer_info_))
		{
			std::this_thread::sleep_for(std::chrono::microseconds(10));
		}
		else
		{
			data = recv_buffer_queue_[recv_buffer_info_.tail].data;
			recv_size = recv_buffer_queue_[recv_buffer_info_.tail].data_length;
			if (data[0] == 0x42 && data[1] == 0x57 && data[2] == PRODUCT_ID_P4 && data[3] == PROTOCOL_ID_MDOP
				&& data[recv_size - 2] == 0x00 && data[recv_size - 1] == 0xff)
			{
				use_gps_src = false;
				protocol_version = *(uint16_t*)&data[PROTOCOL_DATA_PACKAGE_VERSION_OFFSET];
				pkg_count = *(uint32_t*)&data[PROTOCOL_DATA_PACKAGE_COUNT_OFFSET];
				nFrame = *(uint16_t*)&data[PROTOCOL_DATA_FRAME_COUNT_OFFSET];
				uval16 = *(uint16_t*)&data[PROTOCOL_DATA_LINE_NUM_OFFSET];
				nLine = uval16 & 0x7fff;
				last_pkg_mark = uval16 & 0x8000;
				nPoint = *(uint16_t*)&data[PROTOCOL_DATA_POINT_COUNT_OFFSET];
				switch (protocol_version)
				{
				case PROTOCOL_VERSION_AD2_B:
					time_s = *(uint32_t*)&data[PROTOCOL_AD2_B_DATA_TIME_S_OFFSET];
					time_ns = *(uint32_t*)&data[PROTOCOL_AD2_B_DATA_TIME_NS_OFFSET];
					uval8 = *(uint8_t*)&data[PROTOCOL_AD2_B_DATA_ECHO_MODE];
					roi_center_x_flag = *(uint16_t*)&data[PROTOCOL_AD2_B_DATA_ROI_CENTER_X];
					roi_center_y_flag = *(uint16_t*)&data[PROTOCOL_AD2_B_DATA_ROI_CENTER_Y];
					roi_center_x = (float)roi_center_x_flag / 8192.0;
					roi_center_y = (float)roi_center_y_flag / 8192.0;

					header_length = PROTOCOL_AD2_B_DATA_HEADER_LENGHT;

					if ((time_s & 0x80000000) == 0x80000000)
					{
						use_gps_src = true;
					}
					break;

				case PROTOCOL_VERSION_AD2_C:
					time_s = *(uint64_t*)&data[PROTOCOL_AD2_DATA_TIME_S_OFFSET];
					time_ns = *(uint32_t*)&data[PROTOCOL_AD2_DATA_TIME_NS_OFFSET];
					uval8 = *(uint8_t*)&data[PROTOCOL_AD2_DATA_ECHO_MODE];
					roi_center_x_flag = *(int16_t*)&data[PROTOCOL_AD2_DATA_ROI_CENTER_X];
					roi_center_y_flag = *(int16_t*)&data[PROTOCOL_AD2_DATA_ROI_CENTER_Y];
					roi_center_x = (float)roi_center_x_flag / 10.0;
					roi_center_y = (float)roi_center_y_flag / 10.0;
					uval16 = *(uint16_t*)&data[PROTOCOL_AD2_DATA_ROI_WIDTH];
					roi_half_x = (float)uval16 / 20.0;
					uval16 = *(uint16_t*)&data[PROTOCOL_AD2_DATA_ROI_HEIGHT];
					roi_half_y = (float)uval16 / 20.0;

					header_length = PROTOCOL_AD2_DATA_HEADER_LENGHT;

					if ((time_s & 0x8000000000000000) == 0x8000000000000000)
					{
						use_gps_src = true;
					}
					break;

				case PROTOCOL_VERSION_AD2_HH:
					time_s = *(uint64_t*)&data[PROTOCOL_AD2_DATA_TIME_S_OFFSET];
					time_ns = *(uint32_t*)&data[PROTOCOL_AD2_DATA_TIME_NS_OFFSET];
					uval8 = *(uint8_t*)&data[PROTOCOL_AD2_DATA_ECHO_MODE];
					roi_center_x_flag = *(int16_t*)&data[PROTOCOL_AD2_DATA_ROI_CENTER_X];
					roi_center_y_flag = *(int16_t*)&data[PROTOCOL_AD2_DATA_ROI_CENTER_Y];
					roi_center_x = (float)roi_center_x_flag / 10.0;
					roi_center_y = (float)roi_center_y_flag / 10.0;
					uval16 = *(uint16_t*)&data[PROTOCOL_AD2_DATA_ROI_WIDTH];
					roi_half_x = (float)uval16 / 20.0;
					uval16 = *(uint16_t*)&data[PROTOCOL_AD2_DATA_ROI_HEIGHT];
					roi_half_y = (float)uval16 / 20.0;

					header_length = PROTOCOL_AD2_DATA_HEADER_LENGHT;

					if ((time_s & 0x8000000000000000) == 0x8000000000000000)
					{
						use_gps_src = true;
					}
					break;
				default:
					break;
				}
				treatEchoMode(uval8, echo_mode, echo_mark_1, echo_mark_2);
				if (echo_mode == 1)
				{
					data_block_size = 72;
				}
				else
				{
					data_block_size = 136;
				}
				sys_info_.protocol = protocol_version;
				total_bytes = 12 * data_block_size / echo_mode + header_length + 6;

				check_sum = check_sum_with_protocol_version(protocol_version, data, recv_size - 6);
				
				if (memcmp(&check_sum, &data[recv_size - 6], 4) != 0)
				{
#ifdef DEBUG_INFO
					uint32_t recv_check_sum = *(uint32_t*)&data[recv_size - 6];
					std::printf("INFO: package:%d point cloud data package check sum 0x%08x<0x%08x>\n", pkg_count, check_sum, recv_check_sum);
#endif // DEBUG_INFO
					continue;
				}

				if (use_gps_src)
				{
					year = (time_s & 0x3f000000) >> 24;
					month = (time_s & 0x00fc0000) >> 18;
					day = (time_s & 0x0003f000) >> 12;
					hour = (time_s & 0x00000fc0) >> 6;
					minute = (time_s & 0x0000003f);

					utc_time_t utcTime;
					utcTime.year = year + 2000;
					utcTime.month = month;
					utcTime.day = day;
					utcTime.hour = hour;
					utcTime.minute = minute;
					utcTime.second = time_ns / 1000000 % 100;
					time_s = covUTC2UnixTimestamp(&utcTime);

					time_ns = time_ns % 1000000 * 1000;
				}

				if (total_bytes != recv_size)
				{
#ifdef DEBUG_INFO
					std::cout << "INFO: package size " << total_bytes << " / " << recv_size << std::endl;
#endif // DEBUG_INFO
					continue;
				}

				if (first_pkg)
				{
					statistics_start_pkg = pkg_count;
					statistics_start_frame = nFrame;
					pkg_count_last = pkg_count;
					nFrame_last = nFrame;
					first_pkg = false;
				}

				diff = pkg_count - pkg_count_last;
				if (diff > 1)
				{
					total_lost_package += (diff - 1);
					lost_pkgs += (diff - 1);
				}
				pkg_count_last = pkg_count;

				new_frame_received = false;
				last_pkg_received = false;
				diff = uint16_t(nFrame - nFrame_last); // uint16_t transform for value flow out
				if (!enable_time_windows_ && last_pkg_mark == 0x8000)
					last_pkg_received = true;
				if (!enable_time_windows_ && diff >= 1)
					new_frame_received = true;
				else if (enable_time_windows_)
				{
					if (recv_size > 44) {
						memcpy(&time_offset, &data[header_length + 4], sizeof(int));
						if (use_gps_src)
							time_offset *= 1000;
						dt_time = time_s * 1000.0 + time_ns * 0.000001 + time_offset * 0.000001;
					}
					else
						dt_time = time_s * 1000.0 + time_ns * 0.000001;
					frame_win_num = dt_time / frame_interval_;
					current_pkg_num++;
					if (frame_win_num > last_frame_win_num) // when package is empty, dt may be located at last frame and {frame_win_num < last_frame_win_num} may occur in this situation
					{
						new_frame_received = true;
						last_frame_win_num = frame_win_num;
						current_pkg_num = 0;
					}
					else if (current_pkg_num > MDOP_DATA_MAX_PKG_NUM)
					{
						new_frame_received = true;
						last_frame_win_num = frame_win_num;
						current_pkg_num = 0;
#ifdef DEBUG_INFO
						std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
						std::cout << "Abnormal: timestamp exception detected!" << std::endl;
#endif // DEBUG_INFO
					}
				}
				if (new_frame_received)
				{
					processFrameBufferSphereCoord(point_cloud_tmp, point_cloud_backup, sphere_coord_frame_data_buffer, nFrame_last, total_lost_package, diff);
					if (diff > 1)
						lost_frames += (diff - 1);
				}
				nFrame_last = nFrame;
				
				data_block = nPoint;
				for (int i = 0; i < data_block; i++)
				{
					// angle
					val16_ang_h = *(int16_t*)&data[header_length + i * data_block_size];
					azimuth_h = val16_ang_h / 8192.0; // 2^13
					val16_ang_v = *(int16_t*)&data[header_length + i * data_block_size + 2];
					azimuth_v = val16_ang_v / 8192.0;
					switch (protocol_version)
					{
					case PROTOCOL_VERSION_AD2_B:
						sphere_coord_package_data.spherecoordp4_b.h_azimuth = val16_ang_h;
						sphere_coord_package_data.spherecoordp4_b.v_azimuth = val16_ang_v;
						break;
					case PROTOCOL_VERSION_AD2_C:
						sphere_coord_package_data.spherecoordp4_c.h_azimuth = val16_ang_h;
						sphere_coord_package_data.spherecoordp4_c.v_azimuth = val16_ang_v;
						break;
					case PROTOCOL_VERSION_AD2_HH:
						sphere_coord_package_data.spherecoordp4_c.h_azimuth = val16_ang_h;
						sphere_coord_package_data.spherecoordp4_c.v_azimuth = val16_ang_v;
						break;
					default:
						break;
					}
	
					// timestamp
					time_offset = *(int*)&data[header_length + i * data_block_size + 4];
					if (use_gps_src)
						time_offset *= 1000;
					pt.timestamp_s = time_s;

					if (time_offset < 0 && abs(time_offset) > time_ns)
					{
						pt.timestamp_ns = 1000000000 + (int)time_ns + time_offset;
						pt.timestamp_s--;
					}
					else
					{
						pt.timestamp_ns = time_ns + time_offset;
						if (pt.timestamp_ns >= 1000000000)
						{
							pt.timestamp_s += pt.timestamp_ns / 1000000000;
							pt.timestamp_ns %= 1000000000;
						}
					}
					if ((new_frame_received || last_pkg_received) && i == 0)
					{
						std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
						std::chrono::nanoseconds ns = tp.time_since_epoch();
						delay_time_ = ns.count() / 1000 - (long long)pt.timestamp_s * 1000000 - pt.timestamp_ns * 0.001;
					}

					//sphere coord data assignment
					float cos_h = std::cos(azimuth_h);
					float sin_h = std::sin(azimuth_h);
					float cos_v, sin_v;
					switch (protocol_version)
					{
					case PROTOCOL_VERSION_AD2_B:
						sphere_coord_package_data.spherecoordp4_b.time_offset = time_offset;
						sphere_coord_package_data.spherecoordp4_b.timestamp_s = pt.timestamp_s;
						sphere_coord_package_data.spherecoordp4_b.timestamp_ns = pt.timestamp_ns;
						memset(sphere_coord_package_data.spherecoordp4_b.dist, 0, 32 * sizeof(uint32_t));
						memset(sphere_coord_package_data.spherecoordp4_b.intensity, 0, 32 * sizeof(uint8_t));
						break;
					case PROTOCOL_VERSION_AD2_C:
						sphere_coord_package_data.spherecoordp4_c.time_offset = time_offset;
						sphere_coord_package_data.spherecoordp4_c.timestamp_s = pt.timestamp_s;
						sphere_coord_package_data.spherecoordp4_c.timestamp_ns = pt.timestamp_ns;
						memset(sphere_coord_package_data.spherecoordp4_c.dist, 0, 32 * sizeof(uint32_t));
						memset(sphere_coord_package_data.spherecoordp4_c.intensity, 0, 32 * sizeof(uint8_t));
						memset(sphere_coord_package_data.spherecoordp4_c.flag, 0, 32 * sizeof(uint8_t));
						break;
					case PROTOCOL_VERSION_AD2_HH:
						sphere_coord_package_data.spherecoordp4_c.time_offset = time_offset;
						sphere_coord_package_data.spherecoordp4_c.timestamp_s = pt.timestamp_s;
						sphere_coord_package_data.spherecoordp4_c.timestamp_ns = pt.timestamp_ns;
						memset(sphere_coord_package_data.spherecoordp4_c.dist, 0, 32 * sizeof(uint32_t));
						memset(sphere_coord_package_data.spherecoordp4_c.intensity, 0, 32 * sizeof(uint8_t));
						memset(sphere_coord_package_data.spherecoordp4_c.flag, 0, 32 * sizeof(uint8_t));
						break;
					default:
						break;
					}
					
					belongs_to_next_frame = false;
					if (enable_time_windows_)
						belongs_to_next_frame = (((long long)pt.timestamp_s * 1000.0 + pt.timestamp_ns * 0.000001) >= (frame_win_num + 1) * frame_interval_);
					// x y z intensity
					for (int j = 0; j < 16; j++)
					{
						dist32 = 0;
						if (enable_angle_calib_)
						{
							float azimuth_h_calib, azimuth_v_calib;
							switch (protocol_version)
							{
							case PROTOCOL_VERSION_AD2_B:
								angleCalibration_AD2_B(azimuth_h, azimuth_v, j, azimuth_h_calib, azimuth_v_calib);
								break;
							case PROTOCOL_VERSION_AD2_C:
								angleCalibration_AD2_C(azimuth_h, azimuth_v, j, azimuth_h_calib, azimuth_v_calib);
								break;
							case PROTOCOL_VERSION_AD2_HH:
								angleCalibration_AD2_C(azimuth_h, azimuth_v, j, azimuth_h_calib, azimuth_v_calib);
								break;
							default:
								break;
							}

							cos_h = std::cos(azimuth_h_calib);
							sin_h = std::sin(azimuth_h_calib);
							cos_v = std::cos(azimuth_v_calib);
							sin_v = std::sin(azimuth_v_calib);
						}
						else
						{
							cos_v = std::cos(azimuth_v);
							sin_v = std::sin(azimuth_v);
						}

						switch (protocol_version)
						{
						case PROTOCOL_VERSION_AD2_B:
							memcpy(&dist32, &data[header_length + i * data_block_size + 8 + j * point_bytes + 0], 3);
							pt.intensity = *(uint8_t*)&data[header_length + i * data_block_size + 8 + j * point_bytes + 3];

							sphere_coord_package_data.spherecoordp4_b.dist[2 * j] = dist32;
							sphere_coord_package_data.spherecoordp4_b.intensity[2 * j] = pt.intensity;
							break;
						case PROTOCOL_VERSION_AD2_C:
							memcpy(&dist32, &data[header_length + i * data_block_size + 8 + j * point_bytes + 0], 2);
							pt.intensity = *(uint8_t*)&data[header_length + i * data_block_size + 8 + j * point_bytes + 2];
							pt.confidence = (*(uint8_t*)&data[header_length + i * data_block_size + 8 + j * point_bytes + 3]) & 0x03;

							sphere_coord_package_data.spherecoordp4_c.dist[2 * j] = dist32;
							sphere_coord_package_data.spherecoordp4_c.intensity[2 * j] = pt.intensity;
							sphere_coord_package_data.spherecoordp4_c.flag[2 * j] = pt.confidence;
							break;
						case PROTOCOL_VERSION_AD2_HH:
							memcpy(&dist32, &data[header_length + i * data_block_size + 8 + j * point_bytes + 0], 2);
							pt.intensity = *(uint8_t*)&data[header_length + i * data_block_size + 8 + j * point_bytes + 2];
							pt.confidence = (*(uint8_t*)&data[header_length + i * data_block_size + 8 + j * point_bytes + 3]);

							sphere_coord_package_data.spherecoordp4_c.dist[2 * j] = dist32;
							sphere_coord_package_data.spherecoordp4_c.intensity[2 * j] = pt.intensity;
							sphere_coord_package_data.spherecoordp4_c.flag[2 * j] = pt.confidence;
							break;
						default:
							break;
						}
						dist_f = (float)dist32 / 100.0;
						pt.x = dist_f * cos_v * cos_h;
						pt.y = dist_f * cos_v * sin_h;
						pt.z = dist_f * sin_v;
						pt.row = nLine * 16 + (uint16_t)j;
						pt.channel = (uint8_t)j;
						pt.echo = echo_mark_1;

						// roi flag
						if (roi_center_x_flag != 0xFFFF && roi_center_y_flag != 0xFFFF)
						{
							switch (protocol_version)
							{
							case PROTOCOL_VERSION_AD2_B:
								if (std::fabs(azimuth_h - roi_center_x) < roi_half_x && std::fabs(azimuth_v - roi_center_y) < roi_half_y)
									pt.roi = 1;
								else
									pt.roi = 0;
								break;
							case PROTOCOL_VERSION_AD2_C:
								if (std::fabs(azimuth_h * 180.0 / kPI - roi_center_x) < roi_half_x && std::fabs(azimuth_v * 180.0 / kPI - roi_center_y) < roi_half_y)
									pt.roi = 1;
								else
									pt.roi = 0;
								break;
							case PROTOCOL_VERSION_AD2_HH:
								if ((pt.confidence & 0x08) == 0x08)
									pt.roi = 1;
								else
									pt.roi = 0;
								break;
							default:
								break;
							}
						}
						else
							pt.roi = 0;

						if (belongs_to_next_frame)
						{
							point_cloud_backup->points.push_back(pt);
						}
						else
						{
							point_cloud_tmp->points.push_back(pt);
						}

						if (echo_mode == 2)
						{
							switch (protocol_version)
							{
							case PROTOCOL_VERSION_AD2_B:
								memcpy(&dist32, &data[header_length + i * data_block_size + 8 + 64 + j * point_bytes + 0], 3);
								pt.intensity = *(uint8_t*)&data[header_length + i * data_block_size + 8 + 64 + j * point_bytes + 3];

								sphere_coord_package_data.spherecoordp4_b.dist[2 * j + 1] = dist32;
								sphere_coord_package_data.spherecoordp4_b.intensity[2 * j + 1] = pt.intensity;
								break;
							case PROTOCOL_VERSION_AD2_C:
								memcpy(&dist32, &data[header_length + i * data_block_size + 8 + 64 + j * point_bytes + 0], 2);
								pt.intensity = *(uint8_t*)&data[header_length + i * data_block_size + 8 + 64 + j * point_bytes + 2];
								pt.confidence = (*(uint8_t*)&data[header_length + i * data_block_size + 8 + 64 + j * point_bytes + 3]) & 0x03;

								sphere_coord_package_data.spherecoordp4_c.dist[2 * j + 1] = dist32;
								sphere_coord_package_data.spherecoordp4_c.intensity[2 * j + 1] = pt.intensity;
								sphere_coord_package_data.spherecoordp4_c.flag[2 * j + 1] = pt.confidence;
								break;
							case PROTOCOL_VERSION_AD2_HH:
								memcpy(&dist32, &data[header_length + i * data_block_size + 8 + 64 + j * point_bytes + 0], 2);
								pt.intensity = *(uint8_t*)&data[header_length + i * data_block_size + 8 + 64 + j * point_bytes + 2];
								pt.confidence = (*(uint8_t*)&data[header_length + i * data_block_size + 8 + 64 + j * point_bytes + 3]);

								sphere_coord_package_data.spherecoordp4_c.dist[2 * j + 1] = dist32;
								sphere_coord_package_data.spherecoordp4_c.intensity[2 * j + 1] = pt.intensity;
								sphere_coord_package_data.spherecoordp4_c.flag[2 * j + 1] = pt.confidence;
								break;
							default:
								break;
							}
							dist_f = (float)dist32 / 100.0;
							pt.x = dist_f * cos_v * cos_h;
							pt.y = dist_f * cos_v * sin_h;
							pt.z = dist_f * sin_v;
							pt.row = nLine * 16 + (uint16_t)j;
							pt.channel = (uint8_t)j;
							pt.echo = echo_mark_2;

							// roi flag
							if (roi_center_x_flag != 0xFFFF && roi_center_y_flag != 0xFFFF)
							{
								switch (protocol_version)
								{
								case PROTOCOL_VERSION_AD2_B:
									if (std::fabs(azimuth_h - roi_center_x) < roi_half_x && std::fabs(azimuth_v - roi_center_y) < roi_half_y)
										pt.roi = 1;
									else
										pt.roi = 0;
									break;
								case PROTOCOL_VERSION_AD2_C:
									if (std::fabs(azimuth_h * 180.0 / kPI - roi_center_x) < roi_half_x && std::fabs(azimuth_v * 180.0 / kPI - roi_center_y) < roi_half_y)
										pt.roi = 1;
									else
										pt.roi = 0;
									break;
								case PROTOCOL_VERSION_AD2_HH:
									if ((pt.confidence & 0x08) == 0x08)
										pt.roi = 1;
									else
										pt.roi = 0;
									break;
								default:
									break;
								}
							}
							else
								pt.roi = 0;
							
							if (belongs_to_next_frame)
							{
								point_cloud_backup->points.push_back(pt);
							}
							else
							{
								point_cloud_tmp->points.push_back(pt);
							}
						}
						if (!enable_angle_calib_)
						{
							switch (protocol_version)
							{
							case PROTOCOL_VERSION_AD2_B:
								azimuth_v -= ang2Rad(kAngRes_AD2);
								break;
							case PROTOCOL_VERSION_AD2_C:
								azimuth_v -= ang2Rad(kAngRes_AD2 * 2);
								break;
							case PROTOCOL_VERSION_AD2_HH:
								azimuth_v -= ang2Rad(kAngRes_AD2 * 2);
								break;
							default:
								break;
							}
						}	
					}
					sphere_coord_frame_data_buffer->data.push_back(sphere_coord_package_data);
				}
			
				if (last_pkg_received)
				{
					processFrameBufferSphereCoord(point_cloud_tmp, point_cloud_backup, sphere_coord_frame_data_buffer, nFrame, total_lost_package, diff);
					if (diff > 1)
						lost_frames += (diff - 1);
					nFrame_last++;
				}
			}
			else
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::printf("Point cloud data package header/tail error: 0x%02x 0x%02x 0x%02x 0x%02x ... 0x%02x 0x%02x!\n",
					data[0], data[1], data[2], data[3], data[recv_size - 2], data[recv_size - 1]);
			}
			
			// update ring buffer tail
			recv_buffer_info_.tail = (recv_buffer_info_.tail + 1) % MDOP_DATA_RING_BUFFER_MAX_LENGTH;
		}

		statistics_stop = std::chrono::system_clock::now();
		auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(statistics_stop - statistics_start);
		if (dur.count() > 10000)
		{
			// update data statics
			if (pkg_count < statistics_start_pkg)
				pkg_loss_rate_ = (float)lost_pkgs / float((UINT32_MAX - statistics_start_pkg) + pkg_count) * 100.0;
			else if (pkg_count == statistics_start_pkg)
				pkg_loss_rate_ = 100.0;
			else
				pkg_loss_rate_ = (float)lost_pkgs / float(pkg_count - statistics_start_pkg) * 100.0;
			lost_pkgs = 0;
			statistics_start_pkg = pkg_count;


			if (nFrame < statistics_start_frame)
				frame_loss_rate_ = (float)lost_frames / float((UINT16_MAX - statistics_start_frame) + nFrame) * 100.0;
			else if (nFrame == statistics_start_frame)
				frame_loss_rate_ = 100.0;
			else
				frame_loss_rate_ = (float)lost_frames / float(nFrame - statistics_start_frame) * 100.0;
			lost_frames = 0;
			statistics_start_frame = nFrame;

			statistics_start = statistics_stop;
		}
	}
}

void benewake::MDOPProtocolP4::testDataDecode()
{
	test_frame_data_queue_ = std::make_shared<benewake::MsgQueue<benewake::BwTestDataP4Frame::Ptr>>("msg", 5);
	benewake::BwTestDataP4Frame::Ptr test_frame_data_buffer = std::make_shared<benewake::BwTestDataP4Frame>();
	test_frame_data_ = std::make_shared<benewake::BwTestDataP4Frame>();
	unsigned char* data = new unsigned char[PROTOCOL_DATA_PACKAGE_MAX_LENGTH];
	memset(data, 0, PROTOCOL_DATA_PACKAGE_MAX_LENGTH);
	BwTestDataP4 p4_src_data;
	int recv_size = 0, total_bytes = 0, point_bytes = 72;
	uint32_t pkg_count = 0, last_pkg_count = 0;
	uint64_t time_s = 0;
	uint32_t time_ns = 0;
	uint32_t check_sum = 0, valu32 = 0;
	uint16_t nFrame = 0, nLine = 0, nPoint = 0, uval16 = 0, protocol_version = 0, last_pkg_mark = 0;
	uint16_t data_offset = 0;
	int total_num = 0, eff_num = 0;
	uint32_t package_diff = 0;
	bool new_frame_received = false;
	bool last_pkg_received = false;
	uint16_t nFrame_last = 0;

	p4_src_data.bwtestdatap4_b.prism_code = 0;
	p4_src_data.bwtestdatap4_b.galvo_code = 0;
	p4_src_data.bwtestdatap4_b.pulse_width = 0;
	p4_src_data.bwtestdatap4_b.dac_data = 0;
	p4_src_data.bwtestdatap4_b.ch_r_0[16] = { 0 };
	p4_src_data.bwtestdatap4_b.ch_f_0[16] = { 0 };
	p4_src_data.bwtestdatap4_b.ch_r_1[16] = { 0 };
	p4_src_data.bwtestdatap4_b.ch_f_1[16] = { 0 };
	p4_src_data.bwtestdatap4_b.ch_r_2[16] = { 0 };
	p4_src_data.bwtestdatap4_b.ch_f_2[16] = { 0 };
	p4_src_data.bwtestdatap4_b.ch_r_3[16] = { 0 };
	p4_src_data.bwtestdatap4_b.ch_f_3[16] = { 0 };
	p4_src_data.bwtestdatap4_b.ch_r_cnt[16] = { 0 };
	p4_src_data.bwtestdatap4_b.ch_f_cnt[16] = { 0 };

	p4_src_data.bwtestdatap4_c.luminous_period = 0;
	p4_src_data.bwtestdatap4_c.prism_code = 0;
	p4_src_data.bwtestdatap4_c.galvo_code = 0;
	p4_src_data.bwtestdatap4_c.st_ch_r[256] = { 0 };
	p4_src_data.bwtestdatap4_c.st_ch_f[256] = { 0 };
	p4_src_data.bwtestdatap4_c.st_ch_r_cnt[64] = { 0 };
	p4_src_data.bwtestdatap4_c.st_ch_f_cnt[64] = { 0 };
	p4_src_data.bwtestdatap4_c.main_ch_r[4] = { 0 };
	p4_src_data.bwtestdatap4_c.main_ch_f[4] = { 0 };
	p4_src_data.bwtestdatap4_c.main_ch_r_cnt = 0;
	p4_src_data.bwtestdatap4_c.main_ch_f_cnt = 0;
	p4_src_data.bwtestdatap4_c.dac_out[16] = { 0 };
	p4_src_data.bwtestdatap4_c.dac_stage_2 = 0;
	p4_src_data.bwtestdatap4_c.dac_stage_3 = 0;
	p4_src_data.bwtestdatap4_c.dac_stage_4 = 0;

	while (thread_run_)
	{
		recv_size = pudp_->recvSomeData(data, PROTOCOL_DATA_PACKAGE_MAX_LENGTH, 0, 500000);
		if (recv_size < 0)
		{
//#ifdef DEBUG_INFO
//			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
//			std::cout << "Receive test data failed!" << std::endl;
//#endif // DEBUG_INFO
		}
		else if (recv_size < 44)
		{
#ifdef DEBUG_INFO
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Test data package size error! The package should be larger than 44 bytes but only "
				<< recv_size << " bytes received." << std::endl;
#endif // DEBUG_INFO
			continue;
		}
		//else if (recv_size > 400)
		//{
		//	std::cout << "Packet error. " << std::endl;
		//	continue;
		//}
		else
		{
			if (data[0] == 0x42 && data[1] == 0x57 && data[2] == PRODUCT_ID_P4 && data[3] == PROTOCOL_ID_MDOP
				&& data[recv_size - 2] == 0x00 && data[recv_size - 1] == 0xff)
			{
				protocol_version = *(uint16_t*)&data[PROTOCOL_DATA_PACKAGE_VERSION_OFFSET];
				pkg_count = *(uint32_t*)&data[PROTOCOL_DATA_PACKAGE_COUNT_OFFSET];
				nFrame = *(uint16_t*)&data[PROTOCOL_DATA_FRAME_COUNT_OFFSET];
				uval16 = *(uint16_t*)&data[PROTOCOL_DATA_LINE_NUM_OFFSET];
				nLine = uval16 & 0x7fff;
				last_pkg_mark = uval16 & 0x8000;
				nPoint = *(uint16_t*)&data[PROTOCOL_DATA_POINT_COUNT_OFFSET];
				switch (protocol_version)
				{
				case PROTOCOL_VERSION_AD2_B:
					time_s = *(uint32_t*)&data[PROTOCOL_AD2_B_DATA_TIME_S_OFFSET];
					time_ns = *(uint32_t*)&data[PROTOCOL_AD2_B_DATA_TIME_NS_OFFSET];
					data_offset = 38;
					break;
				case PROTOCOL_VERSION_AD2_C:
					time_s = *(uint64_t*)&data[PROTOCOL_AD2_DATA_TIME_S_OFFSET];
					time_ns = *(uint32_t*)&data[PROTOCOL_AD2_DATA_TIME_NS_OFFSET];
					data_offset = 42;
					break;
				case PROTOCOL_VERSION_AD2_HH:
					time_s = *(uint64_t*)&data[PROTOCOL_AD2_DATA_TIME_S_OFFSET];
					time_ns = *(uint32_t*)&data[PROTOCOL_AD2_DATA_TIME_NS_OFFSET];
					data_offset = 42;
					break;
				default:
					break;
				}

				sys_info_.protocol = protocol_version;
				if (sys_info_.protocol == PROTOCOL_VERSION_AD2_B && recv_size > 400) //B样
				{
					std::cout << "Packet error. " << std::endl;
					continue;
				}
				else if ((sys_info_.protocol == PROTOCOL_VERSION_AD2_C || sys_info_.protocol == PROTOCOL_VERSION_AD2_HH) && (recv_size > 1350 || recv_size < 1000)) //C样
				{
					std::cout << "Packet error. " << std::endl;
					continue;
				}
				last_pkg_received = false;
				new_frame_received = false;
				if (last_pkg_mark == 0x8000)
					last_pkg_received = true;
				if (uint16_t(nFrame - nFrame_last) >= 1)
					new_frame_received = true;

				if (new_frame_received && test_frame_data_buffer->data.size() != 0)
				{
					processFrameBufferTestData(test_frame_data_buffer, nFrame_last);
				}
				nFrame_last = nFrame;
				getTestDataFromBuffer(&p4_src_data, data, data_offset);
				src_data_ = p4_src_data;

				package_diff = pkg_count - last_pkg_count;
				if (package_diff > 1)
				{
					std::cout << "Packet loss: " << package_diff << std::endl;
				}
				last_pkg_count = pkg_count;

				test_frame_data_buffer->data.push_back(p4_src_data);

				if (last_pkg_received && test_frame_data_buffer->data.size() != 0)
				{
					processFrameBufferTestData(test_frame_data_buffer, nFrame);
					nFrame_last++;
				}
			}
			else
			{
				std::cout << "recv data size err! size:" << recv_size << std::endl;
			}
		}
	}
	std::cout << "Test data thread close! " << thread_run_ << std::endl;
	delete[] data;
}

void benewake::MDOPProtocolP4::treatEchoMode(uint8_t _echo_flag, int& _echo_mode, uint8_t& _echo_mark_1, uint8_t& _echo_mark_2)
{
	if (_echo_flag <= 0x02)
	{
		_echo_mode = 1;
		if (_echo_flag == 0x00)
			_echo_mark_1 = 0x01;
		else if (_echo_flag == 0x01)
			_echo_mark_1 = 0x02;
		else
			_echo_mark_1 = 0x04;
	}
	else
	{
		_echo_mode = 2;
		if (_echo_flag == 0x03)
		{
			_echo_mark_1 = 0x02;
			_echo_mark_2 = 0x04;
		}
		else if (_echo_flag == 0x04)
		{
			_echo_mark_1 = 0x01;
			_echo_mark_2 = 0x04;
		}
		else
		{
			_echo_mark_1 = 0x01;
			_echo_mark_2 = 0x02;
		}
	}
}

void benewake::MDOPProtocolP4::getTestDataFromBuffer(benewake::BwTestDataP4 * _test_data, unsigned char* _buffer, uint16_t _data_offset)
{
	if (sys_info_.protocol == PROTOCOL_VERSION_AD2_B) //B样
	{
		for (int i = 0; i < 16; i++)
		{
			_test_data->bwtestdatap4_b.ch_r_0[i] = *(uint16_t*)&_buffer[_data_offset];
			_test_data->bwtestdatap4_b.ch_r_1[i] = *(uint16_t*)&_buffer[_data_offset + 2];
			_test_data->bwtestdatap4_b.ch_r_2[i] = *(uint16_t*)&_buffer[_data_offset + 4];
			_test_data->bwtestdatap4_b.ch_r_3[i] = *(uint16_t*)&_buffer[_data_offset + 6];
			_test_data->bwtestdatap4_b.ch_f_0[i] = *(uint16_t*)&_buffer[_data_offset + 8];
			_test_data->bwtestdatap4_b.ch_f_1[i] = *(uint16_t*)&_buffer[_data_offset + 10];
			_test_data->bwtestdatap4_b.ch_f_2[i] = *(uint16_t*)&_buffer[_data_offset + 12];
			_test_data->bwtestdatap4_b.ch_f_3[i] = *(uint16_t*)&_buffer[_data_offset + 14];
			_data_offset += 16;
		}
		for (int i = 0; i < 16; i++)
		{
			//获取通道上升沿下降沿计数
			_test_data->bwtestdatap4_b.ch_r_cnt[i] = *(uint8_t*)&_buffer[_data_offset];
			_test_data->bwtestdatap4_b.ch_f_cnt[i] = *(uint8_t*)&_buffer[_data_offset + 1];
			_data_offset += 2;
		}
		_test_data->bwtestdatap4_b.prism_code = *(uint32_t*)&_buffer[_data_offset];
		_test_data->bwtestdatap4_b.galvo_code = *(uint32_t*)&_buffer[_data_offset + 4];
		_test_data->bwtestdatap4_b.pulse_width = *(uint16_t*)&_buffer[_data_offset + 8];
		_test_data->bwtestdatap4_b.dac_data = *(uint16_t*)&_buffer[_data_offset + 10];
	}
	else if (sys_info_.protocol == PROTOCOL_VERSION_AD2_C || sys_info_.protocol == PROTOCOL_VERSION_AD2_HH) //C样
	{
		//四档阈值通道1~通道16的4回波信息
		for (int i = 0; i < 4; i++)  //回波
		{
			for (int j = 0; j < 4; j++)  //档位
			{
				for (int k = 0; k < 16; k++)  //通道
				{
					_test_data->bwtestdatap4_c.st_ch_r[i * 4 * 16 + j * 16 + k] = *(uint16_t*)&_buffer[_data_offset];
					_test_data->bwtestdatap4_c.st_ch_f[i * 4 * 16 + j * 16 + k] = *(uint16_t*)&_buffer[_data_offset + 2];
					_data_offset += 4;
				}
			}
		}
		//四档阈值通道的回波个数
		for (int i = 0; i < 4; i++)  //档位
		{
			for (int j = 0; j < 16; j++)
			{
				//获取通道上升沿下降沿计数
				_test_data->bwtestdatap4_c.st_ch_r_cnt[i * 16 + j] = *(uint8_t*)&_buffer[_data_offset];
				_test_data->bwtestdatap4_c.st_ch_f_cnt[i * 16 + j] = *(uint8_t*)&_buffer[_data_offset + 1];
				_data_offset += 2;
			}
		}
		//主波通道信息
		for (int i = 0; i < 4; i++)
		{
			_test_data->bwtestdatap4_c.main_ch_r[i] = *(uint16_t*)&_buffer[_data_offset];
			_test_data->bwtestdatap4_c.main_ch_f[i] = *(uint16_t*)&_buffer[_data_offset + 2];
			_data_offset += 4;
		}
		_test_data->bwtestdatap4_c.main_ch_r_cnt = *(uint8_t*)&_buffer[_data_offset];
		_test_data->bwtestdatap4_c.main_ch_f_cnt = *(uint8_t*)&_buffer[_data_offset + 1];
		_data_offset += 2;

		_test_data->bwtestdatap4_c.luminous_period = *(uint16_t*)&_buffer[_data_offset];
		_test_data->bwtestdatap4_c.prism_code = *(uint32_t*)&_buffer[_data_offset + 2];
		_test_data->bwtestdatap4_c.galvo_code = *(uint32_t*)&_buffer[_data_offset + 6];
		_data_offset += 10;

		for (int i = 0; i < 16; i++)
		{
			_test_data->bwtestdatap4_c.dac_out[i] = *(uint8_t*)&_buffer[_data_offset];
			_data_offset += 1;
		}
		_test_data->bwtestdatap4_c.dac_stage_2 = *(uint8_t*)&_buffer[_data_offset];
		_test_data->bwtestdatap4_c.dac_stage_3 = *(uint8_t*)&_buffer[_data_offset + 1];
		_test_data->bwtestdatap4_c.dac_stage_4 = *(uint8_t*)&_buffer[_data_offset + 2];
	}
}

void benewake::MDOPProtocolP4::saveTestData(benewake::BwTestDataP4&_data)
{
	if (sys_info_.protocol == PROTOCOL_VERSION_AD2_B) //B样
	{
		data_out_2_file_ << _data.bwtestdatap4_b.prism_code << " ";
		data_out_2_file_ << _data.bwtestdatap4_b.galvo_code << " ";
		data_out_2_file_ << _data.bwtestdatap4_b.pulse_width << " ";
		data_out_2_file_ << _data.bwtestdatap4_b.dac_data << " ";
		for (int i = 0; i < 16; i++)
		{
			data_out_2_file_ << _data.bwtestdatap4_b.ch_r_0[i] << " ";
			data_out_2_file_ << _data.bwtestdatap4_b.ch_r_1[i] << " ";
			data_out_2_file_ << _data.bwtestdatap4_b.ch_r_2[i] << " ";
			data_out_2_file_ << _data.bwtestdatap4_b.ch_r_3[i] << " ";
			data_out_2_file_ << _data.bwtestdatap4_b.ch_r_cnt[i] << " ";
			data_out_2_file_ << _data.bwtestdatap4_b.ch_f_0[i] << " ";
			data_out_2_file_ << _data.bwtestdatap4_b.ch_f_1[i] << " ";
			data_out_2_file_ << _data.bwtestdatap4_b.ch_f_2[i] << " ";
			data_out_2_file_ << _data.bwtestdatap4_b.ch_f_3[i] << " ";
			data_out_2_file_ << _data.bwtestdatap4_b.ch_f_cnt[i] << " ";
		}
	}
	else if (sys_info_.protocol == PROTOCOL_VERSION_AD2_C || sys_info_.protocol == PROTOCOL_VERSION_AD2_HH) //C样
	{
		data_out_2_file_ << _data.bwtestdatap4_c.luminous_period << " ";
		data_out_2_file_ << _data.bwtestdatap4_c.prism_code << " ";
		data_out_2_file_ << _data.bwtestdatap4_c.galvo_code << " ";
		for (int i = 0; i < 16; i++)
		{
			data_out_2_file_ << _data.bwtestdatap4_c.dac_out[i] << " ";
		}
		data_out_2_file_ << _data.bwtestdatap4_c.dac_stage_2 << " ";
		data_out_2_file_ << _data.bwtestdatap4_c.dac_stage_3 << " ";
		data_out_2_file_ << _data.bwtestdatap4_c.dac_stage_4 << " ";
		for (int i = 0; i < 4; i++)
		{
			data_out_2_file_ << _data.bwtestdatap4_c.main_ch_r[i] << " ";
			data_out_2_file_ << _data.bwtestdatap4_c.main_ch_f[i] << " ";
		}
		data_out_2_file_ << _data.bwtestdatap4_c.main_ch_r_cnt << " ";
		data_out_2_file_ << _data.bwtestdatap4_c.main_ch_f_cnt << " ";
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				for (int k = 0; k < 16; k++)
				{
					data_out_2_file_ << _data.bwtestdatap4_c.st_ch_r[i * 4 * 16 + j * 16 + k] << " ";
					data_out_2_file_ << _data.bwtestdatap4_c.st_ch_f[i * 4 * 16 + j * 16 + k] << " ";
				}
			}
		}
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 16; j++)
			{
				data_out_2_file_ << _data.bwtestdatap4_c.st_ch_r_cnt[i * 16 + j] << " ";
				data_out_2_file_ << _data.bwtestdatap4_c.st_ch_f_cnt[i * 16 + j] << " ";
			}
		}
	}
	data_out_2_file_ << std::endl;
}

void benewake::MDOPProtocolP4::saveTestDataOnceFrame(BwTestDataP4Frame::Ptr _test_data)
{
	for (int i = 0; i < _test_data->data.size(); i++)
	{
		saveTestData(_test_data->data[i]);
	}
}

void benewake::MDOPProtocolP4::saveSphereCoordData(BwSphereCoordP4Frame::Ptr _data)
{
	if (_data == NULL)
	{
		std::cout << "_data == NULL" << std::endl;
		return;
	}

	uint32_t data_len = _data->data.size();
	for (int i = 0; i < data_len; i++)
	{
		if (sys_info_.protocol == PROTOCOL_VERSION_AD2_B) //B样
		{
			data_out_2_file_ << _data->data[i].spherecoordp4_b.h_azimuth << " ";
			data_out_2_file_ << _data->data[i].spherecoordp4_b.v_azimuth << " ";
			data_out_2_file_ << _data->data[i].spherecoordp4_b.time_offset << " ";
			data_out_2_file_ << _data->data[i].spherecoordp4_b.timestamp_s << " ";
			data_out_2_file_ << _data->data[i].spherecoordp4_b.timestamp_ns << " ";
			for (int ch = 0; ch < 32; ch++)
			{
				data_out_2_file_ << _data->data[i].spherecoordp4_b.dist[ch] << " ";
				uint16_t intensity = (uint16_t)_data->data[i].spherecoordp4_b.intensity[ch];
				data_out_2_file_ << intensity << " ";
			}
		}
		else if (sys_info_.protocol == PROTOCOL_VERSION_AD2_C || sys_info_.protocol == PROTOCOL_VERSION_AD2_HH)
		{
			data_out_2_file_ << _data->data[i].spherecoordp4_c.h_azimuth << " ";
			data_out_2_file_ << _data->data[i].spherecoordp4_c.v_azimuth << " ";
			data_out_2_file_ << _data->data[i].spherecoordp4_c.time_offset << " ";
			data_out_2_file_ << _data->data[i].spherecoordp4_c.timestamp_s << " ";
			data_out_2_file_ << _data->data[i].spherecoordp4_c.timestamp_ns << " ";
			for (int ch = 0; ch < 32; ch++)
			{
				data_out_2_file_ << _data->data[i].spherecoordp4_c.dist[ch] << " ";
				uint16_t intensity = (uint16_t)_data->data[i].spherecoordp4_c.intensity[ch];
				uint16_t confidence = (uint16_t)_data->data[i].spherecoordp4_c.flag[ch];
				data_out_2_file_ << intensity << " ";
				data_out_2_file_ << confidence << " ";
			}
		}
		data_out_2_file_ << std::endl;
	}
}

void benewake::MDOPProtocolP4::saveDataThread()
{
	save_data_thread_run_ = true;
	uint32_t end_times = 100;
	while (save_data_thread_run_)
	{
		if (sphere_coord_frame_data_queue_!= NULL && sphere_coord_frame_data_queue_->size() > 0)
		{
			BwSphereCoordP4Frame::Ptr cur_data = sphere_coord_frame_data_queue_->popMsg();
			openDataOutputStream();
			saveSphereCoordData(cur_data);
			closeDataOutputStream();
		}
		else if (test_frame_data_queue_ != NULL && test_frame_data_queue_->size() > 0)
		{
			BwTestDataP4Frame::Ptr cur_data = test_frame_data_queue_->popMsg();
			openDataOutputStream();
			saveTestDataOnceFrame(cur_data);
			closeDataOutputStream();
		}
		else
		{
#ifdef _WIN32
			Sleep(10);
#else
			usleep(10000);
#endif // _WIN32
			end_times--;
			if (end_times == 0)
			{
				save_data_thread_run_ = false;
				break;
			}
		}
	}
	std::cout << "Save data thread terminated." << std::endl;
}

void benewake::MDOPProtocolP4::openDataOutputStream()
{
	std::string file_name;
	mkDir(save_path_);
	if (save_file_name_ == "")
	{
		time_t timep(time(NULL));
		char time_char[256];
		struct tm* nowTime = localtime(&timep);
		strftime(time_char, sizeof(time_char), "/%Y-%m-%d_%H_%M_%S_", nowTime);
		std::string time_str(time_char);
		auto now_epoch = std::chrono::system_clock::now();
		int ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_epoch.time_since_epoch()).count() % 1000;
		file_name = save_path_ + time_str + std::to_string(ms) + ".txt";
	}
	else
	{
		file_name = save_path_ + "/" + save_file_name_ + std::to_string(save_name_flag_) + ".txt";
		save_name_flag_++;
	}
	data_out_2_file_.open(file_name);
	if (protocol_type_ == benewake::BwProtocolType::TEST)
	{
		if (sys_info_.protocol == PROTOCOL_VERSION_AD2_B) //B样
		{
			data_out_2_file_ << "prism_code" << " ";
			data_out_2_file_ << "galvo_code" << " ";
			data_out_2_file_ << "pulse_width" << " ";
			data_out_2_file_ << "dac_data" << " ";
			for (int i = 0; i < 16; i++)
			{
				data_out_2_file_ << "ch_" << i << "_r_0" << " ";
				data_out_2_file_ << "ch_" << i << "_r_1" << " ";
				data_out_2_file_ << "ch_" << i << "_r_2" << " ";
				data_out_2_file_ << "ch_" << i << "_r_3" << " ";
				data_out_2_file_ << "ch_" << i << "_r_cnt" << " ";
				data_out_2_file_ << "ch_" << i << "_f_0" << " ";
				data_out_2_file_ << "ch_" << i << "_f_1" << " ";
				data_out_2_file_ << "ch_" << i << "_f_2" << " ";
				data_out_2_file_ << "ch_" << i << "_f_3" << " ";
				data_out_2_file_ << "ch_" << i << "_f_cnt" << " ";
			}
		}
		else if (sys_info_.protocol == PROTOCOL_VERSION_AD2_C || sys_info_.protocol == PROTOCOL_VERSION_AD2_HH) //C样
		{
			data_out_2_file_ << "luminous_period" << " ";
			data_out_2_file_ << "prism_code" << " ";
			data_out_2_file_ << "galvo_code" << " ";
			for (int i = 0; i < 16; i++)
			{
				data_out_2_file_ << "dac_out" << i << " ";
			}
			data_out_2_file_ << "dac_stage_2" << " ";
			data_out_2_file_ << "dac_stage_3" << " ";
			data_out_2_file_ << "dac_stage_4" << " ";
			for (int i = 0; i < 4; i++)
			{
				data_out_2_file_ << "main_ch_r_" << i << " ";
				data_out_2_file_ << "main_ch_f_" << i << " ";
			}
			data_out_2_file_ << "main_ch_r_cnt" << " ";
			data_out_2_file_ << "main_ch_f_cnt" << " ";
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					for (int k = 0; k < 16; k++)
					{
						data_out_2_file_ << "st" << j << "_ch" << k << "_r_" << i << " ";
						data_out_2_file_ << "st" << j << "_ch" << k << "_f_" << i << " ";
					}
				}
			}
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 16; j++)
				{
					data_out_2_file_ << "st" << i << "_ch" << j << "_r_cnt" << " ";
					data_out_2_file_ << "st" << i << "_ch" << j << "_f_cnt" << " ";
				}
			}
		}
		
		data_out_2_file_ << std::endl;
	}
	else if(protocol_type_ == benewake::BwProtocolType::SPHERE)
	{
		if (sys_info_.protocol == PROTOCOL_VERSION_AD2_B)
		{
			data_out_2_file_ << "h_azimuth" << " ";
			data_out_2_file_ << "v_azimuth" << " ";
			data_out_2_file_ << "time_offset" << " ";
			data_out_2_file_ << "timestamp_s" << " ";
			data_out_2_file_ << "timestamp_ns" << " ";
			for (int ch = 0; ch < 16; ch++)
			{
				data_out_2_file_ << "dist_" << ch << "_echo_0 ";
				data_out_2_file_ << "intensity_" << ch << "_echo_0 ";
				data_out_2_file_ << "dist_" << ch << "_echo_1 ";
				data_out_2_file_ << "intensity_" << ch << "_echo_1 ";
			}
		}
		else if (sys_info_.protocol == PROTOCOL_VERSION_AD2_C || sys_info_.protocol == PROTOCOL_VERSION_AD2_HH)
		{
			data_out_2_file_ << "h_azimuth" << " ";
			data_out_2_file_ << "v_azimuth" << " ";
			data_out_2_file_ << "time_offset" << " ";
			data_out_2_file_ << "timestamp_s" << " ";
			data_out_2_file_ << "timestamp_ns" << " ";
			for (int ch = 0; ch < 16; ch++)
			{
				data_out_2_file_ << "dist_" << ch << "_echo_0 ";
				data_out_2_file_ << "intensity_" << ch << "_echo_0 ";
				data_out_2_file_ << "reserved_" << ch << "_echo_0 ";
				data_out_2_file_ << "dist_" << ch << "_echo_1 ";
				data_out_2_file_ << "intensity_" << ch << "_echo_1 ";
				data_out_2_file_ << "reserved_" << ch << "_echo_1 ";
			}
		}
		data_out_2_file_ << std::endl;
	}
	
	std::cout << "save path: " << file_name << std::endl;
}

void benewake::MDOPProtocolP4::closeDataOutputStream()
{
	data_out_2_file_.close();
}

void benewake::MDOPProtocolP4::processFrameBufferSphereCoord(
	benewake::BwPointCloud::Ptr& _frame_buf, 
	benewake::BwPointCloud::Ptr& _next_frame_buf, 
	benewake::BwSphereCoordP4Frame::Ptr& _backup_sphere_buf, 
	uint16_t& _frame_id, 
	uint32_t& _lost_pkg_count, 
	uint32_t& _diff_frame)
{
	float pkg_loss_rate = 192.0 * (float)_lost_pkg_count / (float)(_frame_buf->points.size() + 192.0 * _lost_pkg_count);
#ifdef _WIN32
	WaitForSingleObject(occup_, INFINITE);
#else
	sem_wait(&occup_);
#endif // _WIN32
	sphere_coord_frame_data_.reset();
	sphere_coord_frame_data_ = _backup_sphere_buf;
	point_cloud_buffer_.reset();
	point_cloud_buffer_ = _frame_buf;
	point_cloud_buffer_->pkg_loss = pkg_loss_rate;
	frame_id_ = _frame_id;
	new_frame_ = true;
#ifdef _WIN32
	ReleaseSemaphore(occup_, 1, NULL);
#else
	sem_post(&occup_);
#endif // _WIN32
	// push to time window data queue
	if (enable_time_windows_)
		data_queue_->pushMsg(_frame_buf);

	_frame_buf.reset(new BwPointCloud());
	_frame_buf->points.reserve(1100000);
	if (enable_time_windows_)
	{
		_frame_buf->points.insert(_frame_buf->points.end(), _next_frame_buf->points.begin(), _next_frame_buf->points.end());
		_next_frame_buf->points.clear();
	}

	if (callback_enable_)
	{
		callback_func_(point_cloud_buffer_, frame_id_, callback_pointer_);
	}

	_backup_sphere_buf.reset(new BwSphereCoordP4Frame());

	if (_lost_pkg_count > 0)
	{
		std::cout << "INFO: Package lost in last frame " << _lost_pkg_count << std::endl;
		_lost_pkg_count = 0;
	}
	if (_diff_frame > 1)
	{
		std::cout << "INFO: Frame Lost " << (_diff_frame - 1) << std::endl;
	}

	if (save_times_ > 0)
	{
		if (!save_data_thread_run_)
		{
			save_data_thread_ = new std::thread(&MDOPProtocolP4::saveDataThread, this);
		}
		sphere_coord_frame_data_queue_->pushMsg(sphere_coord_frame_data_);
		save_times_--;
	}
}

void benewake::MDOPProtocolP4::processFrameBufferTestData(benewake::BwTestDataP4Frame::Ptr& _frame_buf, uint16_t& _frame_id)
{
#ifdef _WIN32
	WaitForSingleObject(occup_, INFINITE);
#else
	sem_wait(&occup_);
#endif // _WIN32
	//test_frame_data_.reset();
	test_frame_data_ = _frame_buf;

	frame_id_ = _frame_id;
	new_frame_ = true;

#ifdef _WIN32
	ReleaseSemaphore(occup_, 1, NULL);
#else
	sem_post(&occup_);
#endif // _WIN32
	if (save_times_ > 0)
	{
		if (!save_data_thread_run_)
		{
			save_data_thread_ = new std::thread(&MDOPProtocolP4::saveDataThread, this);
		}

		if (test_frame_data_queue_ != NULL)
		{
			test_frame_data_queue_->pushMsg(_frame_buf);
			save_times_--;
		}
		else
		{
			test_frame_data_queue_.reset();
			test_frame_data_queue_ = std::make_shared<benewake::MsgQueue<benewake::BwTestDataP4Frame::Ptr>>();
			test_frame_data_queue_->pushMsg(_frame_buf);
		}
	}

	_frame_buf.reset(new BwTestDataP4Frame());
}

void benewake::MDOPProtocolP4::angleCalibration_AD2_B(float _in_rad_h, float _in_rad_v_ch0, int _in_ch, float& _out_rad_h, float& _out_rad_v)
{
	float ang_h = 180.0 * _in_rad_h / kPI;
	float ang_v = 180.0 * _in_rad_v_ch0 / kPI;
	// mechanical angles
	float galva_deg;
	if (work_mode_ == 0)
		galva_deg = (ang_v + 20) * angle_calib_gal_per_v_mode1 - 13.57;
	else if (work_mode_ == 1)
		galva_deg = (ang_v + 4) * angle_calib_gal_per_v_mode2 - 2.46;
	else if (work_mode_ == 2)
		galva_deg = (ang_v + 6.4) * angle_calib_gal_per_v_mode3 - 4.09;
	float prism_deg = ang_h / 2.0;
	prism_deg = prism_deg < 0 ? (prism_deg + 90) : prism_deg;
	// galva channel offset
	galva_deg += angle_calib_galva_ch_offset[_in_ch];
	// get table value
	float prism_col = (prism_deg - angle_calib_prism_range[0]) / angle_calib_prism_step;
	prism_col = prism_col < 0 ? 0 : prism_col;
	prism_col = prism_col > (angle_calib_prism_num - 1) ? (angle_calib_prism_num - 1) : prism_col;
	float galva_row = (galva_deg - angle_calib_galva_range[0]) / angle_calib_galva_step;
	galva_row = galva_row < 0 ? 0 : galva_row;
	galva_row = galva_row > (angle_calib_galva_num - 1) ? (angle_calib_galva_num - 1) : galva_row;
	float p1 = floor(prism_col);
	float p2 = ceil(prism_col);
	float g1 = floor(galva_row);
	float g2 = ceil(galva_row);
	int idx11 = p1 * angle_calib_galva_num + g1;
	int idx12 = p1 * angle_calib_galva_num + g2;
	int idx21 = p2 * angle_calib_galva_num + g1;
	int idx22 = p2 * angle_calib_galva_num + g2;
	float AZ11 = angle_calib_table[idx11][0] * angle_calib_table_unit;
	float AZ12 = angle_calib_table[idx12][0] * angle_calib_table_unit;
	float AZ21 = angle_calib_table[idx21][0] * angle_calib_table_unit;
	float AZ22 = angle_calib_table[idx22][0] * angle_calib_table_unit;
	float EL11 = angle_calib_table[idx11][1] * angle_calib_table_unit;
	float EL12 = angle_calib_table[idx12][1] * angle_calib_table_unit;
	// interpolation
	float s1 = (p2 == p1) ? 0 : (prism_col - p1);
	float AZ1 = s1 * AZ21 + (1 - s1) * AZ11;
	float AZ2 = s1 * AZ22 + (1 - s1) * AZ12;
	float s2 = (g2 == g1) ? 0 : (galva_row - g1);
	float azimuth = s2 * AZ2 + (1 - s2) * AZ1;
	float elevation = s2 * EL12 + (1 - s2) * EL11;

	_out_rad_h = azimuth * kPI / 180.0;
	_out_rad_v = elevation * kPI / 180.0;
}

void benewake::MDOPProtocolP4::angleCalibration_AD2_C(float _in_azimuth, float _in_elevation, int _in_ch, float& _out_azimuth, float& _out_elevation)
{
	_out_azimuth = _in_azimuth + (_in_elevation - 0.01309) * 0.00166491 * _in_ch;
	_out_elevation = _in_elevation - _in_ch * (0.001745 - abs((_in_elevation - 0.01309) * 0.00016));
}

bool benewake::MDOPProtocolP4::ringBufferIsFull(const RingBufferInfo& _ringbuffer)
{
	return ((_ringbuffer.head + 1) % MDOP_DATA_RING_BUFFER_MAX_LENGTH == _ringbuffer.tail);
}

bool benewake::MDOPProtocolP4::ringBufferIsEmpty(const RingBufferInfo& _ringbuffer)
{
	return (_ringbuffer.head == _ringbuffer.tail);
}

benewake::MDOPProtocolG66::MDOPProtocolG66(std::string _local_ip, int _local_port, std::string _remote_ip, int _remote_port, UDPType _udp_type) :
	MDOPProtocolP4(_local_ip, _local_port, _remote_ip, _remote_port, _udp_type)
{
	protocol_type_ = benewake::BwProtocolType::SPHERE;
	enable_angle_calib_ = false;
}

benewake::MDOPProtocolG66::~MDOPProtocolG66()
{
}

int benewake::MDOPProtocolG66::mdop_enable_decode(bool _enable)
{
	if (_enable)
	{
		if (thread_run_)
		{
			return STATUS_OK;
		}
		thread_run_ = true;
		if (protocol_type_ == benewake::BwProtocolType::SPHERE)
		{
			recv_buffer_info_.head = 0;
			recv_buffer_info_.tail = 0;
			recv_thread_ = new std::thread(&MDOPProtocolG66::recvPackageHandler, this);
			decode_thread_ = new std::thread(&MDOPProtocolG66::sphereDataDecode, this);
		}
		else if (protocol_type_ == benewake::BwProtocolType::TEST)
		{
			//recv_buffer_info_.head = 0;
			//recv_buffer_info_.tail = 0;
			//recv_thread_ = new std::thread(&MDOPProtocolG66::recvPackageHandler, this);
			//decode_thread_ = new std::thread(&MDOPProtocolG66::testDataDecode, this);
			decode_thread_ = new std::thread(&MDOPProtocolG66::testDataDecode, this);
		}
		return STATUS_OK;
	}
	else
	{
		if (thread_run_)
		{
			thread_run_ = false;
			new_frame_ = false;
			if (protocol_type_ == benewake::BwProtocolType::SPHERE)
			{
				recv_thread_->join();
				decode_thread_->join();
			}
		}
		return STATUS_OK;
	}
}

benewake::BwTestDataG66* benewake::MDOPProtocolG66::getSrcDataPtr()
{
	return &src_data_;
}

int benewake::MDOPProtocolG66::mdop_save_data_g66(std::string _save_path, std::string _save_file_name, uint32_t _save_times)
{
	if (_save_times > 0)
	{
		save_path_ = _save_path;
		save_file_name_ = _save_file_name;
		save_name_flag_ = 0;
		save_times_ = _save_times;//开始保存
		return STATUS_OK;
	}
	else
	{
		return STATUS_FAIL;
	}
}

void benewake::MDOPProtocolG66::sphereDataDecode()
{

	BwPointCloud::Ptr point_cloud_tmp;
	BwPoint pt;

	BwPointCloud::Ptr point_cloud_backup;
	uint64_t dt_time = 0;

	BwSphereCoordG66 sphere_coord_package_data;
	benewake::BwSphereCoordG66Frame::Ptr sphere_coord_frame_data_buffer = std::make_shared<BwSphereCoordG66Frame>();
	sphere_coord_frame_data_queue_ = std::make_shared<benewake::MsgQueue<benewake::BwSphereCoordG66Frame::Ptr>>();
	int point_bytes = 4;
	point_cloud_buffer_.reset();
	point_cloud_buffer_ = std::make_shared<BwPointCloud>();
	point_cloud_tmp = std::make_shared<BwPointCloud>();

	std::chrono::system_clock::time_point statistics_start, statistics_stop;
	uint32_t lost_pkgs = 0, statistics_start_pkg = 0, lost_frames = 0, statistics_start_frame = 0;

	unsigned char* data;
	uint16_t data_block_size = 72;
	int header_length = 42;
	int recv_size = 0, total_bytes = 0;
	int echo_mode = 1, data_block = 0;
	uint32_t pkg_count = 0, pkg_count_last = 0;
	uint64_t time_s = 0;
	uint32_t time_ns = 0;
	int time_offset = 0;
	bool use_gps_src = false;
	uint32_t diff = 0, total_lost_package = 0, check_sum = 0;
	uint16_t nFrame = 0, nFrame_last = 0, nLine = 0, nLine_last = 0, nPoint = 0, uval16 = 0, protocol_version = 0, last_pkg_mark = 0;
	int16_t roi_center_x_flag = 0, roi_center_y_flag = 0;
	float roi_center_x = 0, roi_center_y = 0;
	float roi_half_x = kPI * 15.0 / 180.0, roi_half_y = kPI * 6.4 / 180.0;
	int16_t val16 = 0, val16_ang_h = 0, val16_ang_v = 0;
	int32_t dist32 = 0;
	uint8_t uval8 = 0, echo_mark_1 = 0, echo_mark_2 = 0;
	int year = 0, month = 0, day = 0, hour = 0, minute = 0;
	bool new_frame_received = false, last_pkg_received = false, first_pkg = true;
	long long frame_win_num = 0;
	long long last_frame_win_num = 0;
	float azimuth_h, azimuth_v, dist_f;
	int current_pkg_num = 0;

	pkg_loss_rate_ = 0;
	frame_loss_rate_ = 0;
	statistics_start = std::chrono::system_clock::now();

	while (thread_run_)
	{
		if (ringBufferIsEmpty(recv_buffer_info_))
		{
			std::this_thread::sleep_for(std::chrono::microseconds(10));
		}
		else
		{
			data = recv_buffer_queue_[recv_buffer_info_.tail].data;
			recv_size = recv_buffer_queue_[recv_buffer_info_.tail].data_length;
			if (data[0] == 0x42 && data[1] == 0x57 && data[2] == PRODUCT_ID_G66 && data[3] == PROTOCOL_ID_MDOP
				&& data[recv_size - 2] == 0x00 && data[recv_size - 1] == 0xff)
			{
				use_gps_src = false;
				protocol_version = *(uint16_t*)&data[PROTOCOL_DATA_PACKAGE_VERSION_OFFSET];
				pkg_count = *(uint32_t*)&data[PROTOCOL_DATA_PACKAGE_COUNT_OFFSET];
				nFrame = *(uint16_t*)&data[PROTOCOL_DATA_FRAME_COUNT_OFFSET];
				uval16 = *(uint16_t*)&data[PROTOCOL_DATA_LINE_NUM_OFFSET];
				nLine = uval16 & 0x7fff;
				last_pkg_mark = uval16 & 0x8000;
				nPoint = *(uint16_t*)&data[PROTOCOL_DATA_POINT_COUNT_OFFSET];

				data_block_size = *(uint16_t*)&data[PROTOCOL_G66_DATA_BLOCK_SIZE];

				time_s = *(uint64_t*)&data[PROTOCOL_G66_DATA_TIME_S_OFFSET];
				time_ns = *(uint32_t*)&data[PROTOCOL_G66_DATA_TIME_NS_OFFSET];
				uval8 = *(uint8_t*)&data[PROTOCOL_G66_DATA_ECHO_MODE];
				roi_center_x_flag = *(int16_t*)&data[PROTOCOL_G66_DATA_ROI_CENTER_X];
				roi_center_y_flag = *(int16_t*)&data[PROTOCOL_G66_DATA_ROI_CENTER_Y];
				roi_center_x = (float)roi_center_x_flag / 10.0;
				roi_center_y = (float)roi_center_y_flag / 10.0;
				uval16 = *(uint16_t*)&data[PROTOCOL_G66_DATA_ROI_WIDTH];
				roi_half_x = (float)uval16 / 20.0;
				uval16 = *(uint16_t*)&data[PROTOCOL_G66_DATA_ROI_HEIGHT];
				roi_half_y = (float)uval16 / 20.0;

				header_length = PROTOCOL_G66_DATA_HEADER_LENGHT;

				if ((time_s & 0x8000000000000000) == 0x8000000000000000)
				{
					use_gps_src = true;
				}

				treatEchoMode(uval8, echo_mode, echo_mark_1, echo_mark_2);
				sys_info_.protocol = protocol_version;
				total_bytes = 30 * data_block_size / echo_mode + header_length + 6;

				check_sum = check_sum_with_protocol_version(protocol_version, data, recv_size - 6);

				if (memcmp(&check_sum, &data[recv_size - 6], 4) != 0)
				{
#ifdef DEBUG_INFO
					uint32_t recv_check_sum = *(uint32_t*)&data[recv_size - 6];
					std::printf("INFO: package:%d point cloud data package check sum 0x%08x<0x%08x>\n", pkg_count, check_sum, recv_check_sum);
#endif // DEBUG_INFO
					continue;
				}

				if (use_gps_src)
				{
					year = (time_s & 0x3f000000) >> 24;
					month = (time_s & 0x00fc0000) >> 18;
					day = (time_s & 0x0003f000) >> 12;
					hour = (time_s & 0x00000fc0) >> 6;
					minute = (time_s & 0x0000003f);

					utc_time_t utcTime;
					utcTime.year = year + 2000;
					utcTime.month = month;
					utcTime.day = day;
					utcTime.hour = hour;
					utcTime.minute = minute;
					utcTime.second = time_ns / 1000000 % 100;
					time_s = covUTC2UnixTimestamp(&utcTime);

					time_ns = time_ns % 1000000 * 1000;
				}

				if (total_bytes != recv_size)
				{
#ifdef DEBUG_INFO
					std::cout << "INFO: package size " << total_bytes << " / " << recv_size << std::endl;
#endif // DEBUG_INFO
					continue;
				}

				if (first_pkg)
				{
					statistics_start_pkg = pkg_count;
					statistics_start_frame = nFrame;
					pkg_count_last = pkg_count;
					nFrame_last = nFrame;
					first_pkg = false;
				}

				diff = pkg_count - pkg_count_last;
				if (diff > 1)
				{
					total_lost_package += (diff - 1);
					lost_pkgs += (diff - 1);
				}
				pkg_count_last = pkg_count;

				new_frame_received = false;
				last_pkg_received = false;
				diff = uint16_t(nFrame - nFrame_last); // uint16_t transform for value flow out
				if (last_pkg_mark == 0x8000)
					last_pkg_received = true;
				if (diff >= 1)
					new_frame_received = true;

				if (new_frame_received)
				{
					processFrameBufferSphereCoord(point_cloud_tmp, sphere_coord_frame_data_buffer, nFrame_last, total_lost_package, diff);
					if (diff > 1)
						lost_frames += (diff - 1);
				}
				nFrame_last = nFrame;

				data_block = nPoint;
				for (int i = 0; i < data_block; i++)
				{
					// angle
					val16_ang_h = *(int16_t*)&data[header_length + i * data_block_size];
					azimuth_h = val16_ang_h / 8192.0; // 2^13
					val16_ang_v = *(int16_t*)&data[header_length + i * data_block_size + 2];
					azimuth_v = val16_ang_v / 8192.0;
					sphere_coord_package_data.h_azimuth = val16_ang_h;
					sphere_coord_package_data.v_azimuth = val16_ang_v;

					// timestamp
					time_offset = *(int*)&data[header_length + i * data_block_size + 4];
					if (use_gps_src)
						time_offset *= 1000;
					pt.timestamp_s = time_s;

					if (time_offset < 0 && abs(time_offset) > time_ns)
					{
						pt.timestamp_ns = 1000000000 + (int)time_ns + time_offset;
						pt.timestamp_s--;
					}
					else
					{
						pt.timestamp_ns = time_ns + time_offset;
						if (pt.timestamp_ns >= 1000000000)
						{
							pt.timestamp_s += pt.timestamp_ns / 1000000000;
							pt.timestamp_ns %= 1000000000;
						}
					}
					if ((new_frame_received || last_pkg_received) && i == 0)
					{
						std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
						std::chrono::nanoseconds ns = tp.time_since_epoch();
						delay_time_ = ns.count() / 1000 - (long long)pt.timestamp_s * 1000000 - pt.timestamp_ns * 0.001;
					}

					//sphere coord data assignment
					float cos_h = std::cos(azimuth_h);
					float sin_h = std::sin(azimuth_h);
					float cos_v, sin_v;
					sphere_coord_package_data.time_offset = time_offset;
					sphere_coord_package_data.timestamp_s = pt.timestamp_s;
					sphere_coord_package_data.timestamp_ns = pt.timestamp_ns;
					memset(sphere_coord_package_data.dist, 0, 16 * sizeof(uint32_t));
					memset(sphere_coord_package_data.intensity, 0, 16 * sizeof(uint8_t));
					memset(sphere_coord_package_data.flag, 0, 16 * sizeof(uint8_t));

					// x y z intensity
					for (int j = 0; j < 8; j++)
					{
						dist32 = 0;
						cos_v = std::cos(azimuth_v);
						sin_v = std::sin(azimuth_v);

						memcpy(&dist32, &data[header_length + i * data_block_size + 8 + j * point_bytes + 0], 2);
						pt.intensity = *(uint8_t*)&data[header_length + i * data_block_size + 8 + j * point_bytes + 2];
						pt.confidence = *(uint8_t*)&data[header_length + i * data_block_size + 8 + j * point_bytes + 3];

						sphere_coord_package_data.dist[2 * j] = dist32;
						sphere_coord_package_data.intensity[2 * j] = pt.intensity;
						sphere_coord_package_data.flag[2 * j] = pt.confidence;

						dist_f = (float)dist32 / 100.0;
						pt.x = dist_f * cos_v * cos_h;
						pt.y = dist_f * cos_v * sin_h;
						pt.z = dist_f * sin_v;
						pt.row = nLine * 8 + (uint16_t)j;
						pt.channel = (uint8_t)j;
						pt.echo = echo_mark_1;

						// roi flag
						if (roi_center_x_flag != 0xFFFF && roi_center_y_flag != 0xFFFF)
						{
							if ((pt.confidence & 0x08) == 0x08)
								pt.roi = 1;
							else
								pt.roi = 0;
						}
						else
							pt.roi = 0;

						point_cloud_tmp->points.push_back(pt);

						if (echo_mode == 2)
						{
							memcpy(&dist32, &data[header_length + i * data_block_size + 8 + 32 + j * point_bytes + 0], 2);
							pt.intensity = *(uint8_t*)&data[header_length + i * data_block_size + 8 + 32 + j * point_bytes + 2];
							pt.confidence = (*(uint8_t*)&data[header_length + i * data_block_size + 8 + 32 + j * point_bytes + 3]);

							sphere_coord_package_data.dist[2 * j + 1] = dist32;
							sphere_coord_package_data.intensity[2 * j + 1] = pt.intensity;
							sphere_coord_package_data.flag[2 * j + 1] = pt.confidence;

							dist_f = (float)dist32 / 100.0;
							pt.x = dist_f * cos_v * cos_h;
							pt.y = dist_f * cos_v * sin_h;
							pt.z = dist_f * sin_v;
							pt.row = nLine * 8 + (uint16_t)j;
							pt.channel = (uint8_t)j;
							pt.echo = echo_mark_2;

							// roi flag
							if (roi_center_x_flag != 0xFFFF && roi_center_y_flag != 0xFFFF)
							{
								if ((pt.confidence & 0x08) == 0x08)
									pt.roi = 1;
								else
									pt.roi = 0;
							}
							else
								pt.roi = 0;

							point_cloud_tmp->points.push_back(pt);
						}
						azimuth_v -= ang2Rad(kAngRes_G66);
					}
					sphere_coord_frame_data_buffer->data.push_back(sphere_coord_package_data);
				}

				if (last_pkg_received)
				{
					processFrameBufferSphereCoord(point_cloud_tmp, sphere_coord_frame_data_buffer, nFrame, total_lost_package, diff);
					if (diff > 1)
						lost_frames += (diff - 1);
					nFrame_last++;
				}
			}
			else
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				std::printf("Point cloud data package header/tail error: 0x%02x 0x%02x 0x%02x 0x%02x ... 0x%02x 0x%02x!\n",
					data[0], data[1], data[2], data[3], data[recv_size - 2], data[recv_size - 1]);
			}

			// update ring buffer tail
			recv_buffer_info_.tail = (recv_buffer_info_.tail + 1) % MDOP_DATA_RING_BUFFER_MAX_LENGTH;
		}

		statistics_stop = std::chrono::system_clock::now();
		auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(statistics_stop - statistics_start);
		if (dur.count() > 10000)
		{
			// update data statics
			if (pkg_count < statistics_start_pkg)
				pkg_loss_rate_ = (float)lost_pkgs / float((UINT32_MAX - statistics_start_pkg) + pkg_count) * 100.0;
			else if (pkg_count == statistics_start_pkg)
				pkg_loss_rate_ = 100.0;
			else
				pkg_loss_rate_ = (float)lost_pkgs / float(pkg_count - statistics_start_pkg) * 100.0;
			lost_pkgs = 0;
			statistics_start_pkg = pkg_count;


			if (nFrame < statistics_start_frame)
				frame_loss_rate_ = (float)lost_frames / float((UINT16_MAX - statistics_start_frame) + nFrame) * 100.0;
			else if (nFrame == statistics_start_frame)
				frame_loss_rate_ = 100.0;
			else
				frame_loss_rate_ = (float)lost_frames / float(nFrame - statistics_start_frame) * 100.0;
			lost_frames = 0;
			statistics_start_frame = nFrame;

			statistics_start = statistics_stop;
		}
	}
}

//void benewake::MDOPProtocolG66::testDataDecode()
//{
//	test_frame_data_queue_ = std::make_shared<benewake::MsgQueue<benewake::BwTestDataG66Frame::Ptr>>("msg", 5);
//	benewake::BwTestDataG66Frame::Ptr test_frame_data_buffer = std::make_shared<benewake::BwTestDataG66Frame>();
//	unsigned char* data;
//	BwTestDataG66 g66_src_data;
//	int recv_size = 0, total_bytes = 02;
//	uint32_t pkg_count = 0, last_pkg_count = 0;
//	uint64_t time_s = 0;
//	uint32_t time_ns = 0;
//	uint32_t check_sum = 0, valu32 = 0;
//	uint16_t nFrame = 0, nLine = 0, nPoint = 0, uval16 = 0, protocol_version = 0, last_pkg_mark = 0;
//	int header_length = 42;
//	int total_num = 0, eff_num = 0;
//	uint32_t package_diff = 0;
//	bool new_frame_received = false;
//	bool last_pkg_received = false;
//	uint16_t nFrame_last = 0;
//
//	g66_src_data.st_ch_r[96] = { 0 };
//	g66_src_data.st_ch_f[96] = { 0 };
//	g66_src_data.st_ch_r_cnt[24] = { 0 };
//	g66_src_data.st_ch_f_cnt[24] = { 0 };
//	g66_src_data.main_ch_r = 0;
//	g66_src_data.main_ch_f = 0;
//	g66_src_data.main_ch_r_cnt = 0;
//	g66_src_data.main_ch_f_cnt = 0;
//	g66_src_data.luminous_period = 0;
//	g66_src_data.prism_code = 0;
//	g66_src_data.galvo_code = 0;
//	g66_src_data.dac_out[8] = { 0 };
//	g66_src_data.dac_stage_2 = 0;
//	g66_src_data.dac_stage_3 = 0;
//
//	while (thread_run_)
//	{
//		if (ringBufferIsEmpty(recv_buffer_info_))
//		{
//			std::this_thread::sleep_for(std::chrono::microseconds(10));
//		}
//		else
//		{
//			data = recv_buffer_queue_[recv_buffer_info_.tail].data;
//			recv_size = recv_buffer_queue_[recv_buffer_info_.tail].data_length;
//			if (data[0] == 0x42 && data[1] == 0x57 && data[2] == PRODUCT_ID_G66 && data[3] == PROTOCOL_ID_MDOP
//				&& data[recv_size - 2] == 0x00 && data[recv_size - 1] == 0xff)
//			{
//				protocol_version = *(uint16_t*)&data[PROTOCOL_DATA_PACKAGE_VERSION_OFFSET];
//				pkg_count = *(uint32_t*)&data[PROTOCOL_DATA_PACKAGE_COUNT_OFFSET];
//				nFrame = *(uint16_t*)&data[PROTOCOL_DATA_FRAME_COUNT_OFFSET];
//				uval16 = *(uint16_t*)&data[PROTOCOL_DATA_LINE_NUM_OFFSET];
//				nLine = uval16 & 0x7fff;
//				last_pkg_mark = uval16 & 0x8000;
//				nPoint = *(uint16_t*)&data[PROTOCOL_DATA_POINT_COUNT_OFFSET];
//				time_s = *(uint64_t*)&data[PROTOCOL_G66_DATA_TIME_S_OFFSET];
//				time_ns = *(uint32_t*)&data[PROTOCOL_G66_DATA_TIME_NS_OFFSET];
//
//				header_length = PROTOCOL_G66_DATA_HEADER_LENGHT;
//
//				sys_info_.protocol = protocol_version;
//				if (recv_size != 1464) //B样
//				{
//					std::cout << "Packet length error. " << std::endl;
//					continue;
//				}
//
//				last_pkg_received = false;
//				new_frame_received = false;
//				if (last_pkg_mark == 0x8000)
//					last_pkg_received = true;
//				if (uint16_t(nFrame - nFrame_last) >= 1)
//					new_frame_received = true;
//
//				if (new_frame_received && test_frame_data_buffer->data.size() != 0)
//				{
//					processFrameBufferTestData(test_frame_data_buffer);
//				}
//				nFrame_last = nFrame;
//				decodeTestDataFromBuffer(&g66_src_data, data, header_length);
//				src_data_ = g66_src_data;
//
//				package_diff = pkg_count - last_pkg_count;
//				if (package_diff > 1)
//				{
//					std::cout << "Packet loss: " << package_diff << std::endl;
//				}
//				last_pkg_count = pkg_count;
//
//				test_frame_data_buffer->data.push_back(g66_src_data);
//
//				if (last_pkg_received && test_frame_data_buffer->data.size() != 0)
//				{
//					processFrameBufferTestData(test_frame_data_buffer);
//					nFrame_last++;
//				}
//			}
//			else
//			{
//				std::cout << "recv data size err! size:" << recv_size << std::endl;
//			}
//
//			// update ring buffer tail
//			recv_buffer_info_.tail = (recv_buffer_info_.tail + 1) % MDOP_DATA_RING_BUFFER_MAX_LENGTH;
//		}
//	}
//	std::cout << "Test data thread close! " << thread_run_ << std::endl;
//}

void benewake::MDOPProtocolG66::processFrameBufferSphereCoord(benewake::BwPointCloud::Ptr& _frame_buf, benewake::BwSphereCoordG66Frame::Ptr& _backup_sphere_buf, uint16_t& _frame_id, uint32_t& _lost_pkg_count, uint32_t& _diff_frame)
{
	float pkg_loss_rate = 240.0 * (float)_lost_pkg_count / (float)(_frame_buf->points.size() + 240.0 * _lost_pkg_count);
#ifdef _WIN32
	WaitForSingleObject(occup_, INFINITE);
#else
	sem_wait(&occup_);
#endif // _WIN32
	point_cloud_buffer_.reset();
	point_cloud_buffer_ = _frame_buf;
	point_cloud_buffer_->pkg_loss = pkg_loss_rate;
	frame_id_ = _frame_id;
	new_frame_ = true;
#ifdef _WIN32
	ReleaseSemaphore(occup_, 1, NULL);
#else
	sem_post(&occup_);
#endif // _WIN32

	_frame_buf.reset(new BwPointCloud());
	_frame_buf->points.reserve(307200);

	if (callback_enable_)
	{
		callback_func_(point_cloud_buffer_, frame_id_, callback_pointer_);
	}

	if (_lost_pkg_count > 0)
	{
		std::cout << "INFO: Package lost in last frame " << _lost_pkg_count << std::endl;
		_lost_pkg_count = 0;
	}
	if (_diff_frame > 1)
	{
		std::cout << "INFO: Frame Lost " << (_diff_frame - 1) << std::endl;
	}

	//if (save_times_ > 0)
	//{
	//	if (!save_data_thread_run_)
	//	{
	//		save_data_thread_ = new std::thread(&MDOPProtocolG66::saveDataThread, this);
	//	}
	//	sphere_coord_frame_data_queue_->pushMsg(_backup_sphere_buf);
	//	save_times_--;
	//}
	_backup_sphere_buf.reset(new BwSphereCoordG66Frame());
}
//
//void benewake::MDOPProtocolG66::processFrameBufferTestData(benewake::BwTestDataG66Frame::Ptr& _frame_buf)
//{
//	if (save_times_ > 0)
//	{
//		if (!save_data_thread_run_)
//		{
//			save_data_thread_ = new std::thread(&MDOPProtocolG66::saveDataThread, this);
//		}
//
//		if (test_frame_data_queue_ != NULL)
//		{
//			test_frame_data_queue_->pushMsg(_frame_buf);
//			save_times_--;
//		}
//		else
//		{
//			test_frame_data_queue_.reset();
//			test_frame_data_queue_ = std::make_shared<benewake::MsgQueue<benewake::BwTestDataG66Frame::Ptr>>();
//			test_frame_data_queue_->pushMsg(_frame_buf);
//		}
//	}
//
//	_frame_buf.reset(new BwTestDataG66Frame());
//}
//
//void benewake::MDOPProtocolG66::decodeTestDataFromBuffer(benewake::BwTestDataG66* _test_data, unsigned char* _buffer, int _data_offset)
//{
//	//三档阈值通道1~通道16的4回波信息
//	for (int i = 0; i < 4; ++i)  //回波
//	{
//		for (int j = 0; j < 3; ++j)  //档位
//		{
//			for (int k = 0; k < 8; ++k)  //通道
//			{
//				_test_data->st_ch_r[i * 3 * 8 + j * 8 + k] = *(uint16_t*)&_buffer[_data_offset];
//				_test_data->st_ch_f[i * 3 * 8 + j * 8 + k] = *(uint16_t*)&_buffer[_data_offset + 2];
//				_data_offset += 4;
//			}
//		}
//	}
//	//四档阈值通道的回波个数
//	for (int i = 0; i < 3; ++i)  //档位
//	{
//		for (int j = 0; j < 8; ++j)
//		{
//			//获取通道上升沿下降沿计数
//			_test_data->st_ch_r_cnt[i * 8 + j] = *(uint8_t*)&_buffer[_data_offset];
//			_test_data->st_ch_f_cnt[i * 8 + j] = *(uint8_t*)&_buffer[_data_offset + 1];
//			_data_offset += 2;
//		}
//	}
//	//主波通道信息
//	_test_data->main_ch_r = *(uint16_t*)&_buffer[_data_offset];
//	_test_data->main_ch_f = *(uint16_t*)&_buffer[_data_offset + 2];
//	_data_offset += 4;
//
//	_test_data->main_ch_r_cnt = *(uint8_t*)&_buffer[_data_offset];
//	_test_data->main_ch_f_cnt = *(uint8_t*)&_buffer[_data_offset + 1];
//	_data_offset += 2;
//	_data_offset += 2; // reserved
//
//	_test_data->luminous_period = *(uint16_t*)&_buffer[_data_offset];
//	_test_data->prism_code = *(uint32_t*)&_buffer[_data_offset + 2];
//	_test_data->galvo_code = *(uint32_t*)&_buffer[_data_offset + 6];
//	_data_offset += 10;
//
//	for (int i = 0; i < 8; i++)
//	{
//		_test_data->dac_out[i] = *(uint8_t*)&_buffer[_data_offset];
//	}
//	_data_offset += 8;
//	_test_data->dac_stage_2 = *(uint8_t*)&_buffer[_data_offset];
//	_test_data->dac_stage_3 = *(uint8_t*)&_buffer[_data_offset + 1];
//	_data_offset += 2;
//}
//
//void benewake::MDOPProtocolG66::saveDataThread()
//{
//	save_data_thread_run_ = true;
//	uint32_t end_times = 100;
//	while (save_data_thread_run_)
//	{
//		if (sphere_coord_frame_data_queue_ != NULL && sphere_coord_frame_data_queue_->size() > 0)
//		{
//			BwSphereCoordG66Frame::Ptr cur_data = sphere_coord_frame_data_queue_->popMsg();
//			openDataOutputStream();
//			saveSphereCoordData(cur_data);
//			closeDataOutputStream();
//		}
//		else if (test_frame_data_queue_ != NULL && test_frame_data_queue_->size() > 0)
//		{
//			BwTestDataG66Frame::Ptr cur_data = test_frame_data_queue_->popMsg();
//			openDataOutputStream();
//			saveTestDataOnceFrame(cur_data);
//			closeDataOutputStream();
//		}
//		else
//		{
//#ifdef _WIN32
//			Sleep(10);
//#else
//			usleep(10000);
//#endif // _WIN32
//			end_times--;
//			if (end_times == 0)
//			{
//				save_data_thread_run_ = false;
//				break;
//			}
//		}
//	}
//	std::cout << "Save data thread terminated." << std::endl;
//}
//
//void benewake::MDOPProtocolG66::openDataOutputStream()
//{
//	std::string file_name;
//	mkDir(save_path_);
//	if (save_file_name_ == "")
//	{
//		time_t timep(time(NULL));
//		char time_char[256];
//		struct tm* nowTime = localtime(&timep);
//		strftime(time_char, sizeof(time_char), "/%Y-%m-%d_%H_%M_%S_", nowTime);
//		std::string time_str(time_char);
//		auto now_epoch = std::chrono::system_clock::now();
//		int ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_epoch.time_since_epoch()).count() % 1000;
//		file_name = save_path_ + time_str + std::to_string(ms) + ".txt";
//	}
//	else
//	{
//		file_name = save_path_ + "/" + save_file_name_ + std::to_string(save_name_flag_) + ".txt";
//		save_name_flag_++;
//	}
//	data_out_2_file_.open(file_name);
//	if (protocol_type_ == benewake::BwProtocolType::TEST)
//	{
//		data_out_2_file_ << "luminous_period" << " ";
//		data_out_2_file_ << "prism_code" << " ";
//		data_out_2_file_ << "galvo_code" << " ";
//		for (int i = 0; i < 8; i++)
//		{
//			data_out_2_file_ << "dac_out" << i << " ";
//		}
//		data_out_2_file_ << "dac_stage_2" << " ";
//		data_out_2_file_ << "dac_stage_3" << " ";
//		data_out_2_file_ << "main_ch_r_0" << " ";
//		data_out_2_file_ << "main_ch_f_0" << " ";
//		data_out_2_file_ << "main_ch_r_cnt" << " ";
//		data_out_2_file_ << "main_ch_f_cnt" << " ";
//		for (int i = 0; i < 4; i++)
//		{
//			for (int j = 0; j < 3; j++)
//			{
//				for (int k = 0; k < 8; k++)
//				{
//					data_out_2_file_ << "st" << j << "_ch" << k << "_r_" << i << " ";
//					data_out_2_file_ << "st" << j << "_ch" << k << "_f_" << i << " ";
//				}
//			}
//		}
//		for (int i = 0; i < 3; i++)
//		{
//			for (int j = 0; j < 8; j++)
//			{
//				data_out_2_file_ << "st" << i << "_ch" << j << "_r_cnt" << " ";
//				data_out_2_file_ << "st" << i << "_ch" << j << "_f_cnt" << " ";
//			}
//		}
//
//		data_out_2_file_ << std::endl;
//	}
//	else if (protocol_type_ == benewake::BwProtocolType::SPHERE)
//	{
//		data_out_2_file_ << "h_azimuth" << " ";
//		data_out_2_file_ << "v_azimuth" << " ";
//		data_out_2_file_ << "time_offset" << " ";
//		data_out_2_file_ << "timestamp_s" << " ";
//		data_out_2_file_ << "timestamp_ns" << " ";
//		for (int ch = 0; ch < 8; ch++)
//		{
//			data_out_2_file_ << "dist_" << ch << "_echo_0 ";
//			data_out_2_file_ << "intensity_" << ch << "_echo_0 ";
//			data_out_2_file_ << "reserved_" << ch << "_echo_0 ";
//			data_out_2_file_ << "dist_" << ch << "_echo_1 ";
//			data_out_2_file_ << "intensity_" << ch << "_echo_1 ";
//			data_out_2_file_ << "reserved_" << ch << "_echo_1 ";
//		}
//
//		data_out_2_file_ << std::endl;
//	}
//
//	std::cout << "save path: " << file_name << std::endl;
//}
//
//void benewake::MDOPProtocolG66::saveSphereCoordData(BwSphereCoordG66Frame::Ptr _data)
//{
//	if (_data == NULL)
//	{
//		std::cout << "_data == NULL" << std::endl;
//		return;
//	}
//
//	uint32_t data_len = _data->data.size();
//	for (int i = 0; i < data_len; i++)
//	{
//		data_out_2_file_ << _data->data[i].h_azimuth << " ";
//		data_out_2_file_ << _data->data[i].v_azimuth << " ";
//		data_out_2_file_ << _data->data[i].time_offset << " ";
//		data_out_2_file_ << _data->data[i].timestamp_s << " ";
//		data_out_2_file_ << _data->data[i].timestamp_ns << " ";
//		for (int ch = 0; ch < 16; ch++)
//		{
//			data_out_2_file_ << _data->data[i].dist[ch] << " ";
//			uint16_t intensity = (uint16_t)_data->data[i].intensity[ch];
//			uint16_t confidence = (uint16_t)_data->data[i].flag[ch];
//			data_out_2_file_ << intensity << " ";
//			data_out_2_file_ << confidence << " ";
//		}
//
//		data_out_2_file_ << std::endl;
//	}
//}
//
//void benewake::MDOPProtocolG66::saveTestDataOnceFrame(BwTestDataG66Frame::Ptr _data)
//{
//	if (_data == NULL)
//	{
//		std::cout << "_data == NULL" << std::endl;
//		return;
//	}
//
//	uint32_t data_len = _data->data.size();
//	for (int i = 0; i < data_len; i++)
//	{
//		data_out_2_file_ << _data->data[i].luminous_period << " ";
//		data_out_2_file_ << _data->data[i].prism_code << " ";
//		data_out_2_file_ << _data->data[i].galvo_code << " ";
//		for (int i = 0; i < 8; i++)
//		{
//			data_out_2_file_ << _data->data[i].dac_out[i] << " ";
//		}
//		data_out_2_file_ << _data->data[i].dac_stage_2 << " ";
//		data_out_2_file_ << _data->data[i].dac_stage_3 << " ";
//		data_out_2_file_ << _data->data[i].main_ch_r << " ";
//		data_out_2_file_ << _data->data[i].main_ch_f << " ";
//		data_out_2_file_ << _data->data[i].main_ch_r_cnt << " ";
//		data_out_2_file_ << _data->data[i].main_ch_f_cnt << " ";
//		for (int i = 0; i < 4; i++)
//		{
//			for (int j = 0; j < 3; j++)
//			{
//				for (int k = 0; k < 8; k++)
//				{
//					data_out_2_file_ << _data->data[i].st_ch_r[i * 3 * 8 + j * 8 + k] << " ";
//					data_out_2_file_ << _data->data[i].st_ch_f[i * 3 * 8 + j * 8 + k] << " ";
//				}
//			}
//		}
//		for (int i = 0; i < 3; i++)
//		{
//			for (int j = 0; j < 8; j++)
//			{
//				data_out_2_file_ << _data->data[i].st_ch_r_cnt[i * 8 + j] << " ";
//				data_out_2_file_ << _data->data[i].st_ch_f_cnt[i * 8 + j] << " ";
//			}
//		}
//		data_out_2_file_ << std::endl;
//	}
//}
//
