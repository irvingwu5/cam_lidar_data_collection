#include "benewake_dcsp.h"
#include <cstring>
#include <regex>  
#include <vector>

using namespace std;
using namespace benewake;

/** \brief This function check ip format
* \param[in] ip:
* \return true-success
*         false-fail
*/
bool checkIP(string ip)
{
	regex pat{ R"(^(25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[0-9]{1,2})(\.(25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[0-9]{1,2})){3}$)" };
	if (!regex_match(ip, pat))
	{
		return false;
	}
	return true;
}

void splitString(const string& s, vector<string>& v, const string& c)
{
	v.clear();
    string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while (string::npos != pos2)
	{
		if (pos2 != pos1)
			v.push_back(s.substr(pos1, pos2 - pos1));
		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if (pos1 != s.length())
		v.push_back(s.substr(pos1));
}

benewake::DCSPProtocol::DCSPProtocol(std::string _local_ip, int _local_port, std::string _remote_ip, int _remote_port, UDPType _udp_type) :
	Protocol(_local_ip, _local_port, _remote_ip, _remote_port, _udp_type)
{
	request_header_[0] = 'B';
	request_header_[1] = 'W';
	request_header_[2] = PRODUCT_ID_X2;
	request_header_[3] = PROTOCOL_ID_DCSP_REQUEST;

	response_header_[0] = 'B';
	response_header_[1] = 'W';
	response_header_[2] = PRODUCT_ID_X2;
	response_header_[3] = PROTOCOL_ID_DCSP_RESPONSE;

	header_count_len_ = 0;
	response_status_len_ = 0;
}

benewake::DCSPProtocol::~DCSPProtocol()
{
}

int benewake::DCSPProtocol::dcsp_clear_work_time()
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_detect_mode_switch(int32_t _mode)
{
	uint32_t addr = 0x00001000;
	int status = dcsp_write_register(&_mode, addr, 1);

	return status;
}

int benewake::DCSPProtocol::dcsp_download_firmware(std::string _firmware_file)
{
	int status;
	int time_limit = 2;
	struct stat file_stat;
	int result = stat(_firmware_file.data(), &file_stat);
#ifdef _WIN32
	if (result == 0 && (_S_IFREG & file_stat.st_mode))
#else
	if (result == 0 && S_ISREG(file_stat.st_mode))
#endif
	{
		unsigned int filesize = file_stat.st_size;
		ifstream fin(_firmware_file, ios::binary);
		if (!fin)
		{
			cout << endl << "err: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << endl;
			cout << "Open firmware file failed!" << endl;
			return STATUS_FAIL;
		}
		unsigned char *bin_data = new unsigned char[filesize];
		fin.read((char *)bin_data, filesize);

		cout << endl << "Start download firmware, total packages: " << ((int)(filesize / DCSP_DATA_FIRMWARE_DEFAULT_LENGTH) + 1) << endl;
		if (filesize <= DCSP_DATA_FIRMWARE_DEFAULT_LENGTH)
		{
			cout << "    processing package 1 ..." << endl;
			int offset = 0;
			// header
			setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
			offset += (6 + header_count_len_);
			// payload
			request_[offset + 0] = DCSP_CMD_DOWNLOAD_FIRMWARE;
			request_[offset + 1] = (1 + 4 + 4 + filesize) & 0xff;
			request_[offset + 2] = ((1 + 4 + 4 + filesize) >> 8) & 0xff;
			request_[offset + 3] = 0x03;
			memcpy(&request_[offset + 4], &filesize, sizeof(filesize));
			memset(&request_[offset + 8], 0x00, 4 * sizeof(unsigned char));
			offset += 12;
			std::memcpy(&request_[offset], bin_data, filesize);
			offset += filesize;
			// tail
			uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
			padding_tail(offset, request_sum);
			offset += 6;

			time_limit = 60;
			status = tx_rx_check_status(offset, DCSP_CMD_DOWNLOAD_FIRMWARE, time_limit);
		}
		else
		{
			unsigned int cnt = 0;
			for (auto i = 0; i < filesize / DCSP_DATA_FIRMWARE_DEFAULT_LENGTH; ++i, ++cnt)
			{
				cout << "    processing package " << (cnt + 1) << " ..." << endl;
				int offset = 0;
				// header
				setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
				offset += (6 + header_count_len_);
				// payload
				request_[offset + 0] = DCSP_CMD_DOWNLOAD_FIRMWARE;
				request_[offset + 1] = (1 + 4 + 4 + DCSP_DATA_FIRMWARE_DEFAULT_LENGTH) & 0xff;
				request_[offset + 2] = ((1 + 4 + 4 + DCSP_DATA_FIRMWARE_DEFAULT_LENGTH) >> 8) & 0xff;
				request_[offset + 3] = 0x00;
				if (i == 0)
				{
					request_[offset + 3] = 0x01;
					time_limit = 2;
				}
				if ((filesize % DCSP_DATA_FIRMWARE_DEFAULT_LENGTH) == 0 && (i == filesize / DCSP_DATA_FIRMWARE_DEFAULT_LENGTH - 1))
				{
					request_[offset + 3] |= 0x02;
					time_limit = 60;
				}
				memcpy(&request_[offset + 4], &filesize, sizeof(filesize));
				memcpy(&request_[offset + 8], &cnt, sizeof(cnt));
				offset += 12;
				std::memcpy(&request_[offset], &bin_data[cnt * DCSP_DATA_FIRMWARE_DEFAULT_LENGTH], DCSP_DATA_FIRMWARE_DEFAULT_LENGTH);
				offset += DCSP_DATA_FIRMWARE_DEFAULT_LENGTH;
				// tail
				uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
				padding_tail(offset, request_sum);
				offset += 6;

				status = tx_rx_check_status(offset, DCSP_CMD_DOWNLOAD_FIRMWARE, time_limit);
				if (status != STATUS_OK)
				{
					delete[] bin_data;
					return status;
				}
			}

			if ((filesize % DCSP_DATA_FIRMWARE_DEFAULT_LENGTH) != 0)
			{
				cout << "    processing package " << (cnt + 1) << " ..." << endl;
				int residue = filesize % DCSP_DATA_FIRMWARE_DEFAULT_LENGTH;
				int offset = 0;
				// header
				setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
				offset += (6 + header_count_len_);
				// payload
				request_[offset + 0] = DCSP_CMD_DOWNLOAD_FIRMWARE;
				request_[offset + 1] = (1 + 4 + 4 + residue) & 0xff;
				request_[offset + 2] = ((1 + 4 + 4 + residue) >> 8) & 0xff;
				request_[offset + 3] = 0x02;
				memcpy(&request_[offset + 4], &filesize, sizeof(filesize));
				memcpy(&request_[offset + 8], &cnt, sizeof(cnt));
				offset += 12;
				std::memcpy(&request_[offset], &bin_data[cnt * DCSP_DATA_FIRMWARE_DEFAULT_LENGTH], residue);
				offset += residue;
				// tail
				uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
				padding_tail(offset, request_sum);
				offset += 6;

				time_limit = 60;
				status = tx_rx_check_status(offset, DCSP_CMD_DOWNLOAD_FIRMWARE, time_limit);
			}

		}

		delete[] bin_data;
		if (status != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << "Download firmware fail!" << endl;
		}
		return status;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Firmware file doesn't exist!" << endl;
		return STATUS_FAIL;
	}
}

int benewake::DCSPProtocol::dcsp_download_lut(std::string _table_file, bool _check_sn)
{
	if (!loadTable(_table_file)) {
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Read table files failed!" << endl;
		return STATUS_FAIL;
	}
	if (_check_sn)
	{
		std::string ps_ver, pl_ver, sn;
		uint32_t total_num;
		uint16_t line_num, channel_num;
		int stat = dcsp_get_device_information(ps_ver, pl_ver, total_num, line_num, channel_num, sn);
		if (stat != 0)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << "Get device SN failed!" << endl;
			return STATUS_FAIL;
		}
		if (sn != table_sn_)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << "The device's SN is not same with SN within the table!" << endl;
			return STATUS_FAIL;
		}
	}
	int status;
	int offset = 0;
	uint16_t pkg_length = 0;
	int total_send_time = 0;
	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);
	int table_len = table_row_ * table_col_;
	total_send_time = ceil((float)table_.size() / 300);
	// payload
	request_[offset + 0] = DCSP_CMD_DOWNLOAD_LUT;
	request_[offset + 8] = (uint8_t)table_id_;
	memcpy(&request_[offset + 9], &table_len, sizeof(int));
	for (int pkg_idx = 0; pkg_idx < total_send_time; pkg_idx++)
	{
		offset = (6 + header_count_len_);
		int element_num = (table_.size() - pkg_idx * 300) < 300 ? (table_.size() % 300) : 300;
		pkg_length = 10 + 4 * element_num;
		memcpy(&request_[offset + 1], &pkg_length, sizeof(uint16_t));
		request_[offset + 3] = 0x00;
		if (pkg_idx == 0)
			request_[offset + 3] += 0x01;
		if (pkg_idx == (total_send_time - 1))
			request_[offset + 3] += 0x02;
		memcpy(&request_[offset + 4], &pkg_idx, sizeof(int));
		for (int i = 0; i < element_num; i++)
		{
			memcpy(&request_[offset + 13 + 4 * i], &table_[300 * pkg_idx + i], sizeof(int));
		}
		offset += 13 + 4 * element_num;
		// tail
		uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
		padding_tail(offset, request_sum);
		offset += 6;
		status = tx_rx_check_status(offset, DCSP_CMD_DOWNLOAD_LUT, 60);
		if (status != STATUS_OK)
		{
			return status;
		}
	}
	return STATUS_OK;
}

int benewake::DCSPProtocol::dcsp_download_temp_comp_table(const uint8_t* _data, const int _data_len)
{
	int status;
	int offset = 0;
	if (_data_len < 13)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "data error!" << endl;
		return STATUS_FAIL;
	}

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_DOWNLOAD_TEMP_COMP_TABLE;
	request_[offset + 1] = (uint16_t)_data_len;

	std::memcpy(&request_[offset + 3], _data, _data_len);
	offset += 3 + _data_len;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_DOWNLOAD_TEMP_COMP_TABLE);
	if (status == STATUS_OK)
	{

	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Download temp table fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::dcsp_enable(bool _flag)
{
	int status;
	int offset = 0;
	int wait_time = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_START_STOP;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	if (_flag)
	{
		request_[offset + 3] = 0x01;
		wait_time = 30;
	}
	else
	{
		request_[offset + 3] = 0x00;
		wait_time = 2;
	}
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_START_STOP, wait_time);
	if (status != STATUS_OK)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Enable/disable lidar fail!" << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_format_file_system()
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_get_algorithm_version(std::vector<std::string>& _versions)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_get_app_partition(uint8_t& _partition)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_get_autorun_status(bool& _enable)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_AUTORUN_STATUS;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 4 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_AUTORUN_STATUS);
	if (status == STATUS_OK)
	{
		if (response_[6 + header_count_len_ + 3 + response_status_len_] == 0x00)
			_enable = false;
		else
			_enable = true;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get autorun status fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_get_clock_config(bool& _1588_bmc, uint8_t& _1588_domain_nu, bool& _gptp_bmc, uint8_t& _gptp_domain_nu)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_get_coordinate_system(uint8_t & _coordinate)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_COORDINATE_SYSTEM;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 4 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_COORDINATE_SYSTEM);
	if (status == STATUS_OK)
	{
		_coordinate = response_[6 + header_count_len_ + 3 + response_status_len_];
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get coordinate system fail! Error code: " << status << endl;
	}

	return status;
}

uint16_t benewake::DCSPProtocol::dcsp_get_current_sdk_protocol_version()
{
	return dcsp_info_->protocol;
}

int benewake::DCSPProtocol::dcsp_get_DDR_info(uint32_t& _ddr_capacity)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_get_device_information(std::string &_version, std::string &_fpga_version,
	uint32_t &_total_num, uint16_t &_line_num, uint16_t &_channel_num, std::string &_sn, int _timeout_s, int _timeout_us)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_DEVICE_INFORMATION;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 103 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_DEVICE_INFORMATION, _timeout_s, _timeout_us);
	if (status == STATUS_OK)
	{
		// ps version
		std::string project_name, project_branch, project_version_major, project_version_minor, project_version_buid;
		unsigned char buf_8[9] = { 0 }, buf_2[3] = { 0 }, buf_16[17] = { 0 };

		std::memcpy(buf_8, &response_[6 + 3 + header_count_len_ + response_status_len_], 8 * sizeof(char));
		project_name = (char *)buf_8;
		int s = project_name.find_first_not_of(" ");
		s = s >= 0 ? s : 0;
		project_name = project_name.substr(s, 8 - s);

		std::memcpy(buf_8, &response_[6 + 11 + header_count_len_ + response_status_len_], 8 * sizeof(char));
		project_branch = (char *)buf_8;
		s = project_branch.find_first_not_of(" ");
		s = s >= 0 ? s : 0;
		project_branch = project_branch.substr(s, 8 - s);

		std::memcpy(buf_2, &response_[6 + 19 + header_count_len_ + response_status_len_], 2 * sizeof(char));
		project_version_major = (char *)buf_2;

		std::memcpy(buf_2, &response_[6 + 21 + header_count_len_ + response_status_len_], 2 * sizeof(char));
		project_version_minor = (char *)buf_2;

		std::memcpy(buf_2, &response_[6 + 23 + header_count_len_ + response_status_len_], 2 * sizeof(char));
		project_version_buid = (char *)buf_2;

		_version = project_name + "." + project_branch + "." + project_version_major + "." + project_version_minor + "." + project_version_buid;
		// pl version
		std::memcpy(buf_8, &response_[6 + 25 + header_count_len_ + response_status_len_], 8 * sizeof(char));
		project_name = (char *)buf_8;
		s = project_name.find_first_not_of(" ");
		s = s >= 0 ? s : 0;
		project_name = project_name.substr(s, 8 - s);

		std::memcpy(buf_8, &response_[6 + 33 + header_count_len_ + response_status_len_], 8 * sizeof(char));
		project_branch = (char *)buf_8;
		s = project_branch.find_first_not_of(" ");
		s = s >= 0 ? s : 0;
		project_branch = project_branch.substr(s, 8 - s);

		std::memcpy(buf_2, &response_[6 + 41 + header_count_len_ + response_status_len_], 2 * sizeof(char));
		project_version_major = (char *)buf_2;

		std::memcpy(buf_2, &response_[6 + 43 + header_count_len_ + response_status_len_], 2 * sizeof(char));
		project_version_minor = (char *)buf_2;

		std::memcpy(buf_2, &response_[6 + 45 + header_count_len_ + response_status_len_], 2 * sizeof(char));
		project_version_buid = (char *)buf_2;

		_fpga_version = project_name + "." + project_branch + "." + project_version_major + "." + project_version_minor + "." + project_version_buid;
		// total points num
		std::memcpy(&_total_num, &response_[6 + 47 + header_count_len_ + response_status_len_], sizeof(uint32_t));
		// points num of line
		std::memcpy(&_line_num, &response_[6 + 51 + header_count_len_ + response_status_len_], sizeof(uint16_t));
		// channel num
		std::memcpy(&_channel_num, &response_[6 + 53 + header_count_len_ + response_status_len_], sizeof(uint16_t));
		// sn
		std::memcpy(buf_16, &response_[6 + 55 + header_count_len_ + response_status_len_], 16 * sizeof(char));
		_sn = (char *)buf_16;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get device info fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::dcsp_get_DHCP(bool & _enable)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_DHCP;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 4 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_DHCP);
	if (status == STATUS_OK)
	{
		if (response_[6 + 3 + header_count_len_ + response_status_len_] == 0x00)
			_enable = false;
		else
			_enable = true;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get DHCP fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_get_dsop_dest_ip_and_port(std::string& _ip, uint32_t& _port)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_get_dsp_version(std::string& _app_version, std::string& _boot_loader_version)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_DSP_VERSION;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, (6 + header_count_len_) + 3 + response_status_len_ + 60 + 6, DCSP_CMD_GET_DSP_VERSION);
	if (status == STATUS_OK)
	{
		std::string project_name, project_branch, project_version_major, project_version_minor, project_version_buid;
		unsigned char buf_8[9] = { 0 }, buf_2[3] = { 0 };

		std::memcpy(buf_8, &response_[6 + 3 + header_count_len_ + response_status_len_], 8 * sizeof(char));
		project_name = (char*)buf_8;
		int s = project_name.find_first_not_of(" ");
		s = s >= 0 ? s : 0;
		project_name = project_name.substr(s, 8 - s);

		std::memcpy(buf_8, &response_[6 + 11 + header_count_len_ + response_status_len_], 8 * sizeof(char));
		project_branch = (char*)buf_8;
		s = project_branch.find_first_not_of(" ");
		s = s >= 0 ? s : 0;
		project_branch = project_branch.substr(s, 8 - s);

		std::memcpy(buf_2, &response_[6 + 19 + header_count_len_ + response_status_len_], 2 * sizeof(char));
		project_version_major = (char*)buf_2;

		std::memcpy(buf_2, &response_[6 + 21 + header_count_len_ + response_status_len_], 2 * sizeof(char));
		project_version_minor = (char*)buf_2;

		std::memcpy(buf_2, &response_[6 + 23 + header_count_len_ + response_status_len_], 2 * sizeof(char));
		project_version_buid = (char*)buf_2;

		_app_version = project_name + "." + project_branch + "." + project_version_major + "." + project_version_minor + "." + project_version_buid;
		// pl version
		std::memcpy(buf_8, &response_[6 + 25 + header_count_len_ + response_status_len_], 8 * sizeof(char));
		project_name = (char*)buf_8;
		s = project_name.find_first_not_of(" ");
		s = s >= 0 ? s : 0;
		project_name = project_name.substr(s, 8 - s);

		std::memcpy(buf_8, &response_[6 + 33 + header_count_len_ + response_status_len_], 8 * sizeof(char));
		project_branch = (char*)buf_8;
		s = project_branch.find_first_not_of(" ");
		s = s >= 0 ? s : 0;
		project_branch = project_branch.substr(s, 8 - s);

		std::memcpy(buf_2, &response_[6 + 41 + header_count_len_ + response_status_len_], 2 * sizeof(char));
		project_version_major = (char*)buf_2;

		std::memcpy(buf_2, &response_[6 + 43 + header_count_len_ + response_status_len_], 2 * sizeof(char));
		project_version_minor = (char*)buf_2;

		std::memcpy(buf_2, &response_[6 + 45 + header_count_len_ + response_status_len_], 2 * sizeof(char));
		project_version_buid = (char*)buf_2;

		_boot_loader_version = project_name + "." + project_branch + "." + project_version_major + "." + project_version_minor + "." + project_version_buid;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get dsp version fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::dcsp_get_EEPROM_status(uint8_t& _status)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_get_extrinsic_parameters(bool& _enable, std::vector<float>& _params)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_EXTRINSIC_PARAMETERS;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 52 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_EXTRINSIC_PARAMETERS);
	if (status == STATUS_OK)
	{
		_enable = (response_[6 + 3 + header_count_len_ + response_status_len_] == 0x01);
		_params.clear();
		int val;
		for (size_t i = 0; i < 12; ++i)
		{
			memcpy(&val, &response_[6 + 3 + header_count_len_ + response_status_len_ + 1 + i * 4], sizeof(int));
			float fval = (float)val / 1.e7;
			_params.push_back(fval);
		}
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get extrinsic parameters fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::dcsp_get_fallback_status(uint8_t& _status)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_FALLBACK_STATUS;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 4 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_FALLBACK_STATUS);
	if (status == STATUS_OK)
	{
		_status = response_[6 + 3 + header_count_len_ + response_status_len_];
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get fallback status fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::dcsp_get_flash_info(uint32_t& _flash_id, uint32_t& _section_size, uint32_t& _section_num, uint32_t& _page_size, uint32_t& _page_num, uint32_t& _total_size)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_get_gaze_status(bool& _enable)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += 6;

	// payload
	request_[offset + 0] = DCSP_CMD_GET_GAZE;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 4 + 6 + 6, DCSP_CMD_GET_GAZE);
	if (status == STATUS_OK)
	{
		if (response_[6 + 3] == 0x00)
			_enable = false;
		else
			_enable = true;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get Gaze Status fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_get_mac_and_vlan(std::string& _mac, uint16_t& _vlan)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_get_mask_info(char * _data_channel, char * _data_type, char * _algorithm)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_MASK_INFORMATION;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 11 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_MASK_INFORMATION);
	if (status == STATUS_OK)
	{
		std::memcpy(_data_channel, &response_[6 + 3 + header_count_len_ + response_status_len_], 4 * sizeof(char));
		std::memcpy(_data_type, &response_[6 + 7 + header_count_len_ + response_status_len_], 2 * sizeof(char));
		std::memcpy(_algorithm, &response_[6 + 9 + header_count_len_ + response_status_len_], 2 * sizeof(char));
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get mask fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::dcsp_get_mdop_port(uint32_t & _mdop_port)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_MDOP_PORT;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 7 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_MDOP_PORT);
	if (status == STATUS_OK)
	{
		std::memcpy(&_mdop_port, &response_[6 + 3 + header_count_len_ + response_status_len_], 4 * sizeof(char));
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get MDOP port fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::dcsp_get_mode(uint8_t & _mode, int _timeout_s, int _timeout_us)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_LIDAR_MODE;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 4 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_LIDAR_MODE, _timeout_s, _timeout_us);
	if (status == STATUS_OK)
	{
		_mode = response_[6 + 3 + header_count_len_ + response_status_len_];
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get mode fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::dcsp_get_multicast_IP_port(std::string& _ip, uint32_t& _port)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_MULITCAST_IP_PORT;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 11 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_MULITCAST_IP_PORT);
	if (status == STATUS_OK)
	{
		_ip = std::to_string((int)response_[6 + 3 + header_count_len_ + response_status_len_]) + "." + 
			std::to_string((int)response_[6 + 4 + header_count_len_ + response_status_len_]) + "." +
			std::to_string((int)response_[6 + 5 + header_count_len_ + response_status_len_]) + "." + 
			std::to_string((int)response_[6 + 6 + header_count_len_ + response_status_len_]);
		memcpy(&_port, &response_[6 + 7 + header_count_len_ + response_status_len_], sizeof(uint32_t));
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get multicast IP and port fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_get_multicast_status(bool& _enable)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_MULITCAST_STATUS;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 4 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_MULITCAST_STATUS, 0, 50);
	if (status == STATUS_OK)
	{
		if (response_[6 + 3 + header_count_len_ + response_status_len_] == 0x00)
			_enable = false;
		else
			_enable = true;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get multicast status fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_get_network_timeout(uint32_t& _time)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_NETWORK_TIMEOUT;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 7 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_NETWORK_TIMEOUT);
	if (status == STATUS_OK)
	{
		std::memcpy(&_time, &response_[6 + 3 + header_count_len_ + response_status_len_], 4 * sizeof(char));
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get network timeout fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::dcsp_get_ntp_info(uint8_t& _clock_source, uint8_t& _clock_status, std::string& _server_ip, uint32_t& _interval)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_get_offline_logger_info(uint8_t& _file_num)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_get_offline_logger_parameters(uint8_t& _log_level, uint8_t& _file_size, uint8_t& _file_num, bool& _enable_authority, bool& _enable_uart, bool& _enable_udp)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_get_ptp_info(uint8_t& _clock_source, uint8_t& _clock_status, int& _offset_from_master_s, int& _offset_from_master_ns,
	int& _offset_accumulated, int& _path_delay_s, int& _path_delay_ns)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_get_rc_mode(bool &_enabled)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_get_timestamp_format(int& _format)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_TIMESTAMP_FORMAT;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 4 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_TIMESTAMP_FORMAT);
	if (status == STATUS_OK)
	{
		_format = (int)response_[6 + 3 + header_count_len_ + response_status_len_];
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get timestamp format fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::dcsp_get_temp_comp_table(const uint8_t* _data)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_TEMP_COMP_TABLE;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, (6 + header_count_len_) + 1 + 2 + response_status_len_ + 1 + 4 + 8 + 1 + 4 + 64 + 6, DCSP_CMD_GET_TEMP_COMP_TABLE);
	if (status == STATUS_OK)
	{
		// = response_[6 + 3];
		memcpy((void *)_data, &response_[6 + 3 + header_count_len_ + response_status_len_], 82);
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get temp comp table fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::dcsp_get_laser_sn(std::string& _sn)
{
	int recv_len;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_LASER_SN;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	recv_len = tx_rx_uncertain_data(offset, 255 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_LASER_SN);  //19: data len 
	if (recv_len < (15 + response_status_len_ + header_count_len_) )
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get laser sn fail!" << endl;
		return recv_len;
	}
	else
	{
		std::cout << "len: " << (int)response_[6 + header_count_len_ + 1] << std::endl;
		for (int i = 0; i < (int)response_[6 + header_count_len_ + 1]; i++)
		{
			_sn += (char)response_[6 + header_count_len_ + 3 + response_status_len_ + i];
		}
		return STATUS_OK;
	}

}

int benewake::DCSPProtocol::dcsp_get_PL_register(int32_t* _value, uint32_t _address)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_PL_REGISTER;
	request_[offset + 1] = 0x04;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = _address & 0xff;
	request_[offset + 4] = (_address >> 8) & 0xff;
	request_[offset + 5] = (_address >> 16) & 0xff;
	request_[offset + 6] = (_address >> 24) & 0xff;
	offset += 7;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, sizeof(uint32_t) + 3 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_PL_REGISTER);
	if (status == STATUS_OK)
	{
		std::memcpy(_value, &response_[6 + 3 + header_count_len_ + response_status_len_], sizeof(uint32_t));
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Read registers fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::dcsp_read_register(int32_t *_value, uint32_t _address, int _size)
{
	int status;
	int offset = 0;
	if (_size <= 0)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "size <= 0!" << endl;
		return STATUS_FAIL;
	}

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_READ_REGISTER;
	request_[offset + 1] = 0x08;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = _address & 0xff;
	request_[offset + 4] = (_address >> 8) & 0xff;
	request_[offset + 5] = (_address >> 16) & 0xff;
	request_[offset + 6] = (_address >> 24) & 0xff;
	request_[offset + 7] = _size & 0xff;
	request_[offset + 8] = (_size >> 8) & 0xff;
	request_[offset + 9] = (_size >> 16) & 0xff;
	request_[offset + 10] = (_size >> 24) & 0xff;
	offset += 11;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, _size * sizeof(uint32_t) + 3 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_READ_REGISTER);
	if (status == STATUS_OK)
	{
		std::memcpy(_value, &response_[6 + 3 + header_count_len_ + response_status_len_], _size * sizeof(uint32_t));
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Read registers fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::dcsp_read_register(uint32_t* _value, uint32_t _address, int _size)
{
	int status;
	int offset = 0;
	if (_size <= 0)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "size <= 0!" << endl;
		return STATUS_FAIL;
	}

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_READ_REGISTER;
	request_[offset + 1] = 0x08;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = _address & 0xff;
	request_[offset + 4] = (_address >> 8) & 0xff;
	request_[offset + 5] = (_address >> 16) & 0xff;
	request_[offset + 6] = (_address >> 24) & 0xff;
	request_[offset + 7] = _size & 0xff;
	request_[offset + 8] = (_size >> 8) & 0xff;
	request_[offset + 9] = (_size >> 16) & 0xff;
	request_[offset + 10] = (_size >> 24) & 0xff;
	offset += 11;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, _size * sizeof(uint32_t) + 3 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_READ_REGISTER);
	if (status == STATUS_OK)
	{
		std::memcpy(_value, &response_[6 + 3 + header_count_len_ + response_status_len_], _size * sizeof(uint32_t));
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Read registers fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::dcsp_reboot_device()
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_RESTART;
	request_[offset + 1] = 0;
	request_[offset + 2] = 0;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_RESTART);
	if (status == STATUS_OK)
	{
		//cout << "restart success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Restart fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_restore_to_default(void)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_RESTORE;
	request_[offset + 1] = 0;
	request_[offset + 2] = 0;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_RESTORE);
	if (status == STATUS_OK)
	{
		//cout << "restore to default success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Restore to default fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_clear_config()
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_CLEAR_CONFIG;
	request_[offset + 1] = 1;
	request_[offset + 2] = 0;
	request_[offset + 3] = 0x03;
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_CLEAR_CONFIG);
	if (status == STATUS_OK)
	{
		//cout << "restart success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Restart fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_anti_interference(bool _enable)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_set_automobile_info(int _battery_voltage, int _speed, int _time, int _mileage,
	int _outside_temperature, int _altitude, int _rainfall, int _vehicle_status)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_set_autorun_status(bool _enable)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_AUTORUN_STATUS;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	if (_enable)
	{
		request_[offset + 3] = 0x01;
	}
	else
	{
		request_[offset + 3] = 0x00;
	}
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_AUTORUN_STATUS);
	if (status == STATUS_OK)
	{
		//cout << "enable/disable multicast success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Enable/disable autorun fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_clock_config(bool _1588_enable, uint8_t _1588_domain_nu, bool _gptp_enable, uint8_t _gptp_domain_nu)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_set_coordinate_system(uint8_t _coordinate)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_COORDINATE_SYSTEM;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = _coordinate;
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_COORDINATE_SYSTEM);
	if (status == STATUS_OK)
	{
		//cout << "set coordinate success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set coordinate fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_crsd_remote_tcp_server_ip_port(std::string _ip, int _port)
{
	uint8_t ipInt[4];
	if (checkIP(_ip) == false)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect IP format!\n";
		return STATUS_FAIL;
	}
	vector<string> substr;
	splitString(_ip, substr, ".");
	if (substr.size() != 4)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect IP fields!\n";
		return STATUS_FAIL;
	}
	for (auto i = 0; i < 4; ++i)
	{
		ipInt[i] = stoi(substr[i]);
	}

	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_TCP_SERVER_IP_PORT;
	request_[offset + 1] = 0x08;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = ipInt[0];
	request_[offset + 4] = ipInt[1];
	request_[offset + 5] = ipInt[2];
	request_[offset + 6] = ipInt[3];
	std::cout << "ip: " << (int)ipInt[0] << (int)ipInt[1] << (int)ipInt[2] << (int)ipInt[3] << std::endl;
	memcpy(&request_[offset + 7], &_port, sizeof(uint32_t));
	offset += 11;


	// tail   
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_TCP_SERVER_IP_PORT);
	if (status == STATUS_OK)
	{
		//cout << "set ip and port success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set remote tcp server ip and port fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_customer_info(uint16_t _custom_id, char* _manufacture_date, char* _trace_code)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_set_customer_mode(int _mode)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_CUSTOMER_MODE;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = (uint8_t)_mode;

	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_CUSTOMER_MODE);
	if (status == STATUS_OK)
	{

	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set customer mode fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_destination_ip_port(std::string _ip, uint32_t _mdop_port, uint32_t _dcsp_port, uint32_t _dsop_port, 
	std::string _mask, std::string _gateway, std::string _mac, uint16_t _vlan)
{
	int status;
	int offset = 0;
	uint8_t ipInt[4], maskInt[4], gatewayInt[4];
	unsigned char mac[10]; // sscanf translate 1 bytes string to 4 bytes data! need more size to store result or overflow occurred

	if (checkIP(_ip) == false)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect IP format!\n";
		return STATUS_FAIL;
	}
	if (checkIP(_mask) == false)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect mask format!\n";
		return STATUS_FAIL;
	}
	if (checkIP(_gateway) == false)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect gateway format!\n";
		return STATUS_FAIL;
	}
	vector<string> substr;
	splitString(_ip, substr, ".");
	if (substr.size() != 4)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect IP fields!\n";
		return STATUS_FAIL;
	}
	for (auto i = 0; i < 4; ++i)
	{
		ipInt[i] = stoi(substr[i]);
	}
	substr.clear();
	splitString(_mask, substr, ".");
	if (substr.size() != 4)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect mask fields!\n";
		return STATUS_FAIL;
	}
	for (auto i = 0; i < 4; ++i)
	{
		maskInt[i] = stoi(substr[i]);
	}
	substr.clear();
	splitString(_gateway, substr, ".");
	if (substr.size() != 4)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect gateway fields!\n";
		return STATUS_FAIL;
	}
	for (auto i = 0; i < 4; ++i)
	{
		gatewayInt[i] = stoi(substr[i]);
	}
	substr.clear();
	splitString(_mac, substr, ":");
	if (substr.size() != 6)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect MAC fields!\n";
		return STATUS_FAIL;
	}
	sscanf(_mac.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);


	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_DEST_IP_AND_PORT;
	request_[offset + 1] = 0x20;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = ipInt[0];
	request_[offset + 4] = ipInt[1];
	request_[offset + 5] = ipInt[2];
	request_[offset + 6] = ipInt[3];
	memcpy(&request_[offset + 7], &_mdop_port, sizeof(uint32_t));
	memcpy(&request_[offset + 11], &_dsop_port, sizeof(uint32_t));
	memcpy(&request_[offset + 15], &_dcsp_port, sizeof(uint32_t));
	request_[offset + 19] = maskInt[0];
	request_[offset + 20] = maskInt[1];
	request_[offset + 21] = maskInt[2];
	request_[offset + 22] = maskInt[3];
	request_[offset + 23] = gatewayInt[0];
	request_[offset + 24] = gatewayInt[1];
	request_[offset + 25] = gatewayInt[2];
	request_[offset + 26] = gatewayInt[3];
	request_[offset + 27] = mac[0];
	request_[offset + 28] = mac[1];
	request_[offset + 29] = mac[2];
	request_[offset + 30] = mac[3];
	request_[offset + 31] = mac[4];
	request_[offset + 32] = mac[5];
	memcpy(&request_[offset + 33], &_vlan, sizeof(uint16_t));
	offset += 35;

	std::printf("set lidar destination MAC: %02x:%02x:%02x:%02x:%02x:%02x VLAN: %d\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], _vlan);

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_DEST_IP_AND_PORT);
	if (status == STATUS_OK)
	{
		//cout << "set ip and port success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set destination ip and port fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_device_ip_port(std::string _ip, uint32_t _mdop_port, uint32_t _dcsp_port, uint32_t _dsop_port,
	std::string _mask, std::string _gateway)
{
	int status;
	int offset = 0;
	uint8_t ipInt[4], maskInt[4], gatewayInt[4];

	if (checkIP(_ip) == false)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect IP format!\n";
		return STATUS_FAIL;
	}
	if (checkIP(_mask) == false)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect mask format!\n";
		return STATUS_FAIL;
	}
	if (checkIP(_gateway) == false)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect gateway format!\n";
		return STATUS_FAIL;
	}
	vector<string> substr;
	splitString(_ip, substr, ".");
	if (substr.size() != 4)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect IP fields!\n";
		return STATUS_FAIL;
	}
	for (auto i = 0; i < 4; ++i)
	{
		ipInt[i] = stoi(substr[i]);
	}
	substr.clear();
	splitString(_mask, substr, ".");
	if (substr.size() != 4)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect mask fields!\n";
		return STATUS_FAIL;
	}
	for (auto i = 0; i < 4; ++i)
	{
		maskInt[i] = stoi(substr[i]);
	}
	substr.clear();
	splitString(_gateway, substr, ".");
	if (substr.size() != 4)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect gateway fields!\n";
		return STATUS_FAIL;
	}
	for (auto i = 0; i < 4; ++i)
	{
		gatewayInt[i] = stoi(substr[i]);
	}
	substr.clear();
	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_IP_AND_PORT;
	request_[offset + 1] = 0x18;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = ipInt[0];
	request_[offset + 4] = ipInt[1];
	request_[offset + 5] = ipInt[2];
	request_[offset + 6] = ipInt[3];
	memcpy(&request_[offset + 7], &_mdop_port, sizeof(uint32_t));
	memcpy(&request_[offset + 11], &_dsop_port, sizeof(uint32_t));
	memcpy(&request_[offset + 15], &_dcsp_port, sizeof(uint32_t));
	request_[offset + 19] = maskInt[0];
	request_[offset + 20] = maskInt[1];
	request_[offset + 21] = maskInt[2];
	request_[offset + 22] = maskInt[3];
	request_[offset + 23] = gatewayInt[0];
	request_[offset + 24] = gatewayInt[1];
	request_[offset + 25] = gatewayInt[2];
	request_[offset + 26] = gatewayInt[3];
	offset += 27;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_IP_AND_PORT);
	if (status == STATUS_OK)
	{
		//cout << "set ip and port success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set ip and port fail! Error code: " << status << endl;
	}

	return status;

}

int benewake::DCSPProtocol::dcsp_set_device_ip_port_mac(std::string _ip, uint32_t _mdop_port, uint32_t _dcsp_port, uint32_t _dsop_port, 
	std::string _mask, std::string _gateway, bool _use_config_mac, std::string _mac)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_set_DHCP(bool _enable)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_DHCP;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	if (_enable)
	{
		request_[offset + 3] = 0x01;
	}
	else
	{
		request_[offset + 3] = 0x00;
	}
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_DHCP);
	if (status == STATUS_OK)
	{
		//cout << "enable/disable DHCP success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Enable/disable DHCP fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_dsop_dest_ip_and_port(std::string _ip, uint32_t _port)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_set_extrinsic_parameters(bool _enable, const std::vector<float>& _params)
{
	if (_params.size() != 12)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Extrinsic parameters size is incorrect!" << endl;
		return STATUS_FAIL;
	}

	int status;
	int offset = 0;
	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_EXTRINSIC_PARAMETERS;
	request_[offset + 1] = 0x31;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = _enable ? 0x01 : 0x00;
	for (size_t i = 0; i < _params.size(); ++i)
	{
		int val = _params[i] * 1.e7;
		memcpy(&request_[offset + 4 + 4 * i], &val, sizeof(int));
	}
	offset += 52;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_EXTRINSIC_PARAMETERS);
	if (status == STATUS_OK)
	{
		//cout << "set extrinsic parameters success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set extrinsic parameters fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_fallback_status(bool _enable, uint8_t _cpu_select)
{
	int status;
	int offset = 0;
	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);
	// payload
	request_[offset + 0] = DCSP_CMD_SET_FALLBACK_STATUS;
	request_[offset + 1] = 0x02;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = _cpu_select;
	if (_enable)
		request_[offset + 4] = 0x01;
	else
		request_[offset + 4] = 0x00;
	offset += 5;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_FALLBACK_STATUS);
	if (status == STATUS_OK)
	{
		//cout << "set mask success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set fallback status fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_fov_roi_location(uint16_t _horizontal_location, uint16_t _vertical_location)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_set_fov_roi_size(uint16_t _width, uint16_t _height)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_set_gaze_status(bool _enable)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += 6;

	// payload
	request_[offset + 0] = DCSP_CMD_SET_GAZE;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	if (_enable)
	{
		request_[offset + 3] = 0x01;
	}
	else
	{
		request_[offset + 3] = 0x00;
	}
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_GAZE);
	if (status != STATUS_OK)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set Gaze Open fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_mask_info(const char * _data_channel, const char * _data_type, const char * _algorithm)
{
	int status;
	int offset = 0;
	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);
	// payload
	request_[offset + 0] = DCSP_CMD_SET_MASK_INFORMATION;
	request_[offset + 1] = 0x08;
	request_[offset + 2] = 0x00;
	std::memcpy(&request_[offset + 3], _data_channel, 4 * sizeof(char));
	std::memcpy(&request_[offset + 7], _data_type, 2 * sizeof(char));
	std::memcpy(&request_[offset + 9], _algorithm, 2 * sizeof(char));
	offset += 11;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_MASK_INFORMATION);
	if (status == STATUS_OK)
	{
		//cout << "set mask success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set mask fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_mode(uint8_t _mode)
{
	if (_mode > 16)
	{
		cout << "ERROR: Unknow mode set!" << endl;
		return STATUS_FAIL;
	}

	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_LIDAR_MODE;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = _mode;
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_LIDAR_MODE, 5);
	if (status == STATUS_OK)
	{
		//cout << "set lidar work mode success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set lidar work mode fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_multicast_IP_port(std::string _ip, uint32_t _port)
{
	uint8_t ipInt[4];
	if (checkIP(_ip) == false)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect IP format!\n";
		return STATUS_FAIL;
	}
	vector<string> substr;
	splitString(_ip, substr, ".");
	if (substr.size() != 4)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect IP fields!\n";
		return STATUS_FAIL;
	}
	for (auto i = 0; i < 4; ++i)
	{
		ipInt[i] = stoi(substr[i]);
	}

	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_MULITCAST_IP_PORT;
	request_[offset + 1] = 0x08;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = ipInt[0];
	request_[offset + 4] = ipInt[1];
	request_[offset + 5] = ipInt[2];
	request_[offset + 6] = ipInt[3];
	memcpy(&request_[offset + 7], &_port, sizeof(uint32_t));
	offset += 11;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_MULITCAST_IP_PORT);
	if (status == STATUS_OK)
	{
		//cout << "set ip and port success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set multicast ip and port fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_multicast_status(bool _enable)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_MULITCAST_STATUS;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	if (_enable)
	{
		request_[offset + 3] = 0x01;
	}
	else
	{
		request_[offset + 3] = 0x00;
	}
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_MULITCAST_STATUS);
	if (status == STATUS_OK)
	{
		//cout << "enable/disable multicast success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Enable/disable multicast fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_network_timeout(uint32_t _time)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_NETWORK_TIMEOUT;
	request_[offset + 1] = 0x04;
	request_[offset + 2] = 0x00;
	memcpy(&request_[offset + 3], &_time, sizeof(uint32_t));
	offset += 7;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_NETWORK_TIMEOUT);
	if (status == STATUS_OK)
	{
		//cout << "enable/disable multicast success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set network timeout fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_ntp_info(std::string _server_ip, uint32_t _interval)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_set_offline_logger_parameters(uint8_t _log_level, uint8_t _file_size, uint8_t _file_num, bool _enable_authority, bool _enable_uart, bool _enable_udp)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_set_PL_register(const int32_t* _value, uint32_t _address)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_PL_REGISTER;
	request_[offset + 1] = 0x08;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = _address & 0xff;
	request_[offset + 4] = (_address >> 8) & 0xff;
	request_[offset + 5] = (_address >> 16) & 0xff;
	request_[offset + 6] = (_address >> 24) & 0xff;
	std::memcpy(&request_[offset + 7], _value, sizeof(int32_t));
	offset += 11;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_PL_REGISTER);
	if (status == STATUS_OK)
	{
		//cout << "write registers success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Write registers fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::dcsp_set_rc_mode(bool _enable)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_set_roi(bool _enable, const std::vector<float>& _vertices)
{
	if (_enable && (_vertices.size() < 6 || _vertices.size() % 2 != 0 || _vertices.size() > 344))
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "ROI vertices' amount is incorrect!" << endl;
		return STATUS_FAIL;
	}

	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_ROI;
	uint16_t data_length = 2 + _vertices.size() * 4;
	memcpy(&request_[offset + 1], &data_length, sizeof(uint16_t));
	request_[offset + 3] = _enable ? 0x01 : 0x00;
	request_[offset + 4] = (uint8_t)(_vertices.size() / 2);
	for (size_t i = 0; i < _vertices.size(); ++i)
	{
		int val = _vertices[i] * 100;
		memcpy(&request_[offset + 5 + 4 * i], &val, sizeof(int));
	}
	offset += (5 + _vertices.size() * 4);

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_ROI);
	if (status == STATUS_OK)
	{
		//cout << "enable/disable multicast success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set ROI fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_signal_message(char* _msg)
{

	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_SIGNAL_MESSAGE;
	request_[offset + 1] = 0x04;
	request_[offset + 2] = 0x00;
	memcpy(&request_[offset + 3], _msg, 4 * sizeof(char));
	offset += 7;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_SIGNAL_MESSAGE);
	if (status == STATUS_OK)
	{
		//cout << "enable/disable multicast success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set signal message fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_sn(const char * _sn)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_SN;
	request_[offset + 1] = 0x20;
	request_[offset + 2] = 0x00;
	std::memcpy(&request_[offset + 3], _sn, 16);
	memset(&request_[offset + 3 + 16], 0x20, 16);
	offset += 35;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_SN);
	if (status == STATUS_OK)
	{
		//cout << "set SN success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set SN fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_static_arp(bool _enable)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_set_sunlight_resistance(bool _enable)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_set_timestamp_format(int _format)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_TIMESTAMP_FORMAT;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = (uint8_t)_format;
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_TIMESTAMP_FORMAT);
	if (status == STATUS_OK)
	{
		//cout << "set timestamp format success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set timestamp format fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_set_vlan_status(bool _enable)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_set_window_heating(uint8_t _win_enable, uint16_t _voltage,
	int16_t _temp_a, int16_t _temp_b, int16_t _temp_c, int16_t _temp_d,
	uint8_t _tx_enable, int16_t _param_x, int16_t _param_y, int16_t _param_z)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_shutdown_device()
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SHUTDOWN;
	request_[offset + 1] = 0;
	request_[offset + 2] = 0;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SHUTDOWN);
	if (status == STATUS_OK)
	{
		//cout << "shutdown success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Shutdown fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_shutdown_warning(uint8_t _delay_sec)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_store(uint8_t _store_flag)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_STORE;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0;
	request_[offset + 3] = _store_flag;
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_STORE);
	if (status == STATUS_OK)
	{
		//cout << "store success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Store fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocol::dcsp_switch_partition()
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_upload_lut(std::string _table_file, uint8_t _table_id)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Uploading LUT has not been applied to X2 lidar!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_upload_offline_logger(std::string _save_name, uint8_t _file_id)
{
	std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
	cout << "Current device type do not support this CMD!" << endl;
	return STATUS_FAIL;
}

int benewake::DCSPProtocol::dcsp_write_register(const int32_t * _value, uint32_t _address, int _size)
{
	int status;
	int offset = 0;
	if (_size <= 0)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "size <= 0!" << endl;
		return STATUS_FAIL;
	}

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_WRITE_REGISTER;
	request_[offset + 1] = (sizeof(uint32_t) * _size + 8) & 0xff;
	request_[offset + 2] = (sizeof(uint32_t) * _size + 8) >> 8;
	request_[offset + 3] = _address & 0xff;
	request_[offset + 4] = (_address >> 8) & 0xff;
	request_[offset + 5] = (_address >> 16) & 0xff;
	request_[offset + 6] = (_address >> 24) & 0xff;
	request_[offset + 7] = _size & 0xff;
	request_[offset + 8] = (_size >> 8) & 0xff;
	request_[offset + 9] = (_size >> 16) & 0xff;
	request_[offset + 10] = (_size >> 24) & 0xff;
	std::memcpy(&request_[offset + 11], _value, _size * sizeof(uint32_t));
	offset += 11 + sizeof(uint32_t) * _size;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_WRITE_REGISTER);
	if (status == STATUS_OK)
	{
		//cout << "write registers success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Write registers fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::dcsp_write_register(const uint32_t* _value, uint32_t _address, int _size)
{
	int status;
	int offset = 0;
	if (_size <= 0)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "size <= 0!" << endl;
		return STATUS_FAIL;
	}

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_WRITE_REGISTER;
	request_[offset + 1] = (sizeof(uint32_t) * _size + 8) & 0xff;
	request_[offset + 2] = (sizeof(uint32_t) * _size + 8) >> 8;
	request_[offset + 3] = _address & 0xff;
	request_[offset + 4] = (_address >> 8) & 0xff;
	request_[offset + 5] = (_address >> 16) & 0xff;
	request_[offset + 6] = (_address >> 24) & 0xff;
	request_[offset + 7] = _size & 0xff;
	request_[offset + 8] = (_size >> 8) & 0xff;
	request_[offset + 9] = (_size >> 16) & 0xff;
	request_[offset + 10] = (_size >> 24) & 0xff;
	std::memcpy(&request_[offset + 11], _value, _size * sizeof(uint32_t));
	offset += 11 + sizeof(uint32_t) * _size;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_WRITE_REGISTER);
	if (status == STATUS_OK)
	{
		//cout << "write registers success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Write registers fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocol::tx_rx_check_status(int tx_size, uint8_t cmd_response, int _timeout_s, int _timeout_us)
{
	int trytimes = 0;
	int rt;
#ifdef DEBUG_INFO
	std::printf("Execute CMD 0x%02x ...\n", cmd_response);
#endif // DEBUG_INFO
	for (trytimes = 0; trytimes < 3; ++trytimes)
	{
		try
		{
#ifdef DEBUG_INFO
			cout << "    attempts: " << (trytimes + 1) << endl;
#endif // DEBUG_INFO
			rt = pudp_->sendData(request_, tx_size);
			if (rt == STATUS_FAIL)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "Send failed!" << endl;
				return rt;
			}
			else if (rt == STATUS_TIME_OUT)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "Send timeout!" << endl;
				continue;
			}
			memset(response_, 0, PROTOCOL_DATA_PACKAGE_MAX_LENGTH);
			rt = pudp_->recvCMDResponse(response_, 6 + 4 + 6, cmd_response, _timeout_s, _timeout_us);
			if (rt == STATUS_FAIL)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "Receive failed!" << endl;
				return rt;
			}
			else if (rt == STATUS_TIME_OUT)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "Receive timeout!" << endl;
				continue;
			}
		}
		catch (const std::exception& e)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << e.what() << endl;
			return STATUS_FAIL;
		}

		if (memcmp(response_, response_header_, 4) == 0)
		{
			uint16_t protocol_version = 0;
			memcpy(&protocol_version, &response_[PROTOCOL_DATA_PACKAGE_VERSION_OFFSET], sizeof(uint16_t));
			uint32_t response_sum = check_sum_with_protocol_version(protocol_version, response_, 6 + 4);
			if ((response_sum & 0xff) == response_[10] && ((response_sum >> 8) & 0xff) == response_[11] &&
				((response_sum >> 16) & 0xff) == response_[12] && ((response_sum >> 24) & 0xff) == response_[13] &&
				(response_[14] == 0x00) && (response_[15] == 0xff))
			{
				if (response_[9] == STATUS_OK)
				{
					break;
				}
				else
				{
					return STATUS_FAIL;
				}
			}
			else
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "Incorrect data package tail: " <<
					(unsigned int)response_[0] << ", " <<
					(unsigned int)response_[1] << ", " <<
					(unsigned int)response_[2] << ", " <<
					(unsigned int)response_[3] << ", " <<
					(unsigned int)response_[4] << ", " <<
					(unsigned int)response_[5] << ", " <<
					(unsigned int)response_[6] << ", " <<
					(unsigned int)response_[7] << ", " <<
					(unsigned int)response_[8] << ", " <<
					(unsigned int)response_[9] << ", " <<
					(unsigned int)response_[10] << ", " <<
					(unsigned int)response_[11] << ", " <<
					(unsigned int)response_[12] << ", " <<
					(unsigned int)response_[13] << ", " <<
					(unsigned int)response_[14] << ", " << endl;
				cout << "check sum = " << response_sum << endl;
				continue;
			}

		}
		else
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << "Incorrect data package header: " << (unsigned int)response_[0] << ", " <<
				(unsigned int)response_[1] << ", " <<
				(unsigned int)response_[2] << ", " <<
				(unsigned int)response_[3] << ", " <<
				(unsigned int)response_[4] << ", " <<
				(unsigned int)response_[5] << endl;
			continue;
		}
	}
	if (trytimes == 3)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Out of times!" << endl;
		return STATUS_TIME_OUT;
	}
	return STATUS_OK;
}

int benewake::DCSPProtocol::tx_rx_data(int tx_size, int rx_size, uint8_t cmd_response, int _timeout_s, int _timeout_us)
{
	int trytimes = 0;
	int rt;
#ifdef DEBUG_INFO
	std::printf("Execute CMD 0x%02x ...\n", cmd_response);
#endif // DEBUG_INFO
	for (trytimes = 0; trytimes < 3; ++trytimes)
	{
		try
		{
#ifdef DEBUG_INFO
			cout << "    attempts: " << (trytimes + 1) << endl;
#endif // DEBUG_INFO
			rt = pudp_->sendData(request_, tx_size);
			if (rt == STATUS_FAIL)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "Send failed!" << endl;
				return rt;
			}
			else if (rt == STATUS_TIME_OUT)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "Send timeout!" << endl;
				continue;
			}
			memset(response_, 0, PROTOCOL_DATA_PACKAGE_MAX_LENGTH);
			rt = pudp_->recvCMDResponse(response_, rx_size, cmd_response, _timeout_s, _timeout_us);
			if (rt == STATUS_FAIL)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "Receive failed!" << endl;
				return rt;
			}
			else if (rt == STATUS_TIME_OUT)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "Receive timeout!" << endl;
				continue;
			}
		}
		catch (const std::exception& e)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << e.what() << endl;
			return STATUS_FAIL;
		}
		if (memcmp(response_, response_header_, 4) == 0)
		{
			uint16_t protocol_version = 0;
			memcpy(&protocol_version, &response_[PROTOCOL_DATA_PACKAGE_VERSION_OFFSET], sizeof(uint16_t));
			uint32_t response_sum = check_sum_with_protocol_version(protocol_version, response_, rx_size - 6);
			if ((response_sum & 0xff) == response_[rx_size - 6] && ((response_sum >> 8) & 0xff) == response_[rx_size - 5] &&
				((response_sum >> 16) & 0xff) == response_[rx_size - 4] && ((response_sum >> 24) & 0xff) == response_[rx_size - 3] &&
				(response_[rx_size - 2] == 0x00) && (response_[rx_size - 1] == 0xff))
			{
				break;
			}
			else
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "Incorrect data package tail: " << endl;
				cout << "check sum = " << response_sum << endl;
				continue;
			}
		}
		else
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << "Incorrect data package header: " << (unsigned int)response_[0] << ", " <<
				(unsigned int)response_[1] << ", " <<
				(unsigned int)response_[2] << ", " <<
				(unsigned int)response_[3] << ", " <<
				(unsigned int)response_[4] << ", " <<
				(unsigned int)response_[5] << endl;
			continue;
		}
	}
	if (trytimes == 3)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Out of times!" << endl;
		return STATUS_TIME_OUT;
	}
	return STATUS_OK;
}

int benewake::DCSPProtocol::tx_rx_uncertain_data(int tx_size, int rx_size, uint8_t cmd_response, int _timeout_s, int _timeout_us)
{
	int trytimes = 0;
	int rt, recv_size;
#ifdef DEBUG_INFO
	std::printf("Execute CMD 0x%02x ...\n", cmd_response);
#endif // DEBUG_INFO
	for (trytimes = 0; trytimes < 3; ++trytimes)
	{
		try
		{
#ifdef DEBUG_INFO
			cout << "    attempts: " << (trytimes + 1) << endl;
#endif // DEBUG_INFO
			rt = pudp_->sendData(request_, tx_size);
			if (rt == STATUS_FAIL)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "send failed!" << endl;
				return rt;
			}
			else if (rt == STATUS_TIME_OUT)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "send timeout!" << endl;
				continue;
			}
			memset(response_, 0, PROTOCOL_DATA_PACKAGE_MAX_LENGTH);
			rt = pudp_->recvUncertainCMDResponse(response_, rx_size, cmd_response, _timeout_s, _timeout_us);
			if (rt == STATUS_FAIL)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "receive failed!" << endl;
				return rt;
			}
			else if (rt == STATUS_TIME_OUT)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "receive timeout!" << endl;
				continue;
			}
			else if (rt <= 12)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "receive unknow data!" << endl;
				continue;
			}
			recv_size = rt;
		}
		catch (const std::exception& e)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << e.what() << endl;
			return STATUS_FAIL;
		}
		if (memcmp(response_, response_header_, 4) == 0)
		{
			uint16_t protocol_version = 0;
			memcpy(&protocol_version, &response_[PROTOCOL_DATA_PACKAGE_VERSION_OFFSET], sizeof(uint16_t));
			uint32_t response_sum = check_sum_with_protocol_version(protocol_version, response_, recv_size - 6);
			if ((response_sum & 0xff) == response_[recv_size - 6] && ((response_sum >> 8) & 0xff) == response_[recv_size - 5] &&
				((response_sum >> 16) & 0xff) == response_[recv_size - 4] && ((response_sum >> 24) & 0xff) == response_[recv_size - 3] &&
				(response_[recv_size - 2] == 0x00) && (response_[recv_size - 1] == 0xff))
			{
				break;
			}
			else
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "Incorrect data package tail: " << endl;
				cout << "check sum = " << response_sum << endl;
				continue;
			}
		}
		else
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << "Incorrect data package header: " << (unsigned int)response_[0] << ", " <<
				(unsigned int)response_[1] << ", " <<
				(unsigned int)response_[2] << ", " <<
				(unsigned int)response_[3] << ", " <<
				(unsigned int)response_[4] << ", " <<
				(unsigned int)response_[5] << endl;
			continue;
		}
	}
	if (trytimes == 3)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Out of times!" << endl;
		return STATUS_TIME_OUT;
	}
	return recv_size;
}

void benewake::DCSPProtocol::padding_tail(int offset, uint32_t checksum)
{
	request_[offset + 0] = checksum & 0xff;
	request_[offset + 1] = (checksum >> 8) & 0xff;
	request_[offset + 2] = (checksum >> 16) & 0xff;
	request_[offset + 3] = (checksum >> 24) & 0xff;
	request_[offset + 4] = 0x00;
	request_[offset + 5] = 0xff;
}

bool benewake::DCSPProtocol::loadTable(std::string file)
{
	std::string strline;
	std::vector<std::string> str_list;
	std::fstream tableFile(file);
	table_.clear();

	if (tableFile.is_open())
	{
		while (getline(tableFile, strline)) {
			if (strline == "table_id:") {
				getline(tableFile, strline);
				table_id_ = atoi(strline.c_str());
			}
			if (strline == "gen_time:") {
				getline(tableFile, strline);
				splitString(strline, str_list, " ");
				if (str_list.size() != 6)
				{
					std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
					cout << "Table's generation time is incorrect!" << endl;
				}
				for (int i = 0; i < str_list.size(); i++)
				{
					table_.push_back(atoi(str_list[i].c_str()));
				}
				time_t now_time;
				struct tm *utc_time;
				time(&now_time);
				utc_time = localtime(&now_time);
				table_.push_back(utc_time->tm_year + 1900);
				table_.push_back(utc_time->tm_mon + 1);
				table_.push_back(utc_time->tm_mday);
				table_.push_back(utc_time->tm_hour);
				table_.push_back(utc_time->tm_min);
				table_.push_back(utc_time->tm_sec);
			}
			if (strline == "version:") {
				getline(tableFile, strline);
				table_.push_back(atoi(strline.c_str()));
			}
			if (strline == "sn:") {
				getline(tableFile, table_sn_);
			}
			if (strline == "row_num:") {
				getline(tableFile, strline);
				table_row_ = atoi(strline.c_str());
				table_.push_back(table_row_);
			}
			if (strline == "col_num:") {
				getline(tableFile, strline);
				table_col_ = atoi(strline.c_str());
				table_.push_back(table_col_);
			}
			if (strline == "table:") {
				for (int row = 0; row < table_row_; row++)
				{
					if (getline(tableFile, strline))
					{
						splitString(strline, str_list, " ");
						for (int i = 0; i < str_list.size(); i++)
						{
							table_.push_back(atoi(str_list[i].c_str()));
						}
					}
					else
						break;
				}
			}
		}
		tableFile.close();
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cerr << "Cannot open table file!" << std::endl;
		return false;
	}

	if (table_.size() != table_row_ * table_col_ + 15)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cerr << "Table length is incorrect!" << std::endl;
		return false;
	}

	return true;
}

void benewake::DCSPProtocol::setRequestHeader(unsigned char* _request, unsigned char* _request_header, int len)
{
	_request_header[4] = dcsp_info_->protocol & 0xff;
	_request_header[5] = (dcsp_info_->protocol >> 8) & 0xff;
	std::memcpy(_request, _request_header, len);
}

void benewake::DCSPProtocol::setDeviceInfoPtr(SYS_INFO *_info)
{
	dcsp_info_ = _info;
	request_header_[4] = dcsp_info_->protocol & 0xff;
	request_header_[5] = (dcsp_info_->protocol >> 8) & 0xff;

	response_header_[4] = dcsp_info_->protocol & 0xff;
	response_header_[5] = (dcsp_info_->protocol >> 8) & 0xff;
}

benewake::DCSPProtocolP4::DCSPProtocolP4(std::string _send_ip, int _send_port, std::string _recv_ip, int _recv_port, UDPType _udp_type) :
	DCSPProtocol(_send_ip, _send_port, _recv_ip, _recv_port, _udp_type)
{
	request_header_[0] = 'B';
	request_header_[1] = 'W';
	request_header_[2] = PRODUCT_ID_P4;
	request_header_[3] = PROTOCOL_ID_DCSP_REQUEST;

	response_header_[0] = 'B';
	response_header_[1] = 'W';
	response_header_[2] = PRODUCT_ID_P4;
	response_header_[3] = PROTOCOL_ID_DCSP_RESPONSE;

	dcsp_count_ = 0xffffffff;
	header_count_len_ = 4;
	response_status_len_ = 2;
}

int benewake::DCSPProtocolP4::dcsp_clear_work_time()
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_CLEAR_WORK_TIME;
	request_[offset + 1] = 0;
	request_[offset + 2] = 0;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_CLEAR_WORK_TIME);
	if (status == STATUS_OK)
	{
		//cout << "restart success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Clear work time fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_format_file_system()
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_FORMAT_FILE_SYSTEM;
	request_[offset + 1] = 0;
	request_[offset + 2] = 0;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_FORMAT_FILE_SYSTEM);
	if (status == STATUS_OK)
	{
		//cout << "restart success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Format file system fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_get_algorithm_version(std::vector<std::string>& _versions)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_ALGORITHM_VERSION;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 6 + header_count_len_ + 3 + response_status_len_ + 258 + 6, DCSP_CMD_GET_ALGORITHM_VERSION);
	if (status == STATUS_OK)
	{
		uint16_t algo_num = 0;
		memcpy(&algo_num, &response_[6 + header_count_len_ + 3 + response_status_len_], sizeof(uint16_t));
		char* ver = new char[33];
		memset(ver, 0, 33 * sizeof(char));
		_versions.clear();
		for (uint16_t i = 0; i < algo_num; ++i)
		{
			memcpy(ver, &response_[6 + header_count_len_ + 3 + response_status_len_ + 2 + i * 32], 32 * sizeof(char));
			std::string ver_str(ver);
			_versions.push_back(ver_str);
		}
		delete[] ver;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get algorithm version fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocolP4::dcsp_get_app_partition(uint8_t& _partition)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_APP_PARTITION;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 6 + header_count_len_ + 3 + response_status_len_ + 1 + 6, DCSP_CMD_GET_APP_PARTITION);
	if (status == STATUS_OK)
	{
		_partition = response_[6 + header_count_len_ + 3 + response_status_len_];
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get APP partition fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocolP4::dcsp_get_clock_config(bool& _1588_bmc, uint8_t& _1588_domain_nu, bool& _gptp_bmc, uint8_t& _gptp_domain_nu)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_CLOCK_CFG;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 7 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_CLOCK_CFG);
	if (status == STATUS_OK)
	{
		if (response_[6 + 3 + header_count_len_ + response_status_len_] == 0x00)
			_1588_bmc = false;
		else
			_1588_bmc = true;
		if (response_[6 + 3 + header_count_len_ + response_status_len_ + 1] == 0x00)
			_gptp_bmc = false;
		else
			_gptp_bmc = true;
		_1588_domain_nu = response_[6 + 3 + header_count_len_ + response_status_len_ + 2];
		_gptp_domain_nu = response_[6 + 3 + header_count_len_ + response_status_len_ + 3];
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get clock config fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_get_DDR_info(uint32_t& _ddr_capacity)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_DDR_INFO;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 6 + header_count_len_ + 3 + response_status_len_ + 4 + 6, DCSP_CMD_GET_DDR_INFO);
	if (status == STATUS_OK)
	{
		memcpy(&_ddr_capacity, &response_[6 + header_count_len_ + 3 + response_status_len_], sizeof(uint32_t));
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get DDR info fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocolP4::dcsp_get_dsop_dest_ip_and_port(std::string& _ip, uint32_t& _port)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_DSOP_DEST_IP_AND_PORT;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 6 + header_count_len_ + 3 + response_status_len_ + 8 + 6, DCSP_CMD_GET_DSOP_DEST_IP_AND_PORT);
	if (status == STATUS_OK)
	{
		_ip = std::to_string((int)response_[6 + header_count_len_ + 3 + response_status_len_]) + "." +
			std::to_string((int)response_[6 + header_count_len_ + 3 + response_status_len_ + 1]) + "." +
			std::to_string((int)response_[6 + header_count_len_ + 3 + response_status_len_ + 2]) + "." +
			std::to_string((int)response_[6 + header_count_len_ + 3 + response_status_len_ + 3]);
		memcpy(&_port, &response_[6 + header_count_len_ + 3 + response_status_len_ + 4], sizeof(uint32_t));
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get DSOP destination IP and port fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_get_EEPROM_status(uint8_t& _status)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_EEPROM_STATUS;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 6 + header_count_len_ + 3 + response_status_len_ + 1 + 6, DCSP_CMD_GET_EEPROM_STATUS);
	if (status == STATUS_OK)
	{
		_status = response_[6 + header_count_len_ + 3 + response_status_len_];
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get APP partition fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocolP4::dcsp_get_flash_info(uint32_t& _flash_id, uint32_t& _section_size, uint32_t& _section_num, uint32_t& _page_size, uint32_t& _page_num, uint32_t& _total_size)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_FLASH_INFO;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 6 + header_count_len_ + 3 + response_status_len_ + 24 + 6, DCSP_CMD_GET_FLASH_INFO);
	if (status == STATUS_OK)
	{
		memcpy(&_flash_id, &response_[6 + header_count_len_ + 3 + response_status_len_], sizeof(uint32_t));
		memcpy(&_section_size, &response_[6 + header_count_len_ + 3 + response_status_len_ + 4], sizeof(uint32_t));
		memcpy(&_section_num, &response_[6 + header_count_len_ + 3 + response_status_len_ + 8], sizeof(uint32_t));
		memcpy(&_page_size, &response_[6 + header_count_len_ + 3 + response_status_len_ + 12], sizeof(uint32_t));
		memcpy(&_page_num, &response_[6 + header_count_len_ + 3 + response_status_len_ + 16], sizeof(uint32_t));
		memcpy(&_total_size, &response_[6 + header_count_len_ + 3 + response_status_len_ + 20], sizeof(uint32_t));
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get APP partition fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocolP4::dcsp_get_mac_and_vlan(std::string& _mac, uint16_t& _vlan)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_MAC_AND_VLAN;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 6 + header_count_len_ + 3 + response_status_len_ + 8 + 6, DCSP_CMD_GET_MAC_AND_VLAN);
	if (status == STATUS_OK)
	{
		_mac.clear();
		for (int i = 0; i < 6; ++i)
		{
			char buff[3];
			snprintf(buff, sizeof(buff), "%02x", (uint8_t)response_[6 + header_count_len_ + 3 + response_status_len_ + i]);
			_mac += std::string(buff);
			if (i != 5)
				_mac += ":";
		}
		memcpy(&_vlan, &response_[6 + header_count_len_ + 3 + response_status_len_ + 6], sizeof(uint16_t));
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get MAC and VLAN fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocolP4::dcsp_get_ntp_info(uint8_t& _clock_source, uint8_t& _clock_status, std::string& _server_ip, uint32_t& _interval)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_NTP_INFO;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 6 + header_count_len_ + 3 + response_status_len_ + 10 + 6, DCSP_CMD_GET_NTP_INFO);
	if (status == STATUS_OK)
	{
		_clock_source = response_[6 + header_count_len_ + 3 + response_status_len_];
		_clock_status = response_[6 + header_count_len_ + 3 + response_status_len_ + 1];
		_server_ip = std::to_string((int)response_[6 + header_count_len_ + 3 + response_status_len_ + 2]) + "." +
						std::to_string((int)response_[6 + header_count_len_ + 3 + response_status_len_ + 3]) + "." +
						std::to_string((int)response_[6 + header_count_len_ + 3 + response_status_len_ + 4]) + "." +
						std::to_string((int)response_[6 + header_count_len_ + 3 + response_status_len_ + 5]);
		memcpy(&_interval, &response_[6 + header_count_len_ + 3 + response_status_len_ + 6], sizeof(uint32_t));
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get offline logger information fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocolP4::dcsp_get_offline_logger_info(uint8_t& _file_num)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_OFFLINE_LOGGER_INFO;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 6 + header_count_len_ + 3 + response_status_len_ + 1 + 6, DCSP_CMD_GET_OFFLINE_LOGGER_INFO);
	if (status == STATUS_OK)
	{
		_file_num = response_[6 + header_count_len_ + 3 + response_status_len_];
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get offline logger information fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocolP4::dcsp_get_offline_logger_parameters(uint8_t& _log_level, uint8_t& _file_size, uint8_t& _file_num, 
	bool& _enable_authority, bool& _enable_uart, bool& _enable_udp)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_OFFLINE_LOGGER_PARAM;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 6 + header_count_len_ + 3 + response_status_len_ + 6 + 6, DCSP_CMD_GET_OFFLINE_LOGGER_PARAM);
	if (status == STATUS_OK)
	{
		_log_level = response_[6 + header_count_len_ + 3 + response_status_len_];
		_file_size = response_[6 + header_count_len_ + 3 + response_status_len_ + 1];
		_file_num = response_[6 + header_count_len_ + 3 + response_status_len_ + 2];
		_enable_authority = (response_[6 + header_count_len_ + 3 + response_status_len_ + 3] == 0x01);
		_enable_uart = (response_[6 + header_count_len_ + 3 + response_status_len_ + 4] == 0x01);
		_enable_udp = (response_[6 + header_count_len_ + 3 + response_status_len_ + 5] == 0x01);
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get offline logger parameters fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocolP4::dcsp_get_ptp_info(uint8_t& _clock_source, uint8_t& _clock_status, int& _offset_from_master_s, int& _offset_from_master_ns,
	int& _offset_accumulated, int& _path_delay_s, int& _path_delay_ns)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_PTP_INFO;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 6 + header_count_len_ + 3 + response_status_len_ + 22 + 6, DCSP_CMD_GET_PTP_INFO);
	if (status == STATUS_OK)
	{
		_clock_source = response_[6 + header_count_len_ + 3 + response_status_len_];
		_clock_status = response_[6 + header_count_len_ + 3 + response_status_len_ + 1];
		std::memcpy(&_offset_from_master_s, &response_[6 + header_count_len_ + 3 + response_status_len_ + 2], sizeof(int));
		std::memcpy(&_offset_from_master_ns, &response_[6 + header_count_len_ + 3 + response_status_len_ + 6], sizeof(int));
		std::memcpy(&_offset_accumulated, &response_[6 + header_count_len_ + 3 + response_status_len_ + 10], sizeof(int));
		std::memcpy(&_path_delay_s, &response_[6 + header_count_len_ + 3 + response_status_len_ + 14], sizeof(int));
		std::memcpy(&_path_delay_ns, &response_[6 + header_count_len_ + 3 + response_status_len_ + 18], sizeof(int));
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get PTP/gPTP information fail! Error code: " << status << endl;
	}
	return status;
}

int benewake::DCSPProtocolP4::dcsp_get_rc_mode(bool& _enabled)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_GET_RC_MODE;
	request_[offset + 1] = 0x00;
	request_[offset + 2] = 0x00;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_data(offset, 4 + response_status_len_ + (6 + header_count_len_) + 6, DCSP_CMD_GET_RC_MODE);
	if (status == STATUS_OK)
	{
		if (response_[6 + 3 + header_count_len_ + response_status_len_] == 0x00)
			_enabled = true;
		else
			_enabled = false;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Get RC mode fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_set_anti_interference(bool _enable)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_ANTI_INTERFERENCE;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	if (_enable)
	{
		request_[offset + 3] = 0x01;
	}
	else
	{
		request_[offset + 3] = 0x00;
	}
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_ANTI_INTERFERENCE);
	if (status == STATUS_OK)
	{
		//cout << "enable/disable multicast success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Enable/disable anti-interference fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_set_automobile_info(int _battery_voltage, int _speed, int _time, int _mileage,
	int _outside_temperature, int _altitude, int _rainfall, int _vehicle_status)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_AUTOMOBILE_INFO;
	request_[offset + 1] = 0x20;
	request_[offset + 2] = 0x00;
	memcpy(&request_[offset + 3], &_battery_voltage, sizeof(int));
	memcpy(&request_[offset + 7], &_speed, sizeof(int));
	memcpy(&request_[offset + 11], &_time, sizeof(int));
	memcpy(&request_[offset + 15], &_mileage, sizeof(int));
	memcpy(&request_[offset + 19], &_outside_temperature, sizeof(int));
	memcpy(&request_[offset + 23], &_altitude, sizeof(int));
	memcpy(&request_[offset + 27], &_rainfall, sizeof(int));
	memcpy(&request_[offset + 31], &_vehicle_status, sizeof(int));
	offset += 35;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_AUTOMOBILE_INFO);
	if (status == STATUS_OK)
	{
		//cout << "set_signle_point_mode success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set automobile information fail!" << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_set_clock_config(bool _1588_enable, uint8_t _1588_domain_nu, bool _gptp_enable, uint8_t _gptp_domain_nu)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_CLOCK_CFG;
	request_[offset + 1] = 0x04;
	request_[offset + 2] = 0x00;
	if (_1588_enable)
		request_[offset + 3] = 0x01;
	else
		request_[offset + 3] = 0x00;
	if (_gptp_enable)
		request_[offset + 4] = 0x01;
	else
		request_[offset + 4] = 0x00;
	request_[offset + 5] = _1588_domain_nu;
	request_[offset + 6] = _gptp_domain_nu;
	offset += 7;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_CLOCK_CFG);
	if (status == STATUS_OK)
	{
		//cout << "enable/disable multicast success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Enable/disable clock config fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_set_customer_info(uint16_t _custom_id, char* _manufacture_date, char* _trace_code)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_CUSTOMER_INFO;
	request_[offset + 1] = 0x18;
	request_[offset + 2] = 0x00;
	memcpy(&request_[offset + 3], &_custom_id, sizeof(uint16_t));
	memcpy(&request_[offset + 5], _manufacture_date, 4 * sizeof(char));
	memcpy(&request_[offset + 9], _trace_code, 18 * sizeof(char));
	offset += 27;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_CUSTOMER_INFO);
	if (status == STATUS_OK)
	{
		//cout << "set_signle_point_mode success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set customer info fail!" << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_set_device_ip_port_mac(std::string _ip, uint32_t _mdop_port, uint32_t _dcsp_port, uint32_t _dsop_port,
	std::string _mask, std::string _gateway, bool _use_config_mac, std::string _mac)
{
	int status;
	int offset = 0;
	uint8_t ipInt[4], maskInt[4], gatewayInt[4], macInt[6];

	if (checkIP(_ip) == false)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect IP format!\n";
		return STATUS_FAIL;
	}
	if (checkIP(_mask) == false)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect mask format!\n";
		return STATUS_FAIL;
	}
	if (checkIP(_gateway) == false)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect gateway format!\n";
		return STATUS_FAIL;
	}
	vector<string> substr;
	splitString(_ip, substr, ".");
	if (substr.size() != 4)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect IP fields!\n";
		return STATUS_FAIL;
	}
	for (auto i = 0; i < 4; ++i)
	{
		ipInt[i] = stoi(substr[i]);
	}
	substr.clear();
	splitString(_mask, substr, ".");
	if (substr.size() != 4)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect mask fields!\n";
		return STATUS_FAIL;
	}
	for (auto i = 0; i < 4; ++i)
	{
		maskInt[i] = stoi(substr[i]);
	}
	substr.clear();
	splitString(_gateway, substr, ".");
	if (substr.size() != 4)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect gateway fields!\n";
		return STATUS_FAIL;
	}
	for (auto i = 0; i < 4; ++i)
	{
		gatewayInt[i] = stoi(substr[i]);
	}
	substr.clear();
	splitString(_mac, substr, ":");
	if (substr.size() != 6)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect MAC fields!\n";
		return STATUS_FAIL;
	}
	for (auto i = 0; i < 6; ++i)
	{
		macInt[i] = stoi(substr[i], 0, 16);
	}
	substr.clear();

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_IP_AND_PORT;
	request_[offset + 1] = 0x1f;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = ipInt[0];
	request_[offset + 4] = ipInt[1];
	request_[offset + 5] = ipInt[2];
	request_[offset + 6] = ipInt[3];
	memcpy(&request_[offset + 7], &_mdop_port, sizeof(uint32_t));
	memcpy(&request_[offset + 11], &_dsop_port, sizeof(uint32_t));
	memcpy(&request_[offset + 15], &_dcsp_port, sizeof(uint32_t));
	request_[offset + 19] = maskInt[0];
	request_[offset + 20] = maskInt[1];
	request_[offset + 21] = maskInt[2];
	request_[offset + 22] = maskInt[3];
	request_[offset + 23] = gatewayInt[0];
	request_[offset + 24] = gatewayInt[1];
	request_[offset + 25] = gatewayInt[2];
	request_[offset + 26] = gatewayInt[3];
	request_[offset + 27] = _use_config_mac ? 1 : 0;
	request_[offset + 28] = macInt[0];
	request_[offset + 29] = macInt[1];
	request_[offset + 30] = macInt[2];
	request_[offset + 31] = macInt[3];
	request_[offset + 32] = macInt[4];
	request_[offset + 33] = macInt[5];
	offset += 34;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_IP_AND_PORT);
	if (status == STATUS_OK)
	{
		//cout << "set ip and port success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set ip and port fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_set_dsop_dest_ip_and_port(std::string _ip, uint32_t _port)
{
	int status;
	int offset = 0;
	uint8_t ipInt[4];

	if (checkIP(_ip) == false)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect IP format!\n";
		return STATUS_FAIL;
	}
	vector<string> substr;
	splitString(_ip, substr, ".");
	if (substr.size() != 4)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect IP fields!\n";
		return STATUS_FAIL;
	}
	for (auto i = 0; i < 4; ++i)
	{
		ipInt[i] = stoi(substr[i]);
	}
	substr.clear();

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_DSOP_DEST_IP_AND_PORT;
	request_[offset + 1] = 0x08;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = ipInt[0];
	request_[offset + 4] = ipInt[1];
	request_[offset + 5] = ipInt[2];
	request_[offset + 6] = ipInt[3];
	memcpy(&request_[offset + 7], &_port, sizeof(uint32_t));
	offset += 11;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_DSOP_DEST_IP_AND_PORT);
	if (status == STATUS_OK)
	{

	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set DSOP destination IP and port fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_set_fov_roi_location(uint16_t _horizontal_location, uint16_t _vertical_location)
{
	int status;
	int offset = 0;
	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);
	// payload
	request_[offset + 0] = DCSP_CMD_SET_FOV_ROI_LOCATION;
	request_[offset + 1] = 0x04;
	request_[offset + 2] = 0x00;
	memcpy(&request_[offset + 3], &_horizontal_location, sizeof(uint16_t));
	memcpy(&request_[offset + 5], &_vertical_location, sizeof(uint16_t));
	offset += 7;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_FOV_ROI_LOCATION);
	if (status == STATUS_OK)
	{
		//cout << "set mask success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set FOV ROI location fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_set_fov_roi_size(uint16_t _width, uint16_t _height)
{
	int status;
	int offset = 0;
	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);
	// payload
	request_[offset + 0] = DCSP_CMD_SET_FOV_ROI_SIZE;
	request_[offset + 1] = 0x04;
	request_[offset + 2] = 0x00;
	memcpy(&request_[offset + 3], &_width, sizeof(uint16_t));
	memcpy(&request_[offset + 5], &_height, sizeof(uint16_t));
	offset += 7;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_FOV_ROI_SIZE);
	if (status == STATUS_OK)
	{
		//cout << "set mask success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set FOV ROI size fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_set_ntp_info(std::string _server_ip, uint32_t _interval)
{
	uint8_t ipInt[4];
	if (checkIP(_server_ip) == false)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect IP format!\n";
		return STATUS_FAIL;
	}

	vector<string> substr;
	splitString(_server_ip, substr, ".");
	if (substr.size() != 4)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Incorrect IP fields!\n";
		return STATUS_FAIL;
	}
	for (auto i = 0; i < 4; ++i)
	{
		ipInt[i] = stoi(substr[i]);
	}
	substr.clear();

	int status;
	int offset = 0;
	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);
	// payload
	request_[offset + 0] = DCSP_CMD_SET_NTP_INFO;
	request_[offset + 1] = 0x08;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = ipInt[0];
	request_[offset + 4] = ipInt[1];
	request_[offset + 5] = ipInt[2];
	request_[offset + 6] = ipInt[3];
	memcpy(&request_[offset + 7], &_interval, sizeof(uint32_t));
	offset += 11;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_NTP_INFO);
	if (status == STATUS_OK)
	{
		//cout << "set mask success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set NTP info size fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_set_offline_logger_parameters(uint8_t _log_level, uint8_t _file_size, uint8_t _file_num, 
	bool _enable_authority, bool _enable_uart, bool _enable_udp)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_OFFLINE_LOGGER_PARAM;
	request_[offset + 1] = 0x06;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = _log_level;
	request_[offset + 4] = _file_size;
	request_[offset + 5] = _file_num;
	request_[offset + 6] = _enable_authority ? 0x01 : 0x00;
	request_[offset + 7] = _enable_uart ? 0x01 : 0x00;
	request_[offset + 8] = _enable_udp ? 0x01 : 0x00;
	offset += 9;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_OFFLINE_LOGGER_PARAM);
	if (status == STATUS_OK)
	{
		//cout << "set_signle_point_mode success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Set offline logger parameters fail!" << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_set_rc_mode(bool _enable)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_RC_MODE;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	if (_enable)
	{
		request_[offset + 3] = 0x00;
	}
	else
	{
		request_[offset + 3] = 0x01;
	}
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_RC_MODE);
	if (status == STATUS_OK)
	{
		//cout << "enable/disable multicast success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Enable/disable RC mode fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_set_static_arp(bool _enable)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_STATIC_ARP;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	if (_enable)
	{
		request_[offset + 3] = 0x01;
	}
	else
	{
		request_[offset + 3] = 0x00;
	}
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_STATIC_ARP);
	if (status == STATUS_OK)
	{
		//cout << "enable/disable multicast success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Enable/disable static arp fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_set_sunlight_resistance(bool _enable)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_SUNLIGHT_RESISTANCE;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	if (_enable)
	{
		request_[offset + 3] = 0x01;
	}
	else
	{
		request_[offset + 3] = 0x00;
	}
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_SUNLIGHT_RESISTANCE);
	if (status == STATUS_OK)
	{
		//cout << "enable/disable multicast success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Enable/disable sunlight resistance fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_set_vlan_status(bool _enable)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_VLAN_STATUS;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	if (_enable)
	{
		request_[offset + 3] = 0x01;
	}
	else
	{
		request_[offset + 3] = 0x00;
	}
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_VLAN_STATUS);
	if (status == STATUS_OK)
	{
		//cout << "enable/disable multicast success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Enable/disable vlan fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_set_window_heating(uint8_t _win_enable, uint16_t _voltage,
	int16_t _temp_a, int16_t _temp_b, int16_t _temp_c, int16_t _temp_d,
	uint8_t _tx_enable, int16_t _param_x, int16_t _param_y, int16_t _param_z)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SET_WINDOW_HEATING;
	request_[offset + 1] = 0x12;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = _win_enable;
	std::memcpy(&request_[offset + 4], &_voltage, sizeof(uint16_t));
	std::memcpy(&request_[offset + 6], &_temp_a, sizeof(int16_t));
	std::memcpy(&request_[offset + 8], &_temp_b, sizeof(int16_t));
	std::memcpy(&request_[offset + 10], &_temp_c, sizeof(int16_t));
	std::memcpy(&request_[offset + 12], &_temp_d, sizeof(int16_t));
	request_[offset + 14] = _tx_enable;
	std::memcpy(&request_[offset + 15], &_param_x, sizeof(int16_t));
	std::memcpy(&request_[offset + 17], &_param_y, sizeof(int16_t));
	std::memcpy(&request_[offset + 19], &_param_z, sizeof(int16_t));
	offset += 21;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SET_WINDOW_HEATING);
	if (status == STATUS_OK)
	{
		//cout << "enable/disable multicast success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Enable/disable sunlight resistance fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_shutdown_warning(uint8_t _delay_sec)
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SHUTDOWN_WARNING;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = _delay_sec;
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SHUTDOWN_WARNING);
	if (status == STATUS_OK)
	{
		//cout << "set_signle_point_mode success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Inform device shutdown warning fail!" << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_switch_partition()
{
	int status;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_SWITCH_PARTITION;
	request_[offset + 1] = 0;
	request_[offset + 2] = 0;
	offset += 3;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	status = tx_rx_check_status(offset, DCSP_CMD_SWITCH_PARTITION);
	if (status == STATUS_OK)
	{
		//cout << "restart success!" << endl;
	}
	else
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Switch partition fail! Error code: " << status << endl;
	}

	return status;
}

int benewake::DCSPProtocolP4::dcsp_upload_lut(std::string _table_file, uint8_t _table_id)
{
	table_.clear();

	uint16_t response_status;
	int recv_len = 0;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_UPLOAD_LUT;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = _table_id;
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	uint16_t data_size = 0;
	int pkg_cnt = 0, pkg_cnt_get, lut_data, lut_len;
	bool last_pkg = false;
	while (true)
	{
		recv_len = tx_rx_uncertain_data(offset, 1400, DCSP_CMD_UPLOAD_LUT, response_status);
		if (response_status != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << "Upload LUT package error: code " << (int)response_status << endl;
			return response_status;
		}
		else if (recv_len < (12 + header_count_len_ + 3 + response_status_len_))
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << "Error occured while receiving LUT package!" << endl;
			if (recv_len == STATUS_TIME_OUT)
				return STATUS_TIME_OUT;
			else
				return STATUS_FAIL;
		}

		memcpy(&data_size, &response_[6 + header_count_len_ + 1], sizeof(uint16_t));
		if ((response_[6 + 3 + header_count_len_ + response_status_len_] & 0x02) == 0x02)
			last_pkg = true;
		memcpy(&pkg_cnt_get, &response_[6 + 4 + header_count_len_ + response_status_len_], sizeof(int));
		if (pkg_cnt_get != pkg_cnt)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << "Package lost is detected!" << endl;
			return STATUS_FAIL;
		}
		table_id_ = response_[6 + 8 + header_count_len_ + response_status_len_];
		lut_len = (data_size - 10 - response_status_len_) / 4;
		for (int i = 0; i < lut_len; i++)
		{
			memcpy(&lut_data, &response_[6 + header_count_len_ + response_status_len_ + 13 + 4 * i], sizeof(int));
			table_.push_back(lut_data);
		}

		if (last_pkg)
			break;
		pkg_cnt++;
	}

	std::ofstream outStream(_table_file);
	if (outStream.is_open()) {
		int row = table_[13], col = table_[14], col_cnt;
		outStream << "table_id:" << endl
			<< table_id_ << endl
			<< "gen_time:" << endl
			<< table_[0] << " " << table_[1] << " " << table_[2] << " " << table_[3] << " " << table_[4] << " " << table_[5] << endl
			<< "download_time:" << endl
			<< table_[6] << " " << table_[7] << " " << table_[8] << " " << table_[9] << " " << table_[10] << " " << table_[11] << endl
			<< "version:" << endl
			<< table_[12] << endl
			<< "row_num:" << endl
			<< row << endl
			<< "col_num:" << endl
			<< col << endl
			<< "table:" << endl;
		col_cnt = 0;
		for (std::vector<int>::iterator it = table_.begin() + 15; it != table_.end(); it++)
		{
			outStream << *it;
			col_cnt++;
			if (col_cnt == col)
			{
				outStream << endl;
				col_cnt = 0;
			}
			else
				outStream << " ";
		}
		outStream.close();
	}
	return STATUS_OK;
}

int benewake::DCSPProtocolP4::dcsp_upload_offline_logger(std::string _save_name, uint8_t _file_id)
{
	std::ofstream fw(_save_name, ios::app);
	if (!fw.is_open())
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Error occured when open saving file!" << endl;
		return STATUS_FAIL;
	}

	uint16_t response_status;
	int recv_len = 0;
	int offset = 0;

	// header
	setRequestHeader(request_, request_header_, (6 + header_count_len_) * sizeof(unsigned char));
	offset += (6 + header_count_len_);

	// payload
	request_[offset + 0] = DCSP_CMD_UPLOAD_OFFLINE_LOGGER;
	request_[offset + 1] = 0x01;
	request_[offset + 2] = 0x00;
	request_[offset + 3] = _file_id;
	offset += 4;

	// tail
	uint32_t request_sum = check_sum_with_protocol_version(dcsp_info_->protocol, request_, offset);
	padding_tail(offset, request_sum);
	offset += 6;

	uint16_t data_size = 0;
	int pkg_cnt = 1, pkg_cnt_get, file_data_len;
	bool last_pkg = false;
	char* p_data = (char*)malloc(sizeof(char) * 1024);
	while (true)
	{
		recv_len = tx_rx_uncertain_data(offset, 1400, DCSP_CMD_UPLOAD_OFFLINE_LOGGER, response_status);
		if (response_status != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << "Upload logger package error: code " << (int)response_status << endl;
			free(p_data);
			fw.close();
			return response_status;
		}
		else if (recv_len < (12 + header_count_len_ + 3 + response_status_len_))
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << "Error occured while receiving logger package!" << endl;
			free(p_data);
			fw.close();
			if (recv_len == STATUS_TIME_OUT)
				return STATUS_TIME_OUT;
			else
				return STATUS_FAIL;
		}

		memcpy(&data_size, &response_[6 + header_count_len_ + 1], sizeof(uint16_t));
		if ((response_[6 + 3 + header_count_len_ + response_status_len_] & 0x02) == 0x02)
			last_pkg = true;
		memcpy(&pkg_cnt_get, &response_[6 + 4 + header_count_len_ + response_status_len_], sizeof(int));
		if (pkg_cnt_get != pkg_cnt)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << "Package lost is detected!" << endl;
			free(p_data);
			fw.close();
			return STATUS_FAIL;
		}
		if (_file_id != response_[6 + 8 + header_count_len_ + response_status_len_])
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << "File ID error is detected!" << endl;
			free(p_data);
			fw.close();
			return STATUS_FAIL;
		}
		file_data_len = data_size - 6 - response_status_len_;
		memcpy(p_data, &response_[6 + 9 + header_count_len_ + response_status_len_], file_data_len * sizeof(char));
		for (int i = 0; i < file_data_len; ++i)
			fw << p_data[i];

		if (last_pkg)
			break;
		pkg_cnt++;
	}

	free(p_data);
	fw.close();
	return STATUS_OK;
}

int benewake::DCSPProtocolP4::tx_rx_uncertain_data(int _tx_size, int _rx_size, uint8_t _cmd_response, int _timeout_s, int _timeout_us)
{
	int trytimes = 0;
	int rt, recv_size;
#ifdef DEBUG_INFO
	std::printf("Execute CMD 0x%02x ...\n", _cmd_response);
#endif // DEBUG_INFO
	for (trytimes = 0; trytimes < 3; ++trytimes)
	{
		try
		{
#ifdef DEBUG_INFO
			cout << "    attempts: " << (trytimes + 1) << endl;
#endif // DEBUG_INFO
			rt = pudp_->sendData(request_, _tx_size);
			if (rt == STATUS_FAIL)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "send failed!" << endl;
				return rt;
			}
			else if (rt == STATUS_TIME_OUT)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "send timeout!" << endl;
				continue;
			}
			memset(response_, 0, PROTOCOL_DATA_PACKAGE_MAX_LENGTH);
			rt = pudp_->recvUncertainCMDResponseP4(response_, _rx_size, _cmd_response, dcsp_count_, _timeout_s, _timeout_us);
			if (rt == STATUS_FAIL)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "receive failed!" << endl;
				return rt;
			}
			else if (rt == STATUS_TIME_OUT)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "receive timeout!" << endl;
				continue;
			}
			else if (rt < 21)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "receive unknow data!" << endl;
				continue;
			}
			recv_size = rt;
		}
		catch (const std::exception& e)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << e.what() << endl;
			return STATUS_FAIL;
		}
		if (memcmp(response_, response_header_, 4) == 0)
		{
			uint16_t protocol_version = 0;
			memcpy(&protocol_version, &response_[PROTOCOL_DATA_PACKAGE_VERSION_OFFSET], sizeof(uint16_t));
			uint32_t response_sum = check_sum_with_protocol_version(protocol_version, response_, recv_size - 6);
			if ((response_sum & 0xff) == response_[recv_size - 6] && ((response_sum >> 8) & 0xff) == response_[recv_size - 5] &&
				((response_sum >> 16) & 0xff) == response_[recv_size - 4] && ((response_sum >> 24) & 0xff) == response_[recv_size - 3] &&
				(response_[recv_size - 2] == 0x00) && (response_[recv_size - 1] == 0xff))
			{
				break;
			}
			else
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "Incorrect data package tail: " << endl;
				cout << "check sum = " << response_sum << endl;
				continue;
			}
		}
		else
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << "Incorrect data package header: " << (unsigned int)response_[0] << ", " <<
				(unsigned int)response_[1] << ", " <<
				(unsigned int)response_[2] << ", " <<
				(unsigned int)response_[3] << ", " <<
				(unsigned int)response_[4] << ", " <<
				(unsigned int)response_[5] << endl;
			continue;
		}
	}
	if (trytimes == 3)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Out of times!" << endl;
		return STATUS_TIME_OUT;
	}
	return recv_size;
}

int benewake::DCSPProtocolP4::tx_rx_uncertain_data(int _tx_size, int _rx_size, uint8_t _cmd_response, uint16_t& _response_status, int _timeout_s, int _timeout_us)
{
	int trytimes = 0;
	int rt, recv_size;
#ifdef DEBUG_INFO
	std::printf("Execute CMD 0x%02x ...\n", _cmd_response);
#endif // DEBUG_INFO
	for (trytimes = 0; trytimes < 3; ++trytimes)
	{
		try
		{
#ifdef DEBUG_INFO
			cout << "    attempts: " << (trytimes + 1) << endl;
#endif // DEBUG_INFO
			rt = pudp_->sendData(request_, _tx_size);
			if (rt == STATUS_FAIL)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "send failed!" << endl;
				return rt;
			}
			else if (rt == STATUS_TIME_OUT)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "send timeout!" << endl;
				continue;
			}
			memset(response_, 0, PROTOCOL_DATA_PACKAGE_MAX_LENGTH);
			rt = pudp_->recvUncertainCMDResponseP4(response_, _rx_size, _cmd_response, dcsp_count_, _timeout_s, _timeout_us);
			if (rt == STATUS_FAIL)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "receive failed!" << endl;
				return rt;
			}
			else if (rt == STATUS_TIME_OUT)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "receive timeout!" << endl;
				continue;
			}
			else if (rt < 21)
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "receive unknow data!" << endl;
				continue;
			}
			recv_size = rt;
		}
		catch (const std::exception& e)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << e.what() << endl;
			return STATUS_FAIL;
		}
		if (memcmp(response_, response_header_, 4) == 0)
		{
			uint16_t protocol_version = 0;
			memcpy(&protocol_version, &response_[PROTOCOL_DATA_PACKAGE_VERSION_OFFSET], sizeof(uint16_t));
			uint32_t response_sum = check_sum_with_protocol_version(protocol_version, response_, recv_size - 6);
			if ((response_sum & 0xff) == response_[recv_size - 6] && ((response_sum >> 8) & 0xff) == response_[recv_size - 5] &&
				((response_sum >> 16) & 0xff) == response_[recv_size - 4] && ((response_sum >> 24) & 0xff) == response_[recv_size - 3] &&
				(response_[recv_size - 2] == 0x00) && (response_[recv_size - 1] == 0xff))
			{
				_response_status = *(uint16_t*)&response_[13];
				break;
			}
			else
			{
				std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				cout << "Incorrect data package tail: " << endl;
				cout << "check sum = " << response_sum << endl;
				continue;
			}
		}
		else
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			cout << "Incorrect data package header: " << (unsigned int)response_[0] << ", " <<
				(unsigned int)response_[1] << ", " <<
				(unsigned int)response_[2] << ", " <<
				(unsigned int)response_[3] << ", " <<
				(unsigned int)response_[4] << ", " <<
				(unsigned int)response_[5] << endl;
			continue;
		}
	}
	if (trytimes == 3)
	{
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		cout << "Out of times!" << endl;
		return STATUS_TIME_OUT;
	}
	return recv_size;
}

int benewake::DCSPProtocolP4::tx_rx_data(int _tx_size, int _rx_size, uint8_t _cmd_response, int _timeout_s, int _timeout_us)
{
	uint16_t cmd_status;
	int ret;
	int recv_len = tx_rx_uncertain_data(_tx_size, _rx_size, _cmd_response, cmd_status, _timeout_s, _timeout_us);
	if (recv_len > 0 && recv_len != _rx_size && cmd_status == 0x0000)
	{
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Command status is OK but response data length is not equal to the length expected."
			<< _rx_size << " bytes required but " << recv_len << " bytes received." << std::endl;
#endif // DEBUG_INFO
		ret = STATUS_FAIL;
	}
	else if (recv_len == _rx_size || recv_len == 21)
	{
		ret = cmd_status;
	}
	else if (recv_len >= 0)
	{
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Received data is not equal to request. "
			<< _rx_size << " bytes required but " << recv_len << " bytes received." << std::endl;
#endif // DEBUG_INFO
		ret = STATUS_FAIL;
	}
	else // only STATUS_FAIL or STATUS_TIME_OUT
	{
		ret = recv_len;
	}
	return ret;
}

int benewake::DCSPProtocolP4::tx_rx_check_status(int _tx_size, uint8_t _cmd_response, int _timeout_s, int _timeout_us)
{
	uint16_t cmd_status;
	int ret;
	int recv_len = tx_rx_uncertain_data(_tx_size, 21, _cmd_response, cmd_status, _timeout_s, _timeout_us);
	if (recv_len == 21)
	{
		ret = cmd_status;
	}
	else if (recv_len >= 0)
	{
#ifdef DEBUG_INFO
		std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cout << "Received data is not equal to request. 21 bytes required but " << recv_len << " bytes received." << std::endl;
#endif // DEBUG_INFO
		ret = STATUS_FAIL;
	}
	else // only STATUS_FAIL or STATUS_TIME_OUT
	{
		ret = recv_len;
	}
	return ret;
}

void benewake::DCSPProtocolP4::setRequestHeader(unsigned char* _request, unsigned char* _request_header, int len)
{
	_request_header[4] = dcsp_info_->protocol & 0xff;
	_request_header[5] = (dcsp_info_->protocol >> 8) & 0xff;
	dcsp_count_++;
	uint32_t count = dcsp_count_;
	_request_header[6] = count & 0xff;
	_request_header[7] = (count >> 8) & 0xff;
	_request_header[8] = (count >> 16) & 0xff;
	_request_header[9] = (count >> 24) & 0xff;
	std::memcpy(_request, _request_header, len);
}

benewake::DCSPProtocolG66::DCSPProtocolG66(std::string _send_ip, int _send_port, std::string _recv_ip, int _recv_port, UDPType _udp_type) :
	DCSPProtocolP4(_send_ip, _send_port, _recv_ip, _recv_port, _udp_type)
{
	request_header_[0] = 'B';
	request_header_[1] = 'W';
	request_header_[2] = PRODUCT_ID_G66;
	request_header_[3] = PROTOCOL_ID_DCSP_REQUEST;

	response_header_[0] = 'B';
	response_header_[1] = 'W';
	response_header_[2] = PRODUCT_ID_G66;
	response_header_[3] = PROTOCOL_ID_DCSP_RESPONSE;

	dcsp_count_ = 0xffffffff;
	header_count_len_ = 4;
	response_status_len_ = 2;
}

