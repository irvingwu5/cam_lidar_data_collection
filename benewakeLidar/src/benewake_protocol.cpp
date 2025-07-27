#include "benewake_protocol.h"

benewake::Protocol::Protocol(std::string _local_ip, int _local_port, std::string _remote_ip, int _remote_port, UDPType _udp_type) :
	local_ip_(_local_ip), local_port_(_local_port), remote_ip_(_remote_ip), remote_port_(_remote_port), connection_type_(_udp_type)
{
	pudp_ = std::unique_ptr<Udp>(new Udp(local_ip_, local_port_, remote_ip_, remote_port_));
}

benewake::Protocol::~Protocol()
{
	pudp_->disconnect();
	pudp_.reset();
}

int benewake::Protocol::close()
{
	int status = pudp_->disconnect();
	return status;
}

int benewake::Protocol::open()
{
	int status = pudp_->connect(connection_type_);
	return status;
}

int benewake::Protocol::open(UDPType _udp_type)
{
	pudp_->disconnect();
	connection_type_ = _udp_type;
	int status = pudp_->connect(connection_type_);
	return status;
}

int benewake::Protocol::reset_connection(std::string _local_ip, int _local_port, std::string _remote_ip, int _remote_port, UDPType _udp_type)
{
	int status;
	local_ip_ = _local_ip;
	local_port_ = _local_port;
	remote_ip_ = _remote_ip;
	remote_port_ = _remote_port;
	connection_type_ = _udp_type;
	if (pudp_ != nullptr)
	{
		status = pudp_->disconnect();
		if (status != STATUS_OK)
		{
			std::cout << std::endl << "Error: " << __FILE__ << " : " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cout << "Close previous connection filed!" << std::endl;
			return status;
		}
		pudp_.reset();
	}
	pudp_ = std::unique_ptr<Udp>(new Udp(local_ip_, local_port_, remote_ip_, remote_port_));
	return STATUS_OK;
}

uint32_t benewake::Protocol::check_sum_with_protocol_version(uint16_t _version, uint8_t* _buffer, size_t _size)
{
	switch (_version)
	{
	case PROTOCOL_CHECKSUM:
		return checksum(_buffer, _size);
		break;

	case PROTOCOL_VERSION_X:
		return check_crc32(_buffer, _size);
		break;

	case PROTOCOL_VERSION_AD2_B:
		return check_crc32_sb8_mode_begin(_buffer, _size);
		break;

	case PROTOCOL_VERSION_AD2_C:
		return check_crc32_sb8_mode_begin(_buffer, _size);
		break;

	case PROTOCOL_VERSION_AD2_HH:
		return check_crc32_sb8_mode_begin(_buffer, _size);
		break;

	case PROTOCOL_VERSION_G66:
		return check_crc32_sb8_mode_begin(_buffer, _size);
		break;

	default:
		break;
	}
	return 0;
}

uint32_t benewake::Protocol::checksum(const uint8_t * _buffer, size_t _size)
{
	uint32_t sum = 0;
	for (auto i = 0; i < _size; ++i)
	{
		sum += _buffer[i];
	}
	return sum;
}

uint32_t benewake::Protocol::check_crc32(const uint8_t * _buffer, size_t _size)
{
	uint32_t crc;
	uint32_t i;

	i = 0;
	crc = 0;

	while (i < _size)
	{
		crc = (crc << 8) ^ const_crc_table[(crc >> 24) ^ _buffer[i]];
		i++;
	}
	return crc;
}

uint32_t benewake::Protocol::check_crc32_sb8_mode_begin(const uint8_t* _buffer, size_t _size)
{
	uint32_t li;
	uint32_t crc, term1, term2;
	uint32_t running_length;
	uint32_t end_bytes;
	uint32_t init_bytes = (uint32_t)_size % 4;
	crc = 0xFFFFFFFF;
	running_length = ((_size - init_bytes) / 8) * 8;
	end_bytes = _size - init_bytes - running_length;

	for (li = 0; li < init_bytes; li++)
		crc = crc_tableil8_o32[(crc ^ *_buffer++) & 0x000000FF] ^ (crc >> 8);
	for (li = 0; li < running_length / 8; li++)
	{
		crc ^= *(uint32_t*)_buffer;
		_buffer += 4;
		term1 = crc_tableil8_o88[crc & 0x000000FF] ^
			crc_tableil8_o80[(crc >> 8) & 0x000000FF];
		term2 = crc >> 16;
		crc = term1 ^
			crc_tableil8_o72[term2 & 0x000000FF] ^
			crc_tableil8_o64[(term2 >> 8) & 0x000000FF];
		term1 = crc_tableil8_o56[(*(uint32_t*)_buffer) & 0x000000FF] ^
			crc_tableil8_o48[((*(uint32_t*)_buffer) >> 8) & 0x000000FF];

		term2 = (*(uint32_t*)_buffer) >> 16;
		crc = crc ^
			term1 ^
			crc_tableil8_o40[term2 & 0x000000FF] ^
			crc_tableil8_o32[(term2 >> 8) & 0x000000FF];
		_buffer += 4;
	}
	for (li = 0; li < end_bytes; li++)
		crc = crc_tableil8_o32[(crc ^ *_buffer++) & 0x000000FF] ^ (crc >> 8);
	return crc;
}

bool benewake::mkDir(const std::string& strPath)
{
	int i = 0;
	int nDirLen = strPath.length();
	if (nDirLen <= 0)
		return false;
	char* pDirTemp = new char[nDirLen + 4];
	strPath.copy(pDirTemp, nDirLen + 1, 0);// +1 to copy '\0'
	pDirTemp[nDirLen] = '\0';
	// add '/' to tail
	if (pDirTemp[nDirLen - 1] != '\\' && pDirTemp[nDirLen - 1] != '/')
	{
		pDirTemp[nDirLen] = '/';
		pDirTemp[nDirLen + 1] = '\0';
		nDirLen++;
	}
	// creat directory
	for (i = 0; i < nDirLen; i++)
	{
		if (pDirTemp[i] == '\\' || pDirTemp[i] == '/')
		{
			pDirTemp[i] = '\0'; // check existence of every subdirectory, if not then create
			int statu;
			statu = ACCESS(pDirTemp, 0);
			if (statu != 0) // the same name file can cause creation failure
			{
				statu = MKDIR(pDirTemp);
				if (statu != 0) // the parent may be file rather than folder and will lead to failure
				{
					return false;
				}
			}
			// for linux, replace \ to / 
			pDirTemp[i] = '/';
		}
	}
	delete[] pDirTemp;
	return true;
}

bool benewake::isLeapYear(uint16_t _year)
{
	bool res = false;

	if (_year % 4 == 0)
	{
		if ((_year % 100 == 0) && (_year % 400 != 0))	
		{
			res = false;
		}
		else
		{
			res = true;
		}
	}
	return res;
}

uint32_t benewake::covUTC2UnixTimestamp(utc_time_t* _UTCTime)
{
	uint32_t daynum = 0, secNum = 0; 
	uint16_t tempYear = 1970, tempMonth = 0;

	while (tempYear < _UTCTime->year)
	{
		if (isLeapYear(tempYear)) 
		{
			daynum += 366;
		}
		else
		{
			daynum += 365;
		}
		tempYear++;
	}

	while (tempMonth < _UTCTime->month - 1)
	{
		if (isLeapYear(_UTCTime->year)) 
		{ 
			daynum += leapMonthDay[tempMonth];
		}
		else 
		{
			daynum += monthDay[tempMonth];
		}
		tempMonth++;
	}

	daynum += (_UTCTime->day - 1);

	secNum = daynum * 24 * 60 * 60;    
	secNum += _UTCTime->hour * 60 * 60;
	secNum += _UTCTime->minute * 60;
	secNum += _UTCTime->second;

	return secNum;
}