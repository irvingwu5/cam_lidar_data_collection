#include "benewake_hornx2_driver.h"

namespace benewake
{
	BenewakeHornX2::BenewakeHornX2(std::string _ip, uint32_t _port, UDPType _type, std::string _local_ip) : 
		BenewakeLidar(_ip, _port, _type, _local_ip)
	{
		// Actual functions are in BenewakeLidar
	}

	BenewakeHornX2::BenewakeHornX2(benewake::SYS_INFO _info, std::string _local_ip) : 
		BenewakeLidar(_info, _local_ip)
	{
	}

	BenewakeHornX2::~BenewakeHornX2()
	{
	}
}
