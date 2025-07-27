/**
* @file       benewake_protocol_comm.h

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
#ifndef INCLUDE_BENEWAKE_PROTOCOL_COMM__
#define INCLUDE_BENEWAKE_PROTOCOL_COMM__

#define BENEWAKE_LIDAR_DRIVER_VERSION_BRANCH "STD"
#define BENEWAKE_LIDAR_DRIVER_VERSION_MAJOR 10
#define BENEWAKE_LIDAR_DRIVER_VERSION_MINOR 0
#define BENEWAKE_LIDAR_DRIVER_VERSION_BUILD 2

#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include <queue>
#include <mutex>
#include <string>

#ifdef _WIN32
#ifdef BENEWAKE_DLL_EXPORT
#define BENEWAKE_API __declspec(dllexport)
#else
#define BENEWAKE_API __declspec(dllimport)
#endif // BENEWAKE_DLL_EXPORT
#else
#include <semaphore.h>
#define BENEWAKE_API
#endif // WIN32

#ifdef _WIN32
#define ACCESS _access
#define MKDIR(a) _mkdir((a))
#else
#define ACCESS access
#define MKDIR(a) mkdir((a), 0755)
#endif // WIN32

class BwPointCloudRGB;
class BwPointCloudGray;

namespace benewake
{
static float kPI = 3.141592;

#define DEBUG_INFO

#define PRODUCT_ID_X2                 0x00
#define PRODUCT_ID_P4                 0x01
#define PRODUCT_ID_G66                0x02

#define PROTOCOL_CHECKSUM             0x00
#define PROTOCOL_VERSION_X            0x01
#define PROTOCOL_VERSION_AD2_B        0x02
#define PROTOCOL_VERSION_AD2_C        0x03
#define PROTOCOL_VERSION_AD2_HH       0x04
#define PROTOCOL_VERSION_AD2_DD       0x05
#define PROTOCOL_VERSION_G66	      0x06

#define PROTOCOL_ID_MDOP		      0x00
#define PROTOCOL_ID_DSOP		      0x01
#define PROTOCOL_ID_DCSP_REQUEST      0x02
#define PROTOCOL_ID_DCSP_RESPONSE     0x03

#define STATUS_OK                     0
#define STATUS_FAIL                   -1
#define STATUS_TIME_OUT				  -2
#define STATUS_ANOMALOUS              -3

#define BW_OK							0
#define BW_COMM_ERROR					-1
#define BW_SYS_ERROR					-2
#define BW_SYS_WARNING					-3
#define BW_SYS_BUSY						-4

/********************************* MDOP data offset *********************************/
constexpr auto PROTOCOL_DATA_PACKAGE_VERSION_OFFSET		= 4;
constexpr auto PROTOCOL_DATA_PACKAGE_COUNT_OFFSET		= 6;
constexpr auto PROTOCOL_DATA_FRAME_COUNT_OFFSET			= 10;
constexpr auto PROTOCOL_DATA_LINE_NUM_OFFSET			= 12;
constexpr auto PROTOCOL_DATA_POINT_COUNT_OFFSET			= 14;

// for X
constexpr auto PROTOCOL_X_DATA_TIME_S_OFFSET			= 16;
constexpr auto PROTOCOL_X_DATA_TIME_NS_OFFSET			= 20;
constexpr auto PROTOCOL_X_DATA_SIGNAL_MESSAGE			= 24;
constexpr auto PROTOCOL_X_DATA_SIGNAL_TIME_OFFSET		= 28;

// for AD2_B
constexpr auto PROTOCOL_AD2_B_DATA_TIME_S_OFFSET		= 16;
constexpr auto PROTOCOL_AD2_B_DATA_TIME_NS_OFFSET		= 20;
constexpr auto PROTOCOL_AD2_B_DATA_ECHO_MODE			= 24;
constexpr auto PROTOCOL_AD2_B_DATA_ROI_CENTER_X			= 25;
constexpr auto PROTOCOL_AD2_B_DATA_ROI_CENTER_Y			= 27;
constexpr auto PROTOCOL_AD2_B_DATA_HEADER_LENGHT		= 38;

// for AD2_C and AD2_HH
constexpr auto PROTOCOL_AD2_DATA_TIME_S_OFFSET			= 16;
constexpr auto PROTOCOL_AD2_DATA_TIME_NS_OFFSET			= 24;
constexpr auto PROTOCOL_AD2_DATA_ECHO_MODE				= 28;
constexpr auto PROTOCOL_AD2_DATA_ROI_CENTER_X			= 29;
constexpr auto PROTOCOL_AD2_DATA_ROI_CENTER_Y			= 31;
constexpr auto PROTOCOL_AD2_DATA_ROI_WIDTH				= 33;
constexpr auto PROTOCOL_AD2_DATA_ROI_HEIGHT				= 35;
constexpr auto PROTOCOL_AD2_DATA_HEADER_LENGHT			= 42;

// for G66
constexpr auto PROTOCOL_G66_DATA_TIME_S_OFFSET			= 16;
constexpr auto PROTOCOL_G66_DATA_TIME_NS_OFFSET			= 24;
constexpr auto PROTOCOL_G66_DATA_ECHO_MODE				= 28;
constexpr auto PROTOCOL_G66_DATA_ROI_CENTER_X			= 29;
constexpr auto PROTOCOL_G66_DATA_ROI_CENTER_Y			= 31;
constexpr auto PROTOCOL_G66_DATA_ROI_WIDTH				= 33;
constexpr auto PROTOCOL_G66_DATA_ROI_HEIGHT				= 35;
constexpr auto PROTOCOL_G66_DATA_BLOCK_SIZE				= 37;
constexpr auto PROTOCOL_G66_DATA_HEADER_LENGHT			= 42;
/*************************************** END ****************************************/

/******************************** Fixed data length ********************************/
constexpr auto PROTOCOL_DATA_PACKAGE_MAX_LENGTH		= 1500;
constexpr auto DCSP_DATA_FIRMWARE_DEFAULT_LENGTH	= 1024;
constexpr auto MDOP_DATA_MAX_PKG_NUM				= 50000;
constexpr auto MDOP_DATA_RING_BUFFER_MAX_LENGTH		= 1000;
/*************************************** END ****************************************/

/********************************** DCSP cmd mark ***********************************/
constexpr auto DCSP_CMD_GET_DEVICE_INFORMATION		= 0x00;
constexpr auto DCSP_CMD_GET_MASK_INFORMATION		= 0x01;
constexpr auto DCSP_CMD_START_STOP					= 0x02;
constexpr auto DCSP_CMD_GET_COORDINATE_SYSTEM		= 0x03;
constexpr auto DCSP_CMD_SET_COORDINATE_SYSTEM		= 0x04;
constexpr auto DCSP_CMD_GET_EXTRINSIC_PARAMETERS	= 0x05;
constexpr auto DCSP_CMD_SET_EXTRINSIC_PARAMETERS	= 0x06;
constexpr auto DCSP_CMD_SET_IP_AND_PORT				= 0x07;
constexpr auto DCSP_CMD_SET_ROI						= 0x08;
constexpr auto DCSP_CMD_SET_TIME_SYNC				= 0x09;
constexpr auto DCSP_CMD_GET_TIMESTAMP_FORMAT		= 0x0A;
constexpr auto DCSP_CMD_SET_TIMESTAMP_FORMAT		= 0x0B;
constexpr auto DCSP_CMD_GET_LIDAR_MODE				= 0x0C;
constexpr auto DCSP_CMD_SET_LIDAR_MODE				= 0x0D;
constexpr auto DCSP_CMD_GET_LIDAR_POWER				= 0x0E;
constexpr auto DCSP_CMD_SET_LIDAR_POWER				= 0x0F;
constexpr auto DCSP_CMD_SET_DHCP					= 0x10;
constexpr auto DCSP_CMD_GET_DHCP					= 0x11;
constexpr auto DCSP_CMD_SET_MULITCAST_STATUS		= 0x12;
constexpr auto DCSP_CMD_GET_MULITCAST_STATUS		= 0x13;
constexpr auto DCSP_CMD_SET_MULITCAST_IP_PORT		= 0x14;
constexpr auto DCSP_CMD_GET_MULITCAST_IP_PORT		= 0x15;
constexpr auto DCSP_CMD_SET_AUTORUN_STATUS			= 0x16;
constexpr auto DCSP_CMD_GET_AUTORUN_STATUS			= 0x17;
constexpr auto DCSP_CMD_GET_MDOP_PORT				= 0x18;
constexpr auto DCSP_CMD_SET_NETWORK_TIMEOUT			= 0x19;
constexpr auto DCSP_CMD_GET_NETWORK_TIMEOUT			= 0x1A;
constexpr auto DCSP_CMD_SET_DEST_IP_AND_PORT		= 0x1B;
constexpr auto DCSP_CMD_SET_DSOP_DEST_IP_AND_PORT	= 0x1D;
constexpr auto DCSP_CMD_GET_DSOP_DEST_IP_AND_PORT	= 0x1E;
constexpr auto DCSP_CMD_DOWNLOAD_FIRMWARE			= 0x20;
constexpr auto DCSP_CMD_DOWNLOAD_FPGA				= 0x21;
constexpr auto DCSP_CMD_DOWNLOAD_LUT				= 0x22;
constexpr auto DCSP_CMD_UPLOAD_LUT					= 0x23;
constexpr auto DCSP_CMD_GET_RC_MODE					= 0x24;
constexpr auto DCSP_CMD_SET_RC_MODE					= 0x25;
constexpr auto DCSP_CMD_GET_CLOCK_CFG				= 0x26;
constexpr auto DCSP_CMD_SET_CLOCK_CFG				= 0x27;
constexpr auto DCSP_CMD_GET_LASER_SN                = 0x60;
constexpr auto DCSP_CMD_STORE						= 0x70;
constexpr auto DCSP_CMD_RESTORE						= 0x71;
constexpr auto DCSP_CMD_SHUTDOWN					= 0x72;
constexpr auto DCSP_CMD_RESTART						= 0x73;
constexpr auto DCSP_CMD_CLEAR_CONFIG				= 0x74;
constexpr auto DCSP_CMD_SHUTDOWN_WARNING			= 0x75;
constexpr auto DCSP_CMD_SET_MASK_INFORMATION		= 0x80;
constexpr auto DCSP_CMD_SET_SN						= 0x81;
constexpr auto DCSP_CMD_READ_REGISTER				= 0x82;
constexpr auto DCSP_CMD_WRITE_REGISTER				= 0x83;
constexpr auto DCSP_CMD_GET_PTP_INFO				= 0x84;
constexpr auto DCSP_CMD_SET_NTP_INFO				= 0x85;
constexpr auto DCSP_CMD_GET_NTP_INFO				= 0x86;
constexpr auto DCSP_CMD_SET_CUSTOMER_MODE			= 0x90;
constexpr auto DCSP_CMD_SET_TCP_SERVER_IP_PORT		= 0x91;
constexpr auto DCSP_CMD_DOWNLOAD_TEMP_COMP_TABLE    = 0x92;
constexpr auto DCSP_CMD_GET_TEMP_COMP_TABLE         = 0x93;
constexpr auto DCSP_CMD_SET_GAZE					= 0xA1;
constexpr auto DCSP_CMD_GET_GAZE					= 0xA2;
constexpr auto DCSP_CMD_SET_OFFLINE_LOGGER_PARAM	= 0xA3;
constexpr auto DCSP_CMD_GET_OFFLINE_LOGGER_PARAM	= 0xA4;
constexpr auto DCSP_CMD_UPLOAD_OFFLINE_LOGGER		= 0xA5;
constexpr auto DCSP_CMD_GET_OFFLINE_LOGGER_INFO		= 0xA6;
constexpr auto DCSP_CMD_SET_AUTOMOBILE_INFO			= 0xA7;
constexpr auto DCSP_CMD_SET_SIGNAL_MESSAGE			= 0xA8;
constexpr auto DCSP_CMD_SET_FOV_ROI_LOCATION		= 0xAA;
constexpr auto DCSP_CMD_SET_FOV_ROI_SIZE			= 0xAB;
constexpr auto DCSP_CMD_SET_ANTI_INTERFERENCE		= 0xB0;
constexpr auto DCSP_CMD_SET_SUNLIGHT_RESISTANCE		= 0xB1;
constexpr auto DCSP_CMD_SET_WINDOW_HEATING			= 0xB2;
constexpr auto DCSP_CMD_SET_STATIC_ARP				= 0xB3;
constexpr auto DCSP_CMD_SET_CUSTOMER_INFO			= 0xB4;
constexpr auto DCSP_CMD_GET_MAC_AND_VLAN			= 0xB5;
constexpr auto DCSP_CMD_FORMAT_FILE_SYSTEM			= 0xB6;
constexpr auto DCSP_CMD_SWITCH_PARTITION			= 0xB7;
constexpr auto DCSP_CMD_GET_APP_PARTITION			= 0xB8;
constexpr auto DCSP_CMD_SET_VLAN_STATUS				= 0xB9;
constexpr auto DCSP_CMD_GET_FLASH_INFO				= 0xBA;
constexpr auto DCSP_CMD_GET_EEPROM_STATUS			= 0xBB;
constexpr auto DCSP_CMD_GET_DDR_INFO				= 0xBC;
constexpr auto DCSP_CMD_CLEAR_WORK_TIME				= 0xBD;
constexpr auto DCSP_CMD_GET_DSP_VERSION				= 0xE0;
constexpr auto DCSP_CMD_SET_FALLBACK_STATUS			= 0xE1;
constexpr auto DCSP_CMD_GET_FALLBACK_STATUS			= 0xE2;
constexpr auto DCSP_CMD_GET_ALGORITHM_VERSION		= 0xE3;
constexpr auto DCSP_CMD_GET_PL_REGISTER				= 0xF0;
constexpr auto DCSP_CMD_SET_PL_REGISTER				= 0xF1;
/*************************************** END ****************************************/

#define BIT(x)           ((uint32_t)((uint32_t)0x01U<<(x)))
#define BITS(start, end) ((0xFFFFFFFFUL << (start)) & (0xFFFFFFFFUL >> (31U - (uint32_t)(end))))

// LiDAR system error code
enum class SYS_ERROR
{
	ERR_OK = 0x00,

	ERR_FPGA = BIT(0),
	ERR_ARM = BIT(1),
	ERR_CTL_PARA = BIT(2),
	ERR_LUT = BIT(3),
	ERR_APD_IIC_TRANSMIT = BIT(4),
	ERR_APD_IIC_INIT = BIT(5),
	ERR_APD_IIC_OTHER = BIT(6),
	ERR_APD_VOL = BIT(7),

	ERR_TDC_SPI_TRANSMIT = BIT(8),
	ERR_TDC_REG = BIT(9),

	ERR_LASER_UART_TRANSMIT = BIT(12),
	ERR_LASER_UART_DECODE = BIT(13),
	ERR_LASER_SN = BIT(14),

	ERR_PRISM_LOCK = BIT(16),

	ERR_CDMA_IRQ = BIT(24),
	ERR_CDMA_IRQ_QUEUE = BIT(25),
	ERR_CDMA_THREAD_SAMPLE_QUEUE = BIT(26),
	ERR_CDMA_NETWORK = BIT(27),
};

// LiDAR system status code
enum class SYS_STATUS
{
	STAT_INIT = 0X00,			// system is initializing
	STAT_STANDBY = 0X01,		// system is standby
	STAT_RUNNING = 0X02,		// system is measuring
	STAT_WARNING = 0X03,		// system has warn(s)
	STAT_ERROR = 0X04,			// system has error(s)
	STAT_BOOTLOADER = 0X05,		// system is excuting bootloader
	STAT_UNKNOWN = 0X06,		// system status is not recognized
	STAT_STARTUP = 0X07		// system startup
};

// Communication status between LiDAR and SDK
enum class COMM_STATUS
{
	COMM_OK = 0,				// communication ok
	COMM_FAIL = 1,				// has socket error
	COMM_TIMEOUT = 2			// communication timeout
};

// Flags show whether signals is effective. True means effective; otherwise ineffective
struct SIGNAL_STATUS
{
	bool stat_PPS = false;		// PPS signal flag
	bool stat_GPRMC = false;	// GPRMC signal flag
	bool stat_NTP = false;		// NTP signal flag
	bool stat_1588 = false;		// 1588 signal flag
	bool stat_IMU = false;		// IMU signal flag
};

struct DTC_INFO
{
	char time[7] = { 0 };		// time of DTC in hexadecimal format YYYY/MM/DD/HH/MM/SS
	uint8_t DTC_status = 0;		// DTC status
	uint32_t DTC_code = 0;		// DTC code
};

struct DID_INFO
{
	uint16_t id = 0;					// identification
	std::string version;				// version
};

struct FUNCTIONAL_SAFETY
{
	uint8_t FS_version = 0;				// functional safety version
	uint8_t FS_state = 0;				// functional safety status: 
										//		bit0 - func disable if 1; 
										//		bit1 - func degrade if 1; 
										//		bit2 - inside error if 1;
	uint8_t fault_code_type = 0;		// fault code type
	uint8_t fault_index = 0;			// index of current fault in buffer
	uint16_t total_fault_code_num = 0;	// fault code amount in buffer
	uint64_t fault_code = 0;			// fault code
};

// System heart beat information
struct SYS_INFO
{
	std::string device_sn;									// LiDAR SN code
	std::string ip;											// LiDAR IP
	uint8_t device_type = PRODUCT_ID_X2;					// Device type
	uint8_t msg_type = 0x00;								// Current SYS_INFO type: 0x00 - common; 0x01 - functional safety
	uint16_t dcsp_port = 0;									// LiDAR contorl command port
	uint32_t timestamp_s = 0;								// This information's timestamp, second part
	uint32_t timestamp_ns = 0;								// This information's timestamp, nanosecond part
	uint32_t count = 0;										// Heart beat count
	uint16_t protocol;										// Protocol version
	COMM_STATUS comm_stat = COMM_STATUS::COMM_TIMEOUT;		// Communication status between LiDAR and SDK
	SYS_ERROR sys_err = SYS_ERROR::ERR_OK;					// LiDAR system error code
	SYS_STATUS sys_stat = SYS_STATUS::STAT_INIT;			// LiDAR system status
	SIGNAL_STATUS sig_stat;									// LiDAR signals flag
	FUNCTIONAL_SAFETY func_safety;							// Functional safety information
	bool multicast = false;									// LiDAR upload data through multicast or not
	uint16_t fan_pwn;										// LiDAR fan pwm
	uint16_t lidar_work_mode;								// LiDAR work mode
	float temperature;										// Temperature in degree Celsius, for X is laser temperature, for HH is temperature inseide
	int16_t voltage = 0;									// Voltage in mV
	int16_t current = 0;									// Current in mA
	int16_t humidity = 0;									// Humidity
	uint32_t duration = 0;									// Working duration
	std::vector<uint8_t> ETH_stat;							// Eth status
	std::vector<DTC_INFO> DTC_msgs;							// DTC messages
	std::vector<DID_INFO> DID_msgs;							// DID messages
};

struct BwPoint
{
	float x = 0;
	float y = 0;
	float z = 0;
	/*
	* Timestamp:
	* Timestamp contains 2 parts which stand for timestamp's seconds part and nanoseconds part. Specially, when set timestamp output
	* format as GPS&PPS, BwTimestamp's element \a time_s stand for time part greater than 1 hour in form of \b YYmmDDhh (for example,
	* value 20091810 stand for 2020-09-18's 10:00), and element \a time_ns stand for time part under 1 hour and unit is microsecond.
	*/
	uint32_t timestamp_ns = 0;	// nanoseconds part
	uint64_t timestamp_s = 0;	// seconds part
	uint16_t row = 0;
	uint8_t intensity = 0;
	uint8_t roi = 0;
	uint8_t channel = 0;
	uint8_t echo = 0;			// echo mark: bit0 - first echo; bit1 - strongest echo; bit2 - last echo
	uint8_t confidence = 0;
};

struct BwRGB
{
	uint8_t r = 0;
	uint8_t g = 0;
	uint8_t b = 0;
};

struct BwPulse
{
	uint32_t timestamp_s = 0; ///< seconds part
	uint32_t timestamp_ns = 0; ///< nanoseconds part
	uint16_t ch1_l_r[4];
	uint16_t ch1_l_f[4];
	uint16_t ch1_h_r[4];
	uint16_t ch1_h_f[4];
	uint16_t ch2_l_r[4];
	uint16_t ch2_l_f[4];
	uint16_t ch2_h_r[4];
	uint16_t ch2_h_f[4];
	uint16_t x;
	uint16_t y;
};

struct BwSphereCoord
{
	uint16_t x = 0;
	uint16_t y = 0;
	uint16_t line = 0;
	uint32_t timestamp_s = 0; ///< seconds part
	uint32_t timestamp_ns = 0; ///< nanoseconds part
	uint16_t dist[4] = { 0 }; // ch1_lg, ch1_hg, ch2_lg, ch2_hg
	uint16_t pulse_width[4] = { 0 }; // ch1_lg, ch1_hg, ch2_lg, ch2_hg
};

struct BwSphereCoordAll
{
	uint16_t x = 0;
	uint16_t y = 0;
	uint16_t line = 0;
	uint32_t timestamp_s = 0; ///< seconds part
	uint32_t timestamp_ns = 0; ///< nanoseconds part
	uint16_t ch1_l_dist[3] = { 0 }; 
	uint16_t ch1_l_width[3] = { 0 };
	uint16_t ch1_h_dist[3] = { 0 };
	uint16_t ch1_h_width[3] = { 0 };
	uint16_t ch2_l_dist[3] = { 0 };
	uint16_t ch2_l_width[3] = { 0 };
	uint16_t ch2_h_dist[3] = { 0 };
	uint16_t ch2_h_width[3] = { 0 };
};

typedef union
{
	struct
	{
		uint32_t prism_code;
		uint32_t galvo_code;
		uint16_t pulse_width;  //laser pulse width
		uint16_t dac_data;
		uint16_t ch_r_0[16];
		uint16_t ch_f_0[16];
		uint16_t ch_r_1[16];
		uint16_t ch_f_1[16];
		uint16_t ch_r_2[16];
		uint16_t ch_f_2[16];
		uint16_t ch_r_3[16];
		uint16_t ch_f_3[16];
		uint16_t ch_r_cnt[16];
		uint16_t ch_f_cnt[16];
	}bwtestdatap4_b;
	struct
	{
		uint16_t luminous_period;
		uint32_t prism_code;
		uint32_t galvo_code;
		uint16_t st_ch_r[256];
		uint16_t st_ch_f[256];
		uint16_t st_ch_r_cnt[64];
		uint16_t st_ch_f_cnt[64];
		uint16_t main_ch_r[4];
		uint16_t main_ch_f[4];
		uint16_t main_ch_r_cnt;
		uint16_t main_ch_f_cnt;
		uint16_t dac_out[16];
		uint16_t dac_stage_2;
		uint16_t dac_stage_3;
		uint16_t dac_stage_4;
	}bwtestdatap4_c;
}BwTestDataP4;

typedef union
{
	struct
	{
		uint16_t h_azimuth;
		uint16_t v_azimuth;
		uint32_t time_offset;
		uint32_t timestamp_s;
		uint32_t timestamp_ns;
		uint32_t dist[32];
		uint8_t intensity[32];
	}spherecoordp4_b;
	struct
	{
		uint16_t h_azimuth;
		uint16_t v_azimuth;
		uint32_t time_offset;
		uint32_t timestamp_s;
		uint32_t timestamp_ns;
		uint16_t dist[32];
		uint8_t intensity[32];
		uint8_t flag[32];
	}spherecoordp4_c;
}BwSphereCoordP4;

struct BwSphereCoordG66
{
	uint16_t h_azimuth;
	uint16_t v_azimuth;
	uint32_t time_offset;
	uint32_t timestamp_s;
	uint32_t timestamp_ns;
	uint16_t dist[16] = { 0 };
	uint8_t intensity[16] = { 0 };
	uint8_t flag[16] = { 0 };
};

struct BwTestDataG66
{
	uint16_t st_ch_r[96];
	uint16_t st_ch_f[96];
	uint8_t st_ch_r_cnt[24];
	uint8_t st_ch_f_cnt[24];
	uint16_t main_ch_r;
	uint16_t main_ch_f;
	uint8_t main_ch_r_cnt;
	uint8_t main_ch_f_cnt;
	uint16_t luminous_period;
	uint32_t prism_code;
	uint32_t galvo_code;
	uint8_t dac_out[8];
	uint8_t dac_stage_2;
	uint8_t dac_stage_3;
};

enum class BwProtocolType
{
	CLIENT = 0,
	SPHERE = 1,
	TEST = 2
};

class BENEWAKE_API BwPointCloud
{
public:
	BwPointCloud() {};
	virtual ~BwPointCloud()
	{
		points.clear();
	};

	using Ptr = std::shared_ptr<BwPointCloud>;

	std::vector<BwPoint> points;
	float pkg_loss = 0.0;
};

class BENEWAKE_API BwPointCloudRGB : public BwPointCloud
{
public:
	BwPointCloudRGB() {};
	virtual ~BwPointCloudRGB()
	{
		points.clear();
		rgb.clear();
	};

	using Ptr = std::shared_ptr<BwPointCloudRGB>;

	std::vector<BwRGB> rgb;
};

class BENEWAKE_API BwPointCloudGray : public BwPointCloud
{
public:
	BwPointCloudGray() {};
	virtual ~BwPointCloudGray()
	{
		points.clear();
		gray.clear();
	};

	using Ptr = std::shared_ptr<BwPointCloudGray>;

	std::vector<uint8_t> gray;
};

class BENEWAKE_API BwSphereCoordP4Frame
{
public:
	BwSphereCoordP4Frame() {};
	virtual ~BwSphereCoordP4Frame()
	{
		data.clear();
	};

	using Ptr = std::shared_ptr<BwSphereCoordP4Frame>;

	std::vector<BwSphereCoordP4> data;
};

class BENEWAKE_API BwTestDataP4Frame
{
public:
	BwTestDataP4Frame() {};
	virtual ~BwTestDataP4Frame()
	{
		data.clear();
	};

	using Ptr = std::shared_ptr<BwTestDataP4Frame>;

	std::vector<BwTestDataP4> data;
};

class BENEWAKE_API BwSphereCoordG66Frame
{
public:
	BwSphereCoordG66Frame() {};
	virtual ~BwSphereCoordG66Frame()
	{
		data.clear();
	};

	using Ptr = std::shared_ptr<BwSphereCoordG66Frame>;

	std::vector<BwSphereCoordG66> data;
};

class BENEWAKE_API BwTestDataG66Frame
{
public:
	BwTestDataG66Frame() {};
	virtual ~BwTestDataG66Frame()
	{
		data.clear();
	};

	using Ptr = std::shared_ptr<BwTestDataG66Frame>;

	std::vector<BwTestDataG66> data;
};

/**
* @brief Function pointer for point cloud received event.
*/
typedef void(*PointCloudCallbackFunc)(benewake::BwPointCloud::Ptr _pointcloud, int _frame_id, void* _pData);

typedef void(*HeartBeatCallbcakFunc)(benewake::SYS_INFO _sys_info, void* _pData);

}

#endif