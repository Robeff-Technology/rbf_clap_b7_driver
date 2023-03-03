//
// Created by mehce on 13.12.2022.
//

#pragma once

#include <cstdint>
#include <functional>

#define HEADER_LEN 	 	        28U
#define HEADER_LEN_AGRIC 	 	24U
#define CRC_LEN			 4U
#define MAX_DATA_LEN 	512U

#define FIRST_SYNCH		0xAAU
#define SECOND_SYNCH	0x44U
#define THIRD_SYNCH		0x12U

#define THIRD_SYNCH_GPGGA   0xB5U

#define RAWIMU_MSG_ID    268U
#define INSPVAX_MSG_ID   1465U


typedef enum
{
    SYNCH1_CONTROL,
    SYNCH2_CONTROL,
    SYNCH3_CONTROL,
    CLAP_B7_HEADER_LENGTH,
    CLAP_B7_HEADER_ADD,
    CLAP_B7_HEADER_ADD_AGRIC,
    CLAP_B7_DATA_ADD_AGRIC,
    CLAP_B7_DATA_ADD,
}ClapB7ParseStatus;


#pragma pack(1)
typedef struct
{
    uint8_t SYNCH1;
    uint8_t SYNCH2;
    uint8_t SYNCH3;
    uint8_t headerLength;
    uint16_t messageID;
    uint8_t messageType;
    uint8_t portAddress;
    uint16_t messageLen;
    uint16_t sequence;
    uint8_t idleTime;
    uint8_t timeStatus;
    uint16_t refWeekNumber;
    uint32_t weekMs;
    uint32_t receiverStatus;
    uint16_t reserved;
    uint16_t receiverSwVer;
}ClapB7Header;

#pragma pack(1)
typedef struct
{
    uint8_t SYNCH1;
    uint8_t SYNCH2;
    uint8_t SYNCH3;
    uint8_t cpuIDLE;
    uint16_t messageID;
    uint16_t msgLen;
    uint8_t timeRef;
    uint8_t timeStatus;
    uint16_t refWeekNumber;
    uint32_t weekMs;
    uint32_t reserved;
    uint8_t releaseVer;
    uint8_t leapSec;
    uint16_t outDelay;
}ClapB7Header_Agric;

struct __attribute__((packed)) ClapB7_AgricMsg_ {

    int32_t gnss_char;
    uint8_t command_length;            // The digit length from GNSS to CRC is 232 bytes, Fixed value: 0XE8
    uint8_t year;                      // UTC time -year
    uint8_t month;                     // UTC time -month
    uint8_t day;                       // UTC time -day
    uint8_t hour;                      // UTC time -hour
    uint8_t minute;                    // UTC time -minute
    uint8_t second;                    // UTC time-second

    uint8_t RtkStatus;                 // Rover position status  0: Ineffective: 1: Single point: 2: Pseudo-range differential: 4: Fix solution: 5: Float solution:
    uint8_t HeadingStatus;             // Heading solution status of master and slave antennas  0: Ineffective: 4: Fix solution: 5: Float solution:
    uint8_t numGPSsat;                 // GPS satellite number
    uint8_t numBDSsat;                 // BDS satellite number
    uint8_t numGLOsat;                 // GLONASS satellite number

    float Baseline_N;               // From the base to rover baseline vector, Northing component
    float Baseline_E;               // From the base to rover baseline vector, Easting component
    float Baseline_U;               // From the base to rover baseline vector, zenith direction. component standard deviation
    float Baseline_NStd;            // From the base to rover baseline vector, Northing component standard deviation
    float Baseline_EStd;            // From the base to rover baseline vector, Easting component standard deviation
    float Baseline_UStd;            // From the base to rover baseline vector, zenith direction. component standard deviation

    float Heading;                  // Heading
    float Pitch;                    // Pitch
    float Roll;                     // Roll
    float Speed;                    // Speed
    float Velocity_N;               // Northing velocity
    float Velocity_E;               // Easting velocity
    float Velocity_U;               // velocity

    float Xigema_Vx;                // Northing velocity standard deviation
    float Xigema_Vy;                // Easting velocity standard deviation
    float Xigema_Vz;                // velocity standard deviation

    double lat;                     // Rover station latitude (-90 to 90 degrees)   /  a ‘-’ sign denotes south and a ‘+’ sign denotes north
    double lon;                     // Rover station longitude (-180 to 180 degrees)   /  a ‘-’ sign denotes west and a ‘+’ sign denotes east
    double Het;                     // Rover station height

    double ecef_x;                  // ECEF X value (m)
    double ecef_y;                  // ECEF Y value (m)
    double ecef_z;                  // ECEF Z value (m)

    float Xigema_lat;               // Latitude standard deviation
    float Xigema_lon;               // Longitude standard deviation
    float Xigema_alt;               // Height standard deviation
    float Xigema_ecef_x;            // ECEF_X standard deviation
    float Xigema_ecef_y;            // ECEF_Y standard deviation
    float Xigema_ecef_z;            // ECEF_Z standard deviation

    double base_lat;                // Base station latitude (-90 to 90 degrees)
    double base_lon;                // Base station longitude(-180 to 180 degrees)
    double base_alt;                // Base station height

    double sec_lat;                // Sub-antenna latitude (-90 to 90 degrees)
    double sec_lon;                // Sub-antenna longitude(-180 to 180 degrees)
    double sec_alt;                // Sub-antenna height

    int gps_week_second;           // Number of milliseconds into the GPS reference week
    float diffage;                 // Differential age
    float speed_heading;           // Direction of velocity
    float undulation;              // Height outlier
    float remain_float3;           // Reserved
    float remain_float4;           // Reserved

    uint8_t numGALsat;                // Galileo satellite number
    uint8_t remain_char2;             // Reserved
    uint8_t remain_char3;             // Reserved
    char remain_char4;             // Reserved

    int32_t crc_hex;               // 32 bits CRC (only Binary and ASCII)
};

struct __attribute__((packed)) ClapB7_InspvaxMsgs_ {
    uint32_t ins_status;
    uint32_t pos_type;
    double latitude;
    double longitude;
    double height;
    float undulation;
    double north_velocity;
    double east_velocity;
    double up_velocity;
    double roll;
    double pitch;
    double azimuth;
    float std_dev_latitude;
    float std_dev_longitude;
    float std_dev_height;
    float std_dev_north_velocity;
    float std_dev_east_velocity;
    float std_dev_up_velocity;
    float std_dev_roll;
    float std_dev_pitch;
    float std_dev_azimuth;
    uint32_t extended_solution_stat;
    uint16_t time_since_update;

};

struct __attribute__((packed)) ClapB7_RawimuMsgs_ {
    uint32_t gnssWeek;
    double  secondsIntoWeek;
    uint32_t imu_status;
    int32_t z_accel_output;
    int32_t y_accel_output;
    int32_t x_accel_output;
    int32_t z_gyro_output;
    int32_t y_gyro_output;
    int32_t x_gyro_output;
    int crc;
};

typedef struct
{
    uint16_t dataIndex;
    ClapB7ParseStatus status;
    ClapB7Header header;
    ClapB7Header_Agric header_Agric;
    uint8_t rawData[MAX_DATA_LEN];
    ClapB7_AgricMsg_    clap_ArgicData;
    ClapB7_InspvaxMsgs_ clapData;
    ClapB7_RawimuMsgs_  clap_RawimuMsgs;
    std::function<void()> Parser;
} ClapB7Controller;

void ClapB7Init(ClapB7Controller* p_Controller, const std::function<void()> callBack);
void ClapB7Parser(ClapB7Controller* p_Controller, const uint8_t* p_Data, uint16_t len);