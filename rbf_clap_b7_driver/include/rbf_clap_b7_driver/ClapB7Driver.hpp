//
// Created by arslan on 22.05.2022.
//
#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <cstring>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "TermiosSerial.h"
#include "AsyncSerial.h"

#include "rclcpp/rclcpp.hpp"

#include "nmea_msgs/msg/gpgga.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <rbf_clap_b7_msgs/msg/clap_data.hpp>
#include <ntrip/ntrip_client.h>
#include "ClapB7BinaryParser.h"

#define ACCEL_SCALE_FACTOR (400 / (pow(2, 31)))
#define GYRO_SCALE_FACTOR (2160 / (pow(2, 31)))
#define HZ_TO_SECOND ( 100 )

// ROS Time to GPS time Parameters
#define k_TO_NANO (1e9)
#define k_GPS_SEC_IN_WEEK (60 * 60 * 24 * 7)
#define k_UNIX_OFFSET (315964782000000000)
#define k_MILI_TO_NANO (1e6)

#define INS_INACTIVE    0
#define INS_ALIGNING    1

extern int freq_rawimu;
extern int freq_inspvax;
extern int freq_agric;

class ClapB7Driver : public rclcpp::Node
{
public:
    ClapB7Driver();

    ~ClapB7Driver() override {
        serial_boost.close();
    }

    void serial_receive_callback(const char *data, unsigned int len);

private:
    
    void timer_callback();
    void pub_ClapB7Data();
    int NTRIP_client_start();
    void publish_standart_msgs();
    int64_t ros_time_to_gps_time_nano();
    void publish_standart_msgs_agric();

    std::string clap_data_topic_;
    std::string imu_topic_;
    std::string nav_sat_fix_topic_;
    std::string serial_name_;
    std::string ntrip_server_ip_;
    std::string username_;
    std::string password_;
    std::string mount_point_;
    int ntrip_port_;
    std::string activate_ntrip_;
    long baud_rate_;

    CallbackAsyncSerial serial_boost;

    rclcpp::Publisher<rbf_clap_b7_msgs::msg::ClapData>::SharedPtr pub_clap_data_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_nav_sat_fix_;
    rclcpp::Publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr pub_gnss_orientation_;

    ClapB7Controller clapB7Controller;
    uint8_t ntrip_status_ = 0;
    libntrip::NtripClient ntripClient;
    int t_size;
    const float g_ = 9.81f;

    rclcpp::TimerBase::SharedPtr timer_;
};
