//
// Created by arslan on 22.05.2022.
//
#include "ClapB7Driver.hpp"

using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;

using namespace std;
using namespace chrono_literals;
using rcl_interfaces::msg::ParameterDescriptor;
using rclcpp::ParameterValue;

int freq_rawimu = 0;
int freq_inspvax = 0;
int freq_agric = 0;

ClapB7Driver::ClapB7Driver()
    : Node("rbf_clap_b7_driver"),

      // General Clap Data Topic
      clap_data_topic_{this->declare_parameter(
                               "clap_imu_data",
                               ParameterValue{"/clap_b7_data"},
                               ParameterDescriptor{})
                           .get<std::string>()},
      // Std. imu topic
      imu_topic_{this->declare_parameter(
                         "imu_topic",
                         ParameterValue{"/gnss/imu"},
                         ParameterDescriptor{})
                     .get<std::string>()},
      // std. msgs
      nav_sat_fix_topic_{this->declare_parameter(
                                 "nav_sat_fix_topic",
                                 ParameterValue{"/gnss/gps"},
                                 ParameterDescriptor{})
                             .get<std::string>()},

      // Serial port config
      serial_name_{this->declare_parameter(
                           "serial_name",
                           ParameterValue{"/dev/ttyUSB0"},
                           ParameterDescriptor{})
                       .get<std::string>()},

      baud_rate_{this->declare_parameter(
                         "baud_rate",
                         ParameterValue{460800},
                         ParameterDescriptor{})
                     .get<long>()},

      serial_boost(serial_name_, baud_rate_),



      ntrip_server_ip_{this->declare_parameter("ntrip_ip",
                                                     ParameterValue("212.156.70.42"),
                                                     ParameterDescriptor{})
                                     .get<std::string>()},

      username_{this->declare_parameter("ntrip_user_name",
                                              ParameterValue("K073432501"),
                                              ParameterDescriptor{})
                              .get<std::string>()},

      password_{this->declare_parameter("ntrip_password",
                                              ParameterValue("GR3g4"),
                                              ParameterDescriptor{})
                              .get<std::string>()},

      mount_point_{this->declare_parameter("ntrip_mount_point",
                                                 ParameterValue("VRSRTCM31"),
                                                 ParameterDescriptor{})
                                 .get<std::string>()},

      ntrip_port_{static_cast<int>(this->declare_parameter("ntrip_port",
                                                                 ParameterValue(2101),
                                                                 ParameterDescriptor{})
                                                                 .get<int>())},
      activate_ntrip_{this->declare_parameter("activate_ntrip",
                                              ParameterValue("true"),
                                              ParameterDescriptor{})
                               .get<std::string>()},

      // Publisher
      pub_clap_data_{create_publisher<rbf_clap_b7_msgs::msg::ClapData>(
          clap_data_topic_, rclcpp::QoS{10}, PubAllocT{})},

      pub_imu_{create_publisher<sensor_msgs::msg::Imu>(
          imu_topic_, rclcpp::QoS{10}, PubAllocT{})},

      pub_nav_sat_fix_{create_publisher<sensor_msgs::msg::NavSatFix>(
          nav_sat_fix_topic_, rclcpp::QoS{10}, PubAllocT{})},

      // Timer
      timer_{this->create_wall_timer(
          1000ms, std::bind(&ClapB7Driver::timer_callback, this))}

{
  using namespace std::placeholders;

  // Set serial callback
  serial_boost.setCallback(bind(&ClapB7Driver::serial_receive_callback, this, _1, _2));

  // Init ClapB7
  ClapB7Init(&clapB7Controller, bind(&ClapB7Driver::pub_ClapB7Data, this));

  if (activate_ntrip_ == "true") {
    NTRIP_client_start();
  }

}

void ClapB7Driver::serial_receive_callback(const char *data, unsigned int len)
{
  ClapB7Parser(&clapB7Controller, reinterpret_cast<const uint8_t *>(data), len);
}

void ClapB7Driver::timer_callback()
{

  printf("freq_rawimu_hz = %d\n", freq_rawimu);
  printf("freq_inspvax_hz = %d\n", freq_inspvax);
  printf("freq_agric_hz = %d\n", freq_agric);

  freq_rawimu = 0;
  freq_inspvax = 0;
    freq_agric = 0;
}

void ClapB7Driver::pub_ClapB7Data()
{

  rbf_clap_b7_msgs::msg::ClapData msg_clap_data;

  msg_clap_data.set__ins_status(static_cast<uint32_t>(clapB7Controller.clapData.ins_status));
  msg_clap_data.set__pos_type(static_cast<uint32_t>(clapB7Controller.clapData.pos_type));

  msg_clap_data.set__latitude(static_cast<double>(clapB7Controller.clapData.latitude));
  msg_clap_data.set__longitude(static_cast<double>(clapB7Controller.clapData.longitude));

  msg_clap_data.set__height(static_cast<double>(clapB7Controller.clapData.height));

  msg_clap_data.set__undulation(static_cast<float>(clapB7Controller.clapData.undulation));

  msg_clap_data.set__north_velocity(static_cast<double>(clapB7Controller.clapData.north_velocity));
  msg_clap_data.set__east_velocity(static_cast<double>(clapB7Controller.clapData.east_velocity));
  msg_clap_data.set__up_velocity(static_cast<double>(clapB7Controller.clapData.up_velocity));

  msg_clap_data.set__roll(static_cast<double>(clapB7Controller.clapData.roll));
  msg_clap_data.set__pitch(static_cast<double>(clapB7Controller.clapData.pitch));
  msg_clap_data.set__azimuth(static_cast<double>(clapB7Controller.clapData.azimuth));

  msg_clap_data.set__std_dev_latitude(static_cast<float>(clapB7Controller.clapData.std_dev_latitude));
  msg_clap_data.set__std_dev_longitude(static_cast<float>(clapB7Controller.clapData.std_dev_longitude));

  msg_clap_data.set__std_dev_height(static_cast<float>(clapB7Controller.clapData.std_dev_height));

  msg_clap_data.set__std_dev_north_velocity(static_cast<float>(clapB7Controller.clapData.std_dev_north_velocity));
  msg_clap_data.set__std_dev_east_velocity(static_cast<float>(clapB7Controller.clapData.std_dev_east_velocity));
  msg_clap_data.set__std_dev_up_velocity(static_cast<float>(clapB7Controller.clapData.std_dev_up_velocity));

  msg_clap_data.set__std_dev_roll(static_cast<float>(clapB7Controller.clapData.std_dev_roll));
  msg_clap_data.set__std_dev_pitch(static_cast<float>(clapB7Controller.clapData.std_dev_pitch));
  msg_clap_data.set__std_dev_azimuth(static_cast<float>(clapB7Controller.clapData.std_dev_azimuth));

  msg_clap_data.set__extended_solution_stat(static_cast<uint32_t>(clapB7Controller.clapData.extended_solution_stat));
  msg_clap_data.set__time_since_update(static_cast<uint16_t>(clapB7Controller.clapData.time_since_update));

  msg_clap_data.set__imu_status(static_cast<uint8_t>(clapB7Controller.clap_RawimuMsgs.imu_status));

  msg_clap_data.set__z_accel_output(static_cast<int32_t>(clapB7Controller.clap_RawimuMsgs.z_accel_output * ACCEL_SCALE_FACTOR));
  msg_clap_data.set__y_accel_output(static_cast<int32_t>(clapB7Controller.clap_RawimuMsgs.y_accel_output * (-ACCEL_SCALE_FACTOR)));
  msg_clap_data.set__x_accel_output(static_cast<int32_t>(clapB7Controller.clap_RawimuMsgs.x_accel_output * ACCEL_SCALE_FACTOR));

  msg_clap_data.set__z_gyro_output(static_cast<int32_t>(clapB7Controller.clap_RawimuMsgs.x_gyro_output * GYRO_SCALE_FACTOR));
  msg_clap_data.set__y_gyro_output(static_cast<int32_t>(clapB7Controller.clap_RawimuMsgs.y_gyro_output * (-GYRO_SCALE_FACTOR)));
  msg_clap_data.set__x_gyro_output(static_cast<int32_t>(clapB7Controller.clap_RawimuMsgs.z_gyro_output * GYRO_SCALE_FACTOR));

  pub_clap_data_->publish(msg_clap_data);

  if( (clapB7Controller.clapData.ins_status == INS_INACTIVE) || (clapB7Controller.clapData.ins_status == INS_ALIGNING) )
  {
      publish_standart_msgs_agric();
  }
  else
  {
      publish_standart_msgs();
  }

}

void ClapB7Driver::publish_standart_msgs()
{
  sensor_msgs::msg::Imu msg_imu;
  sensor_msgs::msg::NavSatFix msg_nav_sat_fix;

  // GNSS NavSatFix Message
  msg_nav_sat_fix.header.set__frame_id(static_cast<std::string>("gnss"));
  msg_nav_sat_fix.header.stamp.set__sec(static_cast<int32_t>(this->get_clock()->now().seconds()));
  msg_nav_sat_fix.header.stamp.set__nanosec(static_cast<uint32_t>(this->get_clock()->now().nanoseconds()));

  msg_nav_sat_fix.status.set__status(0);

  msg_nav_sat_fix.set__latitude(static_cast<double>(clapB7Controller.clapData.latitude));
  msg_nav_sat_fix.set__longitude(static_cast<double>(clapB7Controller.clapData.longitude));
  msg_nav_sat_fix.set__altitude(static_cast<double>(clapB7Controller.clapData.height));

  std::array<double, 9> pos_cov{0};
  pos_cov[0] = clapB7Controller.clapData.std_dev_latitude;
  pos_cov[4] = clapB7Controller.clapData.std_dev_longitude;
  pos_cov[8] = clapB7Controller.clapData.std_dev_height;

  msg_nav_sat_fix.set__position_covariance(pos_cov);

  msg_imu.header.set__frame_id(static_cast<std::string>("gnss"));
  msg_imu.header.stamp.set__sec(static_cast<int32_t>(this->get_clock()->now().seconds()));
  msg_imu.header.stamp.set__nanosec(static_cast<uint32_t>(this->get_clock()->now().nanoseconds()));

  tf2::Quaternion quart_orient;

  double t_roll, t_pitch, t_yaw;

  t_pitch = 180 * atan(clapB7Controller.clap_RawimuMsgs.x_accel_output / sqrt(std::pow(clapB7Controller.clap_RawimuMsgs.y_accel_output, 2) + std::pow(clapB7Controller.clap_RawimuMsgs.z_accel_output, 2))) / M_PI;

  t_roll = 180 * atan(clapB7Controller.clap_RawimuMsgs.y_accel_output / sqrt(std::pow(clapB7Controller.clap_RawimuMsgs.x_accel_output, 2) + std::pow(clapB7Controller.clap_RawimuMsgs.z_accel_output, 2))) / M_PI;

  t_yaw = 180 * atan(clapB7Controller.clap_RawimuMsgs.z_accel_output / sqrt(std::pow(clapB7Controller.clap_RawimuMsgs.x_accel_output, 2) + std::pow(clapB7Controller.clap_RawimuMsgs.z_accel_output, 2))) / M_PI;

  std::cout << "Roll: " << clapB7Controller.clapData.roll << std::endl;
  std::cout << "Pitch: " << clapB7Controller.clapData.pitch << std::endl;
  std::cout << "Yaw: " << clapB7Controller.clapData.azimuth << std::endl;
  std::cout << "-------------------" << std::endl;
  quart_orient.setRPY(clapB7Controller.clapData.roll, clapB7Controller.clapData.pitch, clapB7Controller.clapData.azimuth);
  quart_orient.normalize();

  float old = 0;

  float delta_theta_z = (clapB7Controller.clap_RawimuMsgs.z_accel_output - old) * 0.01;

  old = clapB7Controller.clap_RawimuMsgs.z_accel_output;

  t_yaw += delta_theta_z * 180 / M_PI;

  msg_imu.orientation.set__w(static_cast<double>(quart_orient.getW()));
  msg_imu.orientation.set__x(static_cast<double>(quart_orient.getX()));
  msg_imu.orientation.set__y(static_cast<double>(quart_orient.getY()));
  msg_imu.orientation.set__z(static_cast<double>(quart_orient.getZ()));

  msg_imu.angular_velocity.set__x(static_cast<double>(clapB7Controller.clap_RawimuMsgs.x_gyro_output * (M_PI / 180) * GYRO_SCALE_FACTOR * HZ_TO_SECOND));
  msg_imu.angular_velocity.set__y(static_cast<double>(clapB7Controller.clap_RawimuMsgs.y_gyro_output * (M_PI / 180) * GYRO_SCALE_FACTOR * HZ_TO_SECOND));
  msg_imu.angular_velocity.set__z(static_cast<double>(clapB7Controller.clap_RawimuMsgs.z_gyro_output * (M_PI / 180) * GYRO_SCALE_FACTOR * HZ_TO_SECOND));

  msg_imu.linear_acceleration.set__x(static_cast<double>(clapB7Controller.clap_RawimuMsgs.x_accel_output * ACCEL_SCALE_FACTOR * HZ_TO_SECOND));
  msg_imu.linear_acceleration.set__y(static_cast<double>(clapB7Controller.clap_RawimuMsgs.y_accel_output * ACCEL_SCALE_FACTOR * HZ_TO_SECOND));
  msg_imu.linear_acceleration.set__z(static_cast<double>(clapB7Controller.clap_RawimuMsgs.z_accel_output * ACCEL_SCALE_FACTOR * HZ_TO_SECOND));

  pub_imu_->publish(msg_imu);
  pub_nav_sat_fix_->publish(msg_nav_sat_fix);
}

void ClapB7Driver::publish_standart_msgs_agric()
{
    sensor_msgs::msg::NavSatFix msg_nav_sat_fix;

    // GNSS NavSatFix Message
    msg_nav_sat_fix.header.set__frame_id(static_cast<std::string>("gnss"));
    msg_nav_sat_fix.header.stamp.set__sec(static_cast<int32_t>(this->get_clock()->now().seconds()));
    msg_nav_sat_fix.header.stamp.set__nanosec(static_cast<uint32_t>(this->get_clock()->now().nanoseconds()));

    msg_nav_sat_fix.status.set__status(0);

    msg_nav_sat_fix.set__latitude(static_cast<double>(clapB7Controller.clap_ArgicData.lat));
    msg_nav_sat_fix.set__longitude(static_cast<double>(clapB7Controller.clap_ArgicData.lon));
    msg_nav_sat_fix.set__altitude(static_cast<double>(clapB7Controller.clap_ArgicData.Het));

    std::array<double, 9> pos_cov{0};
    pos_cov[0] = clapB7Controller.clap_ArgicData.lat;
    pos_cov[4] = clapB7Controller.clap_ArgicData.lon;
    pos_cov[8] = clapB7Controller.clap_ArgicData.Het;

    msg_nav_sat_fix.set__position_covariance(pos_cov);

    pub_nav_sat_fix_->publish(msg_nav_sat_fix);
}

int ClapB7Driver::NTRIP_client_start()
{

  ntripClient.Init(ntrip_server_ip_, ntrip_port_, username_, password_, mount_point_);
  ntripClient.OnReceived([this](const char *buffer, int size)
                         {

                             serial_boost.write(buffer, size);

                             t_size += size;

                             std::cout << "NTRIP:" << t_size << std::endl; });

  ntripClient.set_location(41.018893949, 28.890924848);

  ntripClient.set_report_interval(0.001);
  ntrip_status_ = ntripClient.Run();
}
