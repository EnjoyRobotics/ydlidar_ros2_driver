/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

#define ROS2Verision "1.0.1"

using namespace std::chrono_literals;

class YDLidarClass: public rclcpp::Node {
  public:
    YDLidarClass(): Node("ydlidar_ros2_driver_node"){

    
      std::string str_optvalue = "/dev/ydlidar";
      this->declare_parameter<std::string>("port");
      this->get_parameter("port", str_optvalue);
      ///lidar port
      laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

      ///ignore array
      str_optvalue = "";
      this->declare_parameter<std::string>("ignore_array");
      this->get_parameter("ignore_array", str_optvalue);
      laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

      frame_id = "laser_frame";
      this->declare_parameter<std::string>("frame_id");
      this->get_parameter("frame_id", frame_id);

      std::string topic = "scan";
      this->declare_parameter<std::string>("topic");
      this->get_parameter("topic", topic);


      //////////////////////int property/////////////////
      /// lidar baudrate
      int optval = 230400;
      this->declare_parameter<int>("baudrate");
      this->get_parameter("baudrate", optval);
      laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
      /// tof lidar
      optval = TYPE_TRIANGLE;
      this->declare_parameter<int>("lidar_type");
      this->get_parameter("lidar_type", optval);
      laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
      /// device type
      optval = YDLIDAR_TYPE_SERIAL;
      this->declare_parameter<int>("device_type");
      this->get_parameter("device_type", optval);
      laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
      /// sample rate
      optval = 9;
      this->declare_parameter<int>("sample_rate");
      this->get_parameter("sample_rate", optval);
      laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
      /// abnormal count
      optval = 4;
      this->declare_parameter<int>("abnormal_check_count");
      this->get_parameter("abnormal_check_count", optval);
      laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
        

      //////////////////////bool property/////////////////
      /// fixed angle resolution
      bool b_optvalue = false;
      this->declare_parameter<bool>("fixed_resolution");
      this->get_parameter("fixed_resolution", b_optvalue);
      laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
      /// rotate 180
      b_optvalue = true;
      this->declare_parameter<bool>("reversion");
      this->get_parameter("reversion", b_optvalue);
      laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
      /// Counterclockwise
      b_optvalue = true;
      this->declare_parameter<bool>("inverted");
      this->get_parameter("inverted", b_optvalue);
      laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
      b_optvalue = true;
      this->declare_parameter<bool>("auto_reconnect");
      this->get_parameter("auto_reconnect", b_optvalue);
      laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
      /// one-way communication
      b_optvalue = false;
      this->declare_parameter<bool>("isSingleChannel");
      this->get_parameter("isSingleChannel", b_optvalue);
      laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
      /// intensity
      b_optvalue = false;
      this->declare_parameter<bool>("intensity");
      this->get_parameter("intensity", b_optvalue);
      laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
      /// Motor DTR
      b_optvalue = false;
      this->declare_parameter<bool>("support_motor_dtr");
      this->get_parameter("support_motor_dtr", b_optvalue);
      laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
      
      
      //////////////////////float property/////////////////
      /// unit: °
      float f_optvalue = 180.0f;
      this->declare_parameter<float>("angle_max");
      this->get_parameter("angle_max", f_optvalue);
      laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
      f_optvalue = -180.0f;
      this->declare_parameter<float>("angle_min");
      this->get_parameter("angle_min", f_optvalue);
      laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
      /// unit: m
      f_optvalue = 64.f;
      this->declare_parameter<float>("range_max");
      this->get_parameter("range_max", f_optvalue);
      laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
      f_optvalue = 0.1f;
      this->declare_parameter<float>("range_min");
      this->get_parameter("range_min", f_optvalue);
      laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
      /// unit: Hz
      f_optvalue = 10.f;
      this->declare_parameter<float>("frequency");
      this->get_parameter("frequency", f_optvalue);
      laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

      bool invalid_range_is_inf = false;
      this->declare_parameter<bool>("invalid_range_is_inf");
      this->get_parameter("invalid_range_is_inf", invalid_range_is_inf);

      this->declare_parameter<float>("scaling_factor");
      this->get_parameter("scaling_factor", scaling_factor);

      this->declare_parameter<float>("intercept_factor");
      this->get_parameter("intercept_factor", intercept_factor);

      param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
      

      auto param_callback = [this](const rclcpp::Parameter & p) {
        if (strcmp(p.get_name().c_str(), std::string("scaling_factor").c_str()) == 0) {
          scaling_factor = (float) p.as_double();
        } else if(strcmp(p.get_name().c_str(), std::string("intercept_factor").c_str()) == 0) {
          intercept_factor = (float) p.as_double();
        } 
      };
      param_callback_handler_scaling_factor_ = param_subscriber_->add_parameter_callback("scaling_factor", param_callback);
      param_callback_handler_intercept_factor_ = param_subscriber_->add_parameter_callback("intercept_factor", param_callback);


      laser_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic, rclcpp::SensorDataQoS());

      bool ret = laser.initialize();
      if (ret) {
        ret = laser.turnOn();
      } else {
        RCLCPP_ERROR(this->get_logger(), "%s\n", laser.DescribeError());
      }
      
      timer_ = this->create_wall_timer(50ms, std::bind(&YDLidarClass::tick, this));

    }
    void tick() {
      LaserScan scan;

      if (laser.doProcessSimple(scan)) {

      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      scan_msg->header.stamp.nanosec =  scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = frame_id;
      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;
      
      int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
      scan_msg->ranges.resize(size);
      scan_msg->intensities.resize(size);
      for(size_t i=0; i < scan.points.size(); i++) {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
        if(index >=0 && index < size) {
          if (scan.points[i].range > 0.0) {
            scan_msg->ranges[index] = intercept_factor + scan.points[i].range * scaling_factor;
          } else {
            scan_msg->ranges[index] = 0.0;
          }
          scan_msg->intensities[index] = scan.points[i].intensity;
        }
      }

      laser_publisher_->publish(*scan_msg);


    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to get scan");
    }

    }
    ~YDLidarClass() {
      RCLCPP_INFO(this->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
      laser.turnOff();
      laser.disconnecting();
    }
  
  private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> param_callback_handler_scaling_factor_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> param_callback_handler_intercept_factor_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher_;
  CYdLidar laser;
  std::string frame_id;
  float scaling_factor = 1.0f;
  float intercept_factor = 0.0f;

};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<YDLidarClass>());
  rclcpp::shutdown();
  return 0;
}
