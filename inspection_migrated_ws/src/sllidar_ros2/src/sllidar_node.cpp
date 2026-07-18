/*
 *  SLLIDAR ROS2 NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include "sl_lidar.h"
#include "math.h"

#include <algorithm>
#include <chrono>
#include <signal.h>
#include <thread>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

#define ROS2VERSION "1.0.1"

using namespace sl;

bool need_exit = false;

class SLlidarNode : public rclcpp::Node
{
  public:
    SLlidarNode()
    : Node("sllidar_node")
    {

      scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::KeepLast(10)));
      
    }

  private:    
    void init_param()
    {
        this->declare_parameter<std::string>("channel_type","serial");
        this->declare_parameter<std::string>("tcp_ip", "192.168.0.7");
        this->declare_parameter<int>("tcp_port", 20108);
        this->declare_parameter<std::string>("udp_ip","192.168.11.2");
        this->declare_parameter<int>("udp_port",8089);
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("serial_baudrate",1000000);
        this->declare_parameter<std::string>("frame_id","laser_frame");
        this->declare_parameter<bool>("inverted", false);
        this->declare_parameter<bool>("angle_compensate", false);
        this->declare_parameter<std::string>("scan_mode",std::string());
        this->declare_parameter<float>("scan_frequency",10);
        this->declare_parameter<bool>("auto_recovery_enabled", true);
        this->declare_parameter<int>("scan_timeout_recovery_count", 3);
        this->declare_parameter<int>("scan_recovery_stop_delay_ms", 3000);
        this->declare_parameter<int>("max_scan_recovery_attempts", 2);
        
        this->get_parameter_or<std::string>("channel_type", channel_type, "serial");
        this->get_parameter_or<std::string>("tcp_ip", tcp_ip, "192.168.0.7"); 
        this->get_parameter_or<int>("tcp_port", tcp_port, 20108);
        this->get_parameter_or<std::string>("udp_ip", udp_ip, "192.168.11.2"); 
        this->get_parameter_or<int>("udp_port", udp_port, 8089);
        this->get_parameter_or<std::string>("serial_port", serial_port, "/dev/ttyUSB0"); 
        this->get_parameter_or<int>("serial_baudrate", serial_baudrate, 1000000/*256000*/);//ros run for A1 A2, change to 256000 if A3
        this->get_parameter_or<std::string>("frame_id", frame_id, "laser_frame");
        this->get_parameter_or<bool>("inverted", inverted, false);
        this->get_parameter_or<bool>("angle_compensate", angle_compensate, false);
        this->get_parameter_or<std::string>("scan_mode", scan_mode, std::string());
        if(channel_type == "udp")
            this->get_parameter_or<float>("scan_frequency", scan_frequency, 20.0);
        else
            this->get_parameter_or<float>("scan_frequency", scan_frequency, 10.0);
        this->get_parameter_or<bool>("auto_recovery_enabled", auto_recovery_enabled, true);
        this->get_parameter_or<int>("scan_timeout_recovery_count", scan_timeout_recovery_count, 3);
        this->get_parameter_or<int>("scan_recovery_stop_delay_ms", scan_recovery_stop_delay_ms, 3000);
        this->get_parameter_or<int>("max_scan_recovery_attempts", max_scan_recovery_attempts, 2);
        scan_timeout_recovery_count = std::min(std::max(scan_timeout_recovery_count, 1), 20);
        scan_recovery_stop_delay_ms = std::min(std::max(scan_recovery_stop_delay_ms, 500), 10000);
        max_scan_recovery_attempts = std::min(std::max(max_scan_recovery_attempts, 0), 5);
    }

    sl_result start_configured_scan(LidarScanMode & current_scan_mode)
    {
        if (scan_mode.empty()) {
            return drv->startScan(false, true, 0, &current_scan_mode);
        }

        std::vector<LidarScanMode> all_supported_scan_modes;
        sl_result result = drv->getAllSupportedScanModes(all_supported_scan_modes);
        if (SL_IS_FAIL(result)) {
            return result;
        }

        sl_u16 selected_scan_mode = sl_u16(-1);
        for (const auto & supported_scan_mode : all_supported_scan_modes) {
            if (supported_scan_mode.scan_mode == scan_mode) {
                selected_scan_mode = supported_scan_mode.id;
                break;
            }
        }

        if (selected_scan_mode == sl_u16(-1)) {
            RCLCPP_ERROR(this->get_logger(), "scan mode `%s' is not supported by lidar, supported modes:", scan_mode.c_str());
            for (const auto & supported_scan_mode : all_supported_scan_modes) {
                RCLCPP_ERROR(
                    this->get_logger(), "\t%s: max_distance: %.1f m, Point number: %.1fK",
                    supported_scan_mode.scan_mode, supported_scan_mode.max_distance,
                    (1000 / supported_scan_mode.us_per_sample));
            }
            return SL_RESULT_OPERATION_FAIL;
        }

        return drv->startScanExpress(false, selected_scan_mode, 0, &current_scan_mode);
    }

    bool getSLLIDARDeviceInfo(ILidarDriver * drv)
    {
        sl_result     op_result;
        sl_lidar_response_device_info_t devinfo;

        op_result = drv->getDeviceInfo(devinfo);
        if (SL_IS_FAIL(op_result)) {
            if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
                RCLCPP_ERROR(this->get_logger(),"Error, operation time out. SL_RESULT_OPERATION_TIMEOUT! ");
            } else {
                RCLCPP_ERROR(this->get_logger(),"Error, unexpected error, code: %x",op_result);
            }
            return false;
        }

        // print out the device serial number, firmware and hardware version number..
        char sn_str[37] = {'\0'}; 
        for (int pos = 0; pos < 16 ;++pos) {
            sprintf(sn_str + (pos * 2),"%02X", devinfo.serialnum[pos]);
        }
        RCLCPP_INFO(this->get_logger(),"SLLidar S/N: %s",sn_str);
        RCLCPP_INFO(this->get_logger(),"Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
        RCLCPP_INFO(this->get_logger(),"Hardware Rev: %d",(int)devinfo.hardware_version);
        return true;
    }

    bool checkSLLIDARHealth(ILidarDriver * drv)
    {
        sl_result     op_result;
        sl_lidar_response_device_health_t healthinfo;
        op_result = drv->getHealth(healthinfo);
        if (SL_IS_OK(op_result)) { 
            RCLCPP_INFO(this->get_logger(),"SLLidar health status : %d", healthinfo.status);
            switch (healthinfo.status) {
                case SL_LIDAR_STATUS_OK:
                    RCLCPP_INFO(this->get_logger(),"SLLidar health status : OK.");
                    return true;
                case SL_LIDAR_STATUS_WARNING:
                    RCLCPP_INFO(this->get_logger(),"SLLidar health status : Warning.");
                    return true;
                case SL_LIDAR_STATUS_ERROR:
                    RCLCPP_ERROR(this->get_logger(),"Error, SLLidar internal error detected. Please reboot the device to retry.");
                    return false;
                default:
                    RCLCPP_ERROR(this->get_logger(),"Error, Unknown internal error detected. Please reboot the device to retry.");
                    return false;

            }
        } else {
            RCLCPP_ERROR(this->get_logger(),"Error, cannot retrieve SLLidar health code: %x", op_result);
            return false;
        }
    }

    bool stop_motor(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                    std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        (void)req;
        (void)res;

        if(!drv)
            return false;

        RCLCPP_DEBUG(this->get_logger(),"Stop motor");
        scan_stopped_by_service = true;
        drv->stop();
        drv->setMotorSpeed(0);
        return true;
    }

    bool start_motor(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                    std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        (void)req;
        (void)res;

        if(!drv)
           return false;
        if(drv->isConnected())
        {
            RCLCPP_DEBUG(this->get_logger(),"Start motor");
            sl_result ans=drv->setMotorSpeed();
            if (SL_IS_FAIL(ans)) {
                RCLCPP_WARN(this->get_logger(), "Failed to start motor: %08x", ans);
                return false;
            }
        
            LidarScanMode current_scan_mode;
            ans = start_configured_scan(current_scan_mode);
            if (SL_IS_FAIL(ans)) {
                RCLCPP_WARN(this->get_logger(), "Failed to start scan: %08x", ans);
                return false;
            }
            scan_stopped_by_service = false;
        } else {
            RCLCPP_INFO(this->get_logger(),"lost connection");
            return false;
        }

        return true;
    }

    static float getAngle(const sl_lidar_response_measurement_node_hq_t& node)
    {
        return node.angle_z_q14 * 90.f / 16384.f;
    }

    void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub,
                  sl_lidar_response_measurement_node_hq_t *nodes,
                  size_t node_count, rclcpp::Time start,
                  double scan_time, bool inverted,
                  float angle_min, float angle_max,
                  float max_distance,
                  std::string frame_id)
    {
        static int scan_count = 0;
        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

        scan_msg->header.stamp = start;
        scan_msg->header.frame_id = frame_id;
        scan_count++;

        bool reversed = (angle_max > angle_min);
        if ( reversed ) {
            scan_msg->angle_min =  M_PI - angle_max;
            scan_msg->angle_max =  M_PI - angle_min;
        } else {
            scan_msg->angle_min =  M_PI - angle_min;
            scan_msg->angle_max =  M_PI - angle_max;
        }
        scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (double)(node_count-1);

        scan_msg->scan_time = scan_time;
        scan_msg->time_increment = scan_time / (double)(node_count-1);
        scan_msg->range_min = 0.05;
        scan_msg->range_max = max_distance;//8.0;

        scan_msg->intensities.resize(node_count);
        scan_msg->ranges.resize(node_count);
        bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
        if (!reverse_data) {
            for (size_t i = 0; i < node_count; i++) {
                float read_value = (float) nodes[i].dist_mm_q2/4.0f/1000;
                if (read_value == 0.0)
                    scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
                else
                    scan_msg->ranges[i] = read_value;
                scan_msg->intensities[i] = (float) (nodes[i].quality >> 2);
            }
        } else {
            for (size_t i = 0; i < node_count; i++) {
                float read_value = (float)nodes[i].dist_mm_q2/4.0f/1000;
                if (read_value == 0.0)
                    scan_msg->ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
                else
                    scan_msg->ranges[node_count-1-i] = read_value;
                scan_msg->intensities[node_count-1-i] = (float) (nodes[i].quality >> 2);
            }
        }

        pub->publish(*scan_msg);
    }
public:    
    int work_loop()
    {        
        init_param();
        int ver_major = SL_LIDAR_SDK_VERSION_MAJOR;
        int ver_minor = SL_LIDAR_SDK_VERSION_MINOR;
        int ver_patch = SL_LIDAR_SDK_VERSION_PATCH;
        RCLCPP_INFO(this->get_logger(),"SLLidar running on ROS2 package SLLidar.ROS2 SDK Version:" ROS2VERSION ", SLLIDAR SDK Version:%d.%d.%d",ver_major,ver_minor,ver_patch);
    
        sl_result     op_result;

        // create the driver instance
        drv = *createLidarDriver();
        IChannel* _channel;
        if(channel_type == "tcp"){
            _channel = *createTcpChannel(tcp_ip, tcp_port);
        }
        else if(channel_type == "udp"){
            _channel = *createUdpChannel(udp_ip, udp_port);
        }
        else{
            _channel = *createSerialPortChannel(serial_port, serial_baudrate);
        }
        if (SL_IS_FAIL((drv)->connect(_channel))) {
            if(channel_type == "tcp"){
                RCLCPP_ERROR(this->get_logger(),"Error, cannot connect to the ip addr  %s with the tcp port %s.",tcp_ip.c_str(),std::to_string(tcp_port).c_str());
            }
            else if(channel_type == "udp"){
                RCLCPP_ERROR(this->get_logger(),"Error, cannot connect to the ip addr  %s with the udp port %s.",udp_ip.c_str(),std::to_string(udp_port).c_str());
            }
            else{
                RCLCPP_ERROR(this->get_logger(),"Error, cannot bind to the specified serial port %s.",serial_port.c_str());            
            }
            delete drv;
            return -1;
        }
        
        // get sllidar device info
        if (!getSLLIDARDeviceInfo(drv)) {
            return -1;
        }

        // check health...
        if (!checkSLLIDARHealth(drv)) {
            return -1;
        }

        stop_motor_service = this->create_service<std_srvs::srv::Empty>("stop_motor",  
                                std::bind(&SLlidarNode::stop_motor,this,std::placeholders::_1,std::placeholders::_2));
        start_motor_service = this->create_service<std_srvs::srv::Empty>("start_motor", 
                                std::bind(&SLlidarNode::start_motor,this,std::placeholders::_1,std::placeholders::_2));

        drv->setMotorSpeed();

        LidarScanMode current_scan_mode;
        op_result = start_configured_scan(current_scan_mode);

        if(SL_IS_OK(op_result))
        {
            //default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
            int points_per_circle = (int)(1000*1000/current_scan_mode.us_per_sample/scan_frequency);
            angle_compensate_multiple = points_per_circle/360.0  + 1;
            if(angle_compensate_multiple < 1) 
            angle_compensate_multiple = 1.0;
            max_distance = (float)current_scan_mode.max_distance;
            RCLCPP_INFO(this->get_logger(),"current scan mode: %s, sample rate: %d Khz, max_distance: %.1f m, scan frequency:%.1f Hz, ", 
                                current_scan_mode.scan_mode,(int)(1000/current_scan_mode.us_per_sample+0.5),max_distance, scan_frequency);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),"Can not start scan: %08x!", op_result);
        }

        rclcpp::Time start_scan_time;
        rclcpp::Time end_scan_time;
        double scan_duration;
        int consecutive_scan_failures = 0;
        int scan_recovery_attempts = 0;
        while (rclcpp::ok() && !need_exit) {
            sl_lidar_response_measurement_node_hq_t nodes[8192];
            size_t   count = _countof(nodes);

            start_scan_time = this->now();
            op_result = drv->grabScanDataHq(nodes, count);
            end_scan_time = this->now();
            scan_duration = (end_scan_time - start_scan_time).seconds();

            if (op_result == SL_RESULT_OK) {
                consecutive_scan_failures = 0;
                scan_recovery_attempts = 0;
                op_result = drv->ascendScanData(nodes, count);
                float angle_min = DEG2RAD(0.0f);
                float angle_max = DEG2RAD(360.0f);
                if (op_result == SL_RESULT_OK) {
                    if (angle_compensate) {
                        //const int angle_compensate_multiple = 1;
                        const int angle_compensate_nodes_count = 360*angle_compensate_multiple;
                        int angle_compensate_offset = 0;
                        auto angle_compensate_nodes = new sl_lidar_response_measurement_node_hq_t[angle_compensate_nodes_count];
                        memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(sl_lidar_response_measurement_node_hq_t));

                        size_t i = 0, j = 0;
                        for( ; i < count; i++ ) {
                            if (nodes[i].dist_mm_q2 != 0) {
                                float angle = getAngle(nodes[i]);
                                int angle_value = (int)(angle * angle_compensate_multiple);
                                if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                                for (j = 0; j < angle_compensate_multiple; j++) {
                                    int angle_compensate_nodes_index = angle_value-angle_compensate_offset + j;
                                    if(angle_compensate_nodes_index >= angle_compensate_nodes_count)
                                        angle_compensate_nodes_index = angle_compensate_nodes_count - 1;
                                    angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
                                }
                            }
                        }
    
                        publish_scan(scan_pub, angle_compensate_nodes, angle_compensate_nodes_count,
                                start_scan_time, scan_duration, inverted,
                                angle_min, angle_max, max_distance,
                                frame_id);

                        if (angle_compensate_nodes) {
                            delete[] angle_compensate_nodes;
                            angle_compensate_nodes = nullptr;
                        }
                    } else {
                        int start_node = 0, end_node = 0;
                        int i = 0;
                        // find the first valid node and last valid node
                        while (nodes[i++].dist_mm_q2 == 0);
                        start_node = i-1;
                        i = count -1;
                        while (nodes[i--].dist_mm_q2 == 0);
                        end_node = i+1;

                        angle_min = DEG2RAD(getAngle(nodes[start_node]));
                        angle_max = DEG2RAD(getAngle(nodes[end_node]));

                        publish_scan(scan_pub, &nodes[start_node], end_node-start_node +1,
                                start_scan_time, scan_duration, inverted,
                                angle_min, angle_max, max_distance,
                                frame_id);
                    }
                } else if (op_result == SL_RESULT_OPERATION_FAIL) {
                    // All the data is invalid, just publish them
                    float angle_min = DEG2RAD(0.0f);
                    float angle_max = DEG2RAD(359.0f);
                    publish_scan(scan_pub, nodes, count,
                                start_scan_time, scan_duration, inverted,
                                angle_min, angle_max, max_distance,
                                frame_id);
                }
            } else if (!scan_stopped_by_service) {
                consecutive_scan_failures++;
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "No scan frame received (result: %08x, consecutive failures: %d)",
                    op_result, consecutive_scan_failures);

                if (auto_recovery_enabled &&
                    consecutive_scan_failures >= scan_timeout_recovery_count) {
                    if (scan_recovery_attempts >= max_scan_recovery_attempts) {
                        RCLCPP_ERROR_THROTTLE(
                            this->get_logger(), *this->get_clock(), 10000,
                            "SLLidar automatic recovery limit reached; /scan remains unavailable");
                    } else {
                        scan_recovery_attempts++;
                        consecutive_scan_failures = 0;
                        RCLCPP_WARN(
                            this->get_logger(),
                            "Restarting SLLidar scan after repeated timeouts (attempt %d/%d)",
                            scan_recovery_attempts, max_scan_recovery_attempts);
                        drv->stop();
                        drv->setMotorSpeed(0);
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(scan_recovery_stop_delay_ms));

                        sl_result recovery_result = drv->setMotorSpeed();
                        LidarScanMode recovered_scan_mode;
                        if (SL_IS_OK(recovery_result)) {
                            recovery_result = start_configured_scan(recovered_scan_mode);
                        }
                        if (SL_IS_FAIL(recovery_result)) {
                            RCLCPP_ERROR(
                                this->get_logger(), "SLLidar scan recovery failed: %08x",
                                recovery_result);
                        } else {
                            RCLCPP_INFO(
                                this->get_logger(),
                                "SLLidar scan recovery command completed; waiting for frames");
                        }
                    }
                }
            }

            rclcpp::spin_some(shared_from_this());
        }

        // done!
        drv->setMotorSpeed(0);
        drv->stop();
        RCLCPP_INFO(this->get_logger(),"Stop motor");

        return 0;
    }


  private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_motor_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_motor_service;

    std::string channel_type;
    std::string tcp_ip;
    std::string udp_ip;
    std::string serial_port;
    int tcp_port = 20108;
    int udp_port = 8089;
    int serial_baudrate = 115200;
    std::string frame_id;
    bool inverted = false;
    bool angle_compensate = true;
    float max_distance = 8.0;
    size_t angle_compensate_multiple = 1;//it stand of angle compensate at per 1 degree
    std::string scan_mode;
    float scan_frequency;
    bool auto_recovery_enabled = true;
    int scan_timeout_recovery_count = 3;
    int scan_recovery_stop_delay_ms = 3000;
    int max_scan_recovery_attempts = 2;
    bool scan_stopped_by_service = false;

    ILidarDriver * drv;    
};

void ExitHandler(int sig)
{
    (void)sig;
    need_exit = true;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  
  auto sllidar_node = std::make_shared<SLlidarNode>();
  signal(SIGINT,ExitHandler);
  int ret = sllidar_node->work_loop();
  rclcpp::shutdown();
  return ret;
}
