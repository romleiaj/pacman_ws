#ifndef _ROBOTEQ_MOTOR_WRAPPER_H_
#define _ROBOTEQ_MOTOR_WRAPPER_H_

#include <iostream>
#include <stdio.h>
#include <string.h>
#include "roboteq_api/RoboteqDevice.h"
#include "roboteq_api/ErrorCodes.h"
#include "roboteq_api/Constants.h"

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>

class RoboteqMotorWrapper {
    private:
        std::string USB_DEVICE_PARAMETER_NAME = "usb_device";
        std::string WRITE_PERIOD_PARAMETER_NAME = "write_period";
        std::string READ_PERIOD_PARAMETER_NAME = "read_period";
	std::string COMMAND_SUB_TOPIC_PARAMETER_NAME = "command_topic_name";
	std::string ENCODERR_PUB_TOPIC_PARAMETER_NAME = "encoderR_topic_name";
	std::string ENCODERL_PUB_TOPIC_PARAMETER_NAME = "encoderL_topic_name";
    
        ros::NodeHandle nh;
        ros::NodeHandle priv_nh;
        ros::Subscriber motor_command_sub;
        ros::Subscriber estop_sub;
        ros::Publisher  encoderR_pub;
        ros::Publisher  encoderL_pub;
        
        RoboteqDevice device;
        
        std::vector<int> values;
        
        std::string usb_device;
        double write_period;
        double read_period;
        std::string command_sub_topic_name;
        std::string encoderR_pub_topic_name;
        std::string encoderL_pub_topic_name;
        
    public:
        RoboteqMotorWrapper();
        ~RoboteqMotorWrapper();
        
        int initializeHardware();
        
        double getWritePeriod() { return write_period; }
        double getReadPeriod() { return read_period; }
        
        void onMotorCommand(const std_msgs::Int32MultiArray& msg);
        void onEStop(const sensor_msgs::Joy& msg);
        void writeCallback(const ros::TimerEvent&);
        void readRCallback(const ros::TimerEvent&);
        void readLCallback(const ros::TimerEvent&);
        
};

#endif //_ROBOTEQ_MOTOR_WRAPPER_H_
