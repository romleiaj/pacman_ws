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
#include <sensor_msgs/Joy.h>

class RoboteqMotorWrapper {
    private:
        std::string USB_DEVICE_PARAMETER_NAME = "usb_device";
        std::string WRITE_PERIOD_PARAMETER_NAME = "write_period";
		std::string COMMAND_SUB_TOPIC_PARAMETER_NAME = "command_topic_name";
    
        ros::NodeHandle nh;
        ros::NodeHandle priv_nh;
        ros::Subscriber motor_command_sub;
        ros::Subscriber estop_sub;
        
        RoboteqDevice device;
        
        std::vector<int> values;
        
        std::string usb_device;
        double write_period;
        std::string command_sub_topic_name;
        
    public:
        RoboteqMotorWrapper();
        ~RoboteqMotorWrapper();
        
        int initializeHardware();
        
        double getWritePeriod() { return write_period; }
        
        void onMotorCommand(const std_msgs::Int32MultiArray& msg);
        void onEStop(const sensor_msgs::Joy& msg);
        void writeCallback(const ros::TimerEvent&);
        
};

#endif //_ROBOTEQ_MOTOR_WRAPPER_H_
