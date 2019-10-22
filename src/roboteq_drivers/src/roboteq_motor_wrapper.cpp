#include "roboteq_drivers/roboteq_motor_wrapper.h"

RoboteqMotorWrapper::RoboteqMotorWrapper() : nh(""), priv_nh("~") {
    if (!priv_nh.getParam(USB_DEVICE_PARAMETER_NAME, usb_device)) {
		ROS_INFO_STREAM("Param '" << USB_DEVICE_PARAMETER_NAME << 
		    "' was not set. Using default device: '/dev/ttyACM0'");
		usb_device = "/dev/ttyACM0";
	}
    if (!priv_nh.getParam(WRITE_PERIOD_PARAMETER_NAME, write_period)) {
		ROS_INFO_STREAM("Param '" << WRITE_PERIOD_PARAMETER_NAME << 
		    "' was not set. Using default write period of 0.0");
		write_period = 0.0;
	}
	if (!priv_nh.getParam(COMMAND_SUB_TOPIC_PARAMETER_NAME, command_sub_topic_name)) {
		ROS_INFO_STREAM("Param '" << COMMAND_SUB_TOPIC_PARAMETER_NAME << 
		    "' was not set. Using default name: 'motor_command'");
		command_sub_topic_name = "motor_command";
	}
	motor_command_sub = nh.subscribe(command_sub_topic_name, 10, 
	    &RoboteqMotorWrapper::onMotorCommand, this);
}

RoboteqMotorWrapper::~RoboteqMotorWrapper() {
    
}

int RoboteqMotorWrapper::initializeHardware() {
    return device.Connect(usb_device);
}

void RoboteqMotorWrapper::onMotorCommand(const std_msgs::Int32MultiArray& msg) {
    values = msg.data;
}

void RoboteqMotorWrapper::writeCallback(const ros::TimerEvent&) {
    for (size_t i = 0; i < values.size(); i++) {
        int status = device.SetCommand(_GO, i + 1, values[i]);
        if (status != RQ_SUCCESS) {
            ROS_ERROR_STREAM("SetCommand(_GO, " << i + 1 <<", " << values[i] <<") failed: " << status);
        }
        sleepms(10);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "roboteq_motor_wrapper");
    ros::NodeHandle nh("");
    
    RoboteqMotorWrapper motor_wrapper;
    
    int status = motor_wrapper.initializeHardware();
    if (status != RQ_SUCCESS) {
        ROS_ERROR_STREAM("Error connecting to the device: " << status << ".");
        return 0;
    }
    
    ros::Timer write_timer = nh.createTimer(
        ros::Duration(motor_wrapper.getWritePeriod()), 
        &RoboteqMotorWrapper::writeCallback, &motor_wrapper);
        
    ROS_INFO("Spinning...");
    
    ros::spin();
    return 0;
}
