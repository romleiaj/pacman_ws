#include "roboteq_drivers/roboteq_motor_wrapper.h"
#include <std_msgs/Int32.h>

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
    if (!priv_nh.getParam(READ_PERIOD_PARAMETER_NAME, read_period)) {
		ROS_INFO_STREAM("Param '" << READ_PERIOD_PARAMETER_NAME << 
		    "' was not set. Using default read period of 0.01");
		read_period = 0.01;
	}
	if (!priv_nh.getParam(COMMAND_SUB_TOPIC_PARAMETER_NAME, command_sub_topic_name)) {
		ROS_INFO_STREAM("Param '" << COMMAND_SUB_TOPIC_PARAMETER_NAME << 
		    "' was not set. Using default name: 'motor_command'");
		command_sub_topic_name = "motor_command";
	}
	motor_command_sub = nh.subscribe(command_sub_topic_name, 10, 
	    &RoboteqMotorWrapper::onMotorCommand, this);
	if (!priv_nh.getParam(ENCODERL_PUB_TOPIC_PARAMETER_NAME, encoderL_pub_topic_name)) {
		ROS_INFO_STREAM("Param '" << ENCODERL_PUB_TOPIC_PARAMETER_NAME << 
		    "' was not set. Using default name: 'encoderL_pulses'");
		encoderL_pub_topic_name = "encoderL_pulses";
	}
	if (!priv_nh.getParam(ENCODERR_PUB_TOPIC_PARAMETER_NAME, encoderR_pub_topic_name)) {
		ROS_INFO_STREAM("Param '" << ENCODERR_PUB_TOPIC_PARAMETER_NAME << 
		    "' was not set. Using default name: 'encoderR_pulses'");
		encoderR_pub_topic_name = "encoderR_pulses";
	}
    	encoderR_pub = nh.advertise<std_msgs::Int32>(encoderR_pub_topic_name, 10);
	encoderL_pub = nh.advertise<std_msgs::Int32>(encoderL_pub_topic_name, 10);
	estop_sub = nh.subscribe("/joy", 10, 
	    &RoboteqMotorWrapper::onEStop, this);
	while (motor_command_sub.getNumPublishers() < 1){
		ROS_INFO_STREAM("No publishers on " << command_sub_topic_name << ".");	
		ROS_INFO_STREAM("Waiting...");
		sleepms(1000);
	}
}

RoboteqMotorWrapper::~RoboteqMotorWrapper() {
    
}

int RoboteqMotorWrapper::initializeHardware() {
    return device.Connect(usb_device);
}

void RoboteqMotorWrapper::onMotorCommand(const std_msgs::Int32MultiArray& msg) {
    values = msg.data;
}

void RoboteqMotorWrapper::onEStop(const sensor_msgs::Joy& msg) {
    if (msg.buttons[1] == 1) { // EMERGENCY STOP
        int status = device.SetCommand(_EX);
        if (status != RQ_SUCCESS) {
            ROS_ERROR_STREAM("SetCommand(_EX) failed: " << status);
        }
	ROS_INFO("EMERGENCY STOP ACTIVATED");
        sleepms(5);
    } else if (msg.buttons[2] == 1) { // RELEASE ESTOP
        int status = device.SetCommand(_MG);
        if (status != RQ_SUCCESS) {
            ROS_ERROR_STREAM("SetCommand(_MG) failed: " << status);
        }
    	ROS_INFO("Released Emergency Stop.");
        sleepms(5);
    }
}

void RoboteqMotorWrapper::readLCallback(const ros::TimerEvent&) {
    int counts;
    int status = device.GetValue(_ABCNTR, 2, counts);
    if (status != RQ_SUCCESS) {
        ROS_ERROR_STREAM("GetValue(_ABCNTR, " << 1 <<", " <<") failed: " << status);
    }
    sleepms(5);
    std_msgs::Int32 msg;
    msg.data = counts;
    encoderL_pub.publish(msg);
}

void RoboteqMotorWrapper::readRCallback(const ros::TimerEvent&) {
    int counts;
    int status = device.GetValue(_ABCNTR, 1, counts);
    if (status != RQ_SUCCESS) {
        ROS_ERROR_STREAM("GetValue(_ABCNTR, " << 1 <<", " <<") failed: " << status);
    }
    sleepms(5);
    std_msgs::Int32 msg;
    msg.data = counts;
    encoderR_pub.publish(msg);
}

void RoboteqMotorWrapper::writeCallback(const ros::TimerEvent&) {
    std::vector<int> cur_values = values;
    if (motor_command_sub.getNumPublishers() < 1) {
        ROS_ERROR_STREAM("Roboteq Command publisher lost!");
	for (int j=0; j<cur_values.size(); j++){
		cur_values[j] = 0;
	}
    }
    for (size_t i = 0; i < cur_values.size(); i++) {
        int status = device.SetCommand(_GO, i + 1, cur_values[i]);
        if (status != RQ_SUCCESS) {
            ROS_ERROR_STREAM("SetCommand(_GO, " << i + 1 <<", " << cur_values[i] <<") failed: " << status);
        }
        sleepms(5);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "roboteq_motor_wrapper");
    ros::NodeHandle nh("");
    
    RoboteqMotorWrapper motor_wrapper;
    
    int status = motor_wrapper.initializeHardware();
    while (status != RQ_SUCCESS) {
        ROS_ERROR_STREAM("Error connecting to the device: " << status << ".");
        ROS_ERROR_STREAM("Retrying.");
        sleepms(1000);
        status = motor_wrapper.initializeHardware();
    }
    
    ros::Timer write_timer = nh.createTimer(
        ros::Duration(motor_wrapper.getWritePeriod()), 
        &RoboteqMotorWrapper::writeCallback, &motor_wrapper);

    ros::Timer readR_timer = nh.createTimer(
        ros::Duration(motor_wrapper.getReadPeriod()), 
        &RoboteqMotorWrapper::readRCallback, &motor_wrapper);

    ros::Timer readL_timer = nh.createTimer(
        ros::Duration(motor_wrapper.getReadPeriod()), 
        &RoboteqMotorWrapper::readLCallback, &motor_wrapper);
    
    ROS_INFO("Spinning...");
    
    ros::spin();
    return 0;
}
