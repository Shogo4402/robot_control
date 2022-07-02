#include <ros/ros.h>
#include <robot_control/MotorFreqs.h>
#include <fstream>
#include "std_msgs/String.h"
#include <string>
#include <thread>
#include <chrono>

void file_write(robot_control::MotorFreqs msg){

	std::ofstream writing_file_left;
	std::ofstream writing_file_right;
	std::string filename_left = "/dev/rtmotor_raw_l0";
	std::string filename_right= "/dev/rtmotor_raw_r0";
	writing_file_left.open(filename_left,std::ios::app);
	writing_file_right.open(filename_right,std::ios::app);
	writing_file_left << std::to_string(msg.left_hz) << std::endl;
	writing_file_right << std::to_string(msg.right_hz) << std::endl;
	writing_file_left.close();
	writing_file_right.close();
}


void set_power(int n){
	std::ofstream writing_file; 
	std::string filename = "/dev/rtmotoren0";
       	writing_file.open(filename,std::ios::app);
	writing_file << std::to_string(n) << std::endl;
	writing_file.close();
}

void callback_raw_freqs(const robot_control::MotorFreqs msg){
	file_write(msg);	
}


int main(int argc, char** argv){
	robot_control::MotorFreqs begin;
	begin.left_hz = 0;
	begin.right_hz = 0;
	file_write(begin);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	set_power(1);
	ros::init(argc,argv,"motor_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("motor_raw",10,callback_raw_freqs);
	ros::spin();
	set_power(0);
	return 0;
}
