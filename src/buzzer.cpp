#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <iostream>
#include <fstream>
#include <string>


void buzzer_callback(const std_msgs::Int16 msg){
	std::ofstream writing_file;
	std::string filename = "/dev/rtbuzzer0";
	writing_file.open(filename,std::ios::app);
	writing_file << msg.data << "\n";
	writing_file.close();
}

int main(int argc,char **argv){
	ros::init(argc,argv,"buzzer");
	ros::NodeHandle n;
	ros::Subscriber buzzer_sub = n.subscribe("sound_value",10,buzzer_callback);
	ros::spin();
	return 0;
}
