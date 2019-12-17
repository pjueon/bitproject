#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "MainMachine.h"
#include "UtilityFunctions.h"
#include "Mode.h"

//#include "bit_custom_msgs/YOLOBoxInfo.h"

#include <vector>
#include <utility>
#include <iostream>


using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv, "robot_node");
	ros::NodeHandle n;
	
	MainMachine mainMachine(n);

	mainMachine.StartMainLoop();
	mainMachine.test();

	//bit_custom_msgs::YOLOBoxInfo info;

	
	cout << "program end!" << endl;

	return 0;
}
