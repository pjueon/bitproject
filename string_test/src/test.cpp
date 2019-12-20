
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <stdexcept>
#include <vector>

#include <fstream>
#include <string>

using namespace std;

int main(int argc, char **argv){
	ros::init(argc, argv, "test_node");
	ros::NodeHandle n;
	auto testpub = n.advertise<std_msgs::String>("/test_topic", 1);


	constexpr auto filename = "/home/jetbot/catkin_ws/src/bitproject/ImageDB/booknames.txt";
	ifstream fin(filename);

	if (!fin.is_open()) {
		cerr << "fail to open " << filename << endl;
		return 1;
	}

	vector<string> lines;
	string line;

	while (getline(fin, line)) {
		lines.push_back(line);
	}

	cerr << "wait" << endl;
	ros::Duration(1.0).sleep();

	for(const auto& bookname : lines){
		std_msgs::String msg;
		msg.data = bookname;
		testpub.publish(msg);
		cout << "published!";
		cout << msg.data << endl;
	}

	return 0;
}
