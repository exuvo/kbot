
#include "network.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <gflags/gflags.h>

Network::Network(){

}

Network::~Network(){
  delete(node);
	if(m_thread.joinable()) stop(); 
}

void Network::start(int &argc, char* argv[]){
	ros::init(argc, argv, "kbot_tui");
  node = new ros::NodeHandle;
  m_thread = std::thread(&Network::run, this);
}

void Network::run(){
	ros::Rate loop_rate(10);

	while(ros::ok() && !m_stop){
		receive();
		transmit();
		ros::spinOnce();
		loop_rate.sleep();
	} 

}

void Network::receive(){

}

void Network::transmit(){

}

ros::NodeHandle* Network::getNode(){
	return node;
}

void Network::publish(std::string url, std::string value){
	ros::Publisher p = node->advertise<std_msgs::String>(url, 1);
	std_msgs::String msg;
	msg.data = value;
	p.publish(msg);
}

void Network::publish(std::string url, int value){
	ros::Publisher p = node->advertise<std_msgs::Int32>(url, 1);
	std_msgs::String msg;
	msg.data = value;
	p.publish(msg);
}

void Network::publish(std::string url, float value){
	ros::Publisher p = node->advertise<std_msgs::Float32>(url, 1);
	std_msgs::String msg;
	msg.data = value;
	p.publish(msg);
}

void Network::publish(std::string url, bool value){
	ros::Publisher p = node->advertise<std_msgs::Bool>(url, 1);
	std_msgs::String msg;
	msg.data = value;
	p.publish(msg);
}

