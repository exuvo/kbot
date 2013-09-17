
#include "network.h"
#include "std_msgs/String.h"
#include <gflags/gfalgs.h>

Network::Network(){

}

~Network::Network(){

	if(m_thread.joinable()) stop(); 
}

void Network::Run(){
	ros::init("", "", "kbot_tui");
	
	ros::Rate loop_rate(10);

	while(ros::ok() && !m_stop){
		receive();
		transmit();
		ros::spinOnce();
		loop_rate.sleep();
	} 

}

void publish(string url, string value){
	ros::Publisher p = node.advertise<std_msgs::String>(url, 1);
	std_msgs::String msg;
	msg.data = value;
	p.publish(msg);
}

void publish(string url, int value);
	ros::Publisher p = node.advertise<std_msgs::Int32>(url, 1);
	std_msgs::String msg;
	msg.data = value;
	p.publish(msg);
}

void publish(string url, float value);
	ros::Publisher p = node.advertise<std_msgs::Float32>(url, 1);
	std_msgs::String msg;
	msg.data = value;
	p.publish(msg);
}

void publish(string url, bool value);
	ros::Publisher p = node.advertise<std_msgs::Bool>(url, 1);
	std_msgs::String msg;
	msg.data = value;
	p.publish(msg);
}

