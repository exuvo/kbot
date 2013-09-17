#ifndef NETWORK_H
#define NETWORK_H

#include "ros/ros.h"

using namespace std;

template <class T, class Y>
void publish(string url, T value);
	ros::Publisher p = node.advertise<std_msgs::Y>(url, 1);
	std_msgs::String msg;
	msg.data = value;
	p.publish(msg);
}

class Network{
public:
	Network();
	virtual ~Network();
	void start() { m_thread = thread(&Network::run, this); }
	void stop() { m_stop = true; }
	void join() { m_thread.join(); }

/*
 	void publish(string url, string value);
	void publish(string url, int value);
	void publish(string url, double value);
	void publish(string url, bool value);
*/
	

private:
	atomic<bool> m_stop;
	thread m_thread;
	ros::NodeHandle node;

	void run();
};

#endif /* NETWORK_H */
