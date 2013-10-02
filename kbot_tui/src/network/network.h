#ifndef NETWORK_H
#define NETWORK_H

#include "ros/ros.h"
#include <string>

using namespace std;


class Network{
public:
	Network();
	virtual ~Network();
	void start() { m_thread = thread(&Network::run, this); }
	void stop() { m_stop = true; }
	void join() { m_thread.join(); }
	Node getNode();
/*
template <class T, class Y>
void publish(string url, T value);
	ros::Publisher p = node.advertise<Y>(url, 1);
	Y msg;
	msg.data = value;
	p.publish(msg);
}
*/
 	void publish(string url, string value);
	void publish(string url, int value);
	void publish(string url, double value);
	void publish(string url, bool value);
	
private:
	atomic<bool> m_stop;
	thread m_thread;
	ros::NodeHandle node;

	void run();
};

#endif /* NETWORK_H */
