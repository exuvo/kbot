#ifndef NETWORK_H
#define NETWORK_H

#include <ros/ros.h>
#include <string>
#include <thread>
#include <atomic>

class Network{
public:
	Network();
	virtual ~Network();
	void start(int &argc, char* argv[]);
	void stop() { m_stop = true; }
	void join() { m_thread.join(); }
	ros::NodeHandle* getNode();
  void receive();
  void transmit();
/*
template <class T, class Y>
void publish(string url, T value);
	ros::Publisher p = node.advertise<Y>(url, 1);
	Y msg;
	msg.data = value;
	p.publish(msg);
}
*/
 	void publish(std::string url, std::string value);
	void publish(std::string url, int value);
	void publish(std::string url, float value);
	void publish(std::string url, bool value);
	
private:
	std::atomic<bool> m_stop;
	std::thread m_thread;
	ros::NodeHandle* node;

	void run();
};

#endif /* NETWORK_H */
