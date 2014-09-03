#include "subscriptions.h"
#include "serial.h"
#include "std_msgs/String.h"
#include "node.h"

ros::Subscriber rawSend;

void rawSendCallback(const std_msgs::String::ConstPtr& msg) {
	transmit(msg->data);
}

void initSubscriptions(void){
  rawSend = node->subscribe<std_msgs::String>("send", 100, rawSendCallback);
}
