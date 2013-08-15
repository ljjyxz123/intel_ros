// voice_synthesis.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <string>
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "MyVoiceSynthesis.h"

MyVoiceSynthesis speaker;

void speakerCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Received: [%s]", msg->data.c_str());
	speaker.speak(msg->data);
}

int _tmain(int argc, char** argv)
{
	ros::init(argc, argv, "speaker");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("voice_syn", 1000, speakerCallback);
	speaker.speak(L"Voice synthesis system start...");
	ros::spin();
	return 0;
}

