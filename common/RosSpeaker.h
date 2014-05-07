#pragma once
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <atlbase.h>

class RosSpeaker
{
public:
	ros::NodeHandle n;
	ros::Publisher pub;

	RosSpeaker(void)
	{
		pub = n.advertise<std_msgs::String>("voice_synthesis", 1);
	}

	~RosSpeaker(void){}

 	//template<typename T>
	void speak(const char* sentence)
	{
		std_msgs::String msg;
		std::stringstream ss;
		ss << sentence;
		msg.data = ss.str();
		pub.publish(msg);
		//ROS_INFO("Speaker: [%s]", msg.data.c_str());
	}

	void speak(const wchar_t* sentence)
	{
		ATL::CW2A csentence(sentence);
		speak(csentence);
	}
};

