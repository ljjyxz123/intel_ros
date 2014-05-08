// VoiceSynthesis.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "VoiceSynthesis.h"

// for ATL::CA2W
#include <atlbase.h>

class MyVoiceSynthesis : public VoiceSynthesis
{
public:
	void dealInfo(char* info)
	{
		ROS_INFO("%s", info);
	}

	void dealError(char* error)
	{
		ROS_ERROR("%s", error);
	}
};

MyVoiceSynthesis* speaker;

void speakerCallback(const std_msgs::String::ConstPtr& msg)
{
	speaker->speak(msg->data.c_str());
}

int _tmain(int argc, char** argv)
{
	ros::init(argc, argv, "speaker");
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("voice_synthesis", 10, speakerCallback);
	
	pxcCHAR* sentence = L"�����ϳ�ϵͳ �����ɹ���";
	speaker = new MyVoiceSynthesis();
	speaker->speak(sentence);

	ros::spin();

	delete speaker;
	return 0;
}
