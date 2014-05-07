// VoiceRecognition.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "VoiceRecognition.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "RosSpeaker.h"

class MyVoiceRecognition : public VoiceRecognition
{
	void dealInfo(std::wstring info)
	{
		ROS_INFO("[INFO]: %s\n", info.c_str());
	}

	void dealError(std::wstring error)
	{
		ROS_ERROR("[ERROR]: %s\n", error.c_str());
	}

};

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int _tmain(int argc, char** argv)
{
	VoiceRecognition voiceRec;
	voiceRec.startLoop();


	ros::init(argc, argv, "voice_recognition");
	ros::NodeHandle n;
	ROS_INFO("hello");
	RosSpeaker speaker;
	while (1)
	{
	
	speaker.speak(L"你好，我说话了");
	Sleep(1000);
	}
	//ros::Subscriber sub = n.subscribe("voice_synthesis", 1, chatterCallback);
	//ros::spin();

	return 0;
}

