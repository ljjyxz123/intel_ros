// gesture_recognition.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include "MyGestureRecognition.h"

// Ros includes
#include <ros/ros.h>
#include "std_msgs/String.h"

int _tmain(int argc, char** argv)
{
	ros::init(argc, argv, "intel_gesture_recognizer");
	ros::NodeHandle node;
	ros::Publisher gestureRecPub = node.advertise<std_msgs::String>("/recognizer/output", 1000);
	ros::Rate loop_rate(10); // pub rate at 20Hz

	//UtilRender render(L"Color Stream");
	UtilPipeline *pp=0;

	pp=new UtilPipeline();
	//pp->QueryCapture()->SetFilter(GetCheckedDevice(hwndDlg));

	pp->EnableGesture();

	/* Init */
	if (pp->Init()) {
		while(ros::ok())
		{
			if (!pp->AcquireFrame(true)) break;
			PXCGesture *gesture=pp->QueryGesture();
			PXCImage *depth=pp->QueryImage(PXCImage::IMAGE_TYPE_DEPTH);
			
			PXCGesture::Gesture gestures[2]={0};
			gesture->QueryGestureData(0,PXCGesture::GeoNode::LABEL_BODY_HAND_PRIMARY,0,&gestures[0]);
			gesture->QueryGestureData(0,PXCGesture::GeoNode::LABEL_BODY_HAND_SECONDARY,0,&gestures[1]);

			std::string gestureCommand;
			switch (gestures[0].label)
			{
			case PXCGesture::Gesture::LABEL_POSE_THUMB_UP:
				gestureCommand = "follow me";
				break;
			case PXCGesture::Gesture::LABEL_POSE_THUMB_DOWN:
				gestureCommand = "stop";
				break;
			case PXCGesture::Gesture::LABEL_POSE_PEACE:
				gestureCommand = "follow me";
				break;
			case PXCGesture::Gesture::LABEL_POSE_BIG5:
				gestureCommand = "stop";
				break;
			case PXCGesture::Gesture::LABEL_HAND_WAVE:
				gestureCommand = "LABEL_HAND_WAVE";
				break;
			case PXCGesture::Gesture::LABEL_HAND_CIRCLE:
				gestureCommand = "LABEL_HAND_CIRCLE";
				break;
			case PXCGesture::Gesture::LABEL_NAV_SWIPE_LEFT:
				gestureCommand = "turn left";
				break;
			case PXCGesture::Gesture::LABEL_NAV_SWIPE_RIGHT:
				gestureCommand = "turn right";
				break;
			case PXCGesture::Gesture::LABEL_NAV_SWIPE_UP:
				gestureCommand = "faster";
				break;
			case PXCGesture::Gesture::LABEL_NAV_SWIPE_DOWN:
				gestureCommand = "slower";
				break;
			default:
				gestureCommand = "NONE";
				break;
			}
			if(gestureCommand != "NONE")
			{
				ROS_INFO(gestureCommand.c_str());
				//std::cout << gestureCommand.c_str() << std::endl;
			}
			
			//render.RenderFrame(depth);
			pp->ReleaseFrame();

			std_msgs::String gestureMsg;
			gestureMsg.data = gestureCommand;
			gestureRecPub.publish(gestureMsg);
			loop_rate.sleep();
		}
		
	}

	pp->Close();
	pp->Release();

// 	MyGestureRecognition pipeline;
// 	pipeline.LoopFrames();
	return 0;
}

