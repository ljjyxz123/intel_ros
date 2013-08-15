#pragma once
#include "stdafx.h"
#include "ros/ros.h"
#include "util_pipeline.h"
#include "RosSpeaker.h"
#include "VoiceCommands.h"

class MyVoiceRecognition :
	public UtilPipeline
{
public:
	~MyVoiceRecognition(void);
	MyVoiceRecognition(PXCSession *session=0);
	virtual void PXCAPI OnRecognized(PXCVoiceRecognition::Recognition *data);
	virtual void  PXCAPI OnAlert(PXCVoiceRecognition::Alert *data);
public:
	RosSpeaker rosSpeaker;
	VoiceCommands voiceCmds;
	ros::NodeHandle node;
	ros::Publisher voiceRecPub;
};

