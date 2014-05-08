#pragma once
#include "PXCUtility.h"
#include "VoiceCommands.h"
#include "ros/ros.h"
#include "RosSpeaker.h"

class VoiceRecognition : public UtilPipeline
{
public:
	PXCSmartPtr<PXCSession> session_;
	PXCUtility* pxcUtility_;
	VoiceCommands voiceCmds_;
	ros::Publisher voiceRecPub_;
	RosSpeaker speaker_;

public:
	~VoiceRecognition();
	VoiceRecognition();
	VoiceRecognition(PXCSession* session);

public:
	void initAll();
	virtual void OnVoiceRecognitionSetup(PXCVoiceRecognition::ProfileInfo* pinfo);
	virtual void  PXCAPI OnRecognized(PXCVoiceRecognition::Recognition* data);
	virtual void  PXCAPI OnAlert(PXCVoiceRecognition::Alert* data);
	virtual void dealInfo(char* info);
	virtual void dealError(char* error);
	virtual void dealInfo(std::wstring info);
	virtual void dealError(std::wstring error);
};

