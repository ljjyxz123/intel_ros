#pragma once
#include "PXCUtility.h"

class VoiceRecognition : public UtilPipeline
{
public:
	~VoiceRecognition();
	VoiceRecognition();
	VoiceRecognition(PXCSession* session);


	PXCSmartPtr<PXCSession> session;
	PXCUtility* pxcUtility;

public:
	void initAll();

	virtual void OnVoiceRecognitionSetup(PXCVoiceRecognition::ProfileInfo* pinfo);
	virtual void  PXCAPI OnRecognized(PXCVoiceRecognition::Recognition* data);
	virtual void  PXCAPI OnAlert(PXCVoiceRecognition::Alert* data);
	void startLoop();
	virtual void dealInfo(char* info);
	virtual void dealError(char* error);
	void dealInfo(std::wstring info);
	void dealError(std::wstring error);
};

