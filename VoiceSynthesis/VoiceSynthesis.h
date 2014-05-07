#pragma once
#include "pxcsmartptr.h"
#include "pxcvoice.h"
class VoiceSynthesis
{
public:
	VoiceSynthesis();
	~VoiceSynthesis();

	virtual void dealInfo(char* info);
	virtual void dealError(char* error);

	void speak(pxcCHAR* sentence);
	PXCSmartPtr<PXCSession> session;
	PXCSmartPtr<PXCVoiceSynthesis> synth;
	PXCVoiceSynthesis::ProfileInfo pinfo;

	void speak(const char* sentence);
	//void speak(std::string sentence);
};

