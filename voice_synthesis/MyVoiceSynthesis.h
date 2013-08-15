#pragma once
#include "pxcsmartptr.h"
#include "pxcvoice.h"
#include <string>

class MyVoiceSynthesis
{
public:
	MyVoiceSynthesis(void);
	~MyVoiceSynthesis(void);
	void speak(wchar_t* sentence);
	PXCSmartPtr<PXCSession> session;
	pxcStatus status;
	PXCSmartPtr<PXCVoiceSynthesis> synth;
	PXCVoiceSynthesis::ProfileInfo pinfo;
	void speak(char* sentence);
	void speak(std::string sentence);
};

