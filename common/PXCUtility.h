#pragma once
#include "util_pipeline.h"

class PXCUtility
{
public:
	PXCUtility(PXCSession* session);
	~PXCUtility();
	pxcCHAR* LanguageToString(PXCVoiceRecognition::ProfileInfo::Language language);
	pxcCHAR* AlertToString(PXCVoiceRecognition::Alert::Label label);
	void initLists();
	void selectDevice(int index);
	void selectModule(int index);
	void selectLanguage(int index);

public:
	PXCSmartPtr<PXCSession> session;
	std::vector<std::wstring> deviceList;
	std::vector<std::wstring> moduleList;
	std::vector<std::wstring> languageList;
	int selectedDevice;
	int selectedModule;
	int selectedLanguage;
};

