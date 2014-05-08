#include "PXCUtility.h"


PXCUtility::PXCUtility(PXCSession* session)
{
	this->session = session;
}


PXCUtility::~PXCUtility()
{
}

pxcCHAR* PXCUtility::LanguageToString(PXCVoiceRecognition::ProfileInfo::Language language)
{
	switch (language)
	{
	case PXCVoiceRecognition::ProfileInfo::LANGUAGE_US_ENGLISH:		return L"US English";
	case PXCVoiceRecognition::ProfileInfo::LANGUAGE_GB_ENGLISH:		return L"British English";
	case PXCVoiceRecognition::ProfileInfo::LANGUAGE_DE_GERMAN:		return L"Deutsch";
	case PXCVoiceRecognition::ProfileInfo::LANGUAGE_IT_ITALIAN:		return L"italiano";
	case PXCVoiceRecognition::ProfileInfo::LANGUAGE_BR_PORTUGUESE:	return L"PORTUGUÊS";
	case PXCVoiceRecognition::ProfileInfo::LANGUAGE_CN_CHINESE:		return L"中文";
	case PXCVoiceRecognition::ProfileInfo::LANGUAGE_FR_FRENCH:		return L"Français";
	case PXCVoiceRecognition::ProfileInfo::LANGUAGE_JP_JAPANESE:	return L"日本語";
	case PXCVoiceRecognition::ProfileInfo::LANGUAGE_US_SPANISH:		return L"español";
	}
	return 0;
}

pxcCHAR* PXCUtility::AlertToString(PXCVoiceRecognition::Alert::Label label)
{
	switch (label)
	{
	case PXCVoiceRecognition::Alert::LABEL_SNR_LOW: return L"SNR low!";
	case PXCVoiceRecognition::Alert::LABEL_SPEECH_UNRECOGNIZABLE: return L"Speech unrecognizable!";
	case PXCVoiceRecognition::Alert::LABEL_VOLUME_HIGH: return L"Volume too high!";
	case PXCVoiceRecognition::Alert::LABEL_VOLUME_LOW: return L"Volume too low!";
	}
	return L"Unknown!";
}

void PXCUtility::initLists()
{
	PXCSession::ImplDesc desc, desc1;

	// init device list
	memset(&desc, 0, sizeof(desc));
	desc.group = PXCSession::IMPL_GROUP_SENSOR;
	desc.subgroup = PXCSession::IMPL_SUBGROUP_AUDIO_CAPTURE;
	for (int i = 0;; i++)
	{
		memset(&desc, 0, sizeof(desc1));
		if (session->QueryImpl(&desc, i, &desc1) < PXC_STATUS_NO_ERROR) break;

		PXCSmartPtr<PXCCapture> capture;
		if (session->CreateImpl<PXCCapture>(&desc1, &capture) < PXC_STATUS_NO_ERROR) continue;
		for (int j = 0;; j++)
		{
			PXCCapture::DeviceInfo dinfo;
			if (capture->QueryDevice(j, &dinfo) < PXC_STATUS_NO_ERROR) break;
			std::wstring deviceName(dinfo.name);
			deviceList.push_back(deviceName);
		}
	}

	// init module list
	memset(&desc, 0, sizeof(desc));
	memset(&desc, 0, sizeof(desc1));
	desc.cuids[0] = PXCVoiceRecognition::CUID;
	int i;
	for (i = 0;; i++)
	{
		if (session->QueryImpl(&desc, i, &desc1)<PXC_STATUS_NO_ERROR) break;
		moduleList.push_back(desc1.friendlyName);
	}

	// init language list
	memset(&desc, 0, sizeof(desc));
	memset(&desc, 0, sizeof(desc1));
	desc.cuids[0] = PXCVoiceRecognition::CUID;
	if (session->QueryImpl(&desc, 0 /*ID_MODULE*/, &desc1) >= PXC_STATUS_NO_ERROR)
	{
		PXCSmartPtr<PXCVoiceRecognition> vrec;
		if (session->CreateImpl<PXCVoiceRecognition>(&desc1, &vrec) >= PXC_STATUS_NO_ERROR)
		{
			for (int k = 0;; k++)
			{
				PXCVoiceRecognition::ProfileInfo pinfo;
				if (vrec->QueryProfile(k, &pinfo) < PXC_STATUS_NO_ERROR) break;
				std::wstring lang(LanguageToString(pinfo.language));
				languageList.push_back(lang);
			}
		}
	}
}

void PXCUtility::selectDevice(int index)
{
	selectedDevice = index;
	//this->QueryCapture()->SetFilter((pxcCHAR*)deviceList[index].c_str());
}

void PXCUtility::selectModule(int index)
{
	selectedModule = index;
}

void PXCUtility::selectLanguage(int index)
{
	selectedLanguage = index;
	//this->pidx = index;
}