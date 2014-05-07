#include "stdafx.h"
#include "VoiceRecognition.h"
#include <sstream>
#include <atlbase.h>

VoiceRecognition::VoiceRecognition()
{
	if (PXCSession_Create(&session) < PXC_STATUS_NO_ERROR)
	{
		dealError(L"Failed to create an SDK session Voice Recognition!");
	}

	initAll();
}

VoiceRecognition::VoiceRecognition(PXCSession* session) : UtilPipeline(session)
{
	this->session = session;
	initAll();
}

void VoiceRecognition::initAll()
{
	//setlocale(LC_CTYPE, ".936"); // display Chinese in Unicode
	EnableVoiceRecognition();
	pxcUtility = new PXCUtility(session);
	pxcUtility->initLists();
}

VoiceRecognition::~VoiceRecognition()
{
	delete pxcUtility;
	this->Close();
	this->Release();
}

void VoiceRecognition::OnVoiceRecognitionSetup(PXCVoiceRecognition::ProfileInfo* pinfo)
{
	QueryVoiceRecognition()->QueryProfile(pxcUtility->selectedLanguage, pinfo);
}

void PXCAPI VoiceRecognition::OnRecognized(PXCVoiceRecognition::Recognition* data)
{
	if (data->label<0)
	{
		dealInfo(data->dictation);
	}
	else 
	{
		for (int i = 0; i<sizeof(data->nBest) / sizeof(data->nBest[0]); i++)
		{
			if (data->nBest[i].label<0 || data->nBest[i].confidence == 0) continue;
			std::wstringstream info;
			info << L"label: " << data->dictation;
			info << L" nBest: " << data->nBest[i].label;
			info << L" confidence: " << data->nBest[i].confidence;
			dealInfo(info.str().c_str());
		}
	}
}



void  PXCAPI VoiceRecognition::OnAlert(PXCVoiceRecognition::Alert *data)
{
	dealInfo(pxcUtility->AlertToString(data->label));
}


void VoiceRecognition::startLoop()
{
	std::vector<std::wstring> deviceList = pxcUtility->deviceList;
	std::vector<std::wstring> languageList = pxcUtility->languageList;
	std::vector<std::wstring> moduleList = pxcUtility->moduleList;

	int size = deviceList.size();
	for (int i = 0; i < size; i++)
	{
		dealInfo(deviceList[i]);
	}

	size = moduleList.size();
	for (int i = 0; i < size; i++)
	{
		dealInfo(moduleList[i]);
	}

	size = languageList.size();
	for (int i = 0; i < size; i++)
	{
		dealInfo(languageList[i]);
	}

	pxcUtility->selectDevice(0);
	pxcUtility->selectModule(0);
	pxcUtility->selectLanguage(1);

	pxcStatus status;
	this->QueryCapture()->SetFilter((pxcCHAR*)pxcUtility->deviceList[pxcUtility->selectedDevice].c_str());
	this->EnableVoiceRecognition(pxcUtility->moduleList[pxcUtility->selectedModule].c_str());

	std::vector<std::wstring> cmds;
	cmds.push_back(L"喂");
	cmds.push_back(L"你好");
	cmds.push_back(L"停下");
	if (cmds.empty())
	{
		dealError(L"No Command List. Dictation instead.");
		this->SetVoiceDictation();
	}
	else
	{
		this->SetVoiceCommands(cmds);
	}

	/* Initialization */
	dealInfo(L"Init Started");
	if (this->Init()) {
		dealInfo(L"Init OK");

		/* Set Audio Volume. The camera microphone is very sensitive. */
		this->QueryCapture()->QueryDevice()->SetProperty(PXCCapture::Device::PROPERTY_AUDIO_MIX_LEVEL, 0.2f);

		/* Recognition Loop */
		while (true)
		{
			if (!this->AcquireFrame(true)) break;
			if (!this->ReleaseFrame())
			{
				dealError(L"Recognition Pipeline Failed!");
				break;
			}
		}
	}
	else
	{
		dealError(L"Init Failed!");
	}
}

void VoiceRecognition::dealInfo(char* info)
{
	printf_s("[INFO]: %s\n", info);
}

void VoiceRecognition::dealError(char* error)
{
	printf_s("[ERROR]: %s\n", error);
}

void VoiceRecognition::dealInfo(std::wstring info)
{
	dealInfo(ATL::CW2A(info.c_str()));
}

void VoiceRecognition::dealError(std::wstring error)
{
	dealError(ATL::CW2A(error.c_str()));
}
