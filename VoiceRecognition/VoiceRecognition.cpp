#include "stdafx.h"

// for Intel part
#include "VoiceRecognition.h"
#include <sstream>
#include <atlbase.h>

// for ROS part
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "RosSpeaker.h"


VoiceRecognition::VoiceRecognition()
{
	if (PXCSession_Create(&session_) < PXC_STATUS_NO_ERROR)
	{
		dealError(L"Failed to create an SDK session Voice Recognition!");
	}

	initAll();
}

VoiceRecognition::VoiceRecognition(PXCSession* session) : UtilPipeline(session), session_(session)
{
	initAll();
}

void VoiceRecognition::initAll()
{
	//setlocale(LC_CTYPE, ".936"); // display Chinese in Unicode 不注释掉会导致输出字符串断词问题
	pxcUtility_ = new PXCUtility(session_);
	pxcUtility_->initLists();
}

VoiceRecognition::~VoiceRecognition()
{
}

void VoiceRecognition::OnVoiceRecognitionSetup(PXCVoiceRecognition::ProfileInfo* pinfo)
{
	QueryVoiceRecognition()->QueryProfile(pxcUtility_->selectedLanguage, pinfo);
}

void PXCAPI VoiceRecognition::OnRecognized(PXCVoiceRecognition::Recognition* data)
{
	//if (data->label<0)
	//{
	//	dealInfo(data->dictation);
	//}
	//else 
	//{
	//	for (int i = 0; i<sizeof(data->nBest) / sizeof(data->nBest[0]); i++)
	//	{
	//		if (data->nBest[i].label<0 || data->nBest[i].confidence == 0) continue;
	//		std::wstringstream info;
	//		info << L"label: " << data->dictation;
	//		info << L" nBest: " << data->nBest[i].label;
	//		info << L" confidence: " << data->nBest[i].confidence;
	//		dealInfo(info.str().c_str());
	//	}
	//}



	int label = data->label;
	if (label < 0)
	{
		ROS_ERROR("Wrowng label: [%d]", label);
		return;
	}
	std::string command = voiceCmds_.cmds[label].command;
	std::string speech = voiceCmds_.cmds[label].speech;
	ROS_INFO("Voice recognized: speach [%s] command [%s]", speech.c_str(), command.c_str());
	if ("time" == command)
	{
		time_t t;
		tm* local;
		tm* gmt;
		char buf[128] = { 0 };
		t = time(NULL);
		local = localtime(&t);
		gmt = gmtime(&t);
		strftime(buf, 128, "现在的时间是%X", gmt);
		speaker_.speak(buf);
	}
	else if ("date" == command)
	{
		time_t t;
		tm* gmt;
		char buf[128] = { 0 };
		t = time(NULL);
		gmt = gmtime(&t);
		strftime(buf, 128, "现在的日期是%x", gmt);
		speaker_.speak(buf);
	}
	else
	{
		std_msgs::String msg;
		msg.data = command;
		voiceRecPub_.publish(msg);
	}
}

void  PXCAPI VoiceRecognition::OnAlert(PXCVoiceRecognition::Alert *data)
{
	dealInfo(pxcUtility_->AlertToString(data->label));
}

void VoiceRecognition::dealInfo(char* info)
{
	//printf_s("[INFO]: %s\n", info);
	ROS_INFO("%s", info);
}

void VoiceRecognition::dealError(char* error)
{
	//printf_s("[ERROR]: %s\n", error);
	ROS_ERROR("%s", error);
}

void VoiceRecognition::dealInfo(std::wstring info)
{
	dealInfo(ATL::CW2A(info.c_str()));
}

void VoiceRecognition::dealError(std::wstring error)
{
	dealError(ATL::CW2A(error.c_str()));
}

//---------------------------------------------------------------------
// main
//---------------------------------------------------------------------

int _tmain(int argc, char** argv)
{
	ros::init(argc, argv, "voice_recognition");
	ros::NodeHandle node;
	std::string path;
	node.param<std::string>("voice_recognition/cmd_csv_path", path, "../common/commands.csv");
	std::cout << path.c_str() << std::endl;
	//voiceRecPub = node.advertise<std_msgs::String>("/recognizer/output", 1000);

	VoiceRecognition* voiceRec = new VoiceRecognition();
	PXCUtility* pxcUtility = voiceRec->pxcUtility_;
	RosSpeaker speaker = voiceRec->speaker_;
	voiceRec->voiceRecPub_ = node.advertise<std_msgs::String>("/recognizer/output", 1);
	std::vector<std::wstring> deviceList = pxcUtility->deviceList;
	std::vector<std::wstring> languageList = pxcUtility->languageList;
	std::vector<std::wstring> moduleList = pxcUtility->moduleList;

	int size = deviceList.size();
	for (int i = 0; i < size; i++)
	{
		voiceRec->dealInfo(deviceList[i]);
	}
	
	size = moduleList.size();
	for (int i = 0; i < size; i++)
	{
		voiceRec->dealInfo(moduleList[i]);
	}

	size = languageList.size();
	for (int i = 0; i < size; i++)
	{
		voiceRec->dealInfo(languageList[i]);
	}

	pxcUtility->selectDevice(0);
	pxcUtility->selectModule(0);
	pxcUtility->selectLanguage(1);

	pxcStatus status;
	voiceRec->QueryCapture()->SetFilter((pxcCHAR*)pxcUtility->deviceList[pxcUtility->selectedDevice].c_str());
	voiceRec->EnableVoiceRecognition(pxcUtility->moduleList[pxcUtility->selectedModule].c_str());

	VoiceCommands& voiceCmds = voiceRec->voiceCmds_;
	voiceCmds.loadCsvFile(path);
	std::string str = boost::lexical_cast<string>(voiceCmds.cmds.size()) + "条命令加载成功!";
	Sleep(1000);
	speaker.speak("语音识别系统启动...");
	speaker.speak(str.c_str());
	std::vector<std::wstring> cmds = voiceCmds.getWCommands();
	if (cmds.empty())
	{
		voiceRec->dealError(L"No Command List. Dictation instead.");
		voiceRec->SetVoiceDictation();
	}
	else
	{
		voiceRec->SetVoiceCommands(cmds);
	}

	/* Initialization */
	voiceRec->dealInfo(L"Init Started");
	if (voiceRec->Init()) 
	{
		voiceRec->dealInfo(L"Init OK");

		/* Set Audio Volume. The camera microphone is very sensitive. */
		voiceRec->QueryCapture()->QueryDevice()->SetProperty(PXCCapture::Device::PROPERTY_AUDIO_MIX_LEVEL, 0.2f);

		/* Recognition Loop */
		while (ros::ok())
		{
			if (!voiceRec->AcquireFrame(true)) break;
			if (!voiceRec->ReleaseFrame())
			{
				voiceRec->dealError(L"Recognition Pipeline Failed!");
				break;
			}
		}
	}
	else
	{
		voiceRec->dealError(L"Init Failed!");
	}

	voiceRec->Close();
	voiceRec->Release();
	return 0;
}
