#include "MyVoiceRecognition.h"
#include <boost/lexical_cast.hpp>

MyVoiceRecognition::~MyVoiceRecognition(void)
{
}

MyVoiceRecognition::MyVoiceRecognition(PXCSession *session) : UtilPipeline(session)
{
	EnableVoiceRecognition();

	string path;
	if (!node.getParam("voice_recognition/cmd_csv_path", path))
	{
		ROS_ERROR("Param [%s] not found. Please modify the file path value in intel_ros/launch/voice_recognition.launch file", "voice_recognition/cmd_csv_path");
		exit(-1);
	}
	voiceCmds.loadCsvFile(path);
	if (voiceCmds.getWCommands().size() != voiceCmds.cmds.size())
	{
		ROS_ERROR("Commands setting error!");
		exit(-1);
	}
	SetVoiceCommands(voiceCmds.getWCommands());
	std::string str = boost::lexical_cast<string>(voiceCmds.cmds.size()) + " commands loaded!";
	rosSpeaker.speak(str.c_str());

	voiceRecPub = node.advertise<std_msgs::String>("/recognizer/output", 1000);
}

void PXCAPI MyVoiceRecognition::OnRecognized(PXCVoiceRecognition::Recognition *data)
{
	int label = data->label;
	if (label < 0)
	{
		ROS_ERROR("Wrowng label: [%d]", label);
		return;
	}
	std::string command = voiceCmds.cmds[label].command;
	std::string speech = voiceCmds.cmds[label].speech;
	ROS_INFO("Voice recognized: speach [%s] command [%s]", speech.c_str(), command.c_str());
	if ("time" == command)
	{
		time_t t;
		tm* local;
		tm* gmt;
		char buf[128]= {0};
		t = time(NULL);
		local = localtime(&t);
		gmt = gmtime(&t);
		strftime(buf, 128, "Current time is %X", gmt);
		rosSpeaker.speak(buf);
	}
	else if ("date" == command)
	{
		time_t t;
		tm* gmt;
		char buf[128]= {0};
		t = time(NULL);
		gmt = gmtime(&t);
		strftime(buf, 128, "Current date is %x", gmt);
		rosSpeaker.speak(buf);
	}
	else
	{
		std_msgs::String msg;
		msg.data = command;
		voiceRecPub.publish(msg);
	}
}

void  PXCAPI MyVoiceRecognition::OnAlert(PXCVoiceRecognition::Alert *data)
{
	std::string alert;
	switch (data->label)
	{
	case PXCVoiceRecognition::Alert::LABEL_SNR_LOW: alert = "SNR low!";
		break;
	case PXCVoiceRecognition::Alert::LABEL_SPEECH_UNRECOGNIZABLE: alert = "Speech unrecognizable!";
		break;
	case PXCVoiceRecognition::Alert::LABEL_VOLUME_HIGH: alert = "Volume too high!";
		break;
	case PXCVoiceRecognition::Alert::LABEL_VOLUME_LOW: alert = "Volume too low!";
		break;
	default:
		alert = "Unknown!";
		break;
	}
	ROS_INFO("Voice unrecognized: [%s]", alert.c_str());
	//rosSpeaker.speak(alert.c_str());
}

int _tmain(int argc, char** argv)
{
	// Ros init
	ros::init(argc, argv, "voice_recog");
	RosSpeaker rosSpeaker;

	// Query audio input devices
	PXCSession *session;
	PXCSession_Create(&session);
	PXCSession::ImplDesc desc1;
	memset(&desc1,0,sizeof(desc1));
	desc1.group=PXCSession::IMPL_GROUP_SENSOR;
	desc1.subgroup=PXCSession::IMPL_SUBGROUP_AUDIO_CAPTURE;
	vector<std::wstring> deviceNames;
	for (int m=0;;m++) {
		PXCSession::ImplDesc desc2;
		if (session->QueryImpl(&desc1,m,&desc2)<PXC_STATUS_NO_ERROR) break;

		PXCSmartPtr<PXCCapture> capture;
		if (session->CreateImpl<PXCCapture>(&desc2,&capture)<PXC_STATUS_NO_ERROR) continue;
		for (int d=0;;d++) {
			PXCCapture::DeviceInfo dinfo;
			if (capture->QueryDevice(d,&dinfo)<PXC_STATUS_NO_ERROR) break;
			std::wstring dname(dinfo.name);
			deviceNames.push_back(dname);
		}
	}

	// display devices and require a selection
	int deviceNum = deviceNames.size();
	int selectedDeviceId = 0;
	wprintf_s(L"Device list:\n");
	for (int i = 0; i < deviceNum; i++)
	{
		wprintf_s(L"Device[%d]: %s\n",i,deviceNames[i].c_str());
	}
	if (deviceNum > 0)
	{
		Sleep(1000);
		while (true)
		{
			rosSpeaker.speak("Please select a audio input device!");
			wprintf_s(L"\nPlease select a device [0]-[%d]\n", deviceNum-1);
			cin >> selectedDeviceId;
			if (selectedDeviceId > -1 && selectedDeviceId < deviceNum)
			{
				wprintf_s(L"You selected device: [%d]%s\n", selectedDeviceId, deviceNames[selectedDeviceId].c_str());
				break;
			}
		}
	}
	else
	{
		wprintf_s(L"No audio device founded!");
		return -1;
	}

	// Voice recogintion start
	MyVoiceRecognition voiceRecog(session);

	// Set audio source
	voiceRecog.QueryCapture()->SetFilter(deviceNames[selectedDeviceId].c_str());

	// Start loop
	if (voiceRecog.Init()) {
		cout << "Voice recognition init OK" << endl;

		// Set Audio Volume
		voiceRecog.QueryCapture()->QueryDevice()->SetProperty(PXCCapture::Device::PROPERTY_AUDIO_MIX_LEVEL,0.2f);

		rosSpeaker.speak(L"Voice recognition system start!");

		// Recognition Loop
		while (true) {
			if (!voiceRecog.AcquireFrame(true)) break;
			voiceRecog.ReleaseFrame();
		}
	} else {
		cout << "Voice recognition init failed!" << endl;
	}
	//voiceRecog.LoopFrames();

	session->Release();
	system("pause");

	return 0;
}