// intel_ros.cpp : 定义控制台应用程序的入口点。
// This project is just for test
// Author: yuanboshe@126.com

#include "stdafx.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
// for ATL::CA2W
#include <atlbase.h>
#include <atlconv.h>

#include "../voice_recognition/VoiceCommands.h"
#include "util_pipeline.h"

using namespace std;

void foo1()
{
	PXCSession *session;
	PXCSession_Create(&session);

	// session is a PXCSession instance
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
		while (true)
		{
			wprintf_s(L"Please select a device from [0]~[%d]\n", deviceNum-1);
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
		return;
	}

	session->Release();
	system("pause");
}

void foo2()
{
	//UtilRender render(L"Color Stream");
	UtilPipeline *pp=0;

	pp=new UtilPipeline();
	//pp->QueryCapture()->SetFilter(GetCheckedDevice(hwndDlg));

	pp->EnableGesture();

	std::string gesturePendding;
	long msec = 0; // mm secs
	// Init
	if (pp->Init()) {
		while(true)
		{
			if (!pp->AcquireFrame(true)) break;
			PXCGesture *gesture=pp->QueryGesture();
			PXCImage *depth=pp->QueryImage(PXCImage::IMAGE_TYPE_DEPTH);

			PXCGesture::Gesture gestures[2]={0};
			gesture->QueryGestureData(0,PXCGesture::GeoNode::LABEL_BODY_HAND_PRIMARY,0,&gestures[0]);
			gesture->QueryGestureData(0,PXCGesture::GeoNode::LABEL_BODY_HAND_SECONDARY,0,&gestures[1]);

			std::string gestureCommand;
			switch (gestures[0].label)
			{
			case PXCGesture::Gesture::LABEL_POSE_THUMB_UP:
				gestureCommand = "stop";
				break;
			case PXCGesture::Gesture::LABEL_POSE_THUMB_DOWN:
				gestureCommand = "stop";
				break;
			case PXCGesture::Gesture::LABEL_POSE_PEACE:
				gestureCommand = "follow me";
				break;
			case PXCGesture::Gesture::LABEL_POSE_BIG5:
				gestureCommand = "stop";
				break;
			case PXCGesture::Gesture::LABEL_HAND_WAVE:
				gestureCommand = "hello";
				break;
			case PXCGesture::Gesture::LABEL_HAND_CIRCLE:
				gestureCommand = "LABEL_HAND_CIRCLE";
				break;
			case PXCGesture::Gesture::LABEL_NAV_SWIPE_LEFT:
				gestureCommand = "turn left";
				break;
			case PXCGesture::Gesture::LABEL_NAV_SWIPE_RIGHT:
				gestureCommand = "turn right";
				break;
			case PXCGesture::Gesture::LABEL_NAV_SWIPE_UP:
				gestureCommand = "faster";
				break;
			case PXCGesture::Gesture::LABEL_NAV_SWIPE_DOWN:
				gestureCommand = "slower";
				break;
			default:
				gestureCommand = "NONE";
				break;
			}
			if(gestureCommand != "NONE")
			{
				printf("Gesture recognized: [%s]", gestureCommand.c_str());

				long tmsec = clock();
				cout << tmsec << endl;

				// Only the gesture is different from last recognized gesture in 2 seconds over the time interval will be processed
				if (gestureCommand != gesturePendding && tmsec - msec > 2000)
				{
					std_msgs::String gestureMsg;
					gestureMsg.data = gestureCommand;
					printf("Gesture pub command: [%s]", gestureCommand.c_str());
					msec = tmsec;
					gesturePendding = gestureCommand;
				}
			}

			//render.RenderFrame(depth);
			pp->ReleaseFrame();
		}
	}

	pp->Close();
	pp->Release();
}

// time
void foo3()
{
	time_t t;  //秒时间
	tm* local; //本地时间 
	tm* gmt;   //格林威治时间
	char buf[128]= {0};

	t = time(NULL); //获取目前秒时间
	local = localtime(&t); //转为本地时间
	strftime(buf, 64, "%Y-%m-%d %H:%M:%S", local);
	std::cout << buf << std::endl;

	gmt = gmtime(&t);//转为格林威治时间
	strftime(buf, 64, "%Y-%m-%d %H:%M:%S", gmt);
	std::cout << buf << std::endl;

	gmt = gmtime(&t);//转为格林威治时间
	strftime(buf, 64, "Today is %A, %d of %B int the year %Y", gmt);
	std::cout << buf << std::endl;

	
	strftime(buf, 64, "Current time is %X", gmt);
 	std::cout << buf << std::endl;

	strftime(buf, 64, " %x", gmt);
	std::cout << buf << std::endl;

	system("pause");
}

int _tmain(int argc, _TCHAR* argv[])
{

	foo3();
	return 0;
}

