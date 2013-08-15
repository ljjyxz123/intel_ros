// test.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
// for ATL::CA2W
#include <atlbase.h>
#include <atlconv.h>

#include "../voice_recognition/VoiceCommands.h"
#include "util_pipeline.h"

using namespace std;
int _tmain(int argc, _TCHAR* argv[])
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
		return -1;
	}

	session->Release();
	system("pause");
	return 0;
}

