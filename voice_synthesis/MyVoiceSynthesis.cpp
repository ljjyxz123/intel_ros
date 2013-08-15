#include "MyVoiceSynthesis.h"
#include "voice_out.h"
#include "mmsystem.h"
#pragma comment(lib,"winmm.lib")

// for ATL::CA2W
#include <atlbase.h>
#include <atlconv.h>

MyVoiceSynthesis::MyVoiceSynthesis(void)
{
	// create session
	status = PXCSession_Create(&session);
	if (status < PXC_STATUS_NO_ERROR) throw "PXCSession_Create failed";

	// impl synthesis
	status = session->CreateImpl<PXCVoiceSynthesis>(&synth);
	if (status<PXC_STATUS_NO_ERROR) throw "CreateImpl<PXCVoiceSynthesis> failed";

	// set pinfo
	synth->QueryProfile(0,&pinfo);
	synth->SetProfile(&pinfo);
	if (status<PXC_STATUS_NO_ERROR) throw "QueryProfile or SetProfile failed";
}


MyVoiceSynthesis::~MyVoiceSynthesis(void)
{
}

void MyVoiceSynthesis::speak(wchar_t* sentence)
{
	pxcUID sid;
	synth->QueueSentence(sentence, sizeof(sentence)/sizeof(pxcCHAR), &sid);

	VoiceOut vo(&pinfo);
	for (;;) {
		PXCSmartSP sp;
		PXCAudio *sample;

		// Request audio data from TTS
		status=synth->ProcessAudioAsync(sid,&sample,&sp);
		if (status<PXC_STATUS_NO_ERROR) break;

		// Make sure there is data that is valid
		status = sp->Synchronize();
		if (status<PXC_STATUS_NO_ERROR) break;

		// send sample to audio output device
		vo.RenderAudio(sample);
	}
}


void MyVoiceSynthesis::speak(char* sentence)
{
	speak(ATL::CA2W(sentence));
}


void MyVoiceSynthesis::speak(std::string sentence)
{
	speak(ATL::CA2W(sentence.c_str()));
}
