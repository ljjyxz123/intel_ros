#include "StdAfx.h"
#include "VoiceSynthesis.h"
#include "voice_out.h"
//#include <iostream>
// for ATL::CA2W
#include <atlbase.h>

VoiceSynthesis::VoiceSynthesis()
{
	//setlocale(LC_CTYPE, ".936");

	// create session
	pxcStatus status;
	status = PXCSession_Create(&session);
	if (status < PXC_STATUS_NO_ERROR) throw "PXCSession_Create failed";

	// impl synthesis
	status = session->CreateImpl<PXCVoiceSynthesis>(&synth);
	if (status<PXC_STATUS_NO_ERROR) throw "CreateImpl<PXCVoiceSynthesis> failed";

	// set pinfo
	synth->QueryProfile(0, &pinfo);
	synth->SetProfile(&pinfo);
	if (status<PXC_STATUS_NO_ERROR) throw "QueryProfile or SetProfile failed";
}

VoiceSynthesis::~VoiceSynthesis()
{
}

void VoiceSynthesis::dealInfo(char* info)
{
	printf_s("[INFO]: %s\n", info);
}

void VoiceSynthesis::dealError(char* error)
{
	printf_s("[ERROR]: %s\n", error);
}

void VoiceSynthesis::speak(pxcCHAR* sentence)
{
	pxcStatus status;
	pxcUID sid = 0;
	synth->QueueSentence(sentence, wcslen(sentence), &sid);
	dealInfo(ATL::CW2A(sentence));
	VoiceOut vo(&pinfo);
	for (;;) {
		PXCSmartSP sp;
		PXCAudio *sample;

		// Request audio data from TTS
		status = synth->ProcessAudioAsync(sid, &sample, &sp);
		if (status<PXC_STATUS_NO_ERROR) break;

		// Make sure there is data that is valid
		status = sp->Synchronize();
		if (status<PXC_STATUS_NO_ERROR) break;

		// send sample to audio output device
		vo.RenderAudio(sample);
	}
}

void VoiceSynthesis::speak(const char* sentence)
{
	speak(ATL::CA2W(sentence));
}

//void VoiceSynthesis::speak(std::string sentence)
//{
//	speak(ATL::CA2W(sentence.c_str()));
//}