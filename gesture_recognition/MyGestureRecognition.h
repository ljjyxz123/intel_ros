#pragma once
#include "util_pipeline.h"
#include "util_render.h"
#include "pxcgesture.h"

class MyGestureRecognition :
	public UtilPipeline
{
public:
	MyGestureRecognition(void);
	~MyGestureRecognition(void);
	virtual void PXCAPI OnGesture(PXCGesture::Gesture* data);
	virtual void PXCAPI OnAlert(PXCGesture::Alert* data);
	virtual bool OnNewFrame(void);
protected:
	UtilRender m_render;
	PXCGesture::Gesture m_gdata;
};

