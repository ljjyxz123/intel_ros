#include "MyGestureRecognition.h"


MyGestureRecognition::MyGestureRecognition(void)
{
	EnableGesture();
}


MyGestureRecognition::~MyGestureRecognition(void)
{
}


void MyGestureRecognition::OnGesture(PXCGesture::Gesture* data)
{
	if (data->active) m_gdata = (*data);
}


void MyGestureRecognition::OnAlert(PXCGesture::Alert* data)
{
	switch (data->label) {
	case PXCGesture::Alert::LABEL_FOV_TOP:
		wprintf_s(L"******** Alert: Hand touches the TOP boundary!\n");
		break;
	case PXCGesture::Alert::LABEL_FOV_BOTTOM:
		wprintf_s(L"******** Alert: Hand touches the BOTTOM boundary!\n");
		break;
	case PXCGesture::Alert::LABEL_FOV_LEFT:
		wprintf_s(L"******** Alert: Hand touches the LEFT boundary!\n");
		break;
	case PXCGesture::Alert::LABEL_FOV_RIGHT:
		wprintf_s(L"******** Alert: Hand touches the RIGHT boundary!\n");
		break;
	}
}


bool MyGestureRecognition::OnNewFrame(void)
{
	PXCGesture *gesture = QueryGesture();
	//m_render.RenderFrame(QueryImage(PXCImage::IMAGE_TYPE_DEPTH));
	return true;
}
