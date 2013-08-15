// rgbd_collector.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

// Intel includes
#include "pxcsession.h"
#include "pxcsmartptr.h"
#include "pxccapture.h"
#include "util_render.h"
#include "util_capture_file.h"
#include "util_pipeline.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

// Ros includes
#include <ros/ros.h>

int _tmain(int argc, char** argv)
{
	// Init ros
	ros::init(argc, argv, "rgbd_collector");
	ros::NodeHandle node;
	int rate;
	if (!node.getParam("rgbd_collector/rate", rate))
	{
		ROS_WARN("Param [%s] not found.", "rgbd_collector/rate");
		rate = 5;
	}
	ROS_INFO("Set rate to [%d]", rate);
	ros::Publisher depthPub = node.advertise<sensor_msgs::Image>("/camera/depth/image_raw", 100);
	ros::Publisher rgbPub = node.advertise<sensor_msgs::Image>("/camera/rgb/image_raw", 100);
	ros::Publisher depthCameraInfoPub = node.advertise<sensor_msgs::CameraInfo>("/camera/depth/camera_info", 100);
	ros::Rate loopRate(rate); // pub rate at 20Hz;

	// Init pipeline
	UtilPipeline pp;
	pp.EnableImage(PXCImage::COLOR_FORMAT_RGB32);
	pp.EnableImage(PXCImage::COLOR_FORMAT_DEPTH);
 	pp.Init();
	UtilRender intelRgbRender(L"Color Stream");
	UtilRender intelDepthRender(L"Depth Stream");
	UINT32 seq = 0;
	while(ros::ok())
	{
		if (!pp.AcquireFrame(true)) break;
		PXCImage *pPxcRgbImg=pp.QueryImage(PXCImage::IMAGE_TYPE_COLOR);
		PXCImage *pPxcDepthImg=pp.QueryImage(PXCImage::IMAGE_TYPE_DEPTH);

		PXCImage::ImageData pxcRgbData;
		PXCImage::ImageData pxcDepthData;
		pPxcRgbImg->AcquireAccess(PXCImage::ACCESS_READ_WRITE, PXCImage::COLOR_FORMAT_RGB24, &pxcRgbData);
		pPxcDepthImg->AcquireAccess(PXCImage::ACCESS_READ,&pxcDepthData);

		PXCImage::ImageInfo pxcRgbInfo;
		PXCImage::ImageInfo pxcDepthInfo;
		pPxcRgbImg->QueryInfo(&pxcRgbInfo);
		pPxcDepthImg->QueryInfo(&pxcDepthInfo);

		// Convert Intel rgb to Ros rgb image
		UCHAR* pRgbData = pxcRgbData.planes[0];
		std_msgs::Header header;
		header.seq = seq;
		header.stamp = ros::Time::now();
		sensor_msgs::Image rosRgbImg;
		rosRgbImg.header = header;
		rosRgbImg.step = pxcRgbInfo.width * sizeof(UCHAR) * 3;
		rosRgbImg.width = pxcRgbInfo.width;
		rosRgbImg.height = pxcRgbInfo.height;
		rosRgbImg.is_bigendian = false;
		rosRgbImg.encoding = sensor_msgs::image_encodings::BGR8;
		size_t sizeRgb = rosRgbImg.step * rosRgbImg.height;
		rosRgbImg.data.resize(sizeRgb);
		memcpy((char*)(&rosRgbImg.data[0]), pRgbData, sizeRgb);

		// Convert Intel depth to Ros depth image
		uint16_t* pDepthData=(uint16_t*)pxcDepthData.planes[0];
		sensor_msgs::CameraInfo rosDepthCameraInfo;
		rosDepthCameraInfo.header = header;
		rosDepthCameraInfo.width = pxcDepthInfo.width;
		rosDepthCameraInfo.height = pxcDepthInfo.height;
		sensor_msgs::Image rosDpthImg;
		rosDpthImg.header = header;
		rosDpthImg.step = pxcDepthInfo.width * sizeof(uint16_t);
		rosDpthImg.width = pxcDepthInfo.width;
		rosDpthImg.height = pxcDepthInfo.height;
		rosDpthImg.is_bigendian = false;
		rosDpthImg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
		size_t sizeDepth = rosDpthImg.step * rosDpthImg.height;
		rosDpthImg.data.resize(sizeDepth);
		memcpy((char*)(&rosDpthImg.data[0]), pDepthData, sizeDepth);

		pPxcRgbImg->ReleaseAccess(&pxcRgbData);
		pPxcDepthImg->ReleaseAccess(&pxcDepthData);

		if (!intelRgbRender.RenderFrame(pPxcRgbImg)) break;
		if (!intelDepthRender.RenderFrame(pPxcDepthImg)) break;
		pp.ReleaseFrame();

		// Ros pub
		rgbPub.publish(rosRgbImg);
		depthPub.publish(rosDpthImg);
		depthCameraInfoPub.publish(rosDepthCameraInfo);
		ROS_INFO("Pub image seq: [%d]", seq);
		loopRate.sleep();
		seq++;
	}

  	pp.Close();

	system("pause");
	return 0;
}

