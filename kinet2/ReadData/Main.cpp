//  this is an example, need Opencv 3.x
#include "MultiSourceReader.h"
#include <iostream>
using namespace std;
using namespace cv;

extern   ThreadEndFlagStruct_MultiReader ThreadEndFlag;

int  main()
{
	//ThreadEndFlag.T_ThreadMapDepthToPointCloud = true;  ///false--begin the point cloud generation, true means close the thread of PointCloud Thread
	//ThreadEndFlag.T_ThreadMapDepthToColor = true;  ////false--means begin the DepthMapedColorImage generation
	//ThreadEndFlag.T_ThreadColor_End = false; /// false means begins the color image generation
	//ThreadEndFlag.T_ThreadDepth_End = false;///false means begins the Depth image generation
	MyMultiSourceReader   multi_reader;
	multi_reader.InitKinectAndRun();
	while (true)
	{
		Sleep(50);
		cv::Mat  ColorImg= multi_reader.UserGetColorImage();  ///CV_8UC4
		cv::Mat  MapedColorImg = multi_reader.UserGetMapedColorImage(); //CV_8UC4 Unit is "mm"
		cv::Mat  DepthImg = multi_reader.UserGetDepthImage();///CV_16CU1
		cv::Mat  PointCloudImg = multi_reader.UserGetPointCloud(); //CV_32FC3  Unit is "m"
		if (!ColorImg.empty() && !DepthImg.empty()&&!PointCloudImg.empty() &&! MapedColorImg.empty())
		{
			printf("Receive Data!\n");
		}
		else
		{
			//printf("return none!\n");
		}
	}
	return 0;
}
