/**************************************************************************
***Copyright  (C) Zhejiang University

***All rights reserved

***Author: Chauncey Wang

***Email: wangyb2100@gmail.com

***Date:2014-11

***Description: Read RGB Depth Data from Kinect V2, Which is relized by multiple thread

***Others: There are some memory access conflict in Debug mode, So I really suggest you run in release mode
**************************************************************************/
#include "MultiSourceReader.h"
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include<iomanip>
#include<float.h>

using namespace std;
using namespace cv;
/////set Image is showed or not
#define ALL_IMAGE_SHOW

// Set Thread Handle
HANDLE h_color_handle;
HANDLE h_depth_handle;
HANDLE h_depth2color;
HANDLE h_frame_arrived;
HANDLE h_depth2point_cloud;

HANDLE h_mutex_color;///create mutes to avoid conflict
HANDLE h_mutex_depth;
HANDLE h_mutex_depth2point_cloud;

HANDLE h_mutex_color_for_user; ///avoid user access and update conflict
HANDLE h_mutex_depth_for_user;
HANDLE h_mutex_MapDepthToColor_user;
HANDLE h_mutex_PointCloud_for_user;
#define LENG_OF(a)  sizeof(a)/sizeof(*a)


//***********************set thread ,false means run(Turn is: )************************
//***T_FrameArrivedEventsLoop_End;
//***T_ThreadColor_End;
//***T_ThreadDepth_End;
// ***T_ThreadMapDepthToColor;
// ***T_ThreadMapDepthToPointCloud;
ThreadEndFlagStruct_MultiReader ThreadEndFlag = { false, false, false, false, false }; ///安全退出,可以设置是否启动该线程（true为线程退出）




/*************************************************
***Function
***Description:  constructor
***Input:         
***Output:        
***Return:         
***Others:  initialization some pointers
*************************************************/
MyMultiSourceReader::MyMultiSourceReader()
{

	m_pKinectSensor = nullptr;
	m_pMultiSourceFrameReader = nullptr;
	m_pCoordinateMapper = nullptr;
	m_hMultiSourceFrameArrived = 0;
	m_hCoordinateMapperChanged = 0;

	m_pMapToColorCoordinates = new ColorSpacePoint[cDepthWidth*cDepthHeight];
	DepthMapedColorImage = new RGBQUAD[cDepthHeight*cDepthWidth];
	m_pMapToCameraPoints = new CameraSpacePoint[cDepthWidth*cDepthHeight];

	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];// create heap storage for color pixel data in RGBX format
	m_pDepthGray = new USHORT[cDepthWidth*cDepthHeight];
}
/*************************************************
***Function
***Description:  destructor
***Input:
***Output:
***Return:
***Others:  close all threads, delete pointers
*************************************************/
MyMultiSourceReader::~MyMultiSourceReader()
{
	ThreadEndFlag = { true, true, true, true, true };
	cv::destroyAllWindows();
	if (m_pColorRGBX)
	{
		delete[] m_pColorRGBX;
		m_pColorRGBX = nullptr;
	}
	if (m_pDepthGray)
	{
		delete[] m_pDepthGray;
		m_pDepthGray = nullptr;
	}
	if (m_pMapToColorCoordinates)
	{
		delete[] m_pMapToColorCoordinates;
		m_pMapToColorCoordinates = nullptr;
	}
	if (DepthMapedColorImage)
	{
		delete[] DepthMapedColorImage;
		DepthMapedColorImage = nullptr;
	}
	if (m_pMapToCameraPoints)
	{
		delete[] m_pMapToCameraPoints;
		m_pMapToCameraPoints = nullptr;
	}
	if (m_hMultiSourceFrameArrived&&m_pMultiSourceFrameReader)
	{
		m_pMultiSourceFrameReader->UnsubscribeMultiSourceFrameArrived(m_hMultiSourceFrameArrived);
		m_hMultiSourceFrameArrived = 0;
	}
	if (m_hCoordinateMapperChanged&&m_pCoordinateMapper)
	{
		m_pCoordinateMapper->UnsubscribeCoordinateMappingChanged(m_hCoordinateMapperChanged);
		m_hCoordinateMapperChanged = 0;
	}
	SafeRelease(m_pMultiSourceFrameReader);
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}
	SafeRelease(m_pCoordinateMapper);
	SafeRelease(m_pKinectSensor);

}
/*************************************************
***Function
***Description:  Kinnect初始化和执行函数
***Input:           none
***Output:
***Return:          if successful, return true
***Others:     涉及线程的创建，挂起和唤醒
*************************************************/
bool  MyMultiSourceReader::InitKinectAndRun()
{
	h_frame_arrived = CreateThread(NULL, 0, MyMultiSourceReader::FrameArrivedEventsLoop, this, CREATE_SUSPENDED, NULL); 
	h_depth2color = CreateThread(NULL, 0, MyMultiSourceReader::MapDepthToColor, this, CREATE_SUSPENDED, NULL);
	h_depth2point_cloud = CreateThread(NULL, 0, MyMultiSourceReader::MapDepthToPointCloud, this, CREATE_SUSPENDED, NULL);
	h_mutex_depth2point_cloud = CreateMutex(NULL, false, NULL);

	h_color_handle = CreateThread(NULL, 0, MyMultiSourceReader::ThreadColor, this, CREATE_SUSPENDED, NULL);
	h_mutex_color = CreateMutex(NULL, false, NULL);
	h_depth_handle = CreateThread(NULL, 0, MyMultiSourceReader::ThreadDepth, this, CREATE_SUSPENDED, NULL);
	h_mutex_depth = CreateMutex(NULL, false, NULL);

	h_mutex_color_for_user = CreateMutex(NULL, false, NULL);  ////avoid 用户访问与 数据帧更新冲突
	h_mutex_depth_for_user = CreateMutex(NULL, false, NULL);
	h_mutex_PointCloud_for_user = CreateMutex(NULL, false, NULL);
	h_mutex_MapDepthToColor_user = CreateMutex(NULL, false, NULL);

	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return false;
	}
	if (m_pKinectSensor)
	{
		hr = m_pKinectSensor->Open();
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_Depth, &m_pMultiSourceFrameReader);	/// different from before , get _reader directly
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pMultiSourceFrameReader->SubscribeMultiSourceFrameArrived(&m_hMultiSourceFrameArrived);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);

		}
		if (SUCCEEDED(hr))
		{
			hr = m_pCoordinateMapper->SubscribeCoordinateMappingChanged(&m_hCoordinateMapperChanged);
		}
	}
	if (!m_pKinectSensor || FAILED(hr))
	{
		printf("No ready Kinect found! \n");
		return false;
	}
	else
	{

		ResumeThread(h_frame_arrived);  ///very important for run
		Sleep(3000);  ///等待初始化成功
		return true;
	}

}
/*************************************************
***Function
***Description: 新的帧数据来临
***Input:           MyMultiSourceReader 对象指针
***Output:
***Return:
***Others:     调用更新帧数据函数
*************************************************/
DWORD WINAPI MyMultiSourceReader::FrameArrivedEventsLoop(LPVOID  thread_para)
{
	MyMultiSourceReader *p_object = (MyMultiSourceReader*)thread_para;
	HANDLE events[] = {
		reinterpret_cast<HANDLE>(p_object->m_hMultiSourceFrameArrived)
		// reinterpret_cast<HANDLE>(m_hCoordinateMapperChanged),
	};
	int Events_Num = LENG_OF(events);
	DWORD UpdateNum = 0;
	while (!ThreadEndFlag.T_FrameArrivedEventsLoop_End) ///在全局变量初始化
	{

		if (MsgWaitForMultipleObjects(LENG_OF(events), events, FALSE, INFINITE, QS_ALLINPUT) == (WAIT_OBJECT_0 + 0))///msg函数返回WAIT_OBJECT_0 + 0~WAIT_OBJECT_0 +Events_Num-1（区分不同的事件） 
		{
			UpdateNum++;
			if (UpdateNum%15==0)
			{
				printf("New Frame......\n");
			}	
			///about 30ms Multiple Frame Events arrive(namely fps--30)
			p_object->UpdateFrame();//// (This function costs about 3.7ms)	
		}
		else
			Sleep(1);

	}
	return 0;
}
/*************************************************
***Function
***Description: 更新帧
***Input:
***Output:
***Return:
***Others:     获取复源帧数据，数据转换线程(all unused Kinect pointer need SafeRelease!(IColorFrame and IDepthFrame must release))
*************************************************/
bool  MyMultiSourceReader::UpdateFrame()
{
	if (!m_pMultiSourceFrameReader)
	{
		return false;
	}
	// 复源临帧事件参数
	IMultiSourceFrameArrivedEventArgs* pArgs = nullptr;
	// 复源帧引用
	IMultiSourceFrameReference* pMSFrameRef = nullptr;
	IMultiSourceFrame*			m_pMultiSourceFrame = nullptr;
	HRESULT  hr = m_pMultiSourceFrameReader->GetMultiSourceFrameArrivedEventData(m_hMultiSourceFrameArrived, &pArgs);
	if (SUCCEEDED(hr)) {
		hr = pArgs->get_FrameReference(&pMSFrameRef);
	}
	if (SUCCEEDED(hr)) {
		hr = pMSFrameRef->AcquireFrame(&m_pMultiSourceFrame);
	}
	if (SUCCEEDED(hr))
	{
		////*******************Access Color Camera Stream********************************
		IColorFrameReference * m_pColorFrameReference_temp = nullptr;
		IColorFrame* m_pColorFrame_temp = nullptr;
		IFrameDescription* m_pColorFrameDescription_temp = nullptr;

		ColorImageFormat color_imageFormat = ColorImageFormat_None;

		UINT nColorBufferSize = 0;

		hr = m_pMultiSourceFrame->get_ColorFrameReference(&m_pColorFrameReference_temp);
		if (SUCCEEDED(hr))
		{
			hr = m_pColorFrameReference_temp->AcquireFrame(&m_pColorFrame_temp);

			SafeRelease(m_pColorFrameReference_temp);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pColorFrame_temp->get_FrameDescription(&m_pColorFrameDescription_temp);
		}


		WaitForSingleObject(h_mutex_color, INFINITE);

		if (SUCCEEDED(hr))
		{
			hr = m_pColorFrameDescription_temp->get_Width(&get_color_width);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pColorFrameDescription_temp->get_Height(&get_color_height);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pColorFrame_temp->get_RawColorImageFormat(&color_imageFormat);
		}

		if (SUCCEEDED(hr))
		{
			if (color_imageFormat == ColorImageFormat_Bgra)
			{

				p_color_buffer = nullptr;
				hr = m_pColorFrame_temp->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&p_color_buffer)); ///so pBuffer contains the color frame data

				ReleaseMutex(h_mutex_color);
				ResumeThread(h_color_handle); ////begin another thread

			}
			else if (m_pColorRGBX) ///  if the color data isn't bgra format, then you can convert it (the data is saved in "pBuffer")
			{

				p_color_buffer = nullptr;
				p_color_buffer = m_pColorRGBX;////  then p_color_buffer point to the same memory space as m_pColorRGBX
				nColorBufferSize = get_color_width * get_color_height * sizeof(RGBQUAD);   ////nColorBufferSize=1920*1080*4=8294400(表示的是字节数，与深度buffer不同)
				hr = m_pColorFrame_temp->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(p_color_buffer), ColorImageFormat_Bgra); 	///so p_color_buffer contains the color frame data

				ReleaseMutex(h_mutex_color);
				ResumeThread(h_color_handle); ////begin another thread

			}
			else
			{
				hr = E_FAIL;
			}
		}/// end of color format conversion 
		if (SUCCEEDED(hr))
		{

			SafeRelease(m_pColorFrameDescription_temp);
			SafeRelease(m_pColorFrame_temp);
		}

		////*******************Access Depth Camera Stream********************************
		IDepthFrame* m_pDepthFrame_temp = nullptr;
		IDepthFrameReference *m_pDepthFrameReference_temp = nullptr;
		IFrameDescription* m_pDepthFrameDescription_temp = nullptr;

		UINT nDepthBufferSize = 0;///max is 2^32-1(about 4*10^9)

		if (SUCCEEDED(hr))
		{

			hr = m_pMultiSourceFrame->get_DepthFrameReference(&m_pDepthFrameReference_temp);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrameReference_temp->AcquireFrame(&m_pDepthFrame_temp);
			SafeRelease(m_pDepthFrameReference_temp);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrame_temp->get_FrameDescription(&m_pDepthFrameDescription_temp);
		}

		WaitForSingleObject(h_mutex_depth, INFINITE);

		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrameDescription_temp->get_Width(&get_depth_width);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrameDescription_temp->get_Height(&get_depth_height);
		}
		if (SUCCEEDED(hr))
		{

			p_depth_buffer = nullptr;
			///p_depth_buffer指向了二维数组大小空间;nDepthBufferSize=217008=512*424(USHORT 占2个字节，但nDepthBufferSize不表示字节数)
			hr = m_pDepthFrame_temp->AccessUnderlyingBuffer(&nDepthBufferSize, &p_depth_buffer);

		}
		if (SUCCEEDED(hr) && get_depth_height == cDepthHeight&&get_depth_width == cDepthWidth)
		{
			hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(
				cDepthHeight*cDepthWidth, (UINT16*)p_depth_buffer, cDepthHeight*cDepthWidth, m_pMapToColorCoordinates);
		}
		if (SUCCEEDED(hr))
		{
			WaitForSingleObject(h_mutex_depth2point_cloud, INFINITE);
			hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(
				cDepthHeight*cDepthWidth, p_depth_buffer, cDepthHeight*cDepthWidth, m_pMapToCameraPoints);
			ReleaseMutex(h_mutex_depth2point_cloud);
		}
		ReleaseMutex(h_mutex_depth);
		ResumeThread(h_depth_handle);

		ResumeThread(h_depth2point_cloud);
		if (SUCCEEDED(hr))
		{
			ColorSpacePoint color_space_point_temp;
			int x, y;
			RGBQUAD rgb_temp;
			///for the safe of thread access
			WaitForSingleObject(h_mutex_MapDepthToColor_user, INFINITE);
			for (UINT i = 0; i < cDepthHeight*cDepthWidth; i++)
			{
				color_space_point_temp = m_pMapToColorCoordinates[i];
				x = static_cast<int>(floor(color_space_point_temp.X + .5F));///to the nearest integer
				y = static_cast<int>(floor(color_space_point_temp.Y + .5F));
				if (x >= 0 && x < get_color_width && y >= 0 && y < get_color_height)////x's, y's meaning similar to Opencv
				{
					rgb_temp = p_color_buffer[y*get_color_width + x];
				}
				else
				{
					rgb_temp = { 0, 0, 0, 0 };
				}
				DepthMapedColorImage[i] = rgb_temp;
			}
			ResumeThread(h_depth2color);
		}
		ReleaseMutex(h_mutex_MapDepthToColor_user);

		if (SUCCEEDED(hr))
		{

			SafeRelease(m_pDepthFrameDescription_temp);
			SafeRelease(m_pDepthFrame_temp);

		}
	}///end of color and depth frame processed

	SafeRelease(m_pMultiSourceFrame);
	SafeRelease(pArgs);
	SafeRelease(pMSFrameRef);

	if (SUCCEEDED(hr))
	{
		return true;
	}
	else
		return false;
}
/*************************************************
***Function
***Description: Color Image 数据转换为Mat并显示
***Input:
***Output:
***Return:
***Others:     可选择关闭显示窗口
*************************************************/
DWORD WINAPI MyMultiSourceReader::ThreadColor(LPVOID  thread_para)
{
	MyMultiSourceReader *p_object = (MyMultiSourceReader*)thread_para;

	while (!ThreadEndFlag.T_ThreadColor_End)
	{
		WaitForSingleObject(h_mutex_color, INFINITE);
		Mat ColorImageCopy;
		if (p_object->GetColorBuffer() && p_object->get_color_width == cColorWidth&& p_object->get_color_height == cColorHeight)
		{
			int height = p_object->get_color_height;
			int width = p_object->get_color_width;
			RGBQUAD* buffer_temp = p_object->GetColorBuffer();
			Mat ColorImage(height, width, CV_8UC4, buffer_temp);
			ColorImageCopy = ColorImage;
			WaitForSingleObject(h_mutex_color_for_user, INFINITE);
			p_object->m_ColorImage = ColorImage.clone();  ////user copy
			ReleaseMutex(h_mutex_color_for_user);
		}
		ReleaseMutex(h_mutex_color);
#ifdef ALL_IMAGE_SHOW
		if (!ColorImageCopy.empty())
		{
			namedWindow("ColorImage", WINDOW_NORMAL);
			imshow("ColorImage", ColorImageCopy);
			waitKey(1);
		}
#endif
		SuspendThread(h_color_handle);/// 
	}
	return 0;
}
/*************************************************
***Function
***Description: 用户获取数据接口
***Input:
***Output:
***Return:
***Others:
*************************************************/
Mat  MyMultiSourceReader::UserGetColorImage() const
{
	WaitForSingleObject(h_mutex_color_for_user, INFINITE);
	Mat  ColorImage_temp = m_ColorImage.clone();
	ReleaseMutex(h_mutex_color_for_user);
	return ColorImage_temp;
}
/*************************************************
***Function
***Description: 深度数据转换接口
***Input:  传入对象指针
***Output:
***Return:
***Others:
*************************************************/
DWORD WINAPI MyMultiSourceReader::ThreadDepth(LPVOID thread_para)
{
	MyMultiSourceReader *p_object = (MyMultiSourceReader*)thread_para;
	while (!ThreadEndFlag.T_ThreadDepth_End)
	{
		if (p_object->GetDepthBuffer() && p_object->get_depth_height == cDepthHeight&&p_object->get_depth_width == cDepthWidth)
		{
			WaitForSingleObject(h_mutex_depth, INFINITE);
			USHORT* p_depth_temp = p_object->GetDepthBuffer();
			Mat DepthImage(cDepthHeight, cDepthWidth, CV_16UC1, p_depth_temp);
			WaitForSingleObject(h_mutex_depth_for_user, INFINITE);
			p_object->m_DepthImage = DepthImage.clone();
			ReleaseMutex(h_mutex_depth_for_user);
			Mat depth_show=DepthImage.clone();
			ReleaseMutex(h_mutex_depth);////less than 12ms
#ifdef ALL_IMAGE_SHOW
			if (!depth_show.empty())
			{
				depth_show.convertTo(depth_show, CV_16UC1, 15, 0);
				cv::namedWindow("DepthImage", WINDOW_NORMAL);
				cv::imshow("DepthImage", depth_show);
				waitKey(1);
			}
#endif
		}
		SuspendThread(h_depth_handle);
	}
	return 0;
}

Mat  MyMultiSourceReader::UserGetDepthImage() const
{
	WaitForSingleObject(h_mutex_depth_for_user, INFINITE);
	Mat depth_temp = m_DepthImage.clone();
	ReleaseMutex(h_mutex_depth_for_user);
	return depth_temp;
}
/*************************************************
***Function
***Description: 深度与ColorRGB坐标映射
***Input:
***Output:
***Return:
***Others:
*************************************************/
DWORD WINAPI MyMultiSourceReader::MapDepthToColor(LPVOID thread_para)
{
	MyMultiSourceReader *p_object = (MyMultiSourceReader*)thread_para;
	while (!ThreadEndFlag.T_ThreadMapDepthToColor)
	{
		WaitForSingleObject(h_mutex_MapDepthToColor_user, INFINITE);
		Mat  mapImage(cDepthHeight, cDepthWidth, CV_8UC4, p_object->DepthMapedColorImage);
		p_object->m_DepthMapedColorImage = mapImage.clone();
		Mat DepthMapedColorImage = p_object->m_DepthMapedColorImage.clone();
		ReleaseMutex(h_mutex_MapDepthToColor_user);
		
#ifdef ALL_IMAGE_SHOW
		if (!DepthMapedColorImage.empty())
		{
			namedWindow("MapColorImage", WINDOW_NORMAL);
			imshow("MapColorImage", DepthMapedColorImage);
			waitKey(1);
		}

#endif

		SuspendThread(h_depth2color);
	}
	return 0;
}
/*************************************************
***Function
***Description: 获取映射后的Color Image
***Input:
***Output:
***Return:
***Others:
*************************************************/
Mat  MyMultiSourceReader::UserGetMapedColorImage() const
{
	WaitForSingleObject(h_mutex_MapDepthToColor_user, INFINITE);
	Mat MapedColorImage = m_DepthMapedColorImage.clone();
	ReleaseMutex(h_mutex_MapDepthToColor_user);
	return MapedColorImage;
}
/*************************************************
***Function
***Description:Acquire Point Cloud XYZ
***Input:
***Output:
***Return:
***Others: unit is “m”, data is saved in m_PointCloud
*************************************************/
DWORD WINAPI MyMultiSourceReader::MapDepthToPointCloud(LPVOID thread_para)
{
	MyMultiSourceReader *p_object = (MyMultiSourceReader*)thread_para;
	static int64  loop_num = 0;
	static  int   file_num = 0;
	int sum_num = cDepthWidth*cDepthHeight;
	stringstream num2str;
	string temp;
	ofstream fout;
	while (!ThreadEndFlag.T_ThreadMapDepthToPointCloud)
	{
		int64 start1 = getTickCount();
		CameraSpacePoint * camera_point_copy = new CameraSpacePoint[sum_num];
		WaitForSingleObject(h_mutex_depth2point_cloud, INFINITE);
		memmove(camera_point_copy, p_object->m_pMapToCameraPoints, sizeof(CameraSpacePoint)*sum_num);
		ReleaseMutex(h_mutex_depth2point_cloud);
		////////********************write in a Mat--3 channels , max cost 16ms *************
		Mat point_cloud(cDepthHeight, cDepthWidth, CV_32FC3, Scalar(0, 0, 0)); ////unit is "m"
		int depth_index;
		for (int r_d = 0; r_d < cDepthHeight; r_d++)
		{
			for (int c_d = 0; c_d < cDepthWidth; c_d++)
			{
				cv::Vec3f *RowData = point_cloud.ptr<Vec3f>(r_d);
				depth_index = r_d*cDepthWidth + c_d;
				if (!_isnan(camera_point_copy[depth_index].X) && _finite(camera_point_copy[depth_index].X) && !_isnan(camera_point_copy[depth_index].Y) && _finite(camera_point_copy[depth_index].Y) && !_isnan(camera_point_copy[depth_index].Z) && _finite(camera_point_copy[depth_index].Z))
				{
					RowData[c_d] = cv::Vec3f(camera_point_copy[depth_index].X, camera_point_copy[depth_index].Y, camera_point_copy[depth_index].Z);
				}
			}
		}
		delete[] camera_point_copy;
		WaitForSingleObject(h_mutex_PointCloud_for_user, INFINITE);
		p_object->m_PointCloud = point_cloud.clone();
		ReleaseMutex(h_mutex_PointCloud_for_user);
		SuspendThread(h_depth2point_cloud);
	}
	return  0;
}
/*************************************************
***Function
***Description: For User Acquire Point Cloud Data
***Input:
***Output:
***Return:
***Others: unit is “m”
*************************************************/
Mat  MyMultiSourceReader::UserGetPointCloud() const
{
	WaitForSingleObject(h_mutex_PointCloud_for_user, INFINITE);
	Mat  point_cloud_temp = m_PointCloud.clone();
	ReleaseMutex(h_mutex_PointCloud_for_user);
	return point_cloud_temp;
}