/***All rights reserved

***Author: Chauncey Wang

***Email: wangyb2100@gmail.com

***Date:2014-11

***Description: Read RGB Depth Data from Kinect V2, Which is relized by multiple thread

***Others: There are some memory access conflict in Debug mode, So I really suggest you run in release mode
***  need Opencv 3.x
**************************************************************************/
#ifndef _MULTI_SOURCE_READER_H
#define _MULTI_SOURCE_READER_H
#include <Kinect.h>
#include <windows.h>
#include <opencv.hpp>
struct ThreadEndFlagStruct_MultiReader
{
	bool T_FrameArrivedEventsLoop_End;
	bool T_ThreadColor_End;
	bool T_ThreadDepth_End;
	bool T_ThreadMapDepthToColor;
	bool  T_ThreadMapDepthToPointCloud;
};
class MyMultiSourceReader
{
public:
	MyMultiSourceReader();
	~MyMultiSourceReader();

	bool  InitKinectAndRun();
	bool  UpdateFrame();

	static DWORD WINAPI FrameArrivedEventsLoop(LPVOID  thread_para);
	static DWORD WINAPI ThreadColor(LPVOID  thread_para);
	static DWORD WINAPI ThreadDepth(LPVOID thread_para);
	static DWORD WINAPI MapDepthToColor(LPVOID thread_para);
	static  DWORD WINAPI  MapDepthToPointCloud(LPVOID thread_para);

	cv::Mat UserGetColorImage() const;  //CV_8UC4
	cv::Mat UserGetDepthImage()const; //CV_16UC1 ,Uint is mm
	cv::Mat  UserGetPointCloud()const; ////CV_32FC3 ,Unit is M
	cv::Mat  UserGetMapedColorImage() const;///CV_8UC4

	inline  RGBQUAD*  GetColorBuffer() const
	{
		return p_color_buffer;
	}
	inline  int const & GetColorWidth() const
	{
		return get_color_width;
	}
	inline  int const & GetColorHeight() const
	{
		return get_color_height;
	}

	inline USHORT*& GetDepthBuffer()
	{
		return p_depth_buffer;
	}
	inline int const &GetDepthHeight() const
	{
		return get_depth_height;
	}
	inline int const &GetDepthWidth() const
	{
		return get_depth_width;
	}
	
private:
	IKinectSensor*          m_pKinectSensor;
	IMultiSourceFrameReader *		m_pMultiSourceFrameReader; /// always exist when program runing

	ICoordinateMapper*			m_pCoordinateMapper;
	
	WAITABLE_HANDLE             m_hMultiSourceFrameArrived;  ////when Arrived this value always be a const(like 600)
	
	WAITABLE_HANDLE             m_hCoordinateMapperChanged;

	CameraSpacePoint*			m_pMapToCameraPoints;///user can get XYZ coordinate by index in depth image
	ColorSpacePoint*            m_pMapToColorCoordinates;///user can establish relationship between coordinate x,y in depth and to u,v in color image

	RGBQUAD*		m_pColorRGBX;
	RGBQUAD *		p_color_buffer;
	RGBQUAD *	   DepthMapedColorImage;/// store depth mapped to color space image
	cv::Mat			m_ColorImage; //// for  users
	cv::Mat         m_DepthMapedColorImage; ////resolution 512*424  //for user
	int			get_color_width;
	int			get_color_height;

	USHORT*			m_pDepthGray;  /// new 512*424 size space to store the depth image
	cv::Mat			m_DepthImage;  ///for users 
	USHORT *		p_depth_buffer;  /// it's every frame's pointer, but we don't know the frame size(according to variable "get_depth_width, get_depth_height")
	int get_depth_width;
	int get_depth_height;

	cv::Mat			m_PointCloud;//// for users


	static const int        cColorWidth = 1920;  ///max is 1920 * 1080
	static const int        cColorHeight = 1080;
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
	static const int DepthMinReliableDistance = 500; /// if the depth resolution is fixed
	static const int DepthMaxReliableDistance = 4500;

};

/*************************************************
***Function
***Description:  
***Input:          
***Output:        void
***Return:         NULL
***Others:
*************************************************/
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != nullptr)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = nullptr;
	}
}


#endif