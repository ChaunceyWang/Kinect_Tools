#include <iostream>
#include <fstream>  //读取txt文件
#include <string>

#include <XnCppWrapper.h>
#include "opencv.hpp"

using namespace std;
using namespace cv ;
void CheckOpenNIError( XnStatus eResult, string sStatus );


int main( int argc, char** argv )
{
	XnStatus eResult = XN_STATUS_OK;
	// 2. initial context
	xn::Context mContext;         //物件：用來管理整個 OpenNI 的環境狀態以及資源
	eResult = mContext.Init();    //使用前必須起始化,这期间，所有 OpenNI相關的模块會被讀取、分析，直到呼叫「Shutdown()」這個函数，才釋放所使用的資源
	CheckOpenNIError( eResult, "initialize context" ); //检测错误
	if(eResult != XN_STATUS_OK) {return 0;}

	// set map mode
	XnMapOutputMode mapMode;
	mapMode.nXRes = 640;
	mapMode.nYRes = 480;
	mapMode.nFPS = 30;

	// 3. create depth generator
	xn::DepthGenerator mDepthGenerator;
	eResult = mDepthGenerator.Create( mContext );
	CheckOpenNIError( eResult, "Create depth generator" );  if(eResult != XN_STATUS_OK) {return 0;}
	eResult = mDepthGenerator.SetMapOutputMode( mapMode );  if(eResult != XN_STATUS_OK) {return 0;}

	// 4. start generate data
	eResult = mContext.StartGeneratingAll();  if(eResult != XN_STATUS_OK) {return 0;}

	// 5. read data
	eResult = mContext.WaitAndUpdateAll();   if(eResult != XN_STATUS_OK) {return 0;}
	if( eResult == XN_STATUS_OK )
	{
		// 5. get the depth map
		const XnDepthPixel*  pDepthMap = mDepthGenerator.GetDepthMap();//pDepthMap是一个指向unsigned short的一维数组（长度：640*480）的指针
		XnPoint3D  proj[30720],real[30720];		
		ofstream fout, f_x,f_y,f_z;
		fout.open("D:/My Project/OpenNI/depthImage.txt" );
		//f_x.open("D:/My Project/OpenNI/x_data.txt");
		//f_y.open("D:/My Project/OpenNI/y_data.txt");
		//f_z.open("D:/My Project/OpenNI/z_data.txt");

		cout<<"writing files..."<<endl;
		for(int i=0;i<480;i++)
		{
			for(int j=0;j<640;j++)
			{
				//proj[j+i*640].X=(XnFloat)j;   proj[j+i*640].Y =(XnFloat)i;    proj[j+i*640].Z =(XnFloat)pDepthMap[j+i*640];
				fout<<pDepthMap[j+i*640]<<" "; 			
			}
			fout<<endl;  

		}
		fout.close();
		
	/*	mDepthGenerator.ConvertProjectiveToRealWorld(640*480,proj,real);
		for(int i=0;i<480;i++)
		{
			for(int j=0;j<640;j++)
			{
				f_x<<real[j+i*640].X<<" ";
			}
			f_x<<endl;		
		}*/
		f_x.close();
		
	}

	// 7. stop
	mContext.StopGeneratingAll();  //停止产生数据
	mContext.Shutdown();           //终止物件

	return 0;
}

void CheckOpenNIError( XnStatus eResult, string sStatus )
{
	if( eResult != XN_STATUS_OK )
		cout << sStatus << " Error: " << xnGetStatusString( eResult ) << endl;
	return;
}