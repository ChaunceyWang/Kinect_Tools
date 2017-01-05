#include <iostream>
#include <fstream>  //��ȡtxt�ļ�
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
	xn::Context mContext;         //������Á�������� OpenNI �ĭh����B�Լ��YԴ
	eResult = mContext.Init();    //ʹ��ǰ�����ʼ��,���ڼ䣬���� OpenNI���P��ģ������xȡ��������ֱ�����С�Shutdown()���@����������ጷ���ʹ�õ��YԴ
	CheckOpenNIError( eResult, "initialize context" ); //������
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
		const XnDepthPixel*  pDepthMap = mDepthGenerator.GetDepthMap();//pDepthMap��һ��ָ��unsigned short��һά���飨���ȣ�640*480����ָ��
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
	mContext.StopGeneratingAll();  //ֹͣ��������
	mContext.Shutdown();           //��ֹ���

	return 0;
}

void CheckOpenNIError( XnStatus eResult, string sStatus )
{
	if( eResult != XN_STATUS_OK )
		cout << sStatus << " Error: " << xnGetStatusString( eResult ) << endl;
	return;
}