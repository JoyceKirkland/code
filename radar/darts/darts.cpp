/*
 * @Author: your name
 * @Date: 2021-11-02 16:33:40
 * @LastEditTime: 2021-11-08 17:01:06
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /radar/Binocular_camera.cpp
 */

#include <opencv2/opencv.hpp>
 using namespace std;
 using namespace cv;
 
int main () 
{
	VideoCapture capture("/home/joyce/视频/闸门闪烁/闸门闪烁6.gif");
	if (!capture.isOpened())
        {
            cout<<"No camera or video input!\n"<<endl;
            return -1;
        }
	Mat frame;
	Mat gray ;
	Mat hsv;
	Mat gray_dilate1 ;
	Mat gray_dilate2 ;
	Mat gray_dilate3 ;
	Mat background,foreground,foreground_BW;
	Mat mid_filer;   //中值滤波法后的照片
	//---------------------------------------------------------------------
	//获取视频的宽度、高度、帧率、总的帧数
	int frameH    = capture.get(CAP_PROP_FRAME_HEIGHT); //获取帧高
    int frameW    = capture.get(CAP_PROP_FRAME_WIDTH);  //获取帧宽
    int fps       = capture.get(CAP_PROP_FPS);          //获取帧率
    int numFrames = capture.get(CAP_PROP_FRAME_COUNT);  //获取整个帧数
	int num=numFrames;
    printf("vedio's \nwidth = %d\t height = %d\n video's fps = %d\t nums = %d", frameW, frameH, fps, numFrames);  
	//---------------------------------------------------------------------
	Mat frame_0,frame_1;//Mat m(3, 5, CV_32FC1, 1);
	//---------------------------------------------------------------------
	while(1)
	{
	   double time = (double)getTickCount();

	   capture>>frame;
	   imshow("frame_resize",frame);
	//    cvtColor( frame,hsv, COLOR_RGB2HSV );
	//    inRange(hsv,Scalar(0,0,249),Scalar(170,120,255),gray);
	//    imshow("inrange",hsv);
	   cvtColor( frame,gray, COLOR_RGB2GRAY );
	   	//    bilateralFilter(gray,mid_filer,10,60,60);
		GaussianBlur(gray, mid_filer, Size(5, 5), 1, 1);	  
	   	//    medianBlur(gray,mid_filer,5);     //中值滤波法

	   //-----------------------------------------------------------------------------------
	   //选择前一帧作为背景（读入第一帧时，第一帧作为背景）
	   if(num==numFrames)
	   {
		   background=mid_filer.clone();
		   frame_0=background;
	   }
	   else
	   {
		    background=frame_0; 
	   }
	   //------------------------------------------------------------------------------------
	   absdiff(mid_filer,background,foreground);//用帧差法求前景
	//    imshow("foreground",foreground);
	//    Mat element = getStructuringElement(MORPH_RECT,Size(3,3));
 
	   
	   threshold( foreground, foreground_BW, 100, 255 , 0 );//二值化通常设置为50  255
	   //threshold(foreground, foreground_BW, 0, 255 ,CV_THRESH_BINARY | CV_THRESH_OTSU) ;  //此处使用大津法  自适应取阈值
	//    imshow("二值化",foreground_BW);
	//    bilateralFilter(foreground_BW,gray_dilate2,5,4,4);
	   	// dilate(mid_filer,gray_dilate1,element);

	//    absdiff(mid_filer,background,foreground);//用帧差法求前景

	   time = ((double)getTickCount() - time) / getTickFrequency();
       int fps = 1 / time;
	   imshow("foreground",foreground_BW);
	//    imshow("gray_dilate2",gray_dilate2);
	   imshow("mid_filer",mid_filer);


	   cout<<"fps:"<<fps<<endl;
	//    imshow("gray_dilate1",gray_dilate1);
	//    dilate(gray_dilate1,gray_dilate2,element);
	//    imshow("gray_dilate2",gray_dilate2);
	//    dilate(gray_dilate2,gray_dilate3,element);
	//    imshow("gray_dilate3",gray_dilate3);
 
	   frame_0=mid_filer.clone();
	   num--;
	   char c = waitKey(1);
	   if( c =='q' ) break;
	   if (num < 1)
		   return -1;
	}
}