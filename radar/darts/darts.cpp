/*
 * @Author: your name
 * @Date: 2021-11-02 16:33:40
 * @LastEditTime: 2021-11-11 21:38:50
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
	Mat frame_1 ;
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
	Mat frame_0;//Mat m(3, 5, CV_32FC1, 1);
	//---------------------------------------------------------------------
	int count_num=0;
	while(1)
	{
	   count_num=count_num+1;
	   capture>>frame;
	   
	   double time = (double)getTickCount();
	//    imshow("frame_resize",frame);
	//    cvtColor( frame,hsv, COLOR_RGB2HSV );
	//    inRange(hsv,Scalar(0,0,249),Scalar(170,120,255),gray);
	//    imshow("inrange",hsv);
	   cvtColor( frame,gray, COLOR_RGB2GRAY );
	   	//    bilateralFilter(gray,mid_filer,10,60,60);
		GaussianBlur(gray, mid_filer, Size(5, 5), 1, 1);	  
	   	//    medianBlur(gray,mid_filer,5);     //中值滤波法
		//    cout<<count_num<<endl;
	   	// frame_0=mid_filer.clone();
	//    threshold( mid_filer, mid_filer, 100, 255 , 0 );
	   imshow("mid_filer",mid_filer);
	   //-----------------------------------------------------------------------------------
	   //选择前一帧作为背景（读入第一帧时，第一帧作为背景）
	//    if(num==numFrames)
	   if(count_num==1)
	   {
		   background=mid_filer.clone();
		   frame_1=mid_filer.clone();
		   frame_0=background;
	   }
	   else
	   {
		    background=frame_0; 
	   }

	   
	   //------------------------------------------------------------------------------------
	//    imshow("frame",frame_1);
	//    imshow("background",background);
	   absdiff(mid_filer,background,foreground);//用帧差法求前景
	//    absdiff(mid_filer,background,foreground);//用帧差法求前景
	   Mat element = getStructuringElement(MORPH_RECT,Size(9,9));

	   threshold( foreground, foreground_BW, 120, 255 , 0 );//二值化通常设置为50  255
	   //threshold(foreground, foreground_BW, 0, 255 ,CV_THRESH_BINARY | CV_THRESH_OTSU) ;  //此处使用大津法  自适应取阈值
	//    imshow("二值化",foreground_BW);
	//    bilateralFilter(foreground_BW,gray_dilate2,5,4,4);
	   	dilate(foreground_BW,foreground_BW,element);

	//    absdiff(mid_filer,background,foreground);//用帧差法求前景
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	// Canny(foreground_BW,foreground_BW,150,204,3);//边缘检测
	findContours(foreground_BW,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE,Point());//寻找并绘制轮廓
	imshow("foreground_BW",foreground_BW);   

    vector<Moments>mu(contours.size());
	for(unsigned i=0;i<contours.size();i++)//计算轮廓面积
    {
        mu[i]=moments(contours[i],false);
		// cout<<mu[i].m00<<endl;
    }
    for(int i=0;i<contours.size();i++)//最小外接矩形
    {
		Rect rect = boundingRect(contours[i]);	//找出轮廓最小外界矩形
			// cout << "矩形框" << rect.width << endl;
			if(mu[i].m00<2000&&mu[i].m00>200)
			{
				rectangle(frame, rect, Scalar(0, 0, 255), 3);	//在原图像上画出矩形
			}
			// if((rect.width > 10) && (rect.width < 400))	//限定矩形框的大小
				// rectangle(frame, rect, Scalar(0, 0, 255), 3);	//在原图像上画出矩形

	}
	

	//  for(int i=0;i<contours.size();i++)//最小外接矩形
    // {
    //     // drawContours(foreground_BW,contours,i,Scalar(255),1,8,hierarchy);
    //     // RotatedRect rect=minAreaRect(contours[i]);
    //     // Point2f P[4];
    //     // rect.points(P);
    //     // for(int j=0;j<=3;j++)
    //     // {
    //     //     line(frame,P[j],P[(j+1)%4],Scalar(255,255,255),2);
    //     // }
	// 	Rect rect = boundingRect(contours[i]);	//找出轮廓最小外界矩形
	// 		// cout << "矩形框" << rect.width << endl;
	// 		if((rect.width > 10) && (rect.width < 400))	//限定矩形框的大小
	// 			rectangle(frame, rect, Scalar(0, 0, 255), 3);	//在原图像上画出矩形

	// }
	   time = ((double)getTickCount() - time) / getTickFrequency();
       int fps = 1 / time;
	   
	   imshow("frame",frame);
	//    imshow("foreground",foreground);

	   cout<<"fps:"<<fps<<endl;
 
	   frame_0=mid_filer.clone();
	   imshow("frame_0",frame_0);
	//    frame_1=foreground.clone();
	   num--;
	   char c = waitKey(30);
	   if( c =='q' ) break;
	   if (num < 1)
		   return -1;
	}
}