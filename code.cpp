/*
 * @Author: your name
 * @Date: 2021-11-18 11:00:18
 * @LastEditTime: 2021-11-19 19:23:31
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /code/code.cpp
 */
#include <iostream>
#include <opencv2/opencv.hpp>
#include "radar/camera/mv_video_capture.hpp"
using namespace std;
using namespace cv;

Mat runCamera(mindvision::VideoCapture* mv_capture_,
               cv::VideoCapture cap_,Mat img)
{
    if (mv_capture_->isindustryimgInput()) 
        {
            img = mv_capture_->image();
        }
        else 
        {
            cap_.read(img);
        }
        if (!img.empty()) 
        {
            return img;
	    }else
        {
            cout<<"\nimg error\n"<<endl;
            exit(0);
        }
}

int main()
{
    mindvision::VideoCapture* mv_capture_ = new mindvision::VideoCapture(
    mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_20000),0);
    mindvision::VideoCapture* mv_capture_1 = new mindvision::VideoCapture(
    mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_20000),1);
    cv::VideoCapture cap_ = cv::VideoCapture(0);
    cv::VideoCapture cap_1 = cv::VideoCapture(1);
    Mat img;
    Mat img1;
    int change=1;
    namedWindow("1",WINDOW_NORMAL);
    setWindowProperty("1", WND_PROP_FULLSCREEN, WINDOW_NORMAL);    

    for(;;)
    {
        double time = (double)getTickCount();
        img=runCamera(mv_capture_,cap_,img);
        img1=runCamera(mv_capture_1,cap_1,img1);           

        time = ((double)getTickCount() - time) / getTickFrequency();
        int fps = 1 / time;
        cout<<"fps:"<<fps<<endl;
        int key = waitKey(1);
        if(char(key) == 27)break;
        if(change==1)
        {
            cv::imshow("1", img);
        }else if(change==2)
        {
            cv::imshow("1", img1);
        }

		if(char(key)==49) 
        {
            change=1;
        }else if(char(key)==50)
        {
            change=2;
        }
	    mv_capture_->cameraReleasebuff();
        mv_capture_1->cameraReleasebuff();
    }
}
