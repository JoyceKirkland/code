/*
 * @Author: your name
 * @Date: 2021-11-18 11:00:18
 * @LastEditTime: 2021-11-18 16:54:20
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /code/code.cpp
 */
#include <iostream>
#include <opencv2/opencv.hpp>
#include "radar/camera/mv_video_capture.hpp"
// #include "fmt/format.h"
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
 using namespace std;
 using namespace cv;

int main()
{
    // int cap_index=find_capture();
	// CWinThread *pthread1, *pthread2;
    pid_t pid;
  pid = fork();					// 创建进程
        // mindvision::VideoCapture* mv_capture_ = new mindvision::VideoCapture(
        // mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_20000));

  if (-1==pid) {					// 创建进程失败
    printf("Error to create new process!\n");
    return 0;
  }else if(pid==0){
      mindvision::VideoCapture* mv_capture_ = new mindvision::VideoCapture(
        mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_20000),0);
        cv::VideoCapture cap_ = cv::VideoCapture(0);
        Mat img;
        namedWindow("1",WINDOW_NORMAL);
        for(;;)
        {
            if (mv_capture_->isindustryimgInput()) {
            img = mv_capture_->image();
        } else {
            cap_.read(img);
        }
		if (!img.empty()) {
			cv::imshow("1", img);
	 		if(cv::waitKey(1) == 'q') {
                break;
            }
		}
		mv_capture_->cameraReleasebuff();
        }

  }else{	
	mindvision::VideoCapture* mv_capture_1 = new mindvision::VideoCapture(
        mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_20000),1);
	    
        cv::VideoCapture cap_1 = cv::VideoCapture(1);
		
        Mat img1;
        namedWindow("2",WINDOW_NORMAL);
	for(;;)
	{
		 if (mv_capture_1->isindustryimgInput()) {
            img1 = mv_capture_1->image();
        } else {
            cap_1.read(img1);
        }
		if (!img1.empty()) {
            cv::imshow("2", img1);
	 		if(cv::waitKey(1) == 'q') {
                break;
            }
		}
		mv_capture_1->cameraReleasebuff();
	}
        cout<<"father"<<endl;

    }

	// system("pause");
}


// video_test.cpp
// #include <opencv2/core/core.hpp>  
// #include <opencv2/imgproc/imgproc.hpp>  
// #include <opencv2/opencv.hpp>  
 
// #include <vector>  
// #include <cstdio>  
// #include <windows.h>   
// #include <stdio.h>
 
// using namespace std;
// using namespace cv;
 
// DWORD WINAPI Fun1(LPVOID lpParamter)
// {
// 	int number = 0;
// 	// 【1】加载分类器  
// 	CascadeClassifier cascade;
// 	cascade.load("F:\\opencv\\haarcascade_frontalface_alt.xml");
 
// 	Mat frame;
// 	Mat image;
// 	Mat grayImage;
 
// 	VideoCapture cap(0);
 
// 	namedWindow("笔记本摄像头检测", 0);
 
// 	cout << "按下esc退出检测！" << endl;
// 	while (cap.read(frame))
// 	{
// 		char c = waitKey(30);
// 		if (c == 27)
// 		{
// 			break;
// 		}
// 		while (number >= 30)
// 		{
// 			number = 0;
// 			frame.copyTo(image);
// 			cvtColor(image, grayImage, CV_BGR2GRAY); // 生成灰度图，提高检测效率  
 
// 			// 定义7种颜色，用于标记人脸  
// 			Scalar colors[] =
// 			{
// 				// 红橙黄绿青蓝紫  
// 				CV_RGB(255, 0, 0),
// 				CV_RGB(255, 97, 0),
// 				CV_RGB(255, 255, 0),
// 				CV_RGB(0, 255, 0),
// 				CV_RGB(0, 255, 255),
// 				CV_RGB(0, 0, 255),
// 				CV_RGB(160, 32, 240)
// 			};
 
// 			// 【3】检测  
// 			vector<Rect> rect;
// 			cascade.detectMultiScale(grayImage, rect, 1.1, 3, 0);  // 分类器对象调用  
 
// 			printf("笔记本摄像头检测到人脸个数：%d\n", rect.size());
 
// 			// 【4】标记--在脸部画圆  
// 			for (int i = 0; i < rect.size(); i++)
// 			{
// 				Point  center;
// 				int radius;
// 				center.x = cvRound((rect[i].x + rect[i].width * 0.5));
// 				center.y = cvRound((rect[i].y + rect[i].height * 0.5));
 
// 				radius = cvRound((rect[i].width + rect[i].height) * 0.25);
// 				circle(frame, center, radius, colors[i % 7], 2);
// 			}
// 		}
// 		number++;
// 		// 【5】显示  
// 		imshow("笔记本摄像头检测", frame);
// 	}
// 	cap.release();//释放资源
// 	return 0L;
// }
 
// DWORD WINAPI Fun2(LPVOID lpParamter)
// {
// 	int number = 0;
// 	// 【1】加载分类器  
// 	CascadeClassifier cascade;
// 	cascade.load("F:\\opencv\\haarcascade_frontalface_alt.xml");
 
// 	Mat frame;
// 	Mat image;
// 	Mat grayImage;
 
// 	VideoCapture cap(1);
 
// 	namedWindow("外接摄像头检测", 0);
 
// 	cout << "按下esc退出检测！" << endl;
// 	while (cap.read(frame))
// 	{
// 		char c = waitKey(30);
// 		if (c == 27)
// 		{
// 			break;
// 		}
// 		while (number >= 30)
// 		{
// 			number = 0;
// 			frame.copyTo(image);
// 			cvtColor(image, grayImage, CV_BGR2GRAY); // 生成灰度图，提高检测效率  
 
// 			// 定义7种颜色，用于标记人脸  
// 			Scalar colors[] =
// 			{
// 				// 红橙黄绿青蓝紫  
// 				CV_RGB(255, 0, 0),
// 				CV_RGB(255, 97, 0),
// 				CV_RGB(255, 255, 0),
// 				CV_RGB(0, 255, 0),
// 				CV_RGB(0, 255, 255),
// 				CV_RGB(0, 0, 255),
// 				CV_RGB(160, 32, 240)
// 			};
 
// 			// 【3】检测  
// 			vector<Rect> rect;
// 			cascade.detectMultiScale(grayImage, rect, 1.1, 3, 0);  // 分类器对象调用  
 
// 			printf("外接摄像头检测到人脸个数：%d\n", rect.size());
 
// 			// 【4】标记--在脸部画圆  
// 			for (int i = 0; i < rect.size(); i++)
// 			{
// 				Point  center;
// 				int radius;
// 				center.x = cvRound((rect[i].x + rect[i].width * 0.5));
// 				center.y = cvRound((rect[i].y + rect[i].height * 0.5));
 
// 				radius = cvRound((rect[i].width + rect[i].height) * 0.25);
// 				circle(frame, center, radius, colors[i % 7], 2);
// 			}
// 		}
// 		number++;
// 		// 【5】显示  
// 		imshow("外接摄像头检测", frame);
// 	}
// 	cap.release();//释放资源
// 	return 0L;
// }
 
// int main()
// {
// 	HANDLE hThread1 = CreateThread(NULL, 0, Fun1, NULL, 0, NULL);
// 	CloseHandle(hThread1);
// 	HANDLE hThread2 = CreateThread(NULL, 0, Fun2, NULL, 0, NULL);
// 	CloseHandle(hThread2);
// 	while (true)
// 	{
// 		cout << "Main Thread Display!" << endl;
// 		Sleep(3000);
// 	}
// 	return 0;
// }
