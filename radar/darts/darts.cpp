/*
 * @Author: your name
 * @Date: 2021-11-02 16:33:40
 * @LastEditTime: 2021-11-03 22:13:03
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /radar/Binocular_camera.cpp
 */

#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <fmt/format.h>
#include <fmt/color.h>
#include "camera/mv_video_capture.hpp"

using namespace std;
using namespace cv;

int main()
{
    mindvision::VideoCapture* mv_capture_ = new mindvision::VideoCapture(
        mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_20000));

    cv::VideoCapture cap_0 = cv::VideoCapture(0);

    Mat img0;

    for(;;)
    {
        double time = (double)getTickCount();
        if (mv_capture_->isindustryimgInput()) {
            img0 = mv_capture_->image();
        } else {
            cap_0.read(img0);
        }
        if(!img0.empty()||!img1.empty())
        {
            Mat imshow_0 = img0.clone();
            time = ((double)getTickCount() - time) / getTickFrequency();
            int fps = 1 / time;
            imshow("img0", imshow_0);
            putText(imshow_0, fmt::format("fps={}", fps), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
            if(waitKey(1) == 'q') 
            {
                break;
            }
        }
        
        mv_capture_->cameraReleasebuff();
    }

}