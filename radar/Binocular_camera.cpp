/*
 * @Author: your name
 * @Date: 2021-11-02 16:33:40
 * @LastEditTime: 2021-11-18 10:49:12
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /radar/Binocular_camera.cpp
 */

#include <opencv2/opencv.hpp>
#include "camera/mv_video_capture.hpp"
// #include "fmt/format.h"
 using namespace std;
 using namespace cv;

int main()
{
	// CWinThread *pthread1, *pthread2;
	mindvision::VideoCapture* mv_capture_ = new mindvision::VideoCapture(
        mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_20000));
	
	    cv::VideoCapture cap_ = cv::VideoCapture(0);

		Mat img;
	for(;;)
	{
		 if (mv_capture_->isindustryimgInput()) {
            img = mv_capture_->image();
        } else {
            cap_.read(img);
        }
		if (!img.empty()) {
			cv::imshow("SJUT", img);
	 		if(cv::waitKey(1) == 'q') {
                break;
            }
		}
		mv_capture_->cameraReleasebuff();

	}
	// system("pause");
 
}