/*
 * @Author: your name
 * @Date: 2021-11-18 11:00:18
 * @LastEditTime: 2021-12-06 21:28:06
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /code/code.cpp
 */
#include <iostream>
#include <cstring>
#include <stdio.h>
#include <iomanip>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstring>
#include <fcntl.h>
#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "radar/camera/mv_video_capture.hpp"
#include "TRTModule.hpp"
#include <fmt/format.h>
#include <fmt/color.h>
#include "KCf/serial/uart_serial.hpp"
#include "KCf/angle_solve/basic_pnp.hpp"

using namespace std;
using namespace cv;
static bool debug = true;




struct Detection_pack{
    /*
     * 打包数据结构，将识别结果、对应的图像、陀螺仪和时间戳对应
     */
    std::vector<bbox_t> detection;
    cv::Mat img;
    std::array<double, 4> q;
    double timestamp;
};

float getDistance(const cv::Point a, const cv::Point b) {
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}
struct armor_data{
    cv::Point2f pts[4];
    float img_center_dist;
    int color_id; // 0: blue, 1: red, 2: gray
    int tag_id;   // 0: guard, 1-5: number, 6: base
    float confidence; // 识别准确率
};

struct ROI {
    bool ROI_selected = false;
    cv::Rect2f ROI_bbox;
    int last_class = -1;

    ROI()=default;
    ROI(cv::Rect2f &bbox, int &last): ROI_selected(true), ROI_bbox(bbox), last_class(last) {}
    inline void clear() {
        ROI_selected = false;
        last_class = -1;
    }
    ~ROI()=default;
};

cv::Point2f points_center(cv::Point2f pts[4]){
    for (int i = 0; i < 4; ++i) {
        for (int j = i+1; j < 4; ++j) {
            if (pts[i] == pts[j]) {
                std::cout << "[Error] Unable to calculate center point." << std::endl;
                return cv::Point2f{0, 0};
            }
        }
    }
    cv::Point2f center(0, 0);
    if (pts[0].x == pts[2].x && pts[1].x == pts[3].x) {
        std::cout << "[Error] Unable to calculate center point." << std::endl;
    }
    else if (pts[0].x == pts[2].x && pts[1].x != pts[3].x) {
        center.x = pts[0].x;
        center.y = (pts[3].y-pts[1].y)/(pts[3].x-pts[1].x)*(pts[0].x-pts[3].x)+pts[3].y;
    }
    else if (pts[1].x == pts[3].x && pts[0].x != pts[2].x) {
        center.x = pts[1].x;
        center.y = (pts[2].y-pts[0].y)/(pts[2].x-pts[0].x)*(pts[1].x-pts[0].x)+pts[0].y;
    }
    else {
        center.x = (((pts[3].y-pts[1].y)/(pts[3].x-pts[1].x)*pts[3].x - pts[3].y + \
                    pts[0].y - (pts[2].y-pts[0].y)/(pts[2].x-pts[0].x)*pts[0].x)) / \
                    ((pts[3].y-pts[1].y)/(pts[3].x-pts[1].x)-(pts[2].y-pts[0].y)/(pts[2].x-pts[0].x));
        center.y = (pts[2].y-pts[0].y)/(pts[2].x-pts[0].x)*(center.x-pts[0].x)+pts[0].y;
    }

    return center;
}

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

// int main()
// {
//     mindvision::VideoCapture* mv_capture_ = new mindvision::VideoCapture(
//     mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_20000),0);
//     mindvision::VideoCapture* mv_capture_1 = new mindvision::VideoCapture(
//     mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_20000),1);
//     cv::VideoCapture cap_ = cv::VideoCapture(0);
//     cv::VideoCapture cap_1 = cv::VideoCapture(1);
//     Mat img;
//     Mat img1;
//     int change=1;
//     // namedWindow("1",WINDOW_NORMAL);
//     // setWindowProperty("1", WND_PROP_FULLSCREEN, WINDOW_NORMAL);    

//     for(;;)
//     {
//         double time = (double)getTickCount();
//         img=runCamera(mv_capture_,cap_,img);
//         img1=runCamera(mv_capture_1,cap_1,img1);           

//         time = ((double)getTickCount() - time) / getTickFrequency();
//         int fps = 1 / time;
//         // cout<<"fps:"<<fps<<endl;
//         int key = waitKey(1);
//         if(char(key) == 27)break;
//         // if(change==1)
//         {
//             cv::imshow("0", img);
//         }
//         // else if(change==2)
//         {
//             cv::imshow("1", img1);
//         }

// 		// if(char(key)==49) 
//         // {
//         //     change=1;
//         // }else if(char(key)==50)
//         // {
//         //     change=2;
//         // }
// 	    mv_capture_->cameraReleasebuff();
//         mv_capture_1->cameraReleasebuff();
//     }
// }

//暂且无用，留作备份
// void armor(mindvision::VideoCapture* mv_capture_,Mat img,cv::VideoCapture cap_,
//                 uart::SerialPort serial_,const cv::Scalar colors[4],
//                 basic_pnp::PnP pnp_,float yaw_angle,float last_yaw_angle,float enemy_robot_v,
//                 float forecast_dist,int compensate_w,double time,int index)
// {

//         TRTModule model("/home/joyce/workplace/rm/2022/code/KCf/asset/model-opt-3.onnx");

//       if (mv_capture_->isindustryimgInput()) {
//             img = mv_capture_->image();
//         } else {
//             cap_.read(img);
//         }
//         if (!img.empty()) 
//         {
//             // count_num=count_num+1;
// 		    // GaussianBlur(img, img, Size(5, 5), 1, 1);	  
// 	        // cvtColor( img,mid_filer, COLOR_RGB2GRAY );

//             // if(count_num==1)
// 	        // {
// 		    //     background=mid_filer.clone();
// 		    //     frame_0=background;
// 	        // }
// 	        // else
// 	        // {
// 		    //     background=frame_0; 
// 	        // }

// 	        // absdiff(mid_filer,background,foreground);//用帧差法求前景
// 	        // threshold( foreground, foreground_BW, 120, 255 , 0 );//二值化通常设置为50  255
// 	   	    // dilate(foreground_BW,foreground_BW,element);

// 	        // vector<vector<Point>> contours;
// 	        // vector<Vec4i> hierarchy;
// 	        // findContours(foreground_BW,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE,Point());//寻找并绘制轮廓
//             // vector<Moments>mu(contours.size());
//             // for(unsigned i=0;i<contours.size();i++)//计算轮廓面积
//             // {
//             //     mu[i]=moments(contours[i],false);
//             // }
//             // for(int i=0;i<contours.size();i++)//最小外接矩形
//             // {
// 	    	//     Rect rect = boundingRect(contours[i]);	//找出轮廓最小外界矩形
// 	    	// 	// cout << "矩形框" << rect.width << endl;
// 	    	// 	// if(mu[i].m00<2000&&mu[i].m00>125)
// 	    	// 	if(mu[i].m00<200)
// 	    	// 	{
// 	    	// 		rectangle(img, rect, Scalar(0, 0, 255), 2);	//在原图像上画出矩形
// 	    	// 	}
// 	        // }
    
//             //--------------------------------------------
//             std::array<double, 4> q;
//             double timestamp = 0.0;
//             std::vector<armor_data> data_armor;
//             // const auto& [img, q, timestamp] = sensor_sub.pop();
//             auto detections = model(img);
//             armor_data armor;
//             // /* publish detection results */

//             /* show detections */
//             if(!detections.empty()) 
//             {
//                 cv::Mat im2show = img.clone();
//                 // for (const auto &b: detections) {
//                 for (int i = 0; i < detections.size(); i++) 
//                 {
//                     cv::line(img, detections[i].pts[0], detections[i].pts[1], colors[2], 2);
//                     cv::line(img, detections[i].pts[1], detections[i].pts[2], colors[2], 2);
//                     cv::line(img, detections[i].pts[2], detections[i].pts[3], colors[2], 2);
//                     cv::line(img, detections[i].pts[3], detections[i].pts[0], colors[2], 2);
//                     cv::putText(img, std::to_string(detections[i].tag_id), detections[i].pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[detections[i].color_id]);
//                     armor.color_id = detections[i].color_id;
//                     armor.tag_id   = detections[i].tag_id;
//                     armor.pts[0]   = detections[i].pts[0];
//                     armor.pts[1]   = detections[i].pts[1];
//                     armor.pts[2]   = detections[i].pts[2];
//                     armor.pts[3]   = detections[i].pts[3];
//                     armor.confidence = detections[i].confidence;
//                     armor.img_center_dist = getDistance((detections[i].pts[0] + detections[i].pts[3]) * 0.5, cv::Point(img.cols * 0.5, img.rows * 0.5 + 100));
//                     if (serial_.returnReceiceColor() != detections[i].color_id && detections[i].confidence > 0.5 /* && detections[i].tag_id != 2 */) {
//                         data_armor.push_back(armor);
//                     }
//                     // std::cout << armor.img_center_dist << std::endl;
//                 }
//             }
//             if (!data_armor.empty()) 
//             {
                
//                 serial_.returnReceivePitch();
//                 yaw_angle = serial_.returnReceiveYaw();
                
//                 // 离枪管最近装甲板
//                 std::sort(data_armor.begin(), data_armor.end(), [](const armor_data &_a, const armor_data &_b) {
//                     return _a.img_center_dist < _b.img_center_dist;
//                 });

//                 if (data_armor.size() > 0) {
//                     cv::line(img, data_armor[0].pts[0], data_armor[0].pts[1], colors[3], 2);
//                     cv::line(img, data_armor[0].pts[1], data_armor[0].pts[2], colors[3], 2);
//                     cv::line(img, data_armor[0].pts[2], data_armor[0].pts[3], colors[3], 2);
//                     cv::line(img, data_armor[0].pts[3], data_armor[0].pts[0], colors[3], 2);
//                     cv::putText(img, std::to_string(data_armor[0].tag_id), data_armor[0].pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[data_armor[0].color_id]);
//                 }

//                 std::vector<cv::Point2f> target_2d;
//                 target_2d.push_back(data_armor[0].pts[0]);
//                 target_2d.push_back(data_armor[0].pts[1]);
//                 target_2d.push_back(data_armor[0].pts[2]);
//                 target_2d.push_back(data_armor[0].pts[3]);
//                 if (data_armor[0].tag_id == 1 || data_armor[0].tag_id == 0) {
//                     pnp_.solvePnP(serial_.returnReceiveBulletVelocity(), 1, target_2d);
//                 } else {
//                     pnp_.solvePnP(serial_.returnReceiveBulletVelocity(), 0, target_2d);
//                 }

//                 if (last_yaw_angle != 0) {

//                     // 敌方机器人当前速度
//                     enemy_robot_v = (yaw_angle - last_yaw_angle) * pnp_.returnDepth();
//                     // 实际预测位置
//                     forecast_dist = enemy_robot_v * (time + 0.2);
//                     // 添加补偿宽度
//                     compensate_w = 8 * pnp_.returnDepth() / forecast_dist;

//                     std::vector<cv::Point2f> k_target_2d;
//                     for (int i = 0; i < target_2d.size(); i++) {
//                         k_target_2d.push_back(cv::Point2f(target_2d[i].x + compensate_w, target_2d[i].y));
//                     }
                    
//                     if (data_armor[0].tag_id == 1 || data_armor[0].tag_id == 0) {
//                         pnp_.solvePnP(serial_.returnReceiveBulletVelocity(), 1, k_target_2d);
//                     } else {
//                         pnp_.solvePnP(serial_.returnReceiveBulletVelocity(), 0, k_target_2d);
//                     }

//                 }

//                 serial_.updataWriteData(pnp_.returnYawAngle(),
//                                         pnp_.returnPitchAngle(),
//                                         pnp_.returnDepth(),
//                                         1,
//                                         0);
//             }
//             serial_.updataWriteData(pnp_.returnYawAngle(),
//                         pnp_.returnPitchAngle(),
//                         pnp_.returnDepth(),
//                         0,
//                         0);
//             last_yaw_angle = yaw_angle;
//             data_armor.clear();
//             data_armor.shrink_to_fit();
//             // fps_count++;
//             // time = ((double)getTickCount() - time) / getTickFrequency();
//             // int fps = 1 / time;
//             // cv::putText(img, fmt::format("fps={}", fps), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
//             // frame_0=mid_filer.clone();

//             // if(index==0)
//             {
//                 cv::imshow("0", img);
//             }
//             // else
//             {
//                 // cv::imshow("1",img);
//             }
//             // if(cv::waitKey(1) == 'q') {
//             //     break;
//             // }
//     }
// }

// void darts_Door(mindvision::VideoCapture* mv_capture_,Mat img,cv::VideoCapture cap_,
//                 int count_num,Mat mid_filer)
// Mat darts_Door(mindvision::VideoCapture* mv_capture_,Mat img,cv::VideoCapture cap_,
//                 int count_num,Mat mid_filer,Mat background,Mat frame_0,Mat foreground,
//                 Mat foreground_BW,Mat element)
// Mat darts_Door(Mat img,int count_num,Mat mid_filer,Mat background,Mat frame_0,Mat foreground,
//                 Mat foreground_BW,Mat element)
//暂且无用，留作备份
// Mat darts_Door(Mat img,Mat element)
// {
    // if (mv_capture_->isindustryimgInput()) {
    //         img = mv_capture_->image();
    //     } else {
    //         cap_.read(img);
    //     }
    //     if (!img.empty()) 
    //     {
            // static int count_num=1;

            // Mat mid_filer,background,frame_0,foreground,foreground_BW;
            // // static int count_num = 1;
                            
            // // count_num+=1;
		    // GaussianBlur(img, img, Size(5, 5), 1, 1);	  
	        // cvtColor( img,mid_filer, COLOR_RGB2GRAY );
            // // // cv::imshow("mid_filer",mid_filer);

            // if(count_num==1)
	        // {
		    //     background=mid_filer.clone();
		    //     frame_0=background;
            //     // cout<<"count_num:"<<count_num<<endl;
	        // }
	        // else
	        // {
		    //     background=frame_0; 
	        // }
            // // cv::imshow("background",background);

	        // absdiff(mid_filer,background,foreground);//用帧差法求前景
            // // imshow("frame_0",frame_0);

	        // threshold( foreground, foreground_BW, 120, 255 , 0 );//二值化通常设置为50  255
           

	   	    // dilate(foreground_BW,foreground_BW,element);
            // // count_num=count_num+1;                
                    
            // // return foreground_BW;
	        // vector<vector<Point>> contours;
	        // vector<Vec4i> hierarchy;
	        // findContours(foreground_BW,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE,Point());//寻找并绘制轮廓
            // vector<Moments>mu(contours.size());
            // for(unsigned i=0;i<contours.size();i++)//计算轮廓面积
            // {
            //     mu[i]=moments(contours[i],false);

            // }
            // for(int i=0;i<contours.size();i++)//最小外接矩形
            // {
	    	//     Rect rect = boundingRect(contours[i]);	//找出轮廓最小外界矩形
            //     cout<<"rect:faasasdsdasda"<<endl;
	    	// 	// cout << "矩形框" << rect.width << endl;
	    	// 	// if(mu[i].m00<2000&&mu[i].m00>125)
	    	// 	// if(mu[i].m00<200)
	    	// 	{
	    	// 		rectangle(img, rect, Scalar(0, 0, 255), 2);	//在原图像上画出矩形
	    	// 	}
	        // }
            // frame_0=mid_filer.clone();

            // count_num+=1;
            // // cout<<"count_num:"<<count_num<<endl;

            // return img;
            // cv::imshow("1",img);
            
        // }
// }


int main () 
{
	// VideoCapture capture("/home/joyce/视频/闸门闪烁/闸门闪烁6.gif");
    TRTModule model("/home/joyce/workplace/rm/2022/code/KCf/asset/model-opt-3.onnx");

    uart::SerialPort serial_ = uart::SerialPort("/home/joyce/workplace/rm/2022/KCf/configs/serial/uart_serial_config.xml");

    basic_pnp::PnP pnp_ = basic_pnp::PnP("/home/joyce/workplace/rm/2022/KCf/configs/camera/mv_camera_config_555.xml", 
                                         "/home/joyce/workplace/rm/2022/KCf/configs/angle_solve/basic_pnp_config.xml");

    mindvision::VideoCapture* mv_capture_ = new mindvision::VideoCapture(
    mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_20000),0);
    cv::VideoCapture cap_ = cv::VideoCapture(0);

    mindvision::VideoCapture* mv_capture_1 = new mindvision::VideoCapture(
    mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_20000),1);
    cv::VideoCapture cap_1 = cv::VideoCapture(1);

    Mat background,foreground,foreground_BW;
    Mat mid_filer;   //中值滤波法后的照片
    Mat frame_0;
    int count_num=0;
    // static int count_num=0;

    ROI roi;
    // cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	// cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
    // cap_.set(cv::CAP_PROP_AUTO_EXPOSURE,1);
    // cap_.set(cv::CAP_PROP_EXPOSURE, 800);

    // int fps = 0, fps_count = 0;
    // auto t1 = system_clock::now();
    // int cnt_useless = -1;
	cv::Mat img;//装甲板
    cv::Mat img1;//飞镖
    cv::Mat darts_roi;
    const cv::Scalar colors[4] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}, {255, 255, 255}};

	// const double fx = 1755.8568155966806899;
    // const double fy = 1756.0870281269756106;
    // const double cx = 649.2466252010661947;
    // const double cy = 487.2552385352113333;
    float yaw_angle;
    float last_yaw_angle;
    float enemy_robot_v;
    float forecast_dist;
    int compensate_w;
    std::vector<cv::Point2f> target_2d;
    Mat element = getStructuringElement(MORPH_RECT,Size(9,9));

    while (true) 
    {

		// 记录起始的时钟周期数
        double time = (double)getTickCount();
        // armor(mv_capture_,img,cap_,
        //         serial_,colors,
        //         pnp_, yaw_angle, last_yaw_angle, enemy_robot_v,
        //         forecast_dist,compensate_w,time,0);

                // imshow("0",img);
        // darts_Door(mv_capture_1,img1,cap_1,
        //         count_num,mid_filer,background,frame_0,foreground,
        //          foreground_BW, element);
    
        if (mv_capture_->isindustryimgInput()
        &&mv_capture_1->isindustryimgInput()) 
        {
            img = mv_capture_->image();
            img1=mv_capture_1->image();
        } else {
            cap_.read(img);
            cap_1.read(img1);
        }
        if (!img.empty()) {
            count_num=count_num+1;
                // img1=darts_Door(img1,count_num,
                // mid_filer,background,frame_0,foreground,
                //  foreground_BW, element);
            darts_roi=img(Rect((img.cols/2)-20,(img.rows/2)-20,140,140));
	    	rectangle(img, Rect((img.cols/2)-20,(img.rows/2)-20,140,140), Scalar(255, 255, 0), 2);	//在原图像上画出矩形
		    GaussianBlur(darts_roi, darts_roi, Size(5, 5), 1, 1);	
            // imshow("darts_roi",darts_roi);  
	        cvtColor( darts_roi,mid_filer, COLOR_RGB2GRAY );

            if(count_num==1)
	        {
		        background=mid_filer.clone();
		        frame_0=background;
	        }
	        else
	        {
		        background=frame_0; 
	        }
	        absdiff(mid_filer,background,foreground);//用帧差法求前景
	        threshold( foreground, foreground_BW, 120, 255 , 0 );//二值化通常设置为50  255
	   	    dilate(foreground_BW,foreground_BW,element);

	        vector<vector<Point>> contours;
	        vector<Vec4i> hierarchy;
	        findContours(foreground_BW,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE,Point());//寻找并绘制轮廓
            vector<Moments>mu(contours.size());
            for(unsigned i=0;i<contours.size();i++)//计算轮廓面积
            {
                mu[i]=moments(contours[i],false);
            }
            for(int i=0;i<contours.size();i++)//最小外接矩形
            {
	    	    Rect rect = boundingRect(contours[i]);	//找出轮廓最小外界矩形
	    		// cout << "矩形框" << rect.width << endl;
	    		// if(mu[i].m00<2000&&mu[i].m00>125)
	    		if(mu[i].m00<300)
	    		{
	    			rectangle(darts_roi, rect, Scalar(0, 255, 0), 3);	//在原图像上画出矩形
	    		}
	        }
            frame_0=mid_filer.clone();
            imshow("img1",img);
            //--------------------------------------------
            std::array<double, 4> q;
            double timestamp = 0.0;
            std::vector<armor_data> data_armor;
            // const auto& [img, q, timestamp] = sensor_sub.pop();
            auto detections = model(img1);
            armor_data armor;
            // /* publish detection results */

            /* show detections */

            if(!detections.empty()) {
                cv::Mat im2show = img.clone();
                // for (const auto &b: detections) {
                for (int i = 0; i < detections.size(); i++) {
                    cv::line(img1, detections[i].pts[0], detections[i].pts[1], colors[2], 2);
                    cv::line(img1, detections[i].pts[1], detections[i].pts[2], colors[2], 2);
                    cv::line(img1, detections[i].pts[2], detections[i].pts[3], colors[2], 2);
                    cv::line(img1, detections[i].pts[3], detections[i].pts[0], colors[2], 2);
                    cv::putText(img, std::to_string(detections[i].tag_id), detections[i].pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[detections[i].color_id]);
                    armor.color_id = detections[i].color_id;
                    armor.tag_id   = detections[i].tag_id;
                    armor.pts[0]   = detections[i].pts[0];
                    armor.pts[1]   = detections[i].pts[1];
                    armor.pts[2]   = detections[i].pts[2];
                    armor.pts[3]   = detections[i].pts[3];
                    armor.confidence = detections[i].confidence;
                    armor.img_center_dist = getDistance((detections[i].pts[0] + detections[i].pts[3]) * 0.5, cv::Point(img1.cols * 0.5, img1.rows * 0.5 + 100));
                    //  && detections[i].tag_id != 2 
                    if (serial_.returnReceiceColor() != detections[i].color_id && detections[i].confidence > 0.5 ) 

                    {
                        data_armor.push_back(armor);
                    }
                    // std::cout << armor.img_center_dist << std::endl;
                }
            }
            if (!data_armor.empty()) {
                
                serial_.returnReceivePitch();
                yaw_angle = serial_.returnReceiveYaw();
                
                // 离枪管最近装甲板
                std::sort(data_armor.begin(), data_armor.end(), [](const armor_data &_a, const armor_data &_b) {
                    return _a.img_center_dist < _b.img_center_dist;
                });

                if (data_armor.size() > 0) {
                    cv::line(img1, data_armor[0].pts[0], data_armor[0].pts[1], colors[3], 2);
                    cv::line(img1, data_armor[0].pts[1], data_armor[0].pts[2], colors[3], 2);
                    cv::line(img1, data_armor[0].pts[2], data_armor[0].pts[3], colors[3], 2);
                    cv::line(img1, data_armor[0].pts[3], data_armor[0].pts[0], colors[3], 2);
                    cv::putText(img1, std::to_string(data_armor[0].tag_id), data_armor[0].pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[data_armor[0].color_id]);
                }

                std::vector<cv::Point2f> target_2d;
                target_2d.push_back(data_armor[0].pts[0]);
                target_2d.push_back(data_armor[0].pts[1]);
                target_2d.push_back(data_armor[0].pts[2]);
                target_2d.push_back(data_armor[0].pts[3]);
                if (data_armor[0].tag_id == 1 || data_armor[0].tag_id == 0) {
                    pnp_.solvePnP(serial_.returnReceiveBulletVelocity(), 1, target_2d);
                } else {
                    pnp_.solvePnP(serial_.returnReceiveBulletVelocity(), 0, target_2d);
                }

                if (last_yaw_angle != 0) {

                    // 敌方机器人当前速度
                    enemy_robot_v = (yaw_angle - last_yaw_angle) * pnp_.returnDepth();
                    // 实际预测位置
                    forecast_dist = enemy_robot_v * (time + 0.2);
                    // 添加补偿宽度
                    compensate_w = 8 * pnp_.returnDepth() / forecast_dist;

                    std::vector<cv::Point2f> k_target_2d;
                    for (int i = 0; i < target_2d.size(); i++) {
                        k_target_2d.push_back(cv::Point2f(target_2d[i].x + compensate_w, target_2d[i].y));
                    }
                    
                    if (data_armor[0].tag_id == 1 || data_armor[0].tag_id == 0) {
                        pnp_.solvePnP(serial_.returnReceiveBulletVelocity(), 1, k_target_2d);
                    } else {
                        pnp_.solvePnP(serial_.returnReceiveBulletVelocity(), 0, k_target_2d);
                    }

                }

                serial_.updataWriteData(pnp_.returnYawAngle(),
                                        pnp_.returnPitchAngle(),
                                        pnp_.returnDepth(),
                                        1,
                                        0);
            }
            serial_.updataWriteData(pnp_.returnYawAngle(),
                        pnp_.returnPitchAngle(),
                        pnp_.returnDepth(),
                        0,
                        0);
            last_yaw_angle = yaw_angle;
            data_armor.clear();
            data_armor.shrink_to_fit();
            // fps_count++;
            time = ((double)getTickCount() - time) / getTickFrequency();
            int fps = 1 / time;
            cv::putText(img1, fmt::format("fps={}", fps), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0));
            cout<<"fps:"<<fps<<endl;
            // frame_0=mid_filer.clone();
            cv::imshow("0", img1);
            if(cv::waitKey(1) == 'q') {
                break;
            }
        }

        mv_capture_->cameraReleasebuff();        
        mv_capture_1->cameraReleasebuff();

    }
}
