/*
 * @Author: your name
 * @Date: 2021-11-18 11:00:18
 * @LastEditTime: 2022-07-28 20:56:56
 * @LastEditors: JoyceKirkland joyce84739879@163.com
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /code/code.cpp
 */
#define CVUI_IMPLEMENTATION
#define WINDOW_NAME "Radar"

#include <iostream>
#include <cstring>
#include <time.h>
#include <sys/timeb.h> 
#include <stdio.h>
#include <stdlib.h>
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
#include <memory>
#include <atomic>
#include <chrono>
#include <exception>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "radar/camera/mv_video_capture.hpp"
#include "TRTModule.hpp"
// #include <opencv4/opencv2/cvv.hpp>
// #include <python3.8/Python.h>
#include <fmt/format.h>
#include <fmt/color.h>
#include "/home/joyce/workplace/rm/2022/code/cvui/cvui.h"
// #include "KCf/serial/uart_serial.hpp"
#include "KCf/angle_solve/basic_pnp.hpp"
#include "math.h"
// #include "KCf/camera/mv_video_capture.hpp"
#include "KCf/devices/new_serial/serial.hpp"
// #include "KCf/devices/serial/uart_serial.hpp"
#include <string.h>
#include <thread>
#include <chrono>
#include "cmdline.h"
#include "livox_sample/lds_lidar.h"
using namespace std;
using namespace cv;
static bool debug = true;


enum Color {
  BLUE,
  RED,
};
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
RoboInf out_robo_inf;
RoboInf out_robo_cmd;

struct HP_information
{
  int R_Hero_HP = 0;
  int R_Engineer_HP = 0;
  int R_Infantry3_HP = 0;
  int R_Infantry4_HP = 0;
  int R_Infantry5_HP = 0;
  int R_Sentry_HP = 0;
  int R_Outpost_HP = 0;
  int R_Base_HP = 0;
  int B_Hero_HP = 0;
  int B_Engineer_HP = 0;
  int B_Infantry3_HP = 0;
  int B_Infantry4_HP = 0;
  int B_Infantry5_HP = 0;
  int B_Sentry_HP = 0;
  int B_Outpost_HP = 0;
  int B_Base_HP = 0;
}HP_info;

// void output_HP()
// {
//     HP_info.R_Hero_HP=out_robo_inf.R_Hero_HP.load();
//     HP_info.R_Engineer_HP=out_robo_inf.R_Engineer_HP.load();
//     HP_info.R_Infantry3_HP=out_robo_inf.R_Infantry3_HP.load();
//     HP_info.R_Infantry4_HP=out_robo_inf.R_Infantry4_HP.load();
//     HP_info.R_Infantry5_HP=out_robo_inf.R_Infantry5_HP.load();
//     HP_info.R_Sentry_HP=out_robo_inf.R_Sentry_HP.load();
//     HP_info.R_Outpost_HP=out_robo_inf.R_Outpost_HP.load();
//     HP_info.R_Base_HP=out_robo_inf.R_Base_HP.load();

//     HP_info.B_Hero_HP=out_robo_inf.B_Hero_HP.load();
//     HP_info.B_Engineer_HP=out_robo_inf.B_Engineer_HP.load();
//     HP_info.B_Infantry3_HP=out_robo_inf.B_Infantry3_HP.load();
//     HP_info.B_Infantry4_HP=out_robo_inf.B_Infantry4_HP.load();
//     HP_info.B_Infantry5_HP=out_robo_inf.B_Infantry5_HP.load();
//     HP_info.B_Sentry_HP=out_robo_inf.B_Sentry_HP.load();
//     HP_info.B_Outpost_HP=out_robo_inf.B_Outpost_HP.load();
//     HP_info.B_Base_HP=out_robo_inf.B_Base_HP.load();

// }

void uartWriteThread(const std::shared_ptr<RoboSerial> &serial,
                     RoboCmd &robo_cmd) {
  while (true) try {
      if(serial->isOpen()) {
        serial->WriteInfo(robo_cmd);
        
        // cout<<"is_left:"<<robo_cmd.is_left<<endl;
        // cout<<"turn_angle:"<<robo_cmd.turn_angle<<endl;

      } else {
        serial->open();
      }
      std::this_thread::sleep_for(1ms);
    } catch (const std::exception &e) {
      serial->close();
      static int serial_read_excepted_times{0};
      if (serial_read_excepted_times++ > 3) {
        std::this_thread::sleep_for(10000ms);
        fmt::print("[{}] read serial excepted to many times, sleep 10s.\n",
                   idntifier_red);
        serial_read_excepted_times = 0;
      }
      fmt::print("[{}] serial exception: {}\n",
                 idntifier_red, e.what());
      std::this_thread::sleep_for(1000ms);
    }
}

void uartReadThread(const std::shared_ptr<RoboSerial> &serial,
                    RoboInf &robo_inf) {
  while (true) try {
    //   cout<<"??????????/"<<endl;
      if(serial->isOpen()) {
        serial->ReceiveInfo(robo_inf);
        // cout<<"readsth"<<endl;
    // std::cout<<"R_Hero_HP_robo:"<<robo_inf.R_Hero_HP<<std::endl;
    out_robo_inf.my_color.store(robo_inf.my_color);
    // std::cout<<"R_Hero_HP_output:"<<out_robo_inf.R_Hero_HP<<std::endl;
    //   atomic<uint16_t> out_robo_inf.R_Hero_HP(robo_inf.R_Hero_HP.load());
    //   out_robo_inf.R_Hero_HP.store(robo_inf.R_Hero_HP);
    //   out_robo_inf.R_Engineer_HP.store(robo_inf.R_Engineer_HP);
    //   out_robo_inf.R_Infantry3_HP.store(robo_inf.R_Infantry3_HP);
    //   out_robo_inf.R_Infantry4_HP.store(robo_inf.R_Infantry4_HP);
    //   out_robo_inf.R_Infantry5_HP.store(robo_inf.R_Infantry5_HP);
    //   out_robo_inf.R_Sentry_HP.store(robo_inf.R_Sentry_HP);
    //   out_robo_inf.R_Outpost_HP.store(robo_inf.R_Outpost_HP);
    //   out_robo_inf.R_Base_HP.store(robo_inf.R_Base_HP);
    //       //sdfas
    //   out_robo_inf.B_Hero_HP.store(robo_inf.B_Hero_HP);
    //   out_robo_inf.B_Engineer_HP.store(robo_inf.B_Engineer_HP);
    //   out_robo_inf.B_Infantry3_HP.store(robo_inf.B_Infantry3_HP);
    //   out_robo_inf.B_Infantry4_HP.store(robo_inf.B_Infantry4_HP);
    //   out_robo_inf.B_Infantry5_HP.store(robo_inf.B_Infantry5_HP);
    //   out_robo_inf.B_Sentry_HP.store(robo_inf.B_Sentry_HP);
    //   out_robo_inf.B_Outpost_HP.store(robo_inf.B_Outpost_HP);
    //   out_robo_inf.B_Base_HP.store(robo_inf.B_Base_HP);
      
    std::cout<<"out_robo_inf_color:"<<out_robo_inf.my_color<<std::endl;
    // std::cout<<"R_Infantry3_HP:"<<out_robo_inf.R_Infantry3_HP<<std::endl;
    // std::cout<<"R_Infantry4_HP:"<<out_robo_inf.R_Infantry4_HP<<std::endl;
    // std::cout<<"R_Infantry5_HP:"<<out_robo_inf.R_Infantry5_HP<<std::endl;
    // std::cout<<"R_Infantry5_HP:"<<out_robo_inf.R_Sentry_HP<<std::endl;
    // std::cout<<"R_Outpost_HP:"<<out_robo_inf.R_Outpost_HP<<std::endl;
    // std::cout<<"R_Base_HP:"<<out_robo_inf.R_Base_HP<<std::endl;

    // std::cout<<"B_Hero_HP:"<<out_robo_inf.B_Hero_HP<<std::endl;
    // std::cout<<"B_Engineer_HP:"<<out_robo_inf.B_Engineer_HP<<std::endl;
    // std::cout<<"B_Infantry3_HP:"<<out_robo_inf.B_Infantry3_HP<<std::endl;
    // std::cout<<"B_Infantry4_HP:"<<out_robo_inf.B_Infantry4_HP<<std::endl;
    // std::cout<<"B_Infantry5_HP:"<<out_robo_inf.B_Infantry5_HP<<std::endl;
    // std::cout<<"B_Sentry_HP:"<<out_robo_inf.B_Sentry_HP<<std::endl;
    // std::cout<<"B_Outpost_HP:"<<out_robo_inf.B_Outpost_HP<<std::endl;
    // std::cout<<"B_Base_HP:"<<out_robo_inf.B_Base_HP<<std::endl;
      }
      std::this_thread::sleep_for(1ms);
    } catch (const std::exception &e) {
      static int serial_read_excepted_times{0};
      if (serial_read_excepted_times++ > 3) {
        std::this_thread::sleep_for(10000ms);
        fmt::print("[{}] read serial excepted to many times, sleep 10s.\n",
                   idntifier_red);
        serial_read_excepted_times = 0;
      }
      fmt::print("[{}] serial exception: {}\n",
                 idntifier_red, e.what());
      std::this_thread::sleep_for(1000ms);
    }
}

// void drawMap(cv::Mat frame,cv::Mat rm_map,int count_num)
// {
//     cvui::image(frame, 20, 20, rm_map);
//     rectangle(rm_map,Rect(37,260,40,210),Scalar(0,255,0),2);//敌方飞坡区域
//     rectangle(rm_map,Rect(82,310,42,50),Scalar(0,255,0),2);//敌方大能量机关击打区域
//     rectangle(rm_map,Rect(390,20,42,50),Scalar(0,255,0),2);//敌方飞镖区域
//     circle(rm_map,Point(410,90),18,Scalar(255,0,0),-1);
//     circle(rm_map,Point(150,340),18,Scalar(255,0,0),-1);
//     circle(rm_map,Point(92,480),18,Scalar(255,0,0),-1);
    
//     // int r_Hero_HP=out_robo_inf.R_Hero_HP.load();
//     output_HP();
//     cvui::printf(frame, 695, 830, 1.3, 0xFFFFFF, "Red:");//红方各兵种、模块血量，蓝方下同
//     cvui::printf(frame, 695, 870, 0.9, 0xFFFFFF, "R1:%d/500",HP_info.R_Hero_HP);//英雄
//     cvui::printf(frame, 695, 900, 0.9, 0xFFFFFF, "R2:%d/500",HP_info.R_Engineer_HP);//工程
//     cvui::printf(frame, 695, 930, 0.9, 0xFFFFFF, "R3:%d/500",HP_info.R_Infantry3_HP);//步兵3
//     cvui::printf(frame, 695, 960, 0.9, 0xFFFFFF, "R4:%d/500",HP_info.R_Infantry4_HP);//步兵4
//     cvui::printf(frame, 695, 990, 0.9, 0xFFFFFF, "R5:%d/500",HP_info.R_Infantry5_HP);//步兵5
//     cvui::printf(frame, 695, 1020, 0.9, 0xFFFFFF, "R7:%d/600",HP_info.R_Sentry_HP);//哨兵
//     cvui::printf(frame, 695, 1080, 0.9, 0xFFFFFF, "---------");
//     cvui::printf(frame, 695, 1110, 0.9, 0xFFFFFF, "RO:%d/1500",HP_info.R_Outpost_HP);//前哨站
//     cvui::printf(frame, 695, 1140, 0.9, 0xFFFFFF, "RB:%d/5000",HP_info.R_Base_HP);//基地

//     //______________________________________________
//     cvui::printf(frame, 1080, 830, 1.3, 0xFFFFFF, "Blue:");
//     cvui::printf(frame, 1080, 870, 0.9, 0xFFFFFF, "B1:%d/500",HP_info.B_Hero_HP);
//     cvui::printf(frame, 1080, 900, 0.9, 0xFFFFFF, "B2:%d/500",HP_info.B_Engineer_HP);
//     cvui::printf(frame, 1080, 930, 0.9, 0xFFFFFF, "B3:%d/500",HP_info.B_Infantry3_HP);
//     cvui::printf(frame, 1080, 960, 0.9, 0xFFFFFF, "B4:%d/500",HP_info.B_Infantry4_HP);
//     cvui::printf(frame, 1080, 990, 0.9, 0xFFFFFF, "B5:%d/500",HP_info.B_Infantry5_HP);
//     cvui::printf(frame, 1080, 1020, 0.9, 0xFFFFFF, "B7:%d/600",HP_info.B_Sentry_HP);
//     cvui::printf(frame, 1080, 1080, 0.9, 0xFFFFFF, "---------");
//     cvui::printf(frame, 1080, 1110, 0.9, 0xFFFFFF, "BO:%d/1500",HP_info.B_Outpost_HP);
//     cvui::printf(frame, 1080, 1140, 0.9, 0xFFFFFF, "BB:%d/5000",HP_info.B_Base_HP);
// }
// void warning_Lights(cv::Mat rm_map,int count_num,int x,int y)
// {
//     // circle(rm_map,Point(x,y),18,Scalar(255,0,0),-1);
//     if((sin((count_num*3.1415926)*16/180))>0)//||(sin((count_num*3.1415926*0.5)/180))<-0.5
//     {
//         circle(rm_map,Point(x,y),18,Scalar(255,0,0),-1);
//         // sleep(1);
//     }
//     else {
//         circle(rm_map,Point(x,y),18,Scalar(0,0,255),-1);
//         // count_num = 2;
//     }
// }
// bool fly_or_engery(int point_x,int point_y,cv::Rect rect)
// {
//     if(point_x>rect.x&&point_x<rect.x+rect.width&&point_y>rect.y&&point_y<rect.y+rect.height)
//     {
//         return true;
//     }else
//     {
//         return false;
//     }
// }
std::string getCurrentTimeStr()
{
  time_t t = time(NULL);
  char ch[64] = {0};
  char result[100] = {0};
  strftime(ch, sizeof(ch) - 1, "%Y-%m-%d--%H:%M:%S", localtime(&t));
  sprintf(result, "%s", ch);
  return std::string(result);
}
// static std::vector<std::string> cmdline_broadcast_code;
// void SetProgramOption(int argc, const char *argv[]) {
//   cmdline::parser cmd;
//   cmd.add<std::string>("code", 'c', "Register device broadcast code", false);
//   cmd.add("log", 'l', "Save the log file");
//   cmd.add("help", 'h', "Show help");
//   cmd.parse_check(argc, const_cast<char **>(argv));
//   if (cmd.exist("code")) {
//     std::string sn_list = cmd.get<std::string>("code");
//     printf("Register broadcast code: %s\n", sn_list.c_str());
//     size_t pos = 0;
//     cmdline_broadcast_code.clear();
//     while ((pos = sn_list.find("&")) != std::string::npos) {
//       cmdline_broadcast_code.push_back(sn_list.substr(0, pos));
//       sn_list.erase(0, pos + 1);
//     }
//     cmdline_broadcast_code.push_back(sn_list);
//   }
//   if (cmd.exist("log")) {
//     printf("Save the log file.\n");
//     SaveLoggerFile();
//   }
//   return;
// }

/*
 *关于串口：1、使用c++11相关的新串口库。2、因为建立了收发信息对应的多线程，因此要去了解原子的坑。
 3、在收信息时可能要写18条store（避免线程互斥的操作，接受之后用于处理数据），所以要用指针遍历结构体，用for。  
    robo_inf.yaw_angle.store(robo_inf_uart_temp.yaw_angle);
4、cmake相关不用单独写一个CMakeList，直接添加到总CMakeList即可。
 */
int main () 
{
    	// VideoCapture capture("/home/joyce/视频/闸门闪烁/闸门闪烁6.gif");
    //无法与相机共同开启使用
    // SetProgramOption(argc, argv);

    // LdsLidar& read_lidar = LdsLidar::GetInstance();

    // int ret = read_lidar.InitLdsLidar(cmdline_broadcast_code);
    // if (!ret) {
    //   printf("Init lds lidar success!\n");
    // } else {
    //   printf("Init lds lidar fail!\n");
    // }

    // printf("Start discovering device.\n");

    // std::this_thread::sleep_for(std::chrono::seconds(100));

    // read_lidar.DeInitLdsLidar();
    // printf("Livox lidar demo end!\n");

    //--------------------------------------------------------------------
    RoboInf robo_inf;
    RoboCmd robo_cmd;
    // auto streamer_ptr = std::make_shared<nadjieb::MJPEGStreamer>();
    // streamer_ptr->start(8080);
    robo_cmd.is_left=2;
    // robo_cmd.turn_angle=2.0;
    auto serial = std::make_shared<RoboSerial>("/dev/ttyUSB0", 115200);

    // std::thread uart_read_thread(uartReadThread, serial, std::ref(robo_inf));
    // uart_read_thread.detach();

    // std::thread uart_write_thread(uartWriteThread, serial, std::ref(robo_cmd));
    // uart_write_thread.detach();


    cout<<"??????????????????????????????"<<endl;
    std::cout<<"main_my_color:"<<robo_inf.my_color<<std::endl;
    if(robo_inf.my_color==0)
    {
        out_robo_inf.my_color=RED;
        cout<<"zeor"<<endl;
    }else
    {
        // out_robo_inf.my_color=BLUE;
        cout<<"one"<<endl;
    }
    
    // std::cout<<"R_Hero_HP:"<<robo_inf.R_Hero_HP<<std::endl;
    // std::cout<<"R_Engineer_HP:"<<robo_inf.R_Engineer_HP<<std::endl;
    // std::cout<<"R_Infantry3_HP:"<<robo_inf.R_Infantry3_HP<<std::endl;
    // std::cout<<"R_Infantry4_HP:"<<robo_inf.R_Infantry4_HP<<std::endl;
    // std::cout<<"R_Infantry5_HP:"<<robo_inf.R_Infantry5_HP<<std::endl;
    // std::cout<<"R_Infantry5_HP:"<<robo_inf.R_Sentry_HP<<std::endl;
    // std::cout<<"R_Outpost_HP:"<<robo_inf.R_Outpost_HP<<std::endl;
    // std::cout<<"R_Base_HP:"<<robo_inf.R_Base_HP<<std::endl;

    // std::cout<<"B_Hero_HP:"<<robo_inf.B_Hero_HP<<std::endl;
    // std::cout<<"B_Engineer_HP:"<<robo_inf.B_Engineer_HP<<std::endl;
    // std::cout<<"B_Infantry3_HP:"<<robo_inf.B_Infantry3_HP<<std::endl;
    // std::cout<<"B_Infantry4_HP:"<<robo_inf.B_Infantry4_HP<<std::endl;
    // std::cout<<"B_Infantry5_HP:"<<robo_inf.B_Infantry5_HP<<std::endl;
    // std::cout<<"B_Sentry_HP:"<<robo_inf.B_Sentry_HP<<std::endl;
    // std::cout<<"B_Outpost_HP:"<<robo_inf.B_Outpost_HP<<std::endl;
    // std::cout<<"B_Base_HP:"<<robo_inf.B_Base_HP<<std::endl;

    //————————————————————————————————————————————————————
    int change=1;
    int change_map=1;

    //视频录制
    string video_file="/home/joyce/workplace/rm/2022/code/";
    // cout<<sin((30*3.1415926)/180)<<endl;

    #if SLOPE_FLYING_RECORD == 1
    string record_date="";
    record_date=getCurrentTimeStr();
    VideoWriter vw;
    vw.open(video_file+"darts-"+record_date+".avi",VideoWriter::fourcc('M', 'J', 'P', 'G'),30,Size(1280,800));
    #endif

    #if DARTS_OPEN_RECORD == 1
    string record_date_1="";
    record_date_1=getCurrentTimeStr();
    VideoWriter vw1;
    vw1.open(video_file+"radar-"+record_date_1+".avi",VideoWriter::fourcc('M', 'J', 'P', 'G'),30,Size(1280,800));
    #endif
    //——————————————————————————————————
    TRTModule model("/home/joyce/workplace/rm/2022/code/KCf/asset/model-opt-3.onnx");

    // uart::SerialPort serial_ = uart::SerialPort("/home/joyce/workplace/rm/2022/KCf/configs/serial/uart_serial_config.xml");

    basic_pnp::PnP pnp_ = basic_pnp::PnP("/home/joyce/workplace/rm/2022/KCf/configs/camera/mv_camera_config_555.xml", 
                                         "/home/joyce/workplace/rm/2022/KCf/configs/angle_solve/basic_pnp_config.xml");

    mindvision::VideoCapture* mv_capture_ = new mindvision::VideoCapture(
    mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_20000),3);
    cv::VideoCapture cap_ = cv::VideoCapture(3);

    mindvision::VideoCapture* mv_capture_1 = new mindvision::VideoCapture(
    mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_20000),4);
    cv::VideoCapture cap_1 = cv::VideoCapture(4);

    VideoCapture cap0(0);
    VideoCapture cap1(2);
    
	cap0.set(CAP_PROP_FRAME_WIDTH,640);
    cap0.set(CAP_PROP_FRAME_HEIGHT,480);
    cap0.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25); // where 0.25 means "manual exposure, manual iris"
    cap0.set(CAP_PROP_EXPOSURE, 50);
    cap0.set(CAP_PROP_BRIGHTNESS,90);

    cap1.set(CAP_PROP_FRAME_WIDTH,640);
    cap1.set(CAP_PROP_FRAME_HEIGHT,480);
    cap1.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25); // where 0.25 means "manual exposure, manual iris"
    cap1.set(CAP_PROP_EXPOSURE, 50);
    cap1.set(CAP_PROP_BRIGHTNESS,90);
    Mat background,foreground,foreground_BW;
    Mat mid_filer;   //中值滤波法后的照片
    Mat frame_0;
    // Mat rm_map_bule=imread("/home/joyce/workplace/rm/2022/code/rm-map-bule.png");
    // Mat rm_map_red=imread("/home/joyce/workplace/rm/2022/code/rm-map-red.png");
    Mat result;
    int count_num=0;
    // static int count_num=0;

    ROI roi;
    // cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	// cap_.set(cv::CAP_P ROP_FRAME_HEIGHT, 800);
    // cap_.set(cv::CAP_PROP_AUTO_EXPOSURE,1);
    // cap_.set(cv::CAP_PROP_EXPOSURE, 800);

    // int fps = 0, fps_count = 0;
    // auto t1 = system_clock::now();
    // int cnt_useless = -1;
	cv::Mat img;//装甲板
    cv::Mat img1;//飞镖.
	cv::Mat img2;//装甲板
    cv::Mat img3;//飞镖.
    // cv::Mat frame = cv::Mat(900, 1500, CV_8UC3);

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
   	// namedWindow("Radar picture",WINDOW_NORMAL);
    // setWindowProperty("Radar picture", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);    
    // cvui::image(frame, 20, 20, rm_map_bule);
            // cv::imshow("map", rm_map_bule);

    // cv::namedWindow(WINDOW_NAME,WINDOW_NORMAL);
    
	// cvui::init("WINDOW_NAME");

    while (true) 
    {

		// 记录起始的时钟周期数
        double time = (double)getTickCount();
        cap0>>img2;
        // cout<<"cap0:"<<cap0.isOpened()<<endl;
        cap1>>img3;
        // frame = cv::Scalar(49, 52, 49);
        // armor(mv_capture_,img,cap_,
        //         serial_,colors,
        //         pnp_, yaw_angle, last_yaw_angle, enemy_robot_v,
        //         forecast_dist,compensate_w,time,0);

                // imshow("0",img);
        // darts_Door(mv_capture_1,img1,cap_1,
        //         count_num,mid_filer,background,frame_0,foreground,
        //          foreground_BW, element);
    
        if (
            mv_capture_->isindustryimgInput()
        &&
        mv_capture_1->isindustryimgInput()
        ) 
        {
            img = mv_capture_->image();
            img1=mv_capture_1->image();

        } else {
            cap_.read(img);
            cap_1.read(img1);
        }
        if (
            (!img1.empty()
            )
        ||
        (!img.empty())
        )  
        {
            //_______________地图阵营选择，默认我方颜色在下方__________________//
            // if(change_map==1)//地图阵营切换
            // {
            //     rm_map=rm_map_red;
            //     drawMap(frame,rm_map,count_num);
                
            // }else if(change_map==2)
            // {
            //     rm_map=rm_map_bule;
            //     drawMap(frame,rm_map,count_num);
            // }
            // if (cvui::button(frame, 20, 1100, "red",1)) //我方阵营选择按钮：红方
            // {
            //     change_map=1;
		    // }
            // if (cvui::button(frame, 180, 1100, "blue",1)) //我方阵营选择按钮：蓝方
            // {
            //     change_map=2;
		    // }
            //______________________________视频录制______________________________//
            #if SLOPE_FLYING_RECORD == 1
                vw.write(img);
            #endif

            #if DARTS_OPEN_RECORD == 1
                vw1.write(img1);
            #endif
            //——————————————————————————————
            // count_num=count_num+1;
            // cv::imshow("0", img);

            // darts_roi=img(Rect((img.cols/2)-20,(img.rows/2)-20,140,140));
	    	// rectangle(img, Rect((img.cols/2)-20,(img.rows/2)-20,140,140), Scalar(255, 255, 0), 2);	//在原图像上画出矩形
		    // GaussianBlur(darts_roi, darts_roi, Size(5, 5), 1, 1);	
            // // imshow("darts_roi",darts_roi);  
	        // cvtColor( darts_roi,mid_filer, COLOR_RGB2GRAY );

            // if(count_num==1)
	        // {
		    //     background=mid_filer.clone();
		    //     frame_0=background;
	        // }
	        // else
	        // {
		    //     background=frame_0; 
	        // }
            // absdiff(mid_filer,background,foreground);//用帧差法求前景
	        // threshold( foreground, foreground_BW, 120, 255 , 0 );//二值化通常设置为50  255
// 	   	    // dilate(foreground_BW,foreground_BW,element);

	        // vector<Vec4i> hierarchy;
            // vector<vector<Point>> contours;
	        // findContours(foreground_BW,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE,Point());//寻找并绘制轮廓
            // // imshow("mask",foreground_BW);

            // vector<Moments>mu(contours.size());
            // for(unsigned i=0;i<contours.size();i++)//计算轮廓面积
            // {
            //     mu[i]=moments(contours[i],false);
            // }
            // for(int i=0;i<contours.size();i++)//最小外接矩形
            // {
	    	//     Rect rect = boundingRect(contours[i]);	//找出轮廓最小外界矩形
	    	// 	// cout << "矩形框" << rect.width << endl;
	    	// 	// if(mu[i].m00<2000&&mu[i].m00>125)
	    	// 	if(mu[i].m00<1300)//范围需要调整
	    	// 	{
	    	// 		rectangle(darts_roi, rect, Scalar(0, 255, 0), 3);	//在原图像上画出矩形
            //         // warning_Lights(rm_map,count_num,410,90);
            //         // if((sin((count_num*3.1415926)*8/180))>0)//||(sin((count_num*3.1415926*0.5)/180))<-0.5
            //         // {
            //         //     circle(rm_map,Point(410,90),18,Scalar(255,0,0),-1);
            //         //     // sleep(1);
            //         // }
            //         // else {
            //         //     circle(rm_map,Point(410,90),18,Scalar(0,0,255),-1);
            //         //     // count_num = 2;
            //         // }	    		
            //     // }else{
            //         // drawMap(frame,rm_map,count_num);
            //     }
	        // }
            // frame_0=mid_filer.clone();
            // putText(img,"darts",Point(5,790),FONT_HERSHEY_PLAIN,2.0,Scalar(255,255,255),2);            
            #if RECORD == 1
                vw.write(img);
            #endif
            // imshow("0",img);
            //--------------------------------------------
            // std::array<double, 4> q;
            // double timestamp = 0.0;
            // std::vector<armor_data> data_armor;
            // // // const auto& [img, q, timestamp] = sensor_sub.pop();
            // auto detections = model(img1);
            // armor_data armor;
            // // /* publish detection results */

            // //串口发送
            // // robo_cmd.is_left.store(1);
            // // robo_cmd.is_left.store(2.0);
            // // cout<<"main_is_left:"<<robo_cmd.is_left<<endl;
            // // cout<<"main_turn_angle:"<<robo_cmd.turn_angle<<endl;

            // // cv::Rect rect_fly(37,260,180,180);
            // // cv::Rect rect_energy(230,310,180,180);
            // /* show detections */
            // if(!detections.empty()) {
            //     cv::Mat im2show = img.clone();
            //     // for (const auto &b: detections) {
            //     for (int i = 0; i < detections.size(); i++) {
            //         cv::line(img1, detections[i].pts[0], detections[i].pts[1], colors[1], 6);
            //         cv::line(img1, detections[i].pts[1], detections[i].pts[2], colors[1], 6);
            //         cv::line(img1, detections[i].pts[2], detections[i].pts[3], colors[1], 6);
            //         cv::line(img1, detections[i].pts[3], detections[i].pts[0], colors[1], 6);
            //         cv::putText(img1, std::to_string(detections[i].tag_id), detections[i].pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[detections[i].color_id]);
            //         armor.color_id = detections[i].color_id;
            //         armor.tag_id   = detections[i].tag_id;
            //         armor.pts[0]   = detections[i].pts[0];
            //         armor.pts[1]   = detections[i].pts[1];
            //         armor.pts[2]   = detections[i].pts[2];
            //         armor.pts[3]   = detections[i].pts[3];
            //         armor.confidence = detections[i].confidence;
            //         armor.img_center_dist = getDistance((detections[i].pts[0] + detections[i].pts[3]) * 0.5, cv::Point(img1.cols * 0.5, img1.rows * 0.5 + 100));
            //         // robo_cmd.is_left.store(1);
            //         // robo_cmd.is_left.store(180);
            //         //-------------------------------//

            //         //  && detections[i].tag_id != 2 
            //         // cout<<"point:("<<detections[i].pts[0]<<","<<detections[i].pts[1]<<")"<<endl;
            //         // for(int j=0;j<4;j++)
            //         // {
            //         //     if(fly_or_engery(detections[i].pts[j].x,detections[i].pts[j].y,rect_fly))//装甲板是否出现在飞坡区域
            //         //     {
            //         //         warning_Lights(rm_map,count_num,92,480);
            //         //     }
            //         //     if(fly_or_engery(detections[i].pts[j].x,detections[i].pts[j].y,rect_energy))//装甲板是否出现在能量机关击打区域
            //         //     {
            //         //         warning_Lights(rm_map,count_num,150,340);
            //         //     }

            //         // }
            //         // if (serial_.returnReceiceColor() != detections[i].color_id && detections[i].confidence > 0.5 ) 
            //         // {
            //         //     data_armor.push_back(armor);
            //         // }
                    
            //     }
            // }
            // putText(img1,"feipo",Point(5,790),FONT_HERSHEY_PLAIN,2.0,Scalar(255,255,255),2);
            // cv::Rect rect_fly(37,260,70,110);
            // cv::Rect rect_energy(112,310,72,150);
            // cvui::printf(frame, 1550, 90, 0.6, 0xff0000, "count_num: %d", count_num);
            // rectangle(img1,rect_fly,Scalar(0,255,0),2);//敌方飞坡区域
            // rectangle(img1,rect_energy,Scalar(0,255,0),2);//敌方大能量机关击打区域
            
            // drawMap(frame,rm_map,count_num);
            
            // // fps_count++;
            time = ((double)getTickCount() - time) / getTickFrequency();
            int fps = 1 / time;
            // // cv::putText(img1, fmt::format("fps={}", fps), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0));
            // cout<<"fps:"<<fps<<endl;                                                                                                                           
            // frame_0=mid_filer.clone();
            // cv::imshow("map", rm_map_bule);
            
            // //_______________地图阵营选择，默认我方颜色在下方__________________//
            // if(change_map==1)//地图阵营切换，
            // {
            //     drawMap(frame,rm_map_red,count_num);
            // }else if(change_map==2)
            // {
            //     drawMap(frame,rm_map_bule,count_num);
            // }
            // if (cvui::button(frame, 20, 1100, "red",1)) //我方阵营选择按钮：红方
            // {
            //     change_map=1;
		    // }
            // if (cvui::button(frame, 180, 1100, "blue",1)) //我方阵营选择按钮：蓝方
            // {
            //     change_map=2;
		    // }
            //__________________________________________________
            // if(change==1)//主视频源视角切换
            {
                // cvui::image(frame, 180, 20, img);
                // putText(img,"飞镖",Point(100,250),FONT_HERSHEY_PLAIN,4.0,Scalar(0,255,255),2);            
            }
            // else if(change==2)
            {
                // cvui::image(frame, 180, 20, img1);
            }
            // if (cvui::button(frame, 1780, 850, "darts",1.2)) //飞镖视角按钮
            {
                // change=1;
                // cvui::image(frame, 700, 20, img);
		    }
            // if (cvui::button(frame, 1780, 950, "feipo",1.2)) //飞坡视角按钮
            {
                // change=2;
                // cvui::image(frame, 700, 20, img1);
		    }

            // if (cvui::button(frame, 1780, 850, "exit",1.2)) 
            {
                // break;
		    }
            // cout<<"???"<<endl;
            //_______________________________
            // cvui::printf(frame, 695, 830, 1.3, 0xff0000, "Red:");
            // cvui::printf(frame, 695, 870, 0.9, 0xFFFFFF, "R1:");
            // cvui::printf(frame, 695, 900, 0.9, 0xFFFFFF, "R2:");
            // cvui::printf(frame, 695, 930, 0.9, 0xFFFFFF, "R3:");
            // cvui::printf(frame, 695, 960, 0.9, 0xFFFFFF, "R4:");
            // cvui::printf(frame, 695, 990, 0.9, 0xFFFFFF, "R5:");
            
            // cvui::printf(frame, 695, 1050, 0.9, 0xFFFFFF, "--------");
            // cvui::printf(frame, 695, 1080, 0.9, 0xFFFFFF, "R7:");
            // cvui::printf(frame, 695, 1110, 0.9, 0xFFFFFF, "RO:");
            // cvui::printf(frame, 695, 1140, 0.9, 0xFFFFFF, "RB:");
            //_______________________________
            // cvui::update();
            
            
            //拼接__________________________//
            // imgs.push_back(img);
            // imgs.push_back(img1);

            hconcat(img,img1,result);
            imshow("3",img3);
            imshow("2",img2);
            // imshow("1",img1);
            // imshow("0",img);

		    cv::imshow(WINDOW_NAME, result);
            // std::cout <<"serial_is:" <<serial_.isEmpty() << std::endl;
            // std::cout <<"serial:" <<serial_.returnReceiceColor() << std::endl;
            if(cv::waitKey(1) == 'q') {
                // uart_read_thread.~thread();
                break;
            }
        }

        mv_capture_->cameraReleasebuff();        
        mv_capture_1->cameraReleasebuff();

    }
}
