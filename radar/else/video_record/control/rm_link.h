/**
 * @file rm_link.h
 * @author GCUROBOT-Visual-Group (GCUROBOT_WOLF@163.com)
 * @brief RM 2019 步兵视觉各接口链接头文件
 * @version 1.1
 * @date 2019-05-06
 * @copyright Copyright (c) 2019 GCU Robot Lab. All Rights Reserved.
 */
#ifndef RM_LINK_H
#define RM_LINK_H

// #include "configure.h"
#include "debug_control.h"
#include "/home/joyce/workplace/rm/2022/code/radar/camera/mv_video_capture.hpp"


class RM_Vision_Init
{
private:
    cv::Mat dst_img_;

public:  
    RM_Vision_Init();
    ~RM_Vision_Init();
    void Run();
    
    void PHOTO_RUN();
    #if ANALYZE_EACH_FRAME == 1
    bool is_continue();
    #endif  

    bool is_exit();

    void close();
    int getFileSizeInOrder(const std::string &path);//(const std::string &path, const std::string &format);
    /** Camera Srart **/
    cv::VideoCapture capture;
    mindvision::VideoCapture cap;
    /** Camera Srart **/

    /** param initial **/
    cv::Mat src_img;

    /** param initial **/
    cv::Mat frame_img;
#if RECORD == 1
    VideoWriter writer;
#endif
    int n = 0;
    int th;
    int energy_refresh_count = 0;
    char filename[200];
};

#endif // RM_LINK_H
