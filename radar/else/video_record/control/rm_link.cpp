#define SAVE_AVI_PATH "./Lvaotian.avi"
#include "rm_link.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <string>
#include <ctime>
#include <chrono>
#include<fstream>
/**
 * @brief Construct a new rm vision init::rm vision init object
 *
 */
RM_Vision_Init::RM_Vision_Init():capture(USB_CAPTURE_DEFULT),cap(ISOPEN_INDUSTRY_CAPTURE){
#if RECORD == 1
        //printf(CONFIG_FILE_PATH);
        String out_path = SAVE_AVI_PATH+to_string(RM_Vision_Init::getFileSizeInOrder(SAVE_AVI_PATH))+".avi";//目标路径
        Size size(640, 480);//要求与摄像头参数一致
        // int fourcc = writer.fourcc('X', 'V', 'I', 'D');   // 设置avi文件对应的编码格式 66 67
        int fourcc = writer.fourcc('M', 'J', 'P', 'G'); // 33 30 48Flv1
        writer.open(out_path,  fourcc, 30, size, true);//CAP_DSHOW = true
        if(writer.isOpened()){
           cout<<"正在录制"<<endl;
        }
        else{
           cout<<"录制失败"<<endl;
        }
#endif
}

/**
 * @brief Destroy the rm vision init::rm vision init object
 *
 */
RM_Vision_Init::~RM_Vision_Init(){
#if RECORD == 1
    writer.release();
#endif
    capture.release();
}

/**
 * @brief 视觉功能执行函数
 *
 */
void RM_Vision_Init::Run(){

#if RECORD == 1
    //get Image
    if(cap.isindustryimgInput()){
        src_img = cvarrToMat(cap.iplImage,true);//这里只是进行指针转换，将IplImage转换成Mat类型
    }else{
        capture >> src_img;
    }
    imshow("src_img",src_img);
    writer << src_img;

    cap.cameraReleasebuff();
#endif

}
void RM_Vision_Init::PHOTO_RUN()
{
#if RECORD == 2
    if(cap.isindustryimgInput()){
        src_img = cvarrToMat(cap.iplImage,true);//这里只是进行指针转换，将IplImage转换成Mat类型
    }else{
        capture >> src_img;
    }
    imshow("frame",src_img);

    cap.cameraReleasebuff();
#endif 
}
#if ANALYZE_EACH_FRAME == 1
/**
 * @brief 程序继续条件
 * 
 * @return true 继续执行
 * @return false 暂停执行
 */
bool RM_Vision_Init::is_continue()
{
    bool go_on=false;
    int key =waitKey(0);
    if((char)key== 32)
    {
        go_on=true;
    }
    else
    {
        go_on=false;
    }
    return go_on;
}
#endif

/**
 * @brief 程序退出条件
 *
 * @return true 结束程序
 * @return false 继续执行
 */
bool RM_Vision_Init::is_exit()
{
    int key = waitKey(1);//8
    #if RECORD == 1
    if(char(key) == 'q')
    {
        writer.release();
        cout<<"录制结束"<<endl;
    }
    #endif

    #if RECORD == 2
    if(char(key) == 's')
    {
        sprintf(filename, "./%.1d.jpg", n++);
        imwrite(filename, src_img);
        cout<<"Saved"<<n<<endl;
    }
    #endif
    if(char(key) == 'q')
        return true;
    else
        return false;
}
void RM_Vision_Init::close(){
    writer.release();
    String out_path = SAVE_AVI_PATH+to_string(RM_Vision_Init::getFileSizeInOrder(SAVE_AVI_PATH))+".avi";//目标路径
    Size size(640, 480);//要求与摄像头参数一致
    // int fourcc = writer.fourcc('X', 'V', 'I', 'D');   // 设置avi文件对应的编码格式 66 67
    int fourcc = writer.fourcc('M', 'J', 'P', 'G'); // 33 30 48Flv1
    writer.open(out_path,  fourcc, 30, size, true);//CAP_DSHOW = true
    writer << src_img;
}
int RM_Vision_Init::getFileSizeInOrder(const std::string &path) {
    std::string filename;
    std::string format=".avi";//avi
    int number = 0;
    while (true) {
        filename = path + std::to_string(number)+format;
        std::fstream _file;
        _file.open(filename, std::ios::in);
        if (!_file) {
            break;
        }
        number++;
    }
    return number;
}
