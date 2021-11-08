// #include <opencv2/core/utility.hpp>
// #include <opencv2/tracking.hpp>
// #include <opencv2/videoio.hpp>
// #include <opencv2/highgui.hpp>
// #include <iostream>
// #include <cstring>
// #include "TRTModule.hpp"
// #include <fmt/format.h>
// #include <fmt/color.h>
// #include "angle_solve/basic_pnp.hpp"
// #include "camera/mv_video_capture.hpp"
// #include "serial/uart_serial.hpp"

// using namespace std;
// using namespace cv;

// static bool debug = true;

// struct Detection_pack{
//     /*
//      * 打包数据结构，将识别结果、对应的图像、陀螺仪和时间戳对应
//      */
//     std::vector<bbox_t> detection;
//     cv::Mat img;
//     std::array<double, 4> q;
//     double timestamp;
// };

// float getDistance(const cv::Point a, const cv::Point b) {
//   return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
// }
// struct armor_data{
//     cv::Point2f pts[4];
//     float img_center_dist;
//     int color_id; // 0: blue, 1: red, 2: gray
//     int tag_id;   // 0: guard, 1-5: number, 6: base
//     float confidence; // 识别准确率
// };

// struct ROI {
//     bool ROI_selected = false;
//     cv::Rect2f ROI_bbox;
//     int last_class = -1;

//     ROI()=default;
//     ROI(cv::Rect2f &bbox, int &last): ROI_selected(true), ROI_bbox(bbox), last_class(last) {}
//     inline void clear() {
//         ROI_selected = false;
//         last_class = -1;
//     }
//     ~ROI()=default;
// };

// // 计算任意四边形的中心
// cv::Point2f points_center(cv::Point2f pts[4]){
//     for (int i = 0; i < 4; ++i) {
//         for (int j = i+1; j < 4; ++j) {
//             if (pts[i] == pts[j]) {
//                 std::cout << "[Error] Unable to calculate center point." << std::endl;
//                 return cv::Point2f{0, 0};
//             }
//         }
//     }
//     cv::Point2f center(0, 0);
//     if (pts[0].x == pts[2].x && pts[1].x == pts[3].x) {
//         std::cout << "[Error] Unable to calculate center point." << std::endl;
//     }
//     else if (pts[0].x == pts[2].x && pts[1].x != pts[3].x) {
//         center.x = pts[0].x;
//         center.y = (pts[3].y-pts[1].y)/(pts[3].x-pts[1].x)*(pts[0].x-pts[3].x)+pts[3].y;
//     }
//     else if (pts[1].x == pts[3].x && pts[0].x != pts[2].x) {
//         center.x = pts[1].x;
//         center.y = (pts[2].y-pts[0].y)/(pts[2].x-pts[0].x)*(pts[1].x-pts[0].x)+pts[0].y;
//     }
//     else {
//         center.x = (((pts[3].y-pts[1].y)/(pts[3].x-pts[1].x)*pts[3].x - pts[3].y + \
//                     pts[0].y - (pts[2].y-pts[0].y)/(pts[2].x-pts[0].x)*pts[0].x)) / \
//                     ((pts[3].y-pts[1].y)/(pts[3].x-pts[1].x)-(pts[2].y-pts[0].y)/(pts[2].x-pts[0].x));
//         center.y = (pts[2].y-pts[0].y)/(pts[2].x-pts[0].x)*(center.x-pts[0].x)+pts[0].y;
//     }

//     return center;
// }

// // 四点转化为矩形
// inline cv::Rect2f get_ROI(armor_data &armor, float coefficient = 1.0f) {
//     auto center = points_center(armor.pts);
//     auto w = std::max({armor.pts[0].x, armor.pts[1].x, armor.pts[2].x, armor.pts[3].x}) -
//             std::min({armor.pts[0].x, armor.pts[1].x, armor.pts[2].x, armor.pts[3].x});
//     auto h = std::max({armor.pts[0].y, armor.pts[1].y, armor.pts[2].y, armor.pts[3].y}) -
//             std::min({armor.pts[0].y, armor.pts[1].y, armor.pts[2].y, armor.pts[3].y});
//     return cv::Rect2f(center.x-w/2, center.y-h/2, w*coefficient, h*coefficient);
// }

// bool last_shoot = false;            // 判断上一次是否击打

// void detection_run(const std::string &onnx_file) {
//     /*
//      * 识别器
//      */

//     // if (debug) std::cout << "============ detection_run ===========" << std::endl;
//     TRTModule model(onnx_file);

//     uart::SerialPort serial_ = uart::SerialPort("/home/xx/code/KCf/configs/serial/uart_serial_config.xml");

//     basic_pnp::PnP pnp_ = basic_pnp::PnP("/home/xx/code/KCf/configs/camera/mv_camera_config_555.xml", 
//                                          "/home/xx/code/KCf/configs/angle_solve/basic_pnp_config.xml");

//     mindvision::VideoCapture* mv_capture_ = new mindvision::VideoCapture(
//         mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_2500));

//     cv::VideoCapture cap_ = cv::VideoCapture(0);
    
//     ROI roi;
//     // cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
// 	// cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
//     armor_data last_sbbox;
//     int fps = 0, fps_count = 0;
//     // auto t1 = system_clock::now();
//     int cnt_useless = -1;
// 	cv::Mat img;
//     const cv::Scalar colors[4] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}, {255, 255, 255}};

// 	const double fx = 1755.8568155966806899;
//     const double fy = 1756.0870281269756106;
//     const double cx = 649.2466252010661947;
//     const double cy = 487.2552385352113333;
    
    
//     std::vector<cv::Point2f> target_2d;
//     while (true) {

// 		// 记录起始的时钟周期数
//         double time = (double)getTickCount();
//         if (mv_capture_->isindustryimgInput()) {
//             img = mv_capture_->image();
//         } else {
//             cap_.read(img);
//         }
//         if (!img.empty()) {
//             std::array<double, 4> q;
//             double timestamp = 0.0;
//             std::vector<armor_data> data_armor;
//             // const auto& [img, q, timestamp] = sensor_sub.pop();
//             auto detections = model(img);
//             armor_data armor;

//             /* show detections */
//             if(!detections.empty()) {
//                 cv::Mat im2show = img.clone();
//                 // for (const auto &b: detections) {
//                 for (int i = 0; i < detections.size(); i++) {
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
//             if (!data_armor.empty()) {
                
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
//             data_armor.clear();
//             data_armor.shrink_to_fit();
//             fps_count++;
//             time = ((double)getTickCount() - time) / getTickFrequency();
//             time = 1 / time;
//             cv::putText(img, fmt::format("fps={}", time), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
//             cv::imshow("SJUT", img);
//             if(cv::waitKey(1) == 'q') {
//                 break;
//             }
//         }
//         mv_capture_->cameraReleasebuff();
//     }
// }

// int main() {
// 	detection_run("/home/xx/code/KCf/asset/model-opt-4.onnx");
// }