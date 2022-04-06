/*
 * @Author: your name
 * @Date: 2022-03-28 21:18:32
 * @LastEditTime: 2022-04-01 22:09:33
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /code/KCf/devices/new_serial/serial.hpp
 */
#pragma once
#include "serial/serial.h"
// #include "utils.hpp"
#include "/home/joyce/workplace/rm/2022/code/utils.hpp"
auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "serial");
auto idntifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "serial");

class RoboSerial : public serial::Serial {
 public:
  RoboSerial(std::string port, unsigned long baud) {
    auto timeout = serial::Timeout::simpleTimeout(serial::Timeout::max());
    this->setPort(port);
    this->setBaudrate(baud);
    this->setTimeout(timeout);
    try {
      this->open();
      fmt::print("[{}] Serial init successed.\n", idntifier_green);
    } catch(const std::exception& e) {
      fmt::print("[{}] Serial init failed, {}.\n", idntifier_red, e.what());
    }
  }

  void WriteInfo(RoboCmd &robo_cmd) {
    RoboCmdUartBuff robo_cmd_uart_temp;
    robo_cmd_uart_temp.yaw_angle = robo_cmd.yaw_angle.load();
    robo_cmd_uart_temp.pitch_angle = robo_cmd.pitch_angle.load();
    robo_cmd_uart_temp.depth = robo_cmd.depth.load();
    robo_cmd_uart_temp.detect_object = robo_cmd.detect_object.load();

    this->write((uint8_t *)&robo_cmd_uart_temp, sizeof(robo_cmd_uart_temp));
  }

  void ReceiveInfo(RoboInf &robo_inf) {
    RoboInfUartBuff robo_inf_uart_temp;
    uint8_t temp='S';
    this->read(&temp, 1);
    // this->read(&temp, sizeof(robo_inf));
    std::cout<<"temp:"<<temp<<std::endl;

    while (temp != 'S')
    {
      this->read(&temp, 3);
      std::cout<<"temp is S?"<<std::endl;
    }
    this->read((uint8_t *)&robo_inf_uart_temp, sizeof(robo_inf_uart_temp));
    // robo_inf.yaw_angle.store(robo_inf_uart_temp.yaw_angle);
    // robo_inf.my_color.store(robo_inf_uart_temp.my_color);
    // for(int i=0;i!=sizeof(robo_inf.Receive_Red_HP);i++)
    std::cout<<"size:"<<sizeof(robo_inf)<<std::endl;
    std::cout<<"red_hero_start:"<<robo_inf_uart_temp.Receive_Information.R_Hero_HP<<std::endl;
    std::cout<<"red_eng_start:"<<robo_inf_uart_temp.Receive_Information.R_Engineer_HP<<std::endl;
    std::cout<<"red_R_Infantry3_HP_start:"<<robo_inf_uart_temp.Receive_Information.R_Infantry3_HP<<std::endl;


    {
      robo_inf.Receive_Information.R_Hero_HP.store(robo_inf_uart_temp.Receive_Information.R_Hero_HP);
      robo_inf.Receive_Information.R_Engineer_HP.store(robo_inf_uart_temp.Receive_Information.R_Engineer_HP);
      robo_inf.Receive_Information.R_Infantry3_HP.store(robo_inf_uart_temp.Receive_Information.R_Infantry3_HP);
      robo_inf.Receive_Information.R_Infantry4_HP.store(robo_inf_uart_temp.Receive_Information.R_Infantry4_HP);
      robo_inf.Receive_Information.R_Infantry5_HP.store(robo_inf_uart_temp.Receive_Information.R_Infantry5_HP);
      robo_inf.Receive_Information.R_Sentry_HP.store(robo_inf_uart_temp.Receive_Information.R_Sentry_HP);
      robo_inf.Receive_Information.R_Outpost_HP.store(robo_inf_uart_temp.Receive_Information.R_Outpost_HP);
      robo_inf.Receive_Information.R_Base_HP.store(robo_inf_uart_temp.Receive_Information.R_Base_HP);
    }
    {
      robo_inf.Receive_Information.B_Hero_HP.store(robo_inf_uart_temp.Receive_Information.B_Hero_HP);
      robo_inf.Receive_Information.B_Hero_HP.store(robo_inf_uart_temp.Receive_Information.B_Hero_HP);
      robo_inf.Receive_Information.B_Engineer_HP.store(robo_inf_uart_temp.Receive_Information.B_Engineer_HP);
      robo_inf.Receive_Information.B_Infantry3_HP.store(robo_inf_uart_temp.Receive_Information.B_Infantry3_HP);
      robo_inf.Receive_Information.B_Infantry4_HP.store(robo_inf_uart_temp.Receive_Information.B_Infantry4_HP);
      robo_inf.Receive_Information.B_Infantry5_HP.store(robo_inf_uart_temp.Receive_Information.B_Infantry5_HP);
      robo_inf.Receive_Information.B_Sentry_HP.store(robo_inf_uart_temp.Receive_Information.B_Sentry_HP);
      robo_inf.Receive_Information.B_Outpost_HP.store(robo_inf_uart_temp.Receive_Information.B_Outpost_HP);
      robo_inf.Receive_Information.B_Base_HP.store(robo_inf_uart_temp.Receive_Information.R_Base_HP);

    }
    std::cout<<"red_hero:"<<robo_inf.Receive_Information.R_Hero_HP<<std::endl;
    std::cout<<"red_eng:"<<robo_inf.Receive_Information.R_Engineer_HP<<std::endl;
    std::cout<<"red_R_Infantry3_HP:"<<robo_inf.Receive_Information.R_Infantry3_HP<<std::endl;


    // return robo_inf_uart_temp;
  }

 private:
};