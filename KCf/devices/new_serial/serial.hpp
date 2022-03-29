/*
 * @Author: your name
 * @Date: 2022-03-28 21:18:32
 * @LastEditTime: 2022-03-29 17:17:17
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
    uint8_t temp;
    this->read(&temp, 1);
    while (temp != 'S')
      this->read(&temp, 1);
    this->read((uint8_t *)&robo_inf_uart_temp, sizeof(robo_inf_uart_temp));
    // robo_inf.yaw_angle.store(robo_inf_uart_temp.yaw_angle);
    robo_inf.my_color.store(robo_inf_uart_temp.my_color);
    // for(int i=0;i!=sizeof(robo_inf.Receive_Red_HP);i++)
    {
      robo_inf.Receive_Red_HP.R_Hero_HP.store(robo_inf_uart_temp.Receive_Red_HP.R_Hero_HP);
      robo_inf.Receive_Red_HP.R_Engineer_HP.store(robo_inf_uart_temp.Receive_Red_HP.R_Engineer_HP);
      robo_inf.Receive_Red_HP.R_Infantry3_HP.store(robo_inf_uart_temp.Receive_Red_HP.R_Infantry3_HP);
      robo_inf.Receive_Red_HP.R_Infantry4_HP.store(robo_inf_uart_temp.Receive_Red_HP.R_Infantry4_HP);
      robo_inf.Receive_Red_HP.R_Infantry5_HP.store(robo_inf_uart_temp.Receive_Red_HP.R_Infantry5_HP);
      robo_inf.Receive_Red_HP.R_Sentry_HP.store(robo_inf_uart_temp.Receive_Red_HP.R_Sentry_HP);
      robo_inf.Receive_Red_HP.R_Outpost_HP.store(robo_inf_uart_temp.Receive_Red_HP.R_Outpost_HP);
      robo_inf.Receive_Red_HP.R_Base_HP.store(robo_inf_uart_temp.Receive_Red_HP.R_Base_HP);
    }
    {
      robo_inf.Receive_Blue_HP.B_Hero_HP.store(robo_inf_uart_temp.Receive_Blue_HP.B_Hero_HP);
      robo_inf.Receive_Blue_HP.B_Hero_HP.store(robo_inf_uart_temp.Receive_Blue_HP.B_Hero_HP);
      robo_inf.Receive_Blue_HP.B_Engineer_HP.store(robo_inf_uart_temp.Receive_Blue_HP.B_Engineer_HP);
      robo_inf.Receive_Blue_HP.B_Infantry3_HP.store(robo_inf_uart_temp.Receive_Blue_HP.B_Infantry3_HP);
      robo_inf.Receive_Blue_HP.B_Infantry4_HP.store(robo_inf_uart_temp.Receive_Blue_HP.B_Infantry4_HP);
      robo_inf.Receive_Blue_HP.B_Infantry5_HP.store(robo_inf_uart_temp.Receive_Blue_HP.B_Infantry5_HP);
      robo_inf.Receive_Blue_HP.B_Sentry_HP.store(robo_inf_uart_temp.Receive_Blue_HP.B_Sentry_HP);
      robo_inf.Receive_Blue_HP.B_Outpost_HP.store(robo_inf_uart_temp.Receive_Blue_HP.B_Outpost_HP);
      robo_inf.Receive_Blue_HP.B_Base_HP.store(robo_inf_uart_temp.Receive_Red_HP.R_Base_HP);

    }
    // cout<<"color:"<<robo_inf.my_color<<endl;
    // return robo_inf_uart_temp;
  }

 private:
};