/*
 * @Author: your name
 * @Date: 2022-03-28 21:18:32
 * @LastEditTime: 2022-07-26 22:21:36
 * @LastEditors: JoyceKirkland joyce84739879@163.com
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
    // std::cout<<"write"<<std::endl;
    RoboCmdUartBuff robo_cmd_uart_temp;
    robo_cmd_uart_temp.is_left = robo_cmd.is_left.load();
    // robo_cmd_uart_temp.turn_angle = robo_cmd.turn_angle.load();

    this->write((uint8_t *)&robo_cmd_uart_temp, sizeof(robo_cmd_uart_temp));
  }

  void ReceiveInfo(RoboInf &robo_inf) {
    //  std::cout<<"????"<<std::endl;
    RoboInfUartBuff robo_inf_uart_temp;
    uint8_t temp;
    this->read(&temp, 1);
    
    // while (temp != 'S')
    {
      this->read(&temp, 1);
      // std::cout<<"temp_:"<<(int8_t)temp<<std::endl;

    std::cout<<"recive"<<std::endl;

      }
          // std::cout<<"temp_:"<<(char)temp<<std::endl;

    std::cout<<"temp:"<<sizeof(temp)<<std::endl;

    this->read((uint8_t *)&robo_inf_uart_temp, sizeof(robo_inf_uart_temp));
    
   
    // for(int i=0;i!=sizeof(robo_inf.Receive_Red_HP);i++)
      robo_inf.my_color.store(robo_inf_uart_temp.my_color);

      // robo_inf.R_Hero_HP.store(robo_inf_uart_temp.R_Hero_HP);
      // robo_inf.R_Engineer_HP.store(robo_inf_uart_temp.R_Engineer_HP);
      // robo_inf.R_Infantry3_HP.store(robo_inf_uart_temp.R_Infantry3_HP);
      // robo_inf.R_Infantry4_HP.store(robo_inf_uart_temp.R_Infantry4_HP);
      // robo_inf.R_Infantry5_HP.store(robo_inf_uart_temp.R_Infantry5_HP);
      // robo_inf.R_Sentry_HP.store(robo_inf_uart_temp.R_Sentry_HP);
      // robo_inf.R_Outpost_HP.store(robo_inf_uart_temp.R_Outpost_HP);
      // robo_inf.R_Base_HP.store(robo_inf_uart_temp.R_Base_HP);
    
      
      // robo_inf.B_Hero_HP.store(robo_inf_uart_temp.B_Hero_HP);
      // robo_inf.B_Engineer_HP.store(robo_inf_uart_temp.B_Engineer_HP);
      // robo_inf.B_Infantry3_HP.store(robo_inf_uart_temp.B_Infantry3_HP);
      // robo_inf.B_Infantry4_HP.store(robo_inf_uart_temp.B_Infantry4_HP);
      // robo_inf.B_Infantry5_HP.store(robo_inf_uart_temp.B_Infantry5_HP);
      // robo_inf.B_Sentry_HP.store(robo_inf_uart_temp.B_Sentry_HP);
      // robo_inf.B_Outpost_HP.store(robo_inf_uart_temp.B_Outpost_HP);
      // robo_inf.B_Base_HP.store(robo_inf_uart_temp.B_Base_HP);

    
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


    // std::cout<<"????"<<std::endl;

    // return robo_inf_uart_temp;
  }

 private:
};