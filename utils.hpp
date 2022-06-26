/*
 * @Author: your name
 * @Date: 2022-03-29 15:10:42
 * @LastEditTime: 2022-06-26 16:05:30
 * @LastEditors: JoyceKirkland joyce84739879@163.com
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /code/utils.hpp
 */
#pragma once
#include <atomic>

struct RoboCmd {
  std::atomic<uint8_t> is_left = 1;
  std::atomic<float> turn_angle = 0.f;
};

struct RoboCmdUartBuff{
  uint8_t start = (unsigned)'S';
  uint8_t   is_left;//是否往哨兵轨道左移
  float   turn_angle;//转头角度
  uint8_t end = (unsigned)'E';

} __attribute__((packed));

struct RoboInf {
  // uint8_t my_color = 0;
//  union Receive_Information
    // {
      // std::atomic<uint16_t> R_Hero_HP=0;
      // std::atomic<uint16_t> R_Engineer_HP=0;
      // std::atomic<uint16_t> R_Infantry3_HP=0;
      // std::atomic<uint16_t> R_Infantry4_HP=0;
      // std::atomic<uint16_t> R_Infantry5_HP=0;
      // std::atomic<uint16_t> R_Sentry_HP=0;
      // std::atomic<uint16_t> R_Outpost_HP=0;
      // std::atomic<uint16_t> R_Base_HP=0;
    
      // std::atomic<uint16_t> B_Hero_HP=0;
      // std::atomic<uint16_t> B_Engineer_HP=0;
      // std::atomic<uint16_t> B_Infantry3_HP=0;
      // std::atomic<uint16_t> B_Infantry4_HP=0;
      // std::atomic<uint16_t> B_Infantry5_HP=0;
      // std::atomic<uint16_t> B_Sentry_HP=0;
      // std::atomic<uint16_t> B_Outpost_HP=0;
      // std::atomic<uint16_t> B_Base_HP=0;

      std::atomic<uint16_t> R_Hero_HP;
      std::atomic<uint16_t> R_Engineer_HP;
      std::atomic<uint16_t> R_Infantry3_HP;
      std::atomic<uint16_t> R_Infantry4_HP;
      std::atomic<uint16_t> R_Infantry5_HP;
      std::atomic<uint16_t> R_Sentry_HP;
      std::atomic<uint16_t> R_Outpost_HP;
      std::atomic<uint16_t> R_Base_HP;
    
      std::atomic<uint16_t> B_Hero_HP;
      std::atomic<uint16_t> B_Engineer_HP;
      std::atomic<uint16_t> B_Infantry3_HP;
      std::atomic<uint16_t> B_Infantry4_HP;
      std::atomic<uint16_t> B_Infantry5_HP;
      std::atomic<uint16_t> B_Sentry_HP;
      std::atomic<uint16_t> B_Outpost_HP;
      std::atomic<uint16_t> B_Base_HP;
    // } 
    // Receive_Information;
    
    RoboInf()
    {
      // my_color = 0;
      R_Hero_HP = 0;
      R_Engineer_HP = 0;
      R_Infantry3_HP = 0;
      R_Infantry4_HP = 0;
      R_Infantry5_HP = 0;
      R_Sentry_HP = 0;
      R_Outpost_HP = 0;
      R_Base_HP = 0;
      B_Hero_HP = 0;
      B_Engineer_HP = 0;
      B_Infantry3_HP = 0;
      B_Infantry4_HP = 0;
      B_Infantry5_HP = 0;
      B_Sentry_HP = 0;
      B_Outpost_HP = 0;
      B_Base_HP = 0;

    }
};

struct RoboInfUartBuff {
  // uint8_t my_color = 0;
  // union Receive_Information
    // {
      uint16_t R_Hero_HP=0;
      uint16_t R_Engineer_HP=0;
      uint16_t R_Infantry3_HP=0;
      uint16_t R_Infantry4_HP=0;
      uint16_t R_Infantry5_HP=0;
      uint16_t R_Sentry_HP=0;
      uint16_t R_Outpost_HP=0;
      uint16_t R_Base_HP=0;
    
      uint16_t B_Hero_HP=0;
      uint16_t B_Engineer_HP=0;
      uint16_t B_Infantry3_HP=0;
      uint16_t B_Infantry4_HP=0;
      uint16_t B_Infantry5_HP=0;
      uint16_t B_Sentry_HP=0;
      uint16_t B_Outpost_HP=0;
      uint16_t B_Base_HP=0;
    // } 
    // Receive_Information;
    // RoboInfUartBuff()
    // {
    //   // my_color = 0;
    //   Receive_Information.R_Hero_HP = 0;
    //   Receive_Information.R_Engineer_HP = 0;
    //   Receive_Information.R_Infantry3_HP = 0;
    //   Receive_Information.R_Infantry4_HP = 0;
    //   Receive_Information.R_Infantry5_HP = 0;
    //   Receive_Information.R_Sentry_HP = 0;
    //   Receive_Information.R_Outpost_HP = 0;
    //   Receive_Information.R_Base_HP = 0;

    //   Receive_Information.B_Hero_HP = 0;
    //   Receive_Information.B_Engineer_HP = 0;
    //   Receive_Information.B_Infantry3_HP = 0;
    //   Receive_Information.B_Infantry4_HP = 0;
    //   Receive_Information.B_Infantry5_HP = 0;
    //   Receive_Information.B_Sentry_HP = 0;
    //   Receive_Information.B_Outpost_HP = 0;
    //   Receive_Information.B_Base_HP = 0;

    // }
} __attribute__((packed));

