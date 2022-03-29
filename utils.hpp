/*
 * @Author: your name
 * @Date: 2022-03-29 15:10:42
 * @LastEditTime: 2022-03-29 16:29:35
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /code/utils.hpp
 */
#pragma once
#include <atomic>

struct RoboCmd {
  std::atomic<float> yaw_angle = 0.f;
  std::atomic<float> pitch_angle = 0.f;
  std::atomic<float> depth = 0.f;
  std::atomic<uint8_t> detect_object = false;
};

struct RoboCmdUartBuff{
  uint8_t start = (unsigned)'S';
  float yaw_angle;
  float pitch_angle;
  float depth;
  uint8_t detect_object;
  uint8_t end = (unsigned)'E';
} __attribute__((packed));

struct RoboInf {
  std::atomic<uint8_t> my_color = 0;
 union Receive_Red_Information
    {
      std::atomic<uint16_t> R_Hero_HP;
      std::atomic<uint16_t> R_Engineer_HP;
      std::atomic<uint16_t> R_Infantry3_HP;
      std::atomic<uint16_t> R_Infantry4_HP;
      std::atomic<uint16_t> R_Infantry5_HP;
      std::atomic<uint16_t> R_Sentry_HP;
      std::atomic<uint16_t> R_Outpost_HP;
      std::atomic<uint16_t> R_Base_HP;
    } Receive_Red_HP;

    // Description of the pitch axis angle of the gyroscope (signed)
    union Receive_Blue_Information
    {
      std::atomic<uint16_t> B_Hero_HP;
      std::atomic<uint16_t> B_Engineer_HP;
      std::atomic<uint16_t> B_Infantry3_HP;
      std::atomic<uint16_t> B_Infantry4_HP;
      std::atomic<uint16_t> B_Infantry5_HP;
      std::atomic<uint16_t> B_Sentry_HP;
      std::atomic<uint16_t> B_Outpost_HP;
      std::atomic<uint16_t> B_Base_HP;
    } Receive_Blue_HP;
    RoboInf()
    {
      my_color = 0;
      Receive_Red_HP.R_Hero_HP = 0;
      Receive_Red_HP.R_Engineer_HP = 0;
      Receive_Red_HP.R_Infantry3_HP = 0;
      Receive_Red_HP.R_Infantry4_HP = 0;
      Receive_Red_HP.R_Infantry5_HP = 0;
      Receive_Red_HP.R_Sentry_HP = 0;
      Receive_Red_HP.R_Outpost_HP = 0;
      Receive_Red_HP.R_Base_HP = 0;

      Receive_Blue_HP.B_Hero_HP = 0;
      Receive_Blue_HP.B_Engineer_HP = 0;
      Receive_Blue_HP.B_Infantry3_HP = 0;
      Receive_Blue_HP.B_Infantry4_HP = 0;
      Receive_Blue_HP.B_Infantry5_HP = 0;
      Receive_Blue_HP.B_Sentry_HP = 0;
      Receive_Blue_HP.B_Outpost_HP = 0;
      Receive_Blue_HP.B_Base_HP = 0;

    }
};

struct RoboInfUartBuff {
  std::atomic<uint8_t> my_color = 0;
  union Receive_Red_Information
    {
      std::atomic<uint16_t> R_Hero_HP;
      std::atomic<uint16_t> R_Engineer_HP;
      std::atomic<uint16_t> R_Infantry3_HP;
      std::atomic<uint16_t> R_Infantry4_HP;
      std::atomic<uint16_t> R_Infantry5_HP;
      std::atomic<uint16_t> R_Sentry_HP;
      std::atomic<uint16_t> R_Outpost_HP;
      std::atomic<uint16_t> R_Base_HP;
    } Receive_Red_HP;

    // Description of the pitch axis angle of the gyroscope (signed)
    union Receive_Blue_Information
    {
      std::atomic<uint16_t> B_Hero_HP;
      std::atomic<uint16_t> B_Engineer_HP;
      std::atomic<uint16_t> B_Infantry3_HP;
      std::atomic<uint16_t> B_Infantry4_HP;
      std::atomic<uint16_t> B_Infantry5_HP;
      std::atomic<uint16_t> B_Sentry_HP;
      std::atomic<uint16_t> B_Outpost_HP;
      std::atomic<uint16_t> B_Base_HP;
    } Receive_Blue_HP;
    RoboInfUartBuff()
    {
      my_color = 0;
      Receive_Red_HP.R_Hero_HP = 0;
      Receive_Red_HP.R_Engineer_HP = 0;
      Receive_Red_HP.R_Infantry3_HP = 0;
      Receive_Red_HP.R_Infantry4_HP = 0;
      Receive_Red_HP.R_Infantry5_HP = 0;
      Receive_Red_HP.R_Sentry_HP = 0;
      Receive_Red_HP.R_Outpost_HP = 0;
      Receive_Red_HP.R_Base_HP = 0;

      Receive_Blue_HP.B_Hero_HP = 0;
      Receive_Blue_HP.B_Engineer_HP = 0;
      Receive_Blue_HP.B_Infantry3_HP = 0;
      Receive_Blue_HP.B_Infantry4_HP = 0;
      Receive_Blue_HP.B_Infantry5_HP = 0;
      Receive_Blue_HP.B_Sentry_HP = 0;
      Receive_Blue_HP.B_Outpost_HP = 0;
      Receive_Blue_HP.B_Base_HP = 0;

    }
} __attribute__((packed));

