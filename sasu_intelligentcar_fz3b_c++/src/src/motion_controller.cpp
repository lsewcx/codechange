#pragma once
/**
 * @file motion_controller.cpp
 * @author lse
 * @brief 运动控制器：PD姿态控制||速度控制
 * @version 0.1
 * @date 2023-07-20
 * @note PD控制器要求稳定的控制周期：~40ms
 * 这里可以增加motion文件里面的数值，我们通过把圆环里面的pd控制器拉出来单独控制
 * @note 添加motion文件方法
 *        [1]先在json文件中添加数值
 *        [2]再在params结构体里面添加数值，初值随便赋无所谓
 *        [3]再在NLOHMANN_DEFINE_TYPE_INTRUSIVE中添加这个参数，使用的话建立结构体，具体使用方法可以查看icar的调用的放法
 * @copyright Copyright (c) 2023
 *
 */

#include "../include/common.hpp"
#include "../include/json.hpp"
#include "controlcenter_cal.cpp"
#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;
using nlohmann::json;

int a = 0;
class MotionController {
private:
  int counterShift = 0; // 变速计数器

public:
  /**
   * @brief 控制器核心参数
   *
   */
  struct Params {
    float speedLow = 1.0;      // 智能车最低速
    float speedHigh = 1.0;     // 智能车最高速
    float speedDown = 0.6;     // 特殊区域降速速度
    float speedBridge = 1.0;   // 坡道（桥）行驶速度
    float speedSlowzone = 1.0; // 慢行区行驶速度
    float speedGarage = 1.0;   // 出入车库速度
    float speedRing = 1.0;     // 圆环
    float speedcross = 0.6;
    float runP1 = 0.9;         // 一阶比例系数：直线控制量
    float runP2 = 0.018;       // 二阶比例系数：弯道控制量
    float runP3 = 0.0;         // 三阶比例系数：弯道控制量
    float turnP = 3.5;         // 一阶比例系数：转弯控制量
    float turnD = 3.5;         // 一阶微分系数：转弯控制量
    float ringP1 = 0.9;
    float ringP2 = 0.018;
    float rightP1 = 1.7;
    float rightP2 = 0.009;
    int ringDirection = 0;
    bool debug = false;         // 调试模式使能
    bool saveImage = false;     // 存图使能
    uint16_t rowCutUp = 10;     // 图像顶部切行
    uint16_t rowCutBottom = 10; // 图像顶部切行
    float disGarageEntry = 0.7; // 车库入库距离(斑马线Image占比)
    bool GarageEnable = true;   // 出入库使能
    bool BridgeEnable = true;   // 坡道使能
    bool FreezoneEnable = true; // 泛行区使能
    bool RingEnable = true;     // 环岛使能
    bool CrossEnable = true;    // 十字使能
    bool GranaryEnable = true;  // 粮仓使能
    bool DepotEnable = true;    // 修车厂使能
    bool FarmlandEnable = true; // 农田使能
    bool SlowzoneEnable = true; // 慢行区使能
    uint16_t circles = 2;       // 智能车运行圈数
    string pathVideo = "../res/samples/sample.mp4"; // 视频路径
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        Params, speedcross, rightP1, rightP2, ringDirection, ringP1, ringP2,
        speedRing, speedLow, speedHigh, speedDown, speedBridge, speedSlowzone,
        speedGarage, runP1, runP2, runP3, turnP, turnD, debug, saveImage,
        rowCutUp, rowCutBottom, disGarageEntry, GarageEnable, BridgeEnable,
        FreezoneEnable, RingEnable, CrossEnable, GranaryEnable, DepotEnable,
        FarmlandEnable, SlowzoneEnable, circles, pathVideo); // 添加构造函数
  };

  Params params;                   // 读取控制参数
  uint16_t servoPwm = PWMSERVOMID; // 发送给舵机的PWM
  float motorSpeed = 1.0;          // 发送给电机的速度
  /**
   * @brief 姿态PD控制器
   *
   * @param controlCenter 智能车控制中心
   */

  double tmp;
  void pdController(int controlCenter) {
    float heightest_line = 0;
    float error = controlCenter - COLSIMAGE / 2; // 图像控制中心转换偏差
    static int errorLast = 0;                    // 记录前一次的偏差
    if (abs(error - errorLast) > COLSIMAGE / 10) {
      error = error > errorLast ? errorLast + COLSIMAGE / 10
                                : errorLast - COLSIMAGE / 10;
    }

    params.turnP = abs(error) * params.runP2 + params.runP1;
    tmp = params.turnP;
    int pwmDiff = (error * params.turnP) + (error - errorLast) * params.turnD;
    errorLast = error;

    servoPwm = (uint16_t)(PWMSERVOMID + pwmDiff); // PWM转换
  }

  void RingpdController(int controlCenter) {
    float error = controlCenter - COLSIMAGE / 2; // 图像控制中心转换偏差
    static int errorLast = 0;                    // 记录前一次的偏差
    if (abs(error - errorLast) > COLSIMAGE / 10) {
      error = error > errorLast ? errorLast + COLSIMAGE / 10
                                : errorLast - COLSIMAGE / 10;
    }
    if (a == 0) {
      params.turnP = tmp;
      a++;
    } else
      params.turnP = abs(error) * params.ringP2 + params.ringP1;
    // turnP = max(turnP,0.2);
    //  if(turnP<0.2){
    //      turnP = 0.2;
    //  }
    // turnP = runP1 + heightest_line * runP2;

    int pwmDiff = (error * params.turnP) + (error - errorLast) * params.turnD;
    errorLast = error;

    servoPwm = (uint16_t)(PWMSERVOMID + pwmDiff); // PWM转换
  }

  void RightRingpdController(int controlCenter) {
    float error = controlCenter - COLSIMAGE / 2; // 图像控制中心转换偏差
    static int errorLast = 0;                    // 记录前一次的偏差
    if (abs(error - errorLast) > COLSIMAGE / 10) {
      error = error > errorLast ? errorLast + COLSIMAGE / 10
                                : errorLast - COLSIMAGE / 10;
    }

    params.turnP = abs(error) * params.rightP2 + params.rightP1;
    // turnP = max(turnP,0.2);
    //  if(turnP<0.2){
    //      turnP = 0.2;
    //  }
    // turnP = runP1 + heightest_line * runP2;

    int pwmDiff = (error * params.turnP) + (error - errorLast) * params.turnD;
    errorLast = error;

    servoPwm = (uint16_t)(PWMSERVOMID + pwmDiff); // PWM转换
  }

  /**
   * @brief 变加速控制
   *
   * @param enable 加速使能
   * @param control
   */
  void speedController(bool enable, bool slowDown, ControlCenterCal control) {
    // 控制率
    uint8_t controlLow = 0;   // 速度控制下限
    uint8_t controlMid = 5;   // 控制率
    uint8_t controlHigh = 10; // 速度控制上限

    if (slowDown) {
      counterShift = controlLow;
      motorSpeed = params.speedDown;
    } else if (enable) // 加速使能
    {
      if (control.centerEdge.size() < 10) {
        motorSpeed = params.speedLow;
        counterShift = controlLow;
        return;
      }
      if (control.centerEdge[control.centerEdge.size() - 1].x > ROWSIMAGE / 2) {
        motorSpeed = params.speedLow;
        counterShift = controlLow;
        return;
      }
      if (abs(control.sigmaCenter) < 100.0) {
        counterShift++;
        if (counterShift > controlHigh)
          counterShift = controlHigh;
      } else {
        counterShift--;
        if (counterShift < controlLow)
          counterShift = controlLow;
      }

      if (counterShift > controlMid)
        motorSpeed = params.speedHigh;
      else
        motorSpeed = params.speedLow;
    } else {
      counterShift = controlLow;
      motorSpeed = params.speedLow;
    }
  }

  /**
   * @brief 加载配置参数Json
   */
  void loadParams() {
    string jsonPath = "../src/config/motion.json";
    std::ifstream config_is(jsonPath);
    if (!config_is.good()) {
      std::cout << "Error: Params file path:[" << jsonPath << "] not find .\n";
      exit(-1);
    }

    json js_value;
    config_is >> js_value;

    try {
      params = js_value.get<Params>();
    } catch (const nlohmann::detail::exception &e) {
      std::cerr << "Json Params Parse failed :" << e.what() << '\n';
      exit(-1);
    }

    motorSpeed = params.speedLow;
    cout << "--- runP1:" << params.runP1 << " | runP2:" << params.runP2
         << " | runP3:" << params.runP3 << endl;
    cout << "--- turnP:" << params.turnP << " | turnD:" << params.turnD << endl;
    cout << "--- speedLow:" << params.speedLow
         << "m/s  |  speedHigh:" << params.speedHigh << "m/s" << endl;
  }
};
