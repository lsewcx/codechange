/**
 * @file icar.cpp
 * @author lse
 * @brief 智能汽车-完全模型组-顶层框架（TOP）
 * @version 0.1
 * @date 2023-7-20
 * @note 添加计时器防止误判
 * @copyright Copyright (c) 2023
 *
 */
#include "../include/common.hpp"            //公共类方法文件
#include "../include/detection.hpp"         //百度Paddle框架移动端部署
#include "../include/uart.hpp"              //串口通信驱动
#include "controlcenter_cal.cpp"            //控制中心计算类
#include "detection/bridge_detection.cpp"   //桥梁AI检测与路径规划类
#include "detection/depot_detection.cpp"    //维修厂AI检测
#include "detection/farmland_detection.cpp" //农田区域AI检测
#include "detection/granary_detection.cpp"  //粮仓AI检测
#include "detection/slowzone_detection.cpp" //慢行区AI检测与路径规划类
#include "image_preprocess.cpp"             //图像预处理类
#include "motion_controller.cpp"            //智能车运动控制类
#include "recognition/cross_recognition.cpp" //十字道路识别与路径规划类
#include "recognition/freezone_recognition.cpp" //泛行区识别类
#include "recognition/garage_recognition.cpp"   //车库及斑马线识别类
#include "recognition/ring_recognition.cpp" //环岛道路识别与路径规划类
#include "recognition/track_recognition.cpp" //赛道识别基础类
#include <chrono>
#include <iostream>
#include <mutex>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <thread>
#include <unistd.h>
using namespace std;
using namespace cv;

void callbackSignal(int signum);
void displayWindowInit(void);
void slowDownEnable(void);
std::shared_ptr<Driver> driver = nullptr; // 初始化串口驱动
bool slowDown = false;                    // 特殊区域减速标志
uint16_t counterSlowDown = 0;             // 减速计数器

bool allowRingState = true;               // 初始设定允许"圆环状态"
bool allowStart = false;

void Timer() {
  // 由于第二圈肯定没有圆环所以第一次5秒后禁止圆环后35秒后才解放(除非学弟学妹们车能上2m)
  std::this_thread::sleep_for(std::chrono::seconds(35));
  allowRingState = true; // 20秒后允许"圆环状态"

  std::this_thread::sleep_for(std::chrono::seconds(15));
  allowRingState = false;
}

void Timer1() {
  std::this_thread::sleep_for(std::chrono::seconds(5));
  allowRingState = false; // 5秒后取消"圆环状态"
}

void Timer2() {
  std::this_thread::sleep_for(std::chrono::seconds(70));
  allowStart = true;
}

// 图像高光选取
cv::Mat HighLight(cv::Mat input, int light) {
  // 生成灰度图
  cv::Mat gray = cv::Mat::zeros(input.size(), CV_32FC1);
  cv::Mat f = input.clone();
  f.convertTo(f, CV_32FC3);
  vector<cv::Mat> pics;
  split(f, pics);
  gray = 0.299f * pics[2] + 0.587 * pics[1] + 0.114 * pics[0];
  gray = gray / 255.f;

  // 确定高光区
  cv::Mat thresh = cv::Mat::zeros(gray.size(), gray.type());
  thresh = gray.mul(gray);
  // 取平均值作为阈值
  Scalar t = mean(thresh);
  cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8UC1);
  mask.setTo(255, thresh >= t[0]);

  // 参数设置
  int max = 4;
  float bright = light / 100.0f / max;
  float mid = 1.0f + max * bright;

  // 边缘平滑过渡
  cv::Mat midrate = cv::Mat::zeros(input.size(), CV_32FC1);
  cv::Mat brightrate = cv::Mat::zeros(input.size(), CV_32FC1);
  for (int i = 0; i < input.rows; ++i) {
    uchar *m = mask.ptr<uchar>(i);
    float *th = thresh.ptr<float>(i);
    float *mi = midrate.ptr<float>(i);
    float *br = brightrate.ptr<float>(i);
    for (int j = 0; j < input.cols; ++j) {
      if (m[j] == 255) {
        mi[j] = mid;
        br[j] = bright;
      } else {
        mi[j] = (mid - 1.0f) / t[0] * th[j] + 1.0f;
        br[j] = (1.0f / t[0] * th[j]) * bright;
      }
    }
  }

  // 高光提亮，获取结果图
  cv::Mat result = cv::Mat::zeros(input.size(), input.type());
  for (int i = 0; i < input.rows; ++i) {
    float *mi = midrate.ptr<float>(i);
    float *br = brightrate.ptr<float>(i);
    uchar *in = input.ptr<uchar>(i);
    uchar *r = result.ptr<uchar>(i);
    for (int j = 0; j < input.cols; ++j) {
      for (int k = 0; k < 3; ++k) {
        float temp = pow(float(in[3 * j + k]) / 255.f, 1.0f / mi[j]) *
                     (1.0 / (1 - br[j]));
        if (temp > 1.0f)
          temp = 1.0f;
        if (temp < 0.0f)
          temp = 0.0f;
        uchar utemp = uchar(255 * temp);
        r[3 * j + k] = utemp;
      }
    }
  }
  return result;
}

enum RoadType {
  BaseHandle = 0, // 基础赛道处理
  RingHandle,     // 环岛赛道处理
  CrossHandle,    // 十字道路处理
  FreezoneHandle, // 泛行区处理
  GarageHandle,   // 车库处理
  GranaryHandle,  // 粮仓处理
  DepotHandle,    // 修车厂处理
  BridgeHandle,   // 坡道(桥)处理
  SlowzoneHandle, // 慢行区（动物出没）处理
  FarmlandHandle, // 农田区域处理
};

int main(int argc, char const *argv[]) {
  std::shared_ptr<Detection> detection = nullptr; // 初始化AI预测模型
  ImagePreprocess imagePreprocess;                // 图像预处理类
  TrackRecognition trackRecognition;              // 赛道识别
  ControlCenterCal controlCenterCal;              // 控制中心计算
  MotionController motionController;              // 运动控制
  RingRecognition ringRecognition;                // 环岛识别
  CrossroadRecognition crossroadRecognition;      // 十字道路处理
  GarageRecognition garageRecognition;            // 车库识别
  FreezoneRecognition freezoneRecognition;        // 泛型区识别类
  FarmlandDetection farmlandDetection;            // 农田区域检测
  DepotDetection depotDetection;                  // 维修厂检测
  GranaryDetection granaryDetection;              // 粮仓检测
  BridgeDetection bridgeDetection;                // 桥梁检测
  SlowZoneDetection slowZoneDetection;            // 慢行区检测
  uint16_t counterRunBegin = 1; // 智能车启动计数器：等待摄像头图像帧稳定
  RoadType roadType = RoadType::BaseHandle; // 初始化赛道类型
  uint16_t counterOutTrackA = 0;            // 车辆冲出赛道计数器A
  uint16_t counterOutTrackB = 0;            // 车辆冲出赛道计数器B
  uint16_t circlesThis = 2;                 // 智能车当前运行的圈数
  uint16_t countercircles = 0;              // 圈数计数器

  // USB转串口的设备名为 / dev/ttyUSB0
  driver = std::make_shared<Driver>("/dev/ttyUSB0", BaudRate::BAUD_115200);
  if (driver == nullptr) {
    std::cout << "Create Uart-Driver Error!" << std::endl;
    return -1;
  }
  // 串口初始化，打开串口设备及配置串口数据格式
  int ret = driver->open();
  if (ret != 0) {
    std::cout << "Uart Open failed!" << std::endl;
    return -1;
  }

  ipm.init(Size(COLSIMAGE, ROWSIMAGE),
           Size(COLSIMAGEIPM, ROWSIMAGEIPM)); // IPM逆透视变换初始化

  signal(SIGINT, callbackSignal);             // 程序退出信号

  motionController.loadParams();              // 读取配置文件
  trackRecognition.rowCutUp = motionController.params.rowCutUp;
  trackRecognition.rowCutBottom = motionController.params.rowCutBottom;
  garageRecognition.disGarageEntry = motionController.params.disGarageEntry;

  if (motionController.params.GarageEnable) // 出入库使能
    roadType = RoadType::GarageHandle;      // 初始赛道元素为出库

  imagePreprocess.imageCorrecteInit();      // 图像矫正参数初始化

  if (motionController.params.debug) {
    displayWindowInit(); // 显示窗口初始化 //显示窗口初始化
    detection = Detection::DetectionInstance(
        motionController.params.pathVideo,
        "../res/model/mobilenet-ssd"); // Video输入源
    printAiEnable = true;              // AI检测结果绘制
  } else {
    cout << "等待发车!!!" << endl;
    detection = Detection::DetectionInstance(
        "/dev/video0", "../res/model/mobilenet-ssd"); // Video输入源
    printAiEnable = false;                            // AI检测结果绘制

    while (!driver->receiveStartSignal()) // 串口接收下位机-比赛开始信号
    {
      ;
    }
    cout << "--------- System start!!! -------" << endl;
    std::thread timer11(Timer2); // 创建一个新线程来倒计时
    timer11.detach();

    for (int i = 0; i < 30; i++)          // 3秒后发车
    {
      driver->carControl(0, PWMSERVOMID); // 智能车停止运动|建立下位机通信
      waitKey(100);
    }
  }

  while (1) {

    bool imshowRec = false; // 特殊赛道图像显示标志

    // 处理帧时长监测把debug注释了可以查看处理每一帧画面的速度
    if (motionController.params.debug) {
      static auto preTime = chrono::duration_cast<chrono::milliseconds>(
                                chrono::system_clock::now().time_since_epoch())
                                .count();
      auto startTime = chrono::duration_cast<chrono::milliseconds>(
                           chrono::system_clock::now().time_since_epoch())
                           .count();
      cout << "run frame time : " << startTime - preTime << "ms" << endl;
      preTime = startTime;
    }

    //[01] 视频源选择
    std::shared_ptr<DetectionResult> resultAI =
        detection->getLastFrame();   // 获取Paddle多线程模型预测数据
    Mat frame = resultAI->rgb_frame; // 获取原始摄像头图像
    if (motionController.params.debug) {
      savePicture(resultAI->det_render_frame);
    } else {
      if (motionController.params.saveImage) // 保存原始图像
        savePicture(frame);
    }

    //[02] 图像预处理
    Mat imgaeCorrect = imagePreprocess.imageCorrection(frame); // RGB
    int light1 = -50;
    Mat src = HighLight(imgaeCorrect, light1);
    Mat imageBinary = imagePreprocess.imageBinaryzation(src); // Gray

    //[03] 基础赛道识别
    trackRecognition.trackRecognition(
        imageBinary); // 赛道线识别   可以尝试修改成八领域巡线
    if (motionController.params.debug) {
      Mat imageTrack = imgaeCorrect.clone(); // RGB
      trackRecognition.drawImage(imageTrack); // 图像显示赛道线识别结果
      imshow("imageTrack", imageTrack);
      savePicture(imageTrack);
    }

    // [04] 出库和入库识别与路径规划
    if (motionController.params.GarageEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::GarageHandle ||
          roadType == RoadType::BaseHandle) {
        countercircles++; // 圈数计数
        if (countercircles > 200)
          countercircles = 200;
        if (garageRecognition.startingCheck(
                resultAI->predictor_results))   // 检测到起点
        {
          if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
            driver->buzzerSound(1);

          bridgeDetection.reset();
          depotDetection.reset();
          farmlandDetection.reset();
          granaryDetection.reset();
          slowZoneDetection.reset();
          crossroadRecognition.reset();
          freezoneRecognition.reset(); // 泛行区识别复位
          ringRecognition.reset();     // 环岛识别初始化

          if (countercircles > 60) {
            circlesThis++;
            countercircles = 0;
          }
        }

        if (circlesThis >= motionController.params.circles &&
            countercircles > 100 && allowStart) // 入库使能：跑完N圈
          garageRecognition.entryEnable = true;

        if (garageRecognition.garageRecognition(trackRecognition,
                                                resultAI->predictor_results)) {
          if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
            driver->buzzerSound(1);             // OK

          roadType = RoadType::GarageHandle;
          if (garageRecognition.garageStep ==
              garageRecognition.GarageEntryFinish) // 入库完成
          {
            cout << ">>>>>>>   入库结束 !!!!!" << endl;
            callbackSignal(0);
          }
          if (motionController.params.debug) {
            Mat imageGarage =
                Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
            garageRecognition.drawImage(trackRecognition, imageGarage);
            imshow("imageRecognition", imageGarage);
            imshowRec = true;
            savePicture(imageGarage);
          }
        } else
          roadType = RoadType::BaseHandle;

        if (garageRecognition.slowDown) // 入库减速
          slowDownEnable();
      }
    }

    //[05] 农田区域检测
    if (motionController.params.FarmlandEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::FarmlandHandle ||
          roadType == RoadType::BaseHandle) {
        if (farmlandDetection.farmlandDetection(trackRecognition,
                                                resultAI->predictor_results)) {
          if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
            driver->buzzerSound(1);             // OK

          roadType = RoadType::FarmlandHandle;
          if (motionController.params.debug) {
            Mat imageFarmland =
                Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
            farmlandDetection.drawImage(trackRecognition, imageFarmland);
            imshow("imageRecognition", imageFarmland);
            imshowRec = true;
            savePicture(imageFarmland);
          }
        } else
          roadType = RoadType::BaseHandle;
      }
    }

    //[06] 维修厂检测
    if (motionController.params.DepotEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::DepotHandle ||
          roadType == RoadType::BaseHandle) {
        if (depotDetection.depotDetection(trackRecognition,
                                          resultAI->predictor_results)) {
          if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
            driver->buzzerSound(1);             // OK

          roadType = RoadType::DepotHandle;
          if (motionController.params.debug) {
            Mat imageDepot =
                Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
            depotDetection.drawImage(trackRecognition, imageDepot);
            imshow("imageRecognition", imageDepot);
            imshowRec = true;
            savePicture(imageDepot);
          }
        } else
          roadType = RoadType::BaseHandle;
      }
    }

    //[07] 粮仓检测
    if (motionController.params.GranaryEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::GranaryHandle ||
          roadType == RoadType::BaseHandle) {
        if (granaryDetection.granaryDetection(trackRecognition,
                                              resultAI->predictor_results)) {
          if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
            driver->buzzerSound(1);             // OK

          roadType = RoadType::GranaryHandle;
          if (motionController.params.debug) {
            Mat imageGranary =
                Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
            granaryDetection.drawImage(trackRecognition, imageGranary);
            imshow("imageRecognition", imageGranary);
            imshowRec = true;
            savePicture(imageGranary);
          }
        } else
          roadType = RoadType::BaseHandle;
      }
    }

    // [08] 坡道（桥）检测与路径规划
    if (motionController.params.BridgeEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::BridgeHandle ||
          roadType == RoadType::BaseHandle) {
        if (bridgeDetection.bridgeDetection(trackRecognition,
                                            resultAI->predictor_results)) {
          if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
            driver->buzzerSound(1);             // OK

          roadType = RoadType::BridgeHandle;
          if (motionController.params.debug) {
            Mat imageBridge =
                Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
            bridgeDetection.drawImage(trackRecognition, imageBridge);
            imshow("imageRecognition", imageBridge);
            imshowRec = true;
            savePicture(imageBridge);
          }
        } else
          roadType = RoadType::BaseHandle;
      }
    }

    // [09] 慢行区检测与路径规划
    if (motionController.params.SlowzoneEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::SlowzoneHandle ||
          roadType == RoadType::BaseHandle) {
        if (slowZoneDetection.slowZoneDetection(trackRecognition,
                                                resultAI->predictor_results)) {
          if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
            driver->buzzerSound(1);             // OK

          roadType = RoadType::SlowzoneHandle;
          if (motionController.params.debug) {
            Mat imageSlow =
                Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
            slowZoneDetection.drawImage(trackRecognition, imageSlow);
            imshow("imageRecognition", imageSlow);
            imshowRec = true;
            savePicture(imageSlow);
          }
        } else
          roadType = RoadType::BaseHandle;
      }
    }

    // [10] 泛行区检测与识别（无泛行区）
    if (motionController.params.FreezoneEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::FreezoneHandle ||
          roadType == RoadType::BaseHandle) {
        if (freezoneRecognition.freezoneRecognition(trackRecognition)) {
          if (roadType == RoadType::BaseHandle) // 初次识别-蜂鸣器提醒
            driver->buzzerSound(1);             // OK

          roadType = RoadType::FreezoneHandle;
          if (motionController.params.debug) {
            Mat imageFreezone =
                Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
            freezoneRecognition.drawImage(trackRecognition, imageFreezone);
            imshow("imageRecognition", imageFreezone);
            imshowRec = true;
            savePicture(imageFreezone);
          }
        } else
          roadType = RoadType::BaseHandle;
      }
    }

    if (motionController.params.RingEnable) // 赛道元素是否使能
    {
      if (allowRingState) {
        if (roadType == RoadType::RingHandle ||
            roadType == RoadType::BaseHandle) {
          if (motionController.params.ringDirection ==
              0) { // 左右圆环准备分开的暂时去除了右圆环
            if (ringRecognition.ringRecognition(trackRecognition,
                                                imageBinary)) {
              if (roadType == RoadType::BaseHandle) {
                driver->buzzerSound(1);
                std::thread timer6(Timer1); // 创建一个新线程来倒计时
                timer6.detach();
                std::thread timer7(Timer);  // 创建一个新线程来倒计时
                timer7.detach();
              }
              roadType = RoadType::RingHandle;
            } else {
              roadType = RoadType::BaseHandle;
            }
          }
        }
      }
    }

    // [12] 十字道路处理
    if (motionController.params.CrossEnable) // 赛道元素是否使能
    {
      if (roadType == RoadType::CrossHandle ||
          roadType == RoadType::BaseHandle) {
        if (crossroadRecognition.crossroadRecognition(
                trackRecognition, resultAI->predictor_results)) {
          roadType = RoadType::CrossHandle;
        } else
          roadType = RoadType::BaseHandle;
      }
      // }
    }

    // [13] 控制中心计算
    if (trackRecognition.pointsEdgeLeft.size() < 30 &&
        trackRecognition.pointsEdgeRight.size() < 30 &&
        roadType != RoadType::BridgeHandle &&
        roadType != RoadType::GranaryHandle &&
        roadType != RoadType::DepotHandle &&
        roadType != RoadType::FarmlandHandle) // 防止车辆冲出赛道
    {
      counterOutTrackA++;
      counterOutTrackB = 0;
      if (counterOutTrackA > 20)
        callbackSignal(0);
    } else {
      counterOutTrackB++;
      if (counterOutTrackB > 50) {
        counterOutTrackA = 0;
        counterOutTrackB = 50;
      }
    }

    controlCenterCal.controlCenterCal(
        trackRecognition); // 根据赛道边缘信息拟合运动控制中心

    // [14] 运动控制
    if (counterRunBegin > 30) ////智能车启动延时：前几场图像不稳定
    {
      // 智能汽车方向控制
      if (roadType != RoadType::RingHandle)
        motionController.pdController(
            controlCenterCal.controlCenter); // PD控制器姿态控制圆环pid单独控制
      else if (roadType == RoadType::RingHandle) {
        if (motionController.params.ringDirection == 0)
          motionController.RingpdController(controlCenterCal.controlCenter);
        else if (motionController.params.ringDirection == 1)
          motionController.RightRingpdController(
              controlCenterCal.controlCenter);
      }
      // 智能汽车速度控制
      switch (roadType) {
      case RoadType::GarageHandle:
        motionController.motorSpeed =
            motionController.params.speedGarage; // 匀速控制
        break;
      case RoadType::BridgeHandle:
        motionController.motorSpeed =
            motionController.params.speedBridge; // 匀速控制
        break;
      case RoadType::SlowzoneHandle:
        motionController.motorSpeed =
            motionController.params.speedSlowzone; // 匀速控制
        break;
      case RoadType::RingHandle:
        motionController.motorSpeed = motionController.params.speedRing;
        break;
      case RoadType::CrossHandle:
        motionController.motorSpeed = motionController.params.speedcross;
        break;
      default:                                              // 基础巡线
        motionController.speedController(true, slowDown,
                                         controlCenterCal); // 变加速控制
        break;
      }

      if (!motionController.params.debug) // 调试模式下不控制车辆运动
      {
        if (roadType == RoadType::DepotHandle) {
          if (depotDetection.depotStep == 4)
            driver->carControl(
                0, motionController.servoPwm); // 串口通信，姿态与速度控制
          else if (depotDetection.depotStep == 5)
            driver->carControl(
                -motionController.motorSpeed,
                motionController.servoPwm); // 串口通信，姿态与速度控制
          else
            driver->carControl(
                motionController.motorSpeed,
                motionController.servoPwm); // 串口通信，姿态与速度控制
        } else
          driver->carControl(
              motionController.motorSpeed,
              motionController.servoPwm); // 串口通信，姿态与速度控制
      }

      // 减速缓冲
      if (slowDown) {
        counterSlowDown++;
        if (counterSlowDown > 50) {
          slowDown = false;
          counterSlowDown = 0;
        }
      }
    } else
      counterRunBegin++;

    // [15]调试模式下图像显示和存图
    if (motionController.params.debug) {
      controlCenterCal.drawImage(trackRecognition, imgaeCorrect);
      switch (roadType) {
      case RoadType::BaseHandle: // 基础赛道处理 // 基础赛道处理
        putText(imgaeCorrect, "[1] Track", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1,
                CV_AA);          // 显示赛道识别类型
        break;
      case RoadType::RingHandle: // 环岛赛道处理 // 环岛赛道处理
        putText(imgaeCorrect, "[2] Ring", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA);           // 显示赛道识别类型
        break;
      case RoadType::CrossHandle: // 十字道路处理 // 十字道路处理
        putText(imgaeCorrect, "[3] Cross", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA);              // 显示赛道识别类型
        break;
      case RoadType::FreezoneHandle: // 泛行区处理 // 泛行区处理
        putText(imgaeCorrect, "[4] Freezone", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA);            // 显示赛道识别类型
        break;
      case RoadType::GarageHandle: // 车库处理 // 车库处理
        putText(imgaeCorrect, "[5] Garage", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA);             // 显示赛道识别类型
        break;
      case RoadType::GranaryHandle: // 粮仓处理 // 加油站处理
        putText(imgaeCorrect, "[6] Granary", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA);           // 显示赛道识别类型
        break;
      case RoadType::DepotHandle: // 修车厂处理 // 施工区处理
        putText(imgaeCorrect, "[7] Depot", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA);            // 显示赛道识别类型
        break;
      case RoadType::BridgeHandle: // 坡道(桥)处理 // 坡道处理
        putText(imgaeCorrect, "[8] Bridge", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA);              // 显示赛道识别类型
        break;
      case RoadType::SlowzoneHandle: // 慢行区（动物出没）处理 // 坡道处理
        putText(imgaeCorrect, "[9] Slowzone", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA);              // 显示赛道识别类型
        break;
      case RoadType::FarmlandHandle: // 农田区域处理 // 坡道处理
        putText(imgaeCorrect, "[10] Farmland", Point(10, 20),
                cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
                CV_AA); // 显示赛道识别类型
        break;
      }

      putText(imgaeCorrect,
              "v: " + formatDoble2String(motionController.motorSpeed, 2),
              Point(COLSIMAGE - 60, 80), FONT_HERSHEY_PLAIN, 1,
              Scalar(0, 0, 255), 1); // 车速

      string str = to_string(circlesThis) + "/" +
                   to_string(motionController.params.circles);
      putText(imgaeCorrect, str, Point(COLSIMAGE - 50, ROWSIMAGE - 20),
              cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
              CV_AA); // 显示圈数
      if (!imshowRec) // 保持调试图像存储顺序和显示一致性
      {
        Mat imageNone =
            Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像
        imshow("imageRecognition", imageNone);
        savePicture(imageNone);
      }
      imshow("imageControl", imgaeCorrect);
      savePicture(imgaeCorrect);

      char c = waitKey(1);
    }
  }

  return 0;
}

/**
 * @brief 系统信号回调函数：系统退出
 *
 * @param signum 信号量
 */
void callbackSignal(int signum) {
  driver->carControl(0, PWMSERVOMID); // 智能车停止运动
  cout << "====System Exit!!!  -->  CarStopping! " << signum << endl;
  exit(signum);
}

/**
 * @brief OpenCV图像显示窗口初始化（详细参数/Debug模式）
 *
 */
void displayWindowInit(void) {
  //[1] 二值化图像：Gray
  string windowName = "imageTrack";
  cv::namedWindow(windowName, WINDOW_NORMAL); // 图像名称
  cv::resizeWindow(windowName, 320, 240);     // 分辨率
  cv::moveWindow(windowName, 10, 10);         // 布局位置

  //[2] 赛道边缘图像：RGB
  windowName = "imageRecognition";
  cv::namedWindow(windowName, WINDOW_NORMAL); // 图像名称
  cv::resizeWindow(windowName, 320, 240);     // 分辨率
  cv::moveWindow(windowName, 10, 320);        // 布局位置

  //[3] 原始图像/矫正后：RGB
  windowName = "imageControl";
  cv::namedWindow(windowName, WINDOW_NORMAL); // 图像名称
  cv::resizeWindow(windowName, 640, 480);     // 分辨率
  cv::moveWindow(windowName, 350, 20);        // 布局位置
}

/**
 * @brief 车辆减速使能
 *
 */
void slowDownEnable(void) {
  slowDown = true;
  counterSlowDown = 0;
}
