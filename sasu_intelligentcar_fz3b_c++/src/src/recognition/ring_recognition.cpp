#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2022; SaiShu.Lcc.; Leo;
 *https://bjsstech.com 版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial
 *transactions(开源学习,请勿商用). The code ADAPTS the corresponding hardware
 *circuit board(代码适配百度Edgeboard-FZ3B), The specific details consult the
 *professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file ring_recognition.cpp
 * @author Leo ()
 * @brief 环岛识别（基于track赛道识别后）
 * @version 0.1
 * @date 2022-02-28
 *
 * @copyright Copyright (c) 2022
 *
 * @note  环岛识别步骤（ringStep）：
 *          1：环岛识别（初始化）
 *          2：入环处理
 *          3：环中处理
 *          4：出环处理
 *          5：出环结束
 */

#include "../../include/common.hpp"
#include "track_recognition.cpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class RingRecognition {
public:
  uint16_t counterShield = 0; // 环岛检测屏蔽计数器：屏蔽车库误检测

  /**
   * @brief 环岛识别初始化|复位
   *
   */
  void reset(void) {
    RingType ringType = RingType::RingLeft; // 环岛类型
    RingStep ringStep = RingStep::None;     // 环岛处理阶段
    int rowRepairLine = 0;                  // 用于环补线的点（行号）
    int colRepairLine = 0;                  // 用于环补线的点（列号）
    counterSpurroad = 0;                    // 岔路计数器
    counterShield = 0;
  }
  /**
   * @brief 环岛识别与行径规划
   *
   * @param track 基础赛道识别结果
   * @param imagePath 赛道路径图像
   */
  bool ringRecognition(TrackRecognition &track, Mat &imagePath) {
    if (counterShield < 40) {
      counterShield++;
      return false;
    }

    bool ringEnable = false;                    // 判环标志
    RingType ringTypeTemp = RingType::RingNone; // 环岛类型：临时变量
    int rowBreakpointLeft = 0;  // 边缘拐点起始行（左）
    int rowBreakpointRight = 0; // 边缘拐点起始行（右）
    int colEnterRing = 0;       // 入环点（图像列序号）
    int rowRepairRingside =
        track.widthBlock.size() - 1; // 环一侧，补线起点（行号）
    int rowRepairStraightside =
        track.widthBlock.size() - 1; // 直道侧，补线起点（行号）
    int rowYendStraightside =
        track.widthBlock.size() - 1; // 直道侧，延长补线终点（行号）
    _index = 0;
    _ringPoint = POINT(0, 0);

    // 算环用布线的候选点
    rowRepairLine = max(rowRepairLine - 5, 0);
    if (ringStep == RingStep::Entering && !track.spurroad.empty()) {
      if (ringType == RingType::RingLeft && track.pointsEdgeLeft.size() > 20) {
        for (int j = max(rowRepairLine - 30, 10);
             j < track.pointsEdgeLeft.size() - 10 && j < rowRepairLine + 30 &&
             track.pointsEdgeLeft[j].x >= track.spurroad[0].x;
             j++) {
          if (track.pointsEdgeLeft[j].y > track.pointsEdgeLeft[j - 10].y &&
              track.pointsEdgeLeft[j].y > track.pointsEdgeLeft[j + 10].y) {
            rowRepairLine = j;
            break;
          }
        }
      } else if (ringType == RingType::RingRight &&
                 track.pointsEdgeRight.size() > 20) {
        for (int j = max(rowRepairLine - 30, 10);
             j < track.pointsEdgeRight.size() - 10 && j < rowRepairLine + 30 &&
             track.pointsEdgeRight[j].x >= track.spurroad[0].x;
             j++) {
          if (track.pointsEdgeRight[j].y < track.pointsEdgeRight[j - 10].y &&
              track.pointsEdgeRight[j].y < track.pointsEdgeRight[j + 10].y) {
            rowRepairLine = j;
            break;
          }
        }
      }
    }

    // 搜索赛道左右边缘满足图像边沿的最高处
    for (int ii = 0; ii < track.pointsEdgeLeft.size(); ++ii) {
      rowBreakpointLeft = track.pointsEdgeLeft[ii].x;
      if (track.pointsEdgeLeft[ii].y > 2)
        break;
    }
    for (int ii = 0; ii < track.pointsEdgeRight.size(); ++ii) {
      rowBreakpointRight = track.pointsEdgeRight[ii].x;
      if (track.pointsEdgeRight[ii].y < COLSIMAGE - 3)
        break;
    }

    // 判环
    int countWide = 0; // 环岛入口变宽区域行数
    for (int i = 1; i < track.widthBlock.size(); ++i) {
      if (track.widthBlock[i].y > track.widthBlock[i - 1].y &&
          track.widthBlock[i].y > COLSIMAGE * 0.6 &&
          track.widthBlock[i].x > 30 &&
          ((track.stdevLeft > 120 && track.stdevRight < 50) ||
           ringStep == RingStep::Entering)) // 搜索突然变宽的路径行数
      {
        ++countWide;
      } else {
        countWide = 0;
      }
      // [1] 入环判断
      if ((ringStep == RingStep::None || ringStep == RingStep::Entering) &&
          countWide >= 5 && !track.spurroad.empty()) {
        if (ringTypeTemp == RingType::RingNone) // 环岛方向判定
        {
          int tmp_flag = 0;
          for (int j = 0; j < track.spurroad.size(); j++) {
            if (track.spurroad[j].x < track.pointsEdgeLeft[i - 5].x) {
              tmp_flag = 1;
            }
          }
          if (tmp_flag == 0) {
            countWide = 0;
            continue;
          }
          if (track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i - 5].y) {
            ringTypeTemp = RingType::RingLeft; // 环岛类型：左入环
            colEnterRing = track.pointsEdgeLeft[i - 5].y; // 入环点列号
            _ringPoint.x = track.pointsEdgeLeft[i - 5].x;
            _ringPoint.y = track.pointsEdgeLeft[i - 5].y;

            rowRepairLine = i; // 用于环补线的行号
            colRepairLine = track.pointsEdgeLeft[i].x; // 用于环补线的列号
          } else if (track.pointsEdgeRight[i].y >
                     track.pointsEdgeRight[i - 5].y) {
            ringTypeTemp = RingType::RingRight; // 环岛类型：右入环
            colEnterRing = track.pointsEdgeRight[i - 5].y; // 入环点列号
            rowRepairLine = i; // 用于环补线的行号
            colRepairLine = track.pointsEdgeRight[i].x; // 用于环补线的列号
          }
        }

        // 内圆检测
        if ((ringTypeTemp == RingType::RingLeft &&
             colEnterRing - track.pointsEdgeLeft[i].y >= 3) ||
            (ringTypeTemp == RingType::RingRight &&
             track.pointsEdgeRight[i].y - colEnterRing >= 3)) {
          ringEnable = true;
          ringStep = RingStep::Entering;
          ringType = ringTypeTemp;
          if (rowRepairStraightside == track.widthBlock.size() - 1) {
            rowRepairStraightside = i - countWide;
          }
        } else {
          countWide = 0;
        }
      }
      /*if(ringStep == RingStep::Entering && ringEnable == false){
          ringEnable = true;
          rowRepairStraightside = rowRepairLine;
      }*/

      if (ringEnable == true && ringStep == RingStep::Entering) {
        if (ringTypeTemp == RingType::RingLeft) {
          if (track.pointsEdgeLeft[i].y <= 2 &&
              i != track.widthBlock.size() - 1) {
            if (rowRepairRingside == track.widthBlock.size() - 1) {
              rowRepairRingside = i;
            }
            rowYendStraightside = track.pointsEdgeLeft[i].x;
          } else if (rowRepairRingside != track.widthBlock.size() - 1) {

            int x = track.pointsEdgeLeft[rowRepairStraightside].x +
                    (rowYendStraightside -
                     track.pointsEdgeRight[rowRepairStraightside].x) *
                        5 / 4;
            int y = (track.pointsEdgeLeft[rowRepairStraightside].y +
                     track.pointsEdgeRight[rowRepairStraightside].y) /
                    2;

            POINT startPoint =
                track.pointsEdgeRight[rowRepairStraightside]; // 补线：起点
            POINT midPoint(x, y);                             // 补线：中点
            POINT endPoint(rowYendStraightside, 0);           // 补线：终点

            vector<POINT> input = {startPoint, midPoint, endPoint};
            vector<POINT> b_modify = Bezier(0.01, input);
            track.pointsEdgeLeft.resize(rowRepairRingside);
            track.pointsEdgeRight.resize(rowRepairStraightside);
            for (int kk = 0; kk < b_modify.size(); ++kk) {
              track.pointsEdgeRight.emplace_back(b_modify[kk]);
            }
            break;
          }
        }
      }
    }

    int tmp_ttttt = 0;
    if (ringEnable == false && ringStep == RingStep::Entering) {
      // 本场没判出环，且没有分叉
      if (!track.spurroad.empty() &&
          rowRepairLine < track.pointsEdgeRight.size() - 1 &&
          rowBreakpointRight > ROWSIMAGE / 2) {

        rowRepairStraightside = rowRepairLine;

        if (ringType == RingType::RingLeft) {
          tmp_ttttt = 1;
          for (int i = rowRepairLine; i < track.pointsEdgeLeft.size() - 1;
               i++) {
            if (track.pointsEdgeLeft[i].y <= 2 &&
                i != track.widthBlock.size() - 1) {
              rowRepairRingside = i;
              break;
              // rowYendStraightside = track.pointsEdgeLeft[i].x;
            }
          }

          for (int i = rowRepairRingside; i < track.pointsEdgeLeft.size() - 1;
               i++) {
            if (track.pointsEdgeLeft[i].y <= 2 &&
                i != track.widthBlock.size() - 1) {
              rowYendStraightside = track.pointsEdgeLeft[i].x;
            } else if (rowRepairRingside != track.widthBlock.size() - 1) {
              int x = track.pointsEdgeLeft[rowRepairStraightside].x +
                      (rowYendStraightside -
                       track.pointsEdgeRight[rowRepairStraightside].x) *
                          5 / 4;
              int y = (track.pointsEdgeLeft[rowRepairStraightside].y +
                       track.pointsEdgeRight[rowRepairStraightside].y) /
                      2;

              POINT startPoint =
                  track.pointsEdgeRight[rowRepairStraightside]; // 补线：起点
              POINT midPoint(x, y);                   // 补线：中点
              POINT endPoint(rowYendStraightside, 0); // 补线：终点

              // for (int i = 0; i < track.spurroad.size(); i++)
              // {
              //     if (track.spurroad[i].y < startPoint.y &&
              //     track.spurroad[i].x < startPoint.x)
              //         endPoint = track.spurroad[i];
              //     break;
              // }

              vector<POINT> input = {startPoint, midPoint, endPoint};
              vector<POINT> b_modify = Bezier(0.02, input);
              track.pointsEdgeLeft.resize(rowRepairRingside);
              track.pointsEdgeRight.resize(rowRepairStraightside);

              for (int kk = 0; kk < b_modify.size(); ++kk) {
                track.pointsEdgeRight.emplace_back(b_modify[kk]);
              }
              break;
            }
          }
        }
      }
      // 本场没判出环，有分叉
      else {
        if (ringType == RingType::RingLeft &&
            track.pointsEdgeRight.size() > 1) {
          tmp_ttttt = 2;
          int x_end = track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x;
          for (int kkk =
                   track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x;
               kkk <
               track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x + 50;
               kkk++) {
            if (imagePath.at<Vec3b>(kkk, 0)[2] > 0) {
              x_end = kkk;
              break;
            }
          }

          POINT startPoint(ROWSIMAGE - 10, COLSIMAGE - 1); // 补线：起点
          POINT endPoint(x_end, 0);                        // 补线：终点

          // for (int i = 0; i < track.spurroad.size(); i++)
          // {
          //     if (track.spurroad[i].y < startPoint.y && track.spurroad[i].x <
          //     startPoint.x)
          //         endPoint = track.spurroad[i];
          //     break;
          // }
          POINT midPoint =
              POINT((startPoint.x + endPoint.x) * 0.5,
                    (startPoint.y + endPoint.y) * 0.5); // 补线：中点
          vector<POINT> input = {startPoint, midPoint, endPoint};
          vector<POINT> b_modify = Bezier(0.02, input);
          track.pointsEdgeRight.resize(0);
          track.pointsEdgeLeft.resize(0);
          for (int kk = 0; kk < b_modify.size(); ++kk) {
            track.pointsEdgeRight.emplace_back(b_modify[kk]);
          }
        }
      }
    }
    // 环中
    if (ringStep == RingStep::Entering && track.spurroad.empty() &&
        counterSpurroad >= 3) {
      ringStep = RingStep::Inside;
    }
    // 出环补线
    if (ringStep == RingStep::Inside) {
      if (ringType == RingType::RingLeft) {
        int rowBreakRight = 0; // 右边缘横坐标连续性(行号)
        for (int i = 0; i < track.pointsEdgeRight.size(); i += 3) {
          if (track.pointsEdgeRight[i].y <=
              track.pointsEdgeRight[rowBreakRight].y) {
            rowBreakRight = i;
            continue;
          }
          if (i > rowBreakRight &&
              track.pointsEdgeRight[i].y -
                      track.pointsEdgeRight[rowBreakRight].y >
                  5) {
            break; // 寻找到出环口：出环补线
          }
        }
        track.pointsEdgeLeft.resize(0); // 单边控制
        int acute_angle_flag = 0;
        if (!track.pointsEdgeRight.empty() &&
            track.pointsEdgeRight[rowBreakRight].y < COLSIMAGE / 4) {
          track.pointsEdgeRight.resize(rowBreakRight); // 前80列不需要补线
        } else if (track.pointsEdgeRight.size() - rowBreakRight > 20) {
          float slopeTop = 0;    // 斜率：分歧点上半部分
          float slopeButtom = 0; // 斜率：分歧点下半部分
          if (track.pointsEdgeRight[rowBreakRight].x !=
              track.pointsEdgeRight[0].x) {
            slopeButtom = (track.pointsEdgeRight[rowBreakRight].y -
                           track.pointsEdgeRight[0].y) *
                          100 /
                          (track.pointsEdgeRight[rowBreakRight].x -
                           track.pointsEdgeRight[0].x);
          }
          if (track.pointsEdgeRight[rowBreakRight].x !=
              track.pointsEdgeRight[rowBreakRight + 20].x) {
            slopeTop = (track.pointsEdgeRight[rowBreakRight + 20].y -
                        track.pointsEdgeRight[rowBreakRight].y) *
                       100 /
                       (track.pointsEdgeRight[rowBreakRight + 20].x -
                        track.pointsEdgeRight[rowBreakRight].x);
          }

          if (slopeButtom * slopeTop <= 0) {
            rowBreakpointLeft = track.pointsEdgeRight[track.validRowsLeft].x;
            POINT p_end(rowBreakpointLeft, 0); // 补线终点为左边有效行顶点
            POINT p_mid(
                (track.pointsEdgeRight[rowBreakRight].x + rowBreakpointLeft) *
                    3 / 8,
                track.pointsEdgeRight[rowBreakRight].y / 2);
            vector<POINT> input = {track.pointsEdgeRight[rowBreakRight], p_mid,
                                   p_end};
            vector<POINT> b_modify = Bezier(0.01, input);
            track.pointsEdgeRight.resize(rowBreakRight);
            for (int kk = 0; kk < b_modify.size(); ++kk) {
              track.pointsEdgeRight.emplace_back(b_modify[kk]);
            }
          }
        } else if (track.pointsEdgeRight.size() - rowBreakRight <= 20) {
          _index = 2;
          POINT p_end(rowBreakpointLeft, 0);
          POINT p_start(max(rowBreakpointRight, ROWSIMAGE - 80), COLSIMAGE);
          POINT p_mid((ROWSIMAGE - 50 + rowBreakpointLeft) / 4, COLSIMAGE / 2);
          vector<POINT> input = {p_start, p_mid, p_end};
          vector<POINT> b_modify = Bezier(0.01, input);
          track.pointsEdgeRight.resize(0);
          for (int kk = 0; kk < b_modify.size(); ++kk) {
            track.pointsEdgeRight.emplace_back(b_modify[kk]);
          }
        }
      } else {
        ;
      }
      if (max(rowBreakpointLeft, rowBreakpointRight) < ROWSIMAGE / 2) {
        ringStep = RingStep::Exiting;
      }
    }
    // 出环完成
    else if (ringStep == RingStep::Exiting) {
      if (ringType == RingType::RingLeft && rowBreakpointLeft < ROWSIMAGE / 2) {
        POINT p_end(rowBreakpointLeft, 0);
        POINT p_start(ROWSIMAGE - 50, COLSIMAGE - 1);
        POINT p_mid((ROWSIMAGE - 50 + rowBreakpointLeft) * 3 / 8,
                    COLSIMAGE / 2);
        vector<POINT> input = {p_start, p_mid, p_end};
        vector<POINT> b_modify = Bezier(0.01, input);
        track.pointsEdgeRight.resize(0);
        track.pointsEdgeLeft.resize(0);
        for (int kk = 0; kk < b_modify.size(); ++kk) {
          track.pointsEdgeRight.emplace_back(b_modify[kk]);
        }
        if (rowBreakpointRight > ROWSIMAGE / 2) {
          ringStep = RingStep::Finish;
        }
      }
    }

    // //清掉边界的edge点
    // vector<POINT> v_temp, v_temp2;
    // for (int jj = 0; jj < track.pointsEdgeLeft.size(); ++jj)
    // {
    //     if (track.pointsEdgeLeft[jj].y > 2)
    //     {
    //         v_temp.push_back(track.pointsEdgeLeft[jj]);
    //     }
    //     else
    //     {
    //         if (jj > track.pointsEdgeLeft.size() * 9 / 10)
    //         {
    //             break;
    //         }
    //     }

    //     if (track.pointsEdgeLeft[jj].y > COLSIMAGE * 9 / 10 && jj <
    //     track.pointsEdgeLeft.size() - 5)
    //     {
    //         break;
    //     }
    // }
    // track.pointsEdgeLeft = v_temp;
    // if (track.pointsEdgeLeft.size() < 5)
    // {
    //     track.pointsEdgeLeft.resize(0);
    // }

    // for (int jj = 0; jj < track.pointsEdgeRight.size(); ++jj)
    // {
    //     if (track.pointsEdgeRight[jj].y < COLSIMAGE - 3)
    //     {
    //         v_temp2.push_back(track.pointsEdgeRight[jj]);
    //     }
    //     else
    //     {
    //         if (jj > track.pointsEdgeRight.size() * 9 / 10)
    //         {
    //             break;
    //         }
    //     }
    //     if (track.pointsEdgeRight[jj].y < COLSIMAGE / 10 && jj <
    //     track.pointsEdgeRight.size() - 5)
    //     {
    //         break;
    //     }
    // }
    // track.pointsEdgeRight = v_temp2;
    // if (track.pointsEdgeRight.size() < 5)
    // {
    //     track.pointsEdgeRight.resize(0);
    // }

    // 出环，切回正常循迹
    if (ringStep == RingStep::Finish) {
      if (track.pointsEdgeLeft.size() > 30 &&
          track.pointsEdgeRight.size() > 30 &&
          abs(track.pointsEdgeRight.size() - track.pointsEdgeLeft.size() <
              track.pointsEdgeRight.size() / 3) &&
          track.spurroad.empty()) {
        ringStep = RingStep::None;
      }
    }

    if (track.spurroad.empty())
      counterSpurroad++;
    else
      counterSpurroad = 0;

    //--------------------------------临时测试----------------------------------
    _ringStep = ringStep;
    _ringEnable = ringEnable;
    _tmp_ttttt = tmp_ttttt;

    // 返回识别结果
    if (ringStep == RingStep::None)
      return false;
    else
      return true;
  }

  /**
   * @brief 绘制环岛识别图像
   *
   * @param ringImage 需要叠加显示的图像
   */
  void drawImage(TrackRecognition track, Mat &ringImage) {
    for (int i = 0; i < track.pointsEdgeLeft.size(); i++) {
      circle(ringImage,
             Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 2,
             Scalar(0, 255, 0), -1); // 绿色点
    }
    for (int i = 0; i < track.pointsEdgeRight.size(); i++) {
      circle(ringImage,
             Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 2,
             Scalar(0, 255, 255), -1); // 黄色点
    }

    for (int i = 0; i < track.spurroad.size(); i++) {
      circle(ringImage, Point(track.spurroad[i].y, track.spurroad[i].x), 5,
             Scalar(0, 0, 255), -1); // 红色点
    }

    putText(ringImage,
            to_string(_ringStep) + " " + to_string(_ringEnable) + " " +
                to_string(_tmp_ttttt),
            Point(COLSIMAGE - 80, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX,
            0.3, cv::Scalar(0, 0, 255), 1, CV_AA);

    putText(ringImage, to_string(_index), Point(80, ROWSIMAGE - 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);

    putText(ringImage,
            to_string(track.validRowsRight) + " | " +
                to_string(track.stdevRight),
            Point(COLSIMAGE - 100, ROWSIMAGE - 50), FONT_HERSHEY_TRIPLEX, 0.3,
            Scalar(0, 0, 255), 1, CV_AA);
    putText(ringImage,
            to_string(track.validRowsLeft) + " | " + to_string(track.stdevLeft),
            Point(30, ROWSIMAGE - 50), FONT_HERSHEY_TRIPLEX, 0.3,
            Scalar(0, 0, 255), 1, CV_AA);

    putText(ringImage, "Ring", Point(COLSIMAGE / 2 - 5, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
            CV_AA); // 显示赛道识别类型

    circle(ringImage, Point(_ringPoint.y, _ringPoint.x), 4, Scalar(255, 0, 0),
           -1); // 红色点
  }

private:
  uint16_t counterSpurroad = 0; // 岔路计数器
  // 临时测试用参数
  int _ringStep;
  int _ringEnable;
  int _tmp_ttttt;
  int _index = 0;
  POINT _ringPoint = POINT(0, 0);

  /**
   * @brief 环岛类型
   *
   */
  enum RingType {
    RingNone = 0, // 未知类型
    RingLeft,     // 左入环岛
    RingRight     // 右入环岛
  };

  /**
   * @brief 环岛运行步骤/阶段
   *
   */
  enum RingStep {
    None = 0, // 未知类型
    Entering, // 入环
    Inside,   // 环中
    Exiting,  // 出环
    Finish    // 环任务结束
  };

  RingType ringType = RingType::RingLeft; // 环岛类型
  RingStep ringStep = RingStep::None;     // 环岛处理阶段
  int rowRepairLine = 0;                  // 用于环补线的点（行号）
  int colRepairLine = 0;                  // 用于环补线的点（列号）
};
