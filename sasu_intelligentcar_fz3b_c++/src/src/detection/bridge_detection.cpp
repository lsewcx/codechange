#pragma once
/**
********************************************************************************************************
*                                               示例代码
*                                             EXAMPLE  CODE
*
*                      (c) Copyright 2022; SaiShu.Lcc.; Leo; https://bjsstech.com
*                                   版权所属[SASU-北京赛曙科技有限公司]
*
*            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
*            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-FZ3B),
*            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
*********************************************************************************************************
* @file slope_detection.cpp
* @author Leo ()
* @brief 坡道（桥）路径处理
* @version 0.1
* @date 2022-06-26
*
* @copyright Copyright (c) 2022
*
*/

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/predictor.hpp"
#include "../recognition/track_recognition.cpp"

using namespace cv;
using namespace std;

class BridgeDetection
{
public:
    /**
     * @brief 初始化
     *
     */
    void reset(void)
    {
        counterSession = 0;   // 图像场次计数器
        counterRec = 0;       // 加油站标志检测计数器
        bridgeEnable = false; // 桥区域使能标志
    }

    bool bridgeDetection(TrackRecognition &track, vector<PredictResult> predict)
    {
        if (bridgeEnable) // 进入桥梁
        {
            if (track.pointsEdgeLeft.size() > ROWSIMAGE / 2 && track.pointsEdgeRight.size() > ROWSIMAGE / 2) // 切行，防止错误前瞻引发转向
            {
                track.pointsEdgeLeft.resize(track.pointsEdgeLeft.size() / 2);
                track.pointsEdgeRight.resize(track.pointsEdgeRight.size() / 2);
            }
            counterSession++;
            if (counterSession > 30) // 上桥40场图像后失效
            {
                counterRec = 0;
                counterSession = 0;
                bridgeEnable = false;
            }

            return true;
        }
        else // 检测桥
        {
            for (int i = 0; i < predict.size(); i++)
            {
                if (predict[i].label == LABEL_BRIDGE)
                {
                    counterRec++;
                    break;
                }
            }

            if (counterRec)
            {
                counterSession++;
                if (counterRec >= 4 && counterSession < 8)
                {
                    counterRec = 0;
                    counterSession = 0;
                    bridgeEnable = true; // 检测到桥标志
                    return true;
                }
                else if (counterSession >= 8)
                {
                    counterRec = 0;
                    counterSession = 0;
                }
            }

            return false;
        }
    }

    /**
     * @brief 识别结果图像绘制
     *
     */
    void drawImage(TrackRecognition track, Mat &image)
    {
        // 赛道边缘
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
                   Scalar(0, 255, 255), -1); // 黄色点
        }

        if (bridgeEnable)
            putText(image, "bridgeEnable", Point(COLSIMAGE / 2 - 10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

private:
    uint16_t counterSession = 0; // 图像场次计数器
    uint16_t counterRec = 0;     // 加油站标志检测计数器
    bool bridgeEnable = false;   // 桥区域使能标志
};