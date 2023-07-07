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
 * @file cross_recognition.cpp
 * @author Leo ()
 * @brief 十字道路识别与图像处理
 * @version 0.1
 * @date 2022-03-14
 *
 * @copyright Copyright (c) 2022
 *
 * @note 十字道路处理步骤：
 *                      [01] 入十字类型识别：track_recognition.cpp
 *                      [02] 补线起止点搜索
 *                      [03] 边缘重计算
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "track_recognition.cpp"
#include "../../include/predictor.hpp"

using namespace cv;
using namespace std;

class CrossroadRecognition
{
public:
    /**
     * @brief 初始化
     *
     */
    void reset(void)
    {
        crossroadType = CrossroadType::None; // 十字道路类型
    }

    /**
     * @brief 十字道路识别与图像处理
     *
     * @param track 赛道识别结果
     * @param imagePath 输入图像
     */
    bool crossroadRecognition(TrackRecognition &track, vector<PredictResult> predict)
    {
        bool repaired = false;               // 十字识别与补线结果
        crossroadType = CrossroadType::None; // 十字道路类型
        pointBreakLU = POINT(0, 0);
        pointBreakLD = POINT(0, 0);
        pointBreakRU = POINT(0, 0);
        pointBreakRD = POINT(0, 0);

        uint16_t counterRec = 0;    // 计数器
        uint16_t counterLinear = 0; // 连续计数器
        _index = 0;

        if (track.pointsEdgeRight.size() < ROWSIMAGE / 2 || track.pointsEdgeLeft.size() < ROWSIMAGE / 2) // 十字有效行限制
            return false;

        _index = 1;
        //----------------------------------------------------------------------------------------------------

        //[01] 左入十字处理
        if (track.stdevRight > 50)
        {
            // 通过色块突变-搜索十字类型
            for (int i = 2; i < track.widthBlock.size() - 10; i++)
            {
                // 左入十字判断
                if (track.pointsEdgeLeft[i].y < 2 && track.widthBlock[i].y > track.widthBlock[i - 2].y)
                    counterRec++;
                if (counterRec)
                {
                    counterLinear++;
                    if (counterLinear > 8)
                    {
                        counterLinear = 0;
                        counterRec = 0;
                    }
                    if (counterRec > 5)
                    {
                        crossroadType = CrossroadType::CrossroadLeft; // 左入十字
                        _index = 2;
                        break;
                    }
                }
            }
        }

        counterRec = 0;
        if (crossroadType == CrossroadType::CrossroadLeft) // 左入十字
        {
            uint16_t rowBreakRightDown = searchBreakRightDown(track.pointsEdgeRight); // 搜索十字赛道突变行（右下）

            if (rowBreakRightDown > 0 && track.pointsEdgeRight[rowBreakRightDown].y > 20)
            {
                pointBreakRD = track.pointsEdgeRight[rowBreakRightDown]; // 右下突变点
                if (track.spurroad.size() > 0)                           //[Step-1] 搜索到岔路
                {
                    int indexSP = 0;

                    for (int i = 0; i < track.spurroad.size(); i++) // 若存在多个岔路点：搜索最优点
                    {
                        if (pointBreakRD.y > track.spurroad[i].y && pointBreakRD.x > track.spurroad[i].x)
                        {
                            indexSP = i;
                            break;
                        }
                    }
                    for (int i = 0; i < track.spurroad.size(); i++) // 若存在多个岔路点：搜索最优点
                    {
                        if (pointBreakRD.y > track.spurroad[i].y && pointBreakRD.x > track.spurroad[i].x)
                        {
                            if (pointBreakRD.x - track.spurroad[i].x < pointBreakRD.x - track.spurroad[indexSP].x)
                                indexSP = i;
                        }
                    }

                    if (pointBreakRD.y > track.spurroad[indexSP].y && pointBreakRD.x > track.spurroad[indexSP].x)
                    {
                        uint16_t rowEnd = rowBreakRightDown; // 赛道重搜索行
                        for (int i = rowBreakRightDown; i < track.pointsEdgeRight.size(); i++)
                        {
                            if (track.pointsEdgeRight[i].x <= track.spurroad[indexSP].x)
                            {
                                rowEnd = i - 1;
                                break;
                            }
                        }

                        POINT startPoint = pointBreakRD;                                                              // 补线起点
                        POINT endPoint = track.spurroad[indexSP];                                                     // 补线终点
                        POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5); // 补线中点
                        vector<POINT> input = {startPoint, midPoint, endPoint};
                        vector<POINT> repair = Bezier(0.04, input);

                        track.pointsEdgeRight.resize(rowBreakRightDown); // 重绘右边缘
                        for (int i = 0; i < repair.size(); i++)
                        {
                            track.pointsEdgeRight.push_back(repair[i]);
                        }

                        if (track.spurroad[indexSP].y > COLSIMAGE / 8)
                            track.trackRecognition(true, rowEnd); // 赛道边缘重新搜索
                        else
                            track.pointsEdgeLeft.resize(rowEnd);

                        repaired = true; // 补线成功
                        _index = 3;
                    }
                }

                if (!repaired && pointBreakRD.y < COLSIMAGE / 2) //[Step-2] 未搜索到岔路
                {
                    _index = 4;
                    uint16_t rowBreakLU = rowBreakRightDown; // 左上拐点
                    for (int i = rowBreakRightDown; i < track.pointsEdgeLeft.size() - 10; i++)
                    {
                        if (track.pointsEdgeLeft[i].y > 1)
                            counterRec++;
                        else
                            counterRec = 0;
                        if (counterRec > 2)
                        {
                            rowBreakLU = i - 2;
                            break;
                        }
                    }

                    POINT endPoint = POINT(pointBreakRD.x, 1); // 补线终点
                    if (rowBreakLU < track.pointsEdgeLeft.size())
                    {
                        pointBreakLU = track.pointsEdgeLeft[rowBreakLU];            // 左上拐点
                        endPoint = POINT((pointBreakRD.x + pointBreakLU.x) / 2, 1); // 补线终点
                        track.pointsEdgeLeft.resize(rowBreakLU);                    // 重绘边缘
                    }

                    POINT startPoint = pointBreakRD;                                                              // 补线起点
                    POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5); // 补线中点
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> repair = Bezier(0.05, input);

                    track.pointsEdgeRight.resize(rowBreakRightDown); // 重绘右边缘
                    for (int i = 0; i < repair.size(); i++)
                    {
                        track.pointsEdgeRight.push_back(repair[i]);
                    }
                    repaired = true; // 补线成功
                }
            }
        }

        // 直入十字处理
        if (!repaired) // 如果写入十字未成功
        {
            // 通过色块突变-搜索十字类型
            uint16_t counterStrightA = 0; // 计数器
            uint16_t counterStrightB = 0; // 计数器
            for (int i = 2; i < track.widthBlock.size() - 10; i++)
            {
                // 直入十字判断
                if (track.spurroad.size() > 0 && track.widthBlock[i].y > COLSIMAGE - 5)
                {
                    counterStrightA++;
                }
                if (counterStrightA)
                {
                    counterStrightB++;
                    if (counterStrightB > 30)
                    {
                        counterStrightB = 0;
                        counterStrightA = 0;
                    }
                    if (counterStrightA > 20) // 连续20行全宽
                    {
                        crossroadType = CrossroadType::CrossroadStraight; // 直入十字
                        _index = 5;
                        break;
                    }
                }
            }

            if (crossroadType == CrossroadType::CrossroadStraight) // 直入十字
            {
                int indexSP = 0;
                for (int i = 0; i < track.spurroad.size(); i++) // 挑选准确的岔路点
                {
                    if (track.spurroad[i].x > track.spurroad[indexSP].y && track.spurroad[i].x < pointBreakRD.x)
                        indexSP = i;
                }

                // 搜索左边缘
                uint16_t rowBreakLU = searchBreakLeftUp(track.pointsEdgeLeft);   // 左上拐点搜索
                uint16_t rowBreakLD = searchBreakLeftDown(track.pointsEdgeLeft); // 左下拐点搜索

                // 优化左边缘
                if (rowBreakLU > rowBreakLD && rowBreakLU < COLSIMAGE / 2 && rowBreakLD < COLSIMAGE / 2)
                {
                    pointBreakLU = track.pointsEdgeLeft[rowBreakLU];
                    pointBreakLD = track.pointsEdgeLeft[rowBreakLD];

                    double k = (double)(pointBreakLU.y - pointBreakLD.y) /
                               (double)(pointBreakLU.x - pointBreakLD.x);
                    double b = pointBreakLU.y - k * pointBreakLU.x;
                    for (int i = rowBreakLD; i <= rowBreakLU; i++)
                    {
                        track.pointsEdgeLeft[i].y = (int)(k * track.pointsEdgeLeft[i].x + b);
                    }
                    repaired = true; // 补线成功
                }

                if (track.spurroad[indexSP].y > COLSIMAGE / 3)
                {
                    // 重新进行Track搜索
                    uint16_t rowEnd = 10;
                    for (int i = 10; i < track.pointsEdgeRight.size(); i++)
                    {
                        if (track.pointsEdgeRight[i].x <= track.spurroad[indexSP].x)
                        {
                            rowEnd = i - 1;
                            break;
                        }
                    }

                    POINT startPoint = track.pointsEdgeRight[0];                                                  // 补线起点
                    POINT endPoint = track.spurroad[indexSP];                                                     // 补线终点
                    POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5); // 补线中点
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> repair = Bezier(0.04, input);

                    track.pointsEdgeRight.clear(); // 重绘右边缘
                    track.pointsEdgeRight = repair;

                    track.trackRecognition(true, rowEnd); // 赛道边缘重新搜索
                    repaired = true;                      // 补线成功
                }
            }
        }

        return repaired;
    }

    /**
     * @brief 绘制十字道路识别结果
     *
     * @param Image 需要叠加显示的图像/RGB
     */
    void drawImage(TrackRecognition track, Mat &Image)
    {
        // 绘制边缘点
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(Image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 2,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(Image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 2,
                   Scalar(0, 255, 255), -1); // 黄色点
        }

        // 绘制岔路点
        for (int i = 0; i < track.spurroad.size(); i++)
        {
            circle(Image, Point(track.spurroad[i].y, track.spurroad[i].x), 6,
                   Scalar(0, 0, 255), -1); // 红色点
        }

        // 斜入十字绘制补线起止点
        if (crossroadType == CrossroadType::CrossroadRight) // 右入十字
        {
            circle(Image, Point(pointBreakLU.y, pointBreakLU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
            circle(Image, Point(pointBreakLD.y, pointBreakLD.x), 5, Scalar(255, 0, 255), -1);  // 下补线点：粉色
            if (pointBreakRU.x > 0)
                circle(Image, Point(pointBreakRU.y, pointBreakRU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
            if (pointBreakRD.x > 0)
                circle(Image, Point(pointBreakRD.y, pointBreakRD.x), 5, Scalar(255, 0, 255), -1); // 下补线点：粉色

            putText(Image, "Right", Point(COLSIMAGE / 2 - 15, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        }
        else if (crossroadType == CrossroadType::CrossroadLeft) // 左入十字
        {
            circle(Image, Point(pointBreakRU.y, pointBreakRU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
            circle(Image, Point(pointBreakRD.y, pointBreakRD.x), 5, Scalar(255, 0, 255), -1);  // 下补线点：粉色
            if (pointBreakLU.x > 0)
                circle(Image, Point(pointBreakLU.y, pointBreakLU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
            if (pointBreakLD.x > 0)
                circle(Image, Point(pointBreakLD.y, pointBreakLD.x), 5, Scalar(255, 0, 255), -1); // 下补线点：粉色

            putText(Image, "Left", Point(COLSIMAGE / 2 - 15, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        }
        else if (crossroadType == CrossroadType::CrossroadStraight) // 直入十字
        {
            circle(Image, Point(pointBreakLU.y, pointBreakLU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
            circle(Image, Point(pointBreakLD.y, pointBreakLD.x), 5, Scalar(255, 0, 255), -1);  // 下补线点：粉色
            circle(Image, Point(pointBreakRU.y, pointBreakRU.x), 5, Scalar(226, 43, 138), -1); // 上补线点：紫色
            circle(Image, Point(pointBreakRD.y, pointBreakRD.x), 5, Scalar(255, 0, 255), -1);  // 下补线点：粉色
            putText(Image, "Straight", Point(COLSIMAGE / 2 - 20, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        }

        putText(Image, "Cross", Point(COLSIMAGE / 2 - 5, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA); // 显示赛道识别类型

        putText(Image, to_string(_index), Point(COLSIMAGE / 2 - 5, ROWSIMAGE - 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 0, 155), 1, CV_AA);
    }

private:
    int _index = 0; // 测试

    POINT pointBreakLU;
    POINT pointBreakLD;
    POINT pointBreakRU;
    POINT pointBreakRD;
    uint16_t counterFild = 0;
    /**
     * @brief 十字道路类型
     *
     */
    enum CrossroadType
    {
        None = 0,
        CrossroadLeft,     // 左斜入十字
        CrossroadRight,    // 右斜入十字
        CrossroadStraight, // 直入十字
    };

    CrossroadType crossroadType = CrossroadType::None; // 十字道路类型

    /**
     * @brief 搜索十字赛道突变行（左上）
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeftUp(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeftUp = pointsEdgeLeft.size() - 5;
        uint16_t counter = 0;
        uint16_t counterFilter = 0;
        for (int i = pointsEdgeLeft.size() - 5; i > 50; i--)
        {
            if (pointsEdgeLeft[i].y > 2 & abs(pointsEdgeLeft[i].y - pointsEdgeLeft[i + 1].y) < 3)
            {
                rowBreakLeftUp = i;
                counter = 0;
                counterFilter++;
            }
            else if (pointsEdgeLeft[i].y <= 2 && counterFilter > 10)
            {
                counter++;
                if (counter > 5)
                    return rowBreakLeftUp;
            }
        }

        return rowBreakLeftUp;
    }
    /**
     * @brief 搜索十字赛道突变行（左下）
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeft = 0;
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeLeft.size() / 2; i++) // 寻找左边跳变点
        {
            if (pointsEdgeLeft[i].y > pointsEdgeLeft[rowBreakLeft].y)
            {
                rowBreakLeft = i;
                counter = 0;
            }
            else if (pointsEdgeLeft[i].y <= pointsEdgeLeft[rowBreakLeft].y) // 突变点计数
            {
                counter++;
                if (counter > 5)
                    return rowBreakLeft;
            }
        }

        return rowBreakLeft;
    }
    /**
     * @brief 搜索十字赛道突变行（右上）
     *
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRightUp(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRightUp = pointsEdgeRight.size() - 5;
        uint16_t counter = 0;
        uint16_t counterFilter = 0;
        for (int i = pointsEdgeRight.size() - 5; i > 50; i--)
        {
            if (pointsEdgeRight[i].y < COLSIMAGE - 2 & abs(pointsEdgeRight[i].y - pointsEdgeRight[i + 1].y) < 3)
            {
                rowBreakRightUp = i;
                counter = 0;
                counterFilter++;
            }
            else if (pointsEdgeRight[i].y >= COLSIMAGE - 2 && counterFilter > 10)
            {
                counter++;
                if (counter > 5)
                    return rowBreakRightUp;
            }
        }

        return rowBreakRightUp;
    }
    /**
     * @brief 搜索十字赛道突变行（右下）
     *
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRightDown = 0;
        uint16_t counter = 0;
        bool start = false;

        for (int i = 0; i < pointsEdgeRight.size() - 10; i++) // 寻找左边跳变点
        {
            if (pointsEdgeRight[i].y < COLSIMAGE - 1)
                counter++;
            else
                counter = 0;

            if (counter > 2)
            {
                start = true;
                counter = 0;
            }

            if (start) // 屏蔽初始行
            {
                if (pointsEdgeRight[i].y > pointsEdgeRight[i - 2].y)
                    counter++;
                else
                    counter = 0;

                if (counter > 2)
                    return i - 3;
            }
        }

        return rowBreakRightDown;
    }

    /**
     * @brief 直入十字搜索
     *
     * @param pointsEdgeLeft 赛道左边缘点集
     * @param pointsEdgeRight 赛道右边缘点集
     * @return true
     * @return false
     */
    bool searchStraightCrossroad(vector<POINT> pointsEdgeLeft, vector<POINT> pointsEdgeRight)
    {
        if (pointsEdgeLeft.size() < ROWSIMAGE * 0.8 || pointsEdgeRight.size() < ROWSIMAGE * 0.8)
        {
            return false;
        }

        uint16_t counterLeft = 0;
        uint16_t counterRight = 0;
        for (int i = pointsEdgeLeft.size() - 10; i > 1; i--) // 搜索上半部分边缘点
        {
            if (pointsEdgeLeft[i].x > ROWSIMAGE / 2)
                break;
            else if (pointsEdgeLeft[i].y < 2)
                counterLeft++;
        }
        for (int i = pointsEdgeRight.size() - 10; i > 1; i--) // 搜索上半部分边缘点
        {
            if (pointsEdgeRight[i].x > ROWSIMAGE / 2)
                break;
            else if (pointsEdgeRight[i].y > COLSIMAGE - 2)
                counterRight++;
        }
        if (counterLeft > 30 && counterRight > 30)
            return true;
        else
            return false;
    }
};