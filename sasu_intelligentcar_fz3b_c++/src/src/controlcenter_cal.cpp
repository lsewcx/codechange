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
 * @file controlcenter_cal.cpp
 * @author your name (you@domain.com)
 * @brief 智能车控制中心计算
 * @version 0.1
 * @date 2022-02-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"
#include "recognition/track_recognition.cpp"
using namespace cv;
using namespace std;

class ControlCenterCal
{
public:
    int controlCenter;           // 智能车控制中心（0~320）
    vector<POINT> centerEdge;    // 赛道中心点集
    uint16_t validRowsLeft = 0;  // 边缘有效行数（左）
    uint16_t validRowsRight = 0; // 边缘有效行数（右）
    double sigmaCenter = 0;      // 中心点集的方差

    /**
     * @brief 控制中心计算
     *
     * @param pointsEdgeLeft 赛道左边缘点集
     * @param pointsEdgeRight 赛道右边缘点集
     */

    void controlCenterCal(TrackRecognition &track)
    {
        sigmaCenter = 0;
        controlCenter = COLSIMAGE / 2;
        centerEdge.clear();
        vector<POINT> v_center(4); // 三阶贝塞尔曲线
        style = "STRIGHT";

        // 边缘斜率重计算（边缘修正之后）
        track.stdevLeft = track.stdevEdgeCal(track.pointsEdgeLeft, ROWSIMAGE);
        track.stdevRight = track.stdevEdgeCal(track.pointsEdgeRight, ROWSIMAGE);

        // 边缘有效行优化
        if ((track.stdevLeft < 80 && track.stdevRight > 50) || (track.stdevLeft > 60 && track.stdevRight < 50))
        {
            validRowsCal(track.pointsEdgeLeft, track.pointsEdgeRight); // 边缘有效行计算
            track.pointsEdgeLeft.resize(validRowsLeft);
            track.pointsEdgeRight.resize(validRowsRight);
        }

        if (track.pointsEdgeLeft.size() > 4 && track.pointsEdgeRight.size() > 4) // 通过双边缘有效点的差来判断赛道类型
        {
            v_center[0] = {(track.pointsEdgeLeft[0].x + track.pointsEdgeRight[0].x) / 2, (track.pointsEdgeLeft[0].y + track.pointsEdgeRight[0].y) / 2};

            v_center[1] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].x + track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].x) / 2,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].y + track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].y) / 2};

            v_center[2] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].x + track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].x) / 2,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].y + track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].y) / 2};

            v_center[3] = {(track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x + track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x) / 2,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y + track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y) / 2};

            centerEdge = Bezier(0.03, v_center);

            style = "STRIGHT";
        }
        // 左单边
        else if ((track.pointsEdgeLeft.size() > 0 && track.pointsEdgeRight.size() <= 4) ||
                 (track.pointsEdgeLeft.size() > 0 && track.pointsEdgeRight.size() > 0 && track.pointsEdgeLeft[0].x - track.pointsEdgeRight[0].x > ROWSIMAGE / 2))
        {
            style = "RIGHT";
            centerEdge = centerCompute(track.pointsEdgeLeft, 0);
        }
        // 右单边
        else if ((track.pointsEdgeRight.size() > 0 && track.pointsEdgeLeft.size() <= 4) ||
                 (track.pointsEdgeRight.size() > 0 && track.pointsEdgeLeft.size() > 0 && track.pointsEdgeRight[0].x - track.pointsEdgeLeft[0].x > ROWSIMAGE / 2))
        {
            style = "LEFT";
            centerEdge = centerCompute(track.pointsEdgeRight, 1);
        }
        else if (track.pointsEdgeLeft.size() > 4 && track.pointsEdgeRight.size() == 0) // 左单边
        {
            v_center[0] = {track.pointsEdgeLeft[0].x, (track.pointsEdgeLeft[0].y + COLSIMAGE - 1) / 2};

            v_center[1] = {track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].x,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() / 3].y + COLSIMAGE - 1) / 2};

            v_center[2] = {track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].x,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() * 2 / 3].y + COLSIMAGE - 1) / 2};

            v_center[3] = {track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x,
                           (track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].y + COLSIMAGE - 1) / 2};

            centerEdge = Bezier(0.02, v_center);

            style = "RIGHT";
        }
        else if (track.pointsEdgeLeft.size() == 0 && track.pointsEdgeRight.size() > 4) // 右单边
        {
            v_center[0] = {track.pointsEdgeRight[0].x, track.pointsEdgeRight[0].y / 2};

            v_center[1] = {track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].x,
                           track.pointsEdgeRight[track.pointsEdgeRight.size() / 3].y / 2};

            v_center[2] = {track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].x,
                           track.pointsEdgeRight[track.pointsEdgeRight.size() * 2 / 3].y / 2};

            v_center[3] = {track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x,
                           track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].y / 2};

            centerEdge = Bezier(0.02, v_center);

            style = "LEFT";
        }

        // 加权控制中心计算
        int controlNum = 1;
        for (auto p : centerEdge)
        {
            if (p.x < ROWSIMAGE / 2)
            {
                controlNum += ROWSIMAGE / 2;
                controlCenter += p.y * ROWSIMAGE / 2;
            }
            else
            {
                controlNum += (ROWSIMAGE - p.x);
                controlCenter += p.y * (ROWSIMAGE - p.x);
            }
        }
        if (controlNum > 1)
        {
            controlCenter = controlCenter / controlNum;
        }

        if (controlCenter > COLSIMAGE)
            controlCenter = COLSIMAGE;
        else if (controlCenter < 0)
            controlCenter = 0;

        // 控制率计算
        if (centerEdge.size() > 20)
        {
            vector<POINT> centerV;
            int filt = centerEdge.size() / 5;
            for (int i = filt; i < centerEdge.size() - filt; i++) // 过滤中心点集前后1/5的诱导性
            {
                centerV.push_back(centerEdge[i]);
            }
            sigmaCenter = sigma(centerV);
        }
        else
            sigmaCenter = 1000;
    }

    /**
     * @brief 显示赛道线识别结果
     *
     * @param centerImage 需要叠加显示的图像
     */
    void drawImage(TrackRecognition track, Mat &centerImage)
    {
        // 赛道边缘绘制
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(centerImage, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1, Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(centerImage, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1, Scalar(0, 255, 255), -1); // 黄色点
        }

        // 绘制中心点集
        for (int i = 0; i < centerEdge.size(); i++)
        {
            circle(centerImage, Point(centerEdge[i].y, centerEdge[i].x), 1, Scalar(0, 0, 255), -1);
        }

        // 绘制加权控制中心：方向
        Rect rect(controlCenter, ROWSIMAGE - 20, 10, 20);
        rectangle(centerImage, rect, Scalar(0, 0, 255), CV_FILLED);

        // 详细控制参数显示
        int dis = 20;
        string str;
        putText(centerImage, style, Point(COLSIMAGE - 60, dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 赛道类型

        str = "Edge: " + formatDoble2String(track.stdevLeft, 1) + " | " + formatDoble2String(track.stdevRight, 1);
        putText(centerImage, str, Point(COLSIMAGE - 150, 2 * dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 斜率：左|右

        str = "Center: " + formatDoble2String(sigmaCenter, 2);
        putText(centerImage, str, Point(COLSIMAGE - 120, 3 * dis), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); // 中心点方差

        putText(centerImage, to_string(controlCenter), Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 40), FONT_HERSHEY_PLAIN, 1.2, Scalar(0, 0, 255), 1); // 中心
    }

private:
    string style = ""; // 赛道类型
    /**
     * @brief 搜索十字赛道突变行（左下）
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft)
    {
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeLeft.size() - 10; i++)
        {
            if (pointsEdgeLeft[i].y >= 2)
            {
                counter++;
                if (counter > 3)
                {
                    return i - 2;
                }
            }
            else
                counter = 0;
        }

        return 0;
    }

    /**
     * @brief 搜索十字赛道突变行（右下）
     *
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight)
    {
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeRight.size() - 10; i++) // 寻找左边跳变点
        {
            if (pointsEdgeRight[i].y < COLSIMAGE - 2)
            {
                counter++;
                if (counter > 3)
                {
                    return i - 2;
                }
            }
            else
                counter = 0;
        }

        return 0;
    }

    /**
     * @brief 赛道中心点计算：单边控制
     *
     * @param pointsEdge 赛道边缘点集
     * @param side 单边类型：左边0/右边1
     * @return vector<POINT>
     */
    vector<POINT> centerCompute(vector<POINT> pointsEdge, int side)
    {
        int step = 4;                    // 间隔尺度
        int offsetWidth = COLSIMAGE / 2; // 首行偏移量
        int offsetHeight = 0;            // 纵向偏移量

        vector<POINT> center; // 控制中心集合

        if (side == 0) // 左边缘
        {
            uint16_t counter = 0, rowStart = 0;
            for (int i = 0; i < pointsEdge.size(); i++) // 删除底部无效行
            {
                if (pointsEdge[i].y > 1)
                {
                    counter++;
                    if (counter > 2)
                    {
                        rowStart = i - 2;
                        break;
                    }
                }
                else
                    counter = 0;
            }

            offsetHeight = pointsEdge[rowStart].x - pointsEdge[0].x;
            counter = 0;
            for (int i = rowStart; i < pointsEdge.size(); i += step)
            {
                int py = pointsEdge[i].y + offsetWidth;
                if (py > COLSIMAGE - 1)
                {
                    counter++;
                    if (counter > 2)
                        break;
                }
                else
                {
                    counter = 0;
                    center.emplace_back(pointsEdge[i].x - offsetHeight, py);
                }
            }
        }
        else if (side == 1) // 右边沿
        {
            uint16_t counter = 0, rowStart = 0;
            for (int i = 0; i < pointsEdge.size(); i++) // 删除底部无效行
            {
                if (pointsEdge[i].y < COLSIMAGE - 1)
                {
                    counter++;
                    if (counter > 2)
                    {
                        rowStart = i - 2;
                        break;
                    }
                }
                else
                    counter = 0;
            }

            offsetHeight = pointsEdge[rowStart].x - pointsEdge[0].x;
            counter = 0;
            for (int i = rowStart; i < pointsEdge.size(); i += step)
            {
                int py = pointsEdge[i].y - offsetWidth;
                if (py < 1)
                {
                    counter++;
                    if (counter > 2)
                        break;
                }
                else
                {
                    counter = 0;
                    center.emplace_back(pointsEdge[i].x - offsetHeight, py);
                }
            }
        }

        return center;
        // return Bezier(0.2,center);
    }

    /**
     * @brief 边缘有效行计算：左/右
     *
     * @param pointsEdgeLeft
     * @param pointsEdgeRight
     */
    void validRowsCal(vector<POINT> pointsEdgeLeft, vector<POINT> pointsEdgeRight)
    {
        int counter = 0;
        if (pointsEdgeRight.size() > 10 && pointsEdgeLeft.size() > 10)
        {
            uint16_t rowBreakLeft = searchBreakLeftDown(pointsEdgeLeft);                                           // 右边缘上升拐点
            uint16_t rowBreakRight = searchBreakRightDown(pointsEdgeRight);                                        // 右边缘上升拐点
            if (pointsEdgeRight[pointsEdgeRight.size() - 1].y < COLSIMAGE / 2 && rowBreakRight - rowBreakLeft > 5) // 左弯道
            {
                if (pointsEdgeLeft.size() > rowBreakRight) // 左边缘有效行重新搜索
                {
                    for (int i = rowBreakRight; i < pointsEdgeLeft.size(); i++)
                    {
                        if (pointsEdgeLeft[i].y < 1)
                        {
                            counter++;
                            if (counter >= 3)
                            {
                                pointsEdgeLeft.resize(i - 3);
                            }
                        }
                        else
                            counter = 0;
                    }
                }
            }

            else if (pointsEdgeLeft[pointsEdgeLeft.size() - 1].y > COLSIMAGE / 2 && rowBreakLeft - rowBreakRight > 5) // 右弯道
            {

                if (pointsEdgeRight.size() > rowBreakLeft) // 左边缘有效行重新搜索
                {
                    for (int i = rowBreakLeft; i < pointsEdgeRight.size(); i++)
                    {
                        if (pointsEdgeRight[i].y > COLSIMAGE - 2)
                        {
                            counter++;
                            if (counter >= 3)
                            {
                                pointsEdgeRight.resize(i - 3);
                            }
                        }
                        else
                            counter = 0;
                    }
                }
            }
        }

        // 左边有效行
        validRowsLeft = 0;
        if (pointsEdgeLeft.size() > 1)
        {
            for (int i = pointsEdgeLeft.size() - 1; i >= 1; i--)
            {
                if (pointsEdgeLeft[i].y > 2 && pointsEdgeLeft[i - 1].y >= 2)
                {
                    validRowsLeft = i + 1;
                    break;
                }
                if (pointsEdgeLeft[i].y < 2 && pointsEdgeLeft[i - 1].y >= 2)
                {
                    validRowsLeft = i + 1;
                    break;
                }
            }
        }

        // 右边有效行
        validRowsRight = 0;
        if (pointsEdgeRight.size() > 1)
        {
            for (int i = pointsEdgeRight.size() - 1; i >= 1; i--)
            {
                if (pointsEdgeRight[i].y <= COLSIMAGE - 2 && pointsEdgeRight[i - 1].y <= COLSIMAGE - 2)
                {
                    validRowsRight = i + 1;
                    break;
                }
                if (pointsEdgeRight[i].y >= COLSIMAGE - 2 && pointsEdgeRight[i - 1].y < COLSIMAGE - 2)
                {
                    validRowsRight = i + 1;
                    break;
                }
            }
        }
    }
};