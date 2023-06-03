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
 * @file farmland_detection.cpp
 * @author Leo ()
 * @brief 农田区域检测与图像处理
 * @version 0.1
 * @date 2022-12-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

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

class FarmlandDetection
{
public:
    /**
     * @brief 初始化
     *
     */
    void reset(void)
    {
        farmlandStep = FarmlandStep::None;
    }

    /**
     * @brief 加油站检测与路径规划
     *
     * @param track 赛道识别结果
     * @param detection AI检测结果
     */
    bool farmlandDetection(TrackRecognition &track, vector<PredictResult> predict)
    {
        indexDebug = 0;
        switch (farmlandStep)
        {
        case FarmlandStep::None: //[01] 农田区域检测
            searchCorn(predict); // 玉米检测
            if (pointCorn.x > 0 || pointCorn.y > 0)
                counterRec++;

            if (counterRec)
            {
                counterSession++;
                if (counterRec > 4 && counterSession < 8)
                {
                    farmlandStep = FarmlandStep::Enable; // 农田区域使能
                    counterRec = 0;
                    counterSession = 0;
                }
                else if (counterSession >= 8)
                {
                    counterRec = 0;
                    counterSession = 0;
                }
            }
            pointsEdgeLeftLast = track.pointsEdgeLeft; // 记录前一场数据
            pointsEdgeRightLast = track.pointsEdgeRight;
            break;
        case FarmlandStep::Enable: //[02] 农田区域使能
        {
            if (track.pointsEdgeLeft.size() < ROWSIMAGE / 2 && track.pointsEdgeRight.size() < ROWSIMAGE / 2)
            {
                counterRec++;
                if (counterRec > 2)
                {
                    counterRec = 0;
                    farmlandStep = FarmlandStep::Enter; // 完全进入农田
                }
            }
            else
            {
                if (track.pointsEdgeLeft.size() > 50)
                {
                    track.pointsEdgeLeft.resize(track.pointsEdgeLeft.size() * 0.7);
                }
                if (track.pointsEdgeRight.size() > 50)
                {
                    track.pointsEdgeRight.resize(track.pointsEdgeRight.size() * 0.7);
                }
            }

            break;
        }
        case FarmlandStep::Enter: //[02] 开进进入
        {
            bool refreshed = true;
            if (track.pointsEdgeLeft.size() < 30 && track.pointsEdgeRight.size() < 30)
            {
                counterRec++;
                if (counterRec > 4)
                {
                    counterRec = 0;
                    farmlandStep = FarmlandStep::Cruise; // 完全进入农田
                }
            }
            conesEdgeLeft.clear();
            conesEdgeRight.clear();
            searchCorn(predict);                                   // 玉米检测
            searchCones(predict);                                  // 检索锥桶位置
            int breakLeft = searchBreakLeft(track.pointsEdgeLeft); // Track边缘校验
            if (breakLeft > 0)
            {
                conesEdgeLeft.push_back(track.pointsEdgeLeft[breakLeft]);
                track.pointsEdgeLeft.resize(breakLeft);
                conesEdgeRight.push_back(track.pointsEdgeRight[track.pointsEdgeRight.size() - 1]);
            }
            else // 初始左右锥桶搜索
            {
                conesEdgeLeft.push_back(POINT(0, 0));         // 左下边缘点
                conesEdgeRight.push_back(POINT(0, 0));        // 右下边缘点
                for (int i = 0; i < pointEdgeDet.size(); i++) // 第一行边缘点搜索
                {
                    if (pointEdgeDet[i].y < COLSIMAGE / 2 && pointEdgeDet[i].x > conesEdgeLeft[0].x)
                        conesEdgeLeft[0] = pointEdgeDet[i];

                    if (pointEdgeDet[i].y > COLSIMAGE / 2 && pointEdgeDet[i].x > conesEdgeRight[0].x)
                        conesEdgeRight[0] = pointEdgeDet[i];
                }

                if (abs(conesEdgeRight[0].y - conesEdgeLeft[0].y) > COLSIMAGE / 3 && abs(conesEdgeRight[0].x - conesEdgeLeft[0].x) > ROWSIMAGE / 3) // 单边
                {
                    if (conesEdgeLeft[0].x > conesEdgeRight[0].x) // 左单边
                    {
                        conesEdgeRight.clear();
                    }
                    else if (conesEdgeLeft[0].x < conesEdgeRight[0].x) // 右单边
                    {
                        conesEdgeLeft.clear();
                    }
                }
            }
            searchConesEdge(pointEdgeDet);                                                  // 锥桶边缘坐标检索
            if (conesEdgeLeft.size() >= conesEdgeRight.size() && conesEdgeLeft.size() >= 2) // 左边补右边
            {
                vector<POINT> pointsRep;
                if (conesEdgeRight.size() > 0 && conesEdgeRight[0].x > ROWSIMAGE / 3)
                {
                    pointsRep = predictEdgeRight(conesEdgeLeft, conesEdgeLeft[0], conesEdgeRight[0]);
                    indexDebug = 1;
                }
                else
                {
                    pointsRep = predictEdgeRight(conesEdgeLeft, conesEdgeLeft[0], POINT(0, 0));
                    indexDebug = 2;
                }
                conesEdgeRight.clear();
                conesEdgeRight = pointsRep;

                POINT startPoint = conesEdgeLeft[0];
                POINT midPoint = conesEdgeLeft[conesEdgeLeft.size() / 2];
                POINT endPoint = conesEdgeLeft[conesEdgeLeft.size() - 1];
                vector<POINT> input = {startPoint, midPoint, endPoint};
                vector<POINT> repair = Bezier(0.05, input);
                conesEdgeLeft.clear();
                conesEdgeLeft = repair;
            }
            else if (conesEdgeLeft.size() < conesEdgeRight.size() && conesEdgeRight.size() >= 2) // 右边补左边
            {
                vector<POINT> pointsRep;
                if (conesEdgeLeft.size() > 0 && conesEdgeLeft[0].x > ROWSIMAGE / 3)
                {
                    pointsRep = predictEdgeLeft(conesEdgeLeft, conesEdgeLeft[0], conesEdgeRight[0]);
                    indexDebug = 3;
                }
                else
                {
                    pointsRep = predictEdgeLeft(conesEdgeLeft, POINT(0, 0), conesEdgeRight[0]);
                    indexDebug = 4;
                }
                conesEdgeLeft.clear();
                conesEdgeLeft = pointsRep;

                POINT startPoint = conesEdgeRight[0];
                POINT midPoint = conesEdgeRight[conesEdgeRight.size() / 2];
                POINT endPoint = conesEdgeRight[conesEdgeRight.size() - 1];
                vector<POINT> input = {startPoint, midPoint, endPoint};
                vector<POINT> repair = Bezier(0.05, input);
                conesEdgeRight.clear();
                conesEdgeRight = repair;
            }
            else
            {
                track.pointsEdgeLeft = pointsEdgeLeftLast;
                track.pointsEdgeRight = pointsEdgeRightLast;
                refreshed = false;
                indexDebug = 5;
            }

            if (refreshed) // 检索到有效的锥桶信息
            {
                for (int i = 0; i < conesEdgeLeft.size(); i++)
                    track.pointsEdgeLeft.push_back(conesEdgeLeft[i]);
                for (int i = 0; i < conesEdgeRight.size(); i++)
                    track.pointsEdgeRight.push_back(conesEdgeRight[i]);
            }

            pointsEdgeLeftLast = track.pointsEdgeLeft; // 记录前一场数据
            pointsEdgeRightLast = track.pointsEdgeRight;
            break;
        }

        case FarmlandStep::Cruise: //[03] 完全驶入农田
        {
            bool refreshed = true;
            if (track.pointsEdgeLeft.size() > 50 && track.pointsEdgeRight.size() > 50)
                counterRec++;
            if (counterRec)
            {
                counterSession++;
                if (counterRec > 3 && counterSession < 6)
                {
                    counterRec = 0;
                    counterSession = 0;
                    farmlandStep = FarmlandStep::None; // 出农田
                }
            }
            if (counterSession > 8)
            {
                counterRec = 0;
                counterSession = 0;
            }

            conesEdgeLeft.clear();
            conesEdgeRight.clear();
            searchCorn(predict);                          // 玉米检测
            searchCones(predict);                         // 检索锥桶位置
            conesEdgeLeft.push_back(POINT(0, 0));         // 左下边缘点
            conesEdgeRight.push_back(POINT(0, 0));        // 右下边缘点
            for (int i = 0; i < pointEdgeDet.size(); i++) // 第一行边缘点搜索
            {
                if (pointEdgeDet[i].y < COLSIMAGE / 2 && pointEdgeDet[i].x > conesEdgeLeft[0].x)
                    conesEdgeLeft[0] = pointEdgeDet[i];

                if (pointEdgeDet[i].y > COLSIMAGE / 2 && pointEdgeDet[i].x > conesEdgeRight[0].x)
                    conesEdgeRight[0] = pointEdgeDet[i];
            }

            if (abs(conesEdgeRight[0].y - conesEdgeLeft[0].y) > COLSIMAGE / 3 && abs(conesEdgeRight[0].x - conesEdgeLeft[0].x) > ROWSIMAGE / 3) // 单边
            {
                if (conesEdgeLeft[0].x > conesEdgeRight[0].x) // 左单边
                {
                    conesEdgeRight.clear();
                }
                else if (conesEdgeLeft[0].x < conesEdgeRight[0].x) // 右单边
                {
                    conesEdgeLeft.clear();
                }
            }

            searchConesEdge(pointEdgeDet);                                                  // 锥桶边缘坐标检索
            if (conesEdgeLeft.size() >= conesEdgeRight.size() && conesEdgeLeft.size() >= 2) // 左边补右边
            {
                vector<POINT> pointsRep;
                if (conesEdgeRight.size() > 0 && conesEdgeRight[0].x > ROWSIMAGE / 3)
                {
                    pointsRep = predictEdgeRight(conesEdgeLeft, conesEdgeLeft[0], conesEdgeRight[0]);
                    indexDebug = 6;
                }
                else
                {
                    pointsRep = predictEdgeRight(conesEdgeLeft, conesEdgeLeft[0], POINT(0, 0));
                    indexDebug = 7;
                }
                conesEdgeRight.clear();
                conesEdgeRight = pointsRep;

                POINT startPoint = conesEdgeLeft[0];
                POINT midPoint = conesEdgeLeft[conesEdgeLeft.size() / 2];
                POINT endPoint = conesEdgeLeft[conesEdgeLeft.size() - 1];
                vector<POINT> input = {startPoint, midPoint, endPoint};
                vector<POINT> repair = Bezier(0.05, input);
                conesEdgeLeft.clear();
                conesEdgeLeft = repair;
            }
            else if (conesEdgeLeft.size() < conesEdgeRight.size() && conesEdgeRight.size() >= 2) // 右边补左边
            {
                vector<POINT> pointsRep;
                if (conesEdgeLeft.size() > 0 && conesEdgeLeft[0].x > ROWSIMAGE / 2)
                {
                    pointsRep = predictEdgeLeft(conesEdgeLeft, conesEdgeLeft[0], conesEdgeRight[0]);
                    indexDebug = 8;
                }
                else
                {
                    pointsRep = predictEdgeLeft(conesEdgeLeft, POINT(0, 0), conesEdgeRight[0]);
                    indexDebug = 9;
                }
                conesEdgeLeft.clear();
                conesEdgeLeft = pointsRep;

                POINT startPoint = conesEdgeRight[0];
                POINT midPoint = conesEdgeRight[conesEdgeRight.size() / 2];
                POINT endPoint = conesEdgeRight[conesEdgeRight.size() - 1];
                vector<POINT> input = {startPoint, midPoint, endPoint};
                vector<POINT> repair = Bezier(0.05, input);
                conesEdgeRight.clear();
                conesEdgeRight = repair;
            }
            else
            {
                track.pointsEdgeLeft = pointsEdgeLeftLast;
                track.pointsEdgeRight = pointsEdgeRightLast;
                refreshed = false;
                indexDebug = 10;
            }

            if (refreshed) // 检索到有效的锥桶信息
            {
                for (int i = 0; i < conesEdgeLeft.size(); i++)
                    track.pointsEdgeLeft.push_back(conesEdgeLeft[i]);
                for (int i = 0; i < conesEdgeRight.size(); i++)
                    track.pointsEdgeRight.push_back(conesEdgeRight[i]);
            }

            pointsEdgeLeftLast = track.pointsEdgeLeft; // 记录前一场数据
            pointsEdgeRightLast = track.pointsEdgeRight;
            break;
        }
        }

        if (farmlandStep == FarmlandStep::None)
            return false;
        else
            return true;
    }

    /**
     * @brief 识别结果图像绘制
     *
     */
    void drawImage(TrackRecognition track, Mat &image)
    {
        // 绘制4象限分割线
        line(image, Point(0, image.rows / 2), Point(image.cols, image.rows / 2), Scalar(255, 255, 255), 1);
        line(image, Point(image.cols / 2, 0), Point(image.cols / 2, image.rows - 1), Scalar(255, 255, 255), 1);

        // 绘制锥桶坐标
        for (int i = 0; i < pointEdgeDet.size(); i++)
        {
            circle(image, Point(pointEdgeDet[i].y, pointEdgeDet[i].x), 4, Scalar(92, 92, 205), -1); // 锥桶坐标：红色
        }

        for (int i = 0; i < conesEdgeLeft.size(); i++)
            circle(image, Point(conesEdgeLeft[i].y, conesEdgeLeft[i].x), 3, Scalar(0, 255, 0), -1); // 锥桶左边缘：绿色
        for (int i = 0; i < conesEdgeRight.size(); i++)
            circle(image, Point(conesEdgeRight[i].y, conesEdgeRight[i].x), 3, Scalar(0, 255, 255), -1); // 锥桶右边缘：黄色

        // if (counters.size() > 1)
        // {
        //     vector<vector<Point>> pointsDet;
        //     pointsDet.push_back(counters);
        //     drawContours(image, pointsDet, -1, Scalar(0, 255, 0), -1);
        // }

        circle(image, Point(pointCorn.y, pointCorn.x), 3, Scalar(8, 112, 247), -1); // 玉米坐标绘制：橙色

        // 赛道边缘
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1, Scalar(0, 255, 0), -1); // 绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1, Scalar(0, 255, 255), -1); // 黄色点
        }

        // 显示施工区状态
        string state = "None";
        switch (farmlandStep)
        {
        case FarmlandStep::Enable:
            state = "FarmlandEnable";
            break;
        case FarmlandStep::Enter:
            state = "FarmlandEnter";
            break;
        case FarmlandStep::Cruise:
            state = "FarmlandCruise";
            break;
        }
        putText(image, state, Point(COLSIMAGE / 2 - 10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);

        putText(image, to_string(indexDebug), Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
    }

private:
    vector<POINT> pointEdgeDet;        // AI元素检测边缘点集
    vector<POINT> conesEdgeLeft;       // 左边缘锥桶点集
    vector<POINT> conesEdgeRight;      // 右边缘锥桶点集
    vector<POINT> pointsEdgeLeftLast;  // 记录前一场左边缘锥桶点集
    vector<POINT> pointsEdgeRightLast; // 记录前一场右边缘锥桶点集
    POINT pointCorn = POINT(0, 0);     // 玉米检测坐标
    uint16_t counterSession = 0;       // 图像场次计数器
    uint16_t counterRec = 0;           // 施工区标志检测计数器
    int indexDebug = 0;
    enum FarmlandStep
    {
        None = 0, // 未触发
        Enable,   // 农田区域检测使能（检测到玉米）
        Enter,    // 完全驶入农田
        Cruise
    };
    FarmlandStep farmlandStep = FarmlandStep::None;

    /**
     * @brief 从AI检测结果中检索锥桶坐标集合
     *
     * @param predict AI检测结果
     */
    void searchCones(vector<PredictResult> predict)
    {
        // AI结果检索
        pointEdgeDet.clear();
        for (int i = 0; i < predict.size(); i++)
        {
            if (predict[i].label == LABEL_CONE) // 锥桶检测
            {
                pointEdgeDet.push_back(POINT(predict[i].y + predict[i].height / 2, predict[i].x + predict[i].width / 2));
            }
        }
    }

    /**
     * @brief 锥桶边缘坐标检索
     *
     * @param cones 锥桶坐标点集
     * @param pointLD 左下点
     * @param pointRD 右下点
     */
    void searchConesEdge(vector<POINT> cones)
    {
        vector<POINT> pointsCones = cones;
        if (conesEdgeLeft.size() == 0 && conesEdgeRight.size() == 0) // 未搜索到起点
            return;

        while (pointsCones.size() > 0)
        {
            bool searched = false;
            int index = -1;
            for (int i = 0; i < pointsCones.size(); i++) // 搜索左边缘
            {
                if (conesEdgeLeft.size() > 0)
                {
                    if (distanceForPoints(conesEdgeLeft[conesEdgeLeft.size() - 1], pointsCones[i]) < conesEdgeLeft[conesEdgeLeft.size() - 1].x / 1.1 && pointsCones[i].x < conesEdgeLeft[conesEdgeLeft.size() - 1].x)
                    {
                        if (index < 0)
                            index = i;
                        else if (distanceForPoints(pointsCones[i], conesEdgeLeft[conesEdgeLeft.size() - 1]) < distanceForPoints(pointsCones[index], conesEdgeLeft[conesEdgeLeft.size() - 1]))
                            index = i;
                    }
                }
                else
                    break;
            }
            if (index >= 0)
            {
                conesEdgeLeft.push_back(pointsCones[index]);
                swap(pointsCones[index], pointsCones[pointsCones.size() - 1]);
                pointsCones.pop_back(); // 删除指定元素
                index = -1;
                searched = true;
            }

            for (int i = 0; i < pointsCones.size(); i++) // 搜索右边缘
            {
                if (conesEdgeRight.size() > 0)
                {
                    if (distanceForPoints(conesEdgeRight[conesEdgeRight.size() - 1], pointsCones[i]) < conesEdgeRight[conesEdgeRight.size() - 1].x / 1.1 && pointsCones[i].x < conesEdgeRight[conesEdgeRight.size() - 1].x)
                    {
                        if (index < 0)
                            index = i;
                        else if (distanceForPoints(pointsCones[i], conesEdgeRight[conesEdgeRight.size() - 1]) < distanceForPoints(pointsCones[index], conesEdgeRight[conesEdgeRight.size() - 1]))
                            index = i;
                    }
                }
                else
                    break;
            }
            if (index >= 0)
            {
                conesEdgeRight.push_back(pointsCones[index]);
                swap(pointsCones[index], pointsCones[pointsCones.size() - 1]);
                pointsCones.pop_back(); // 删除指定元素
                index = -1;
                searched = true;
            }

            if (!searched)
                return;
        }
    }

    /**
     * @brief 玉米坐标检测
     *
     * @param predict
     */
    void searchCorn(vector<PredictResult> predict)
    {
        POINT corn = POINT(0, 0);
        int distance = COLSIMAGE / 2;
        for (int i = 0; i < predict.size(); i++)
        {
            if (predict[i].label == LABEL_CORN && abs(predict[i].x + predict[i].width / 2 - COLSIMAGE / 2) < distance) // 玉米检测
            {
                corn = POINT(predict[i].y + predict[i].height, predict[i].x + predict[i].width / 2);
                distance = abs(predict[i].x + predict[i].width / 2 - COLSIMAGE / 2);
            }
        }

        pointCorn = corn;
    }

    /**
     * @brief 搜索Track左边拐点
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeft(vector<POINT> pointsEdgeLeft)
    {
        if (pointsEdgeLeft.size() < 10)
            return 0;

        uint16_t counter = 0;
        uint16_t counterBottom = 0; // 底行过滤计数器

        for (int i = 0; i < pointsEdgeLeft.size() - 1; i++) // 寻找左边跳变点
        {
            if (pointsEdgeLeft[i + 1].y - pointsEdgeLeft[i].y > 0)
                counterBottom++;
            if (counterBottom)
            {
                counter++;
                if (counter > 8 && counterBottom > 4)
                    return i - 8;
            }

            if (counter > 8)
            {
                counter = 0;
                counterBottom = 0;
            }
        }

        return 0;
    }

    /**
     * @brief 在俯视域由左边缘预测右边缘
     *
     * @param pointsEdgeLeft
     * @return vector<POINT>
     */
    vector<POINT> predictEdgeRight(vector<POINT> &pointsEdgeLeft, POINT pointL, POINT pointR)
    {
        int offset = 120; // 右边缘平移尺度
        vector<POINT> pointsEdgeRight;
        POINT startPoint(0, 0);
        POINT endPoint(0, 0);
        Point2d prefictRight;
        if (pointsEdgeLeft.size() < 3)
            return pointsEdgeRight;

        // Start
        if (pointR.x > 0 && pointR.y > 0)
        {
            startPoint = pointR; // 透视变换
            // Width
            Point2d leftIpm = ipm.homography(Point2d(pointL.y, pointL.x));  // 透视变换
            Point2d rightIpm = ipm.homography(Point2d(pointR.y, pointR.x)); // 透视变换
            offset = rightIpm.x - leftIpm.x;
            if (offset < 30)
                offset = 30;
            else if (offset > 150)
                offset = 150;
        }
        else
        {
            Point2d startIpm = ipm.homography(Point2d(pointsEdgeLeft[0].y, pointsEdgeLeft[0].x)); // 透视变换
            prefictRight = Point2d(startIpm.x + offset, startIpm.y);

            Point2d startIipm = ipm.homographyInv(prefictRight); // 反透视变换
            startPoint = POINT(startIipm.y, startIipm.x);
        }

        // Middle
        Point2d middleIpm = ipm.homography(Point2d(pointsEdgeLeft[pointsEdgeLeft.size() / 2].y, pointsEdgeLeft[pointsEdgeLeft.size() / 2].x)); // 透视变换
        prefictRight = Point2d(middleIpm.x + offset, middleIpm.y);
        Point2d middleIipm = ipm.homographyInv(prefictRight); // 反透视变换
        POINT midPoint = POINT(middleIipm.y, middleIipm.x);   // 补线中点

        // End
        Point2d endIpm = ipm.homography(Point2d(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y, pointsEdgeLeft[pointsEdgeLeft.size() - 1].x)); // 透视变换
        prefictRight = Point2d(endIpm.x + offset, endIpm.y);
        Point2d endtIipm = ipm.homographyInv(prefictRight); // 反透视变换
        endPoint = POINT(endtIipm.y, endtIipm.x);

        // 补线

        vector<POINT> input = {startPoint, midPoint, endPoint};
        vector<POINT> repair = Bezier(0.05, input);

        for (int i = 0; i < repair.size(); i++)
        {
            if (repair[i].x >= ROWSIMAGE)
                repair[i].x = ROWSIMAGE - 1;

            else if (repair[i].x < 0)
                repair[i].x = 0;

            else if (repair[i].y >= COLSIMAGE)
                repair[i].y = COLSIMAGE - 1;
            else if (repair[i].y < 0)
                repair[i].y = 0;

            pointsEdgeRight.push_back(repair[i]);
        }

        return pointsEdgeRight;
    }

    /**
     * @brief 在俯视域由右边缘预测左边缘
     *
     * @param pointsEdgeRight
     * @return vector<POINT>
     */
    vector<POINT> predictEdgeLeft(vector<POINT> &pointsEdgeRight, POINT pointL, POINT pointR)
    {
        int offset = 120; // 右边缘平移尺度
        vector<POINT> pointsEdgeLeft;
        POINT startPoint(0, 0);
        POINT endPoint(0, 0);
        Point2d prefictLeft;
        if (pointsEdgeRight.size() < 3)
            return pointsEdgeLeft;

        // Start
        if (pointL.x > 0 && pointL.y > 0)
        {
            startPoint = pointL; // 透视变换
            // Width
            Point2d leftIpm = ipm.homography(Point2d(pointL.y, pointL.x));  // 透视变换
            Point2d rightIpm = ipm.homography(Point2d(pointR.y, pointR.x)); // 透视变换
            offset = rightIpm.x - leftIpm.x;
            if (offset < 30)
                offset = 30;
            else if (offset > 150)
                offset = 150;
        }
        else
        {
            Point2d startIpm = ipm.homography(Point2d(pointsEdgeRight[0].y, pointsEdgeRight[0].x)); // 透视变换
            prefictLeft = Point2d(startIpm.x - offset, startIpm.y);
            Point2d startIipm = ipm.homographyInv(prefictLeft); // 反透视变换
            startPoint = POINT(startIipm.y, startIipm.x);
        }

        // Middle
        Point2d middleIpm = ipm.homography(Point2d(pointsEdgeRight[pointsEdgeRight.size() / 2].y, pointsEdgeRight[pointsEdgeRight.size() / 2].x)); // 透视变换
        prefictLeft = Point2d(middleIpm.x - offset, middleIpm.y);
        Point2d middleIipm = ipm.homographyInv(prefictLeft); // 反透视变换
        POINT midPoint = POINT(middleIipm.y, middleIipm.x);  // 补线中点

        // End
        Point2d endIpm = ipm.homography(Point2d(pointsEdgeRight[pointsEdgeRight.size() - 1].y, pointsEdgeRight[pointsEdgeRight.size() - 1].x)); // 透视变换
        prefictLeft = Point2d(endIpm.x - offset, endIpm.y);
        Point2d endtIipm = ipm.homographyInv(prefictLeft); // 反透视变换
        endPoint = POINT(endtIipm.y, endtIipm.x);

        // 补线

        vector<POINT> input = {startPoint, midPoint, endPoint};
        vector<POINT> repair = Bezier(0.05, input);

        for (int i = 0; i < repair.size(); i++)
        {
            if (repair[i].x >= ROWSIMAGE)
                repair[i].x = ROWSIMAGE - 1;

            else if (repair[i].x < 0)
                repair[i].x = 0;

            else if (repair[i].y >= COLSIMAGE)
                repair[i].y = COLSIMAGE - 1;
            else if (repair[i].y < 0)
                repair[i].y = 0;

            pointsEdgeLeft.push_back(repair[i]);
        }

        return pointsEdgeLeft;
    }
};