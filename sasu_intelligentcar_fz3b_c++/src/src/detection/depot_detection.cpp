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
 * @file depot_detection.cpp
 * @author Leo ()
 * @brief 维修厂检测
 * @version 0.1
 * @date 2022-12-13
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

class DepotDetection
{
public:
    bool carStoping = false;
    enum DepotStep
    {
        DepotNone = 0, // 未触发
        DepotEnable,   // 维修厂操作使能（标志识别成功）
        DepotEnter,    // 维修厂进站
        DepotCruise,   // 维修站巡航
        DepotStop,     // 停车
        DepotExit      // 维修厂出站
    };

    DepotStep depotStep = DepotStep::DepotNone;
    /**
     * @brief 维修厂检测初始化
     *
     */
    void reset(void)
    {
        carStoping = false;
        depotStep = DepotStep::DepotNone;
        counterSession = 0;         // 图像场次计数器
        counterRec = 0;             // 维修厂标志检测计数器
        lastPointsEdgeLeft.clear(); // 记录上一场边缘点集（丢失边）
        lastPointsEdgeRight.clear();
        counterExit = 0;
        counterImmunity = 0;
    }

    /**
     * @brief 维修厂检测与路径规划
     *
     * @param track 赛道识别结果
     * @param detection AI检测结果
     */
    bool depotDetection(TrackRecognition &track, vector<PredictResult> predict)
    {
        _pointNearCone = POINT(0, 0);
        _distance = 0;
        pointEdgeDet.clear();
        levelCones = 0;
        indexDebug = 0;

        switch (depotStep)
        {
        case DepotStep::DepotNone: //[01] 维修厂标志检测
            if (counterImmunity > 20)
            {
                for (int i = 0; i < predict.size(); i++)
                {
                    if (predict[i].label == LABEL_TRACTOR) // 拖拉机标志检测
                    {
                        counterRec++;
                        break;
                    }
                }
                if (counterRec)
                {
                    counterSession++;
                    if (counterRec > 3 && counterSession < 8)
                    {
                        depotStep = DepotStep::DepotEnable; // 维修厂使能
                        counterRec = 0;
                        counterSession = 0;
                    }
                    else if (counterSession >= 8)
                    {
                        counterRec = 0;
                        counterSession = 0;
                    }
                }
            }
            else
                counterImmunity++;
            break;

        case DepotStep::DepotEnable: //[02] 维修厂使能
        {
            counterExit++;
            if (counterExit > 60)
            {
                reset();
                return false;
            }

            searchCones(predict);
            _pointNearCone = searchNearestCone(track.pointsEdgeLeft, pointEdgeDet); // 搜索右下锥桶
            if (_pointNearCone.x > ROWSIMAGE * 0.6)                                 // 当车辆开始靠近右边锥桶：准备入库
            {
                counterRec++;
                if (counterRec >= 2)
                {
                    depotStep = DepotStep::DepotEnter; // 进站使能
                    counterRec = 0;
                    counterSession = 0;
                }
            }
            break;
        }
        case DepotStep::DepotEnter: //[03] 进站使能
        {
            if (track.pointsEdgeLeft.size() > ROWSIMAGE / 2 && track.pointsEdgeRight.size() > ROWSIMAGE / 2)
            {
                counterExit++;
            }

            if (track.pointsEdgeLeft.size() < ROWSIMAGE / 2 && track.pointsEdgeRight.size() < ROWSIMAGE / 2) // 赛道还未丢失
            {
                counterRec++;
                if (counterRec > 10)
                {
                    counterRec = 0;
                    depotStep = DepotStep::DepotCruise; // 巡航使能
                }
            }

            POINT start = POINT(ROWSIMAGE - 40, COLSIMAGE - 1);
            POINT end = POINT(50, 0);
            POINT middle = POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.6);
            vector<POINT> input = {start, middle, end};
            track.pointsEdgeRight = Bezier(0.05, input);                   // 补线
            track.pointsEdgeLeft = predictEdgeLeft(track.pointsEdgeRight); // 由右边缘补偿左边缘
            lastPointsEdgeLeft = track.pointsEdgeLeft;
            lastPointsEdgeRight = track.pointsEdgeRight;

            pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
            pathsEdgeRight.push_back(track.pointsEdgeRight);

            break;
        }

        case DepotStep::DepotCruise: //[04] 巡航使能
        {
            if (track.pointsEdgeLeft.size() > ROWSIMAGE / 2 && track.pointsEdgeRight.size() > ROWSIMAGE / 2)
            {
                counterExit++;
                if (counterExit > 30)
                {
                    counterExit = 0;
                    depotStep = DepotStep::DepotStop; // 停车使能
                    counterRec = 0;
                }
            }

            searchCones(predict);            // 计算锥桶的坐标
            searchHighestCone(pointEdgeDet); // 搜索顶点锥桶
            if (heighestCone.x > 0 && heighestCone.y > 0)
            {
                if (heighestCone.y >= COLSIMAGE / 3) // 顶角锥桶靠近右边→继续右转
                {
                    vector<POINT> points = searchLeftCones(pointEdgeDet); // 搜索以最高点为分界线左边的锥桶
                    if (points.size() >= 3)                               // 曲线补偿
                    {
                        pointsSortForY(points); // 排序
                        vector<POINT> input = {points[0], points[points.size() / 2], points[points.size() - 1]};
                        track.pointsEdgeLeft = Bezier(0.05, input);       // 补线
                        track.pointsEdgeRight = predictEdgeRight(points); // 由左边缘补偿右边缘

                        indexDebug = 1;
                    }
                    else if (points.size() >= 2)
                    {
                        pointsSortForY(points); // 排序
                        POINT middle = POINT((points[0].x + points[points.size() - 1].x) / 2, (points[0].y + points[points.size() - 1].y) / 2);
                        vector<POINT> input = {points[0], middle, points[points.size() - 1]};
                        track.pointsEdgeLeft = Bezier(0.05, input);       // 补线
                        track.pointsEdgeRight = predictEdgeRight(points); // 由左边缘补偿右边缘
                        indexDebug = 2;
                    }
                    else if (points.size() >= 1)
                    {
                        if (points[0].x > ROWSIMAGE / 2)
                            depotStep = DepotStep::DepotExit; // 出站使能
                    }
                }
                else // 顶角锥桶靠近左边→继续右转
                {
                    indexDebug = 3;
                    track.pointsEdgeLeft = lastPointsEdgeLeft;
                    track.pointsEdgeRight = lastPointsEdgeRight;
                }

                // 计算所有锥桶的平均高度
                int num = 0;
                for (int i = 0; i < pointEdgeDet.size(); i++)
                    num += pointEdgeDet[i].x;

                levelCones = num / pointEdgeDet.size();
                if (levelCones > ROWSIMAGE * 0.24 || levelCones == 0)
                {
                    counterRec++;
                    if (counterRec > 3)
                    {
                        depotStep = DepotStep::DepotStop; // 停车使能
                        counterRec = 0;
                    }
                }
            }
            else
            {
                indexDebug = 4;
                track.pointsEdgeLeft = lastPointsEdgeLeft;
                track.pointsEdgeRight = lastPointsEdgeRight;
            }

            lastPointsEdgeLeft = track.pointsEdgeLeft;
            lastPointsEdgeRight = track.pointsEdgeRight;

            pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
            pathsEdgeRight.push_back(track.pointsEdgeRight);
            break;
        }

        case DepotStep::DepotStop: //[05] 停车使能
        {
            carStoping = true;
            counterRec++;
            if (counterRec > 40) // 停车：40场 = 2s
            {
                carStoping = false;
                depotStep = DepotStep::DepotExit; // 出站使能
                counterRec = 0;
            }
            break;
        }

        case DepotStep::DepotExit: //[06] 出站使能
        {
            if (pathsEdgeLeft.size() < 1 || pathsEdgeRight.size() < 1)
            {
                depotStep = DepotStep::DepotNone; // 出厂完成
                reset();
            }
            else
            {
                track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];
                track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
                pathsEdgeLeft.pop_back();
                pathsEdgeRight.pop_back();
            }
            break;
        }
        }

        if (depotStep == DepotStep::DepotNone) // 返回维修厂控制模式标志
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
        // 绘制锥桶坐标
        for (int i = 0; i < pointEdgeDet.size(); i++)
        {
            circle(image, Point(pointEdgeDet[i].y, pointEdgeDet[i].x), 2, Scalar(92, 92, 205), -1); // 锥桶坐标：红色
        }

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

        // 显示维修厂状态
        string state = "None";
        switch (depotStep)
        {
        case DepotStep::DepotEnable:
            state = "DepotEnable";
            break;
        case DepotStep::DepotEnter:
            state = "DepotEnter";
            break;
        case DepotStep::DepotCruise:
            state = "DepotCruise";
            break;
        case DepotStep::DepotStop:
            state = "DepotStop";
            break;
        case DepotStep::DepotExit:
            state = "DepotExit";
            break;
        }
        putText(image, state, Point(COLSIMAGE / 2 - 10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);

        putText(image, to_string(_distance), Point(COLSIMAGE / 2 - 15, 40), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA); // 显示锥桶距离
        if (_pointNearCone.x > 0)
            circle(image, Point(_pointNearCone.y, _pointNearCone.x), 3, Scalar(200, 200, 200), -1);

        if (levelCones > 0)
            line(image, Point(0, levelCones), Point(image.cols, levelCones), Scalar(255, 255, 255), 1);

        putText(image, to_string(indexDebug), Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
    }

private:
    double _distance = 0;
    int levelCones = 0; // 锥桶的平均高度
    POINT _pointNearCone;
    POINT heighestCone;
    POINT pointHCone;
    vector<POINT> pointEdgeDet;       // AI元素检测边缘点集
    vector<POINT> lastPointsEdgeLeft; // 记录上一场边缘点集（丢失边）
    vector<POINT> lastPointsEdgeRight;

    vector<vector<POINT>> pathsEdgeLeft; // 记录入库路径
    vector<vector<POINT>> pathsEdgeRight;
    int indexDebug = 0;

    uint16_t counterSession = 0;  // 图像场次计数器
    uint16_t counterRec = 0;      // 维修厂标志检测计数器
    uint16_t counterExit = 0;     // 标志结束计数器
    uint16_t counterImmunity = 0; // 屏蔽计数器
    /**
     * @brief 从AI检测结果中检索锥桶坐标集合
     *
     * @param predict AI检测结果
     * @return vector<POINT>
     */
    void
    searchCones(vector<PredictResult> predict)
    {
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
     * @brief 搜索距离赛道左边缘最近的锥桶坐标
     *
     * @param pointsEdgeLeft 赛道边缘点集
     * @param predict AI检测结果
     * @return POINT
     */
    POINT searchNearestCone(vector<POINT> pointsEdgeLeft, vector<POINT> pointsCone)
    {
        POINT point(0, 0);
        double disMin = 50; // 右边缘锥桶离赛道左边缘最小距离

        if (pointsCone.size() <= 0 || pointsEdgeLeft.size() < 10)
            return point;

        POINT a = pointsEdgeLeft[pointsEdgeLeft.size() * 0.5];
        POINT b = pointsEdgeLeft[pointsEdgeLeft.size() * 0.8];

        for (int i = 0; i < pointsCone.size(); i++)
        {
            double dis = distanceForPoint2Line(a, b, pointsCone[i]);
            if (dis < disMin && pointsCone[i].x > point.x)
            {
                point = pointsCone[i];
                _distance = dis;
            }
        }

        return point;
    }

    /**
     * @brief 搜索坐标最高的锥桶
     *
     * @param pointsEdgeLeft 赛道边缘点集
     * @param predict AI检测结果
     * @return POINT
     */
    void searchHighestCone(vector<POINT> pointsCone)
    {
        heighestCone = POINT(0, 0);
        if (pointsCone.size() <= 0)
            return;
        heighestCone = pointsCone[0];
        for (int i = 0; i < pointsCone.size(); i++)
        {
            if (pointsCone[i].x < heighestCone.x)
                heighestCone = pointsCone[i];
        }
    }

    /**
     * @brief 搜索以最高点为分界线左边的锥桶
     *
     * @param pointsEdgeLeft 赛道边缘点集
     * @param predict AI检测结果
     * @return POINT
     */
    vector<POINT> searchLeftCones(vector<POINT> pointsCone)
    {
        vector<POINT> points;
        if (pointsCone.size() <= 0)
            return points;

        for (int i = 0; i < pointsCone.size(); i++)
        {
            if (pointsCone[i].y <= heighestCone.y)
                points.push_back(pointsCone[i]);
        }

        return points;
    }

    /**
     * @brief 在俯视域由左边缘预测右边缘
     *
     * @param pointsEdgeLeft
     * @return vector<POINT>
     */
    vector<POINT> predictEdgeRight(vector<POINT> &pointsEdgeLeft)
    {
        int offset = 120; // 右边缘平移尺度
        vector<POINT> pointsEdgeRight;
        if (pointsEdgeLeft.size() < 3)
            return pointsEdgeRight;

        // Start
        Point2d startIpm = ipm.homography(Point2d(pointsEdgeLeft[0].y, pointsEdgeLeft[0].x)); // 透视变换
        Point2d prefictRight = Point2d(startIpm.x + offset, startIpm.y);
        Point2d startIipm = ipm.homographyInv(prefictRight); // 反透视变换
        POINT startPoint = POINT(startIipm.y, startIipm.x);

        // Middle
        Point2d middleIpm = ipm.homography(Point2d(pointsEdgeLeft[pointsEdgeLeft.size() / 2].y, pointsEdgeLeft[pointsEdgeLeft.size() / 2].x)); // 透视变换
        prefictRight = Point2d(middleIpm.x + offset, middleIpm.y);
        Point2d middleIipm = ipm.homographyInv(prefictRight); // 反透视变换
        POINT midPoint = POINT(middleIipm.y, middleIipm.x);   // 补线中点

        // End
        Point2d endIpm = ipm.homography(Point2d(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y, pointsEdgeLeft[pointsEdgeLeft.size() - 1].x)); // 透视变换
        prefictRight = Point2d(endIpm.x + offset, endIpm.y);
        Point2d endtIipm = ipm.homographyInv(prefictRight); // 反透视变换
        POINT endPoint = POINT(endtIipm.y, endtIipm.x);

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
    vector<POINT> predictEdgeLeft(vector<POINT> &pointsEdgeRight)
    {
        int offset = 120; // 右边缘平移尺度
        vector<POINT> pointsEdgeLeft;
        if (pointsEdgeRight.size() < 3)
            return pointsEdgeLeft;

        // Start
        Point2d startIpm = ipm.homography(Point2d(pointsEdgeRight[0].y, pointsEdgeRight[0].x)); // 透视变换
        Point2d prefictLeft = Point2d(startIpm.x - offset, startIpm.y);
        Point2d startIipm = ipm.homographyInv(prefictLeft); // 反透视变换
        POINT startPoint = POINT(startIipm.y, startIipm.x);

        // Middle
        Point2d middleIpm = ipm.homography(Point2d(pointsEdgeRight[pointsEdgeRight.size() / 2].y, pointsEdgeRight[pointsEdgeRight.size() / 2].x)); // 透视变换
        prefictLeft = Point2d(middleIpm.x - offset, middleIpm.y);
        Point2d middleIipm = ipm.homographyInv(prefictLeft); // 反透视变换
        POINT midPoint = POINT(middleIipm.y, middleIipm.x);  // 补线中点

        // End
        Point2d endIpm = ipm.homography(Point2d(pointsEdgeRight[pointsEdgeRight.size() - 1].y, pointsEdgeRight[pointsEdgeRight.size() - 1].x)); // 透视变换
        prefictLeft = Point2d(endIpm.x - offset, endIpm.y);
        Point2d endtIipm = ipm.homographyInv(prefictLeft); // 反透视变换
        POINT endPoint = POINT(endtIipm.y, endtIipm.x);

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

    /**
     * @brief 按照坐标点的y排序
     *
     * @param points
     * @return vector<int>
     */
    void pointsSortForY(vector<POINT> &points)
    {
        int n = points.size();
        bool flag = true;

        for (int i = 0; i < n - 1 && flag; i++)
        {
            flag = false;
            for (int j = 0; j < n - i - 1; j++)
            {
                if (points[j].y > points[j + 1].y)
                {
                    POINT temp = points[j];
                    points[j] = points[j + 1];
                    points[j + 1] = temp;
                    flag = true; // 每次循环i有修改，这里为true   如果跑了一次I没有发生交换的情况，说明已经排序完成，不需要再跑后面的i
                }
            }
        }
    }
};