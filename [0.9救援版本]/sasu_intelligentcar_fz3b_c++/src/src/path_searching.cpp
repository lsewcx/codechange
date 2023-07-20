/**
 * @file path_searching.cpp
 * @author lse
 * @brief 路径搜索：分割智能车行驶区域
 * @version 0.1
 * @date 2023-07-20
 * @note 路径分割容易看不见元素，建议双摄再用
 * @copyright Copyright (c) 2023
 *
 */
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

/**
**[1] 输入二值化
**[2] 图像二值化
*/

class PathSearching {
public:
  /**
   * @brief 赛道图像搜索
   *
   * @param imageBinary   输入二值化图像
   * @return Mat  输出完整赛道图像（可行使区域）
   */
  Mat pathSearch(Mat &imageBinary) {
    Point pointFloodFill = cv::Point(60, 220); // 左:()  右：(60,220)
    Mat imagePath = Mat::zeros(imageBinary.size(), CV_8UC3);

    if (imageBinary.at<uchar>(220, 160) > 128) {
      pointFloodFill.x = 220;
      pointFloodFill.y = 160;
    } else if (imageBinary.at<uchar>(220, 60) > 128) {
      pointFloodFill.x = 220;
      pointFloodFill.y = 60;
    } else if (imageBinary.at<uchar>(220, 260) > 128) {
      pointFloodFill.x = 220;
      pointFloodFill.y = 260;
    }

    vector<vector<Point>> points; // 赛道轮廓搜索
    findContours(imageBinary, points, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 绘制轮廓

    int indexAreaMax = 0;
    double areaMax = 0;

    for (uint16_t i = 0; i < points.size(); i++) // 遍历
    {
      // 计算面积和周长
      //  double length = arcLength(points[i], true);
      double area = contourArea(points[i]);
      if (area > areaMax) {
        areaMax = area;
        indexAreaMax = i;
      }
    }
    drawContours(imagePath, points, indexAreaMax, Scalar(238, 238, 175), 1);
    // imshow("imageContours", imagePath);

  findMark:
    for (int i = 0; i < points[indexAreaMax].size(); i++) {
      if (points[indexAreaMax][i].x == pointFloodFill.y &&
          points[indexAreaMax][i].y == pointFloodFill.x) {
        cout << "-------------------------------Counter!" << endl;
        if (pointFloodFill.y < 160) {
          pointFloodFill.y += 5;
          pointFloodFill.x += 5;
          goto findMark;
        } else {
          pointFloodFill.y -= 5;
          pointFloodFill.x += 5;
          goto findMark;
        }

        break;
      }
    }
    // vector<vector<Point>> pointsPoly(points.size()); //多边形点集
    // approxPolyDP(points[i], pointsPoly[i], 4, true); //多边形逼近

    floodFill(imagePath, cv::Point(pointFloodFill.y, pointFloodFill.x),
              Scalar(0, 0, 255));

    circle(imagePath, Point(pointFloodFill.y, pointFloodFill.x), 2,
           Scalar(100, 100, 100), -1); // 显示采样点

    return imagePath;
  }
};
