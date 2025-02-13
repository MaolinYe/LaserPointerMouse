#include <windows.h>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

class LaserPointMouse {
    int screenWidth;                      // 屏幕宽度
    int screenHeight;                     // 屏幕高度
    std::vector<cv::Point> originPoints;  // 屏幕坐标
    std::vector<cv::Point> screenPoints;  // 屏幕在帧中的坐标
    cv::Point laserPoint;                 // 激光点在帧中的坐标
    cv::Point laserScreenPoint;           // 激光点在屏幕中的坐标

    void moveMouse(const cv::Point& point) const {
        SetCursorPos(point.x, point.y);
        std::cout << "move mouse to" << std::endl;
        std::cout << "x: " << point.x << " y: " << point.y << std::endl;
    }

    void showFrameWithPause(cv::Mat& frame) const {
        // 检查是否成功获取了帧
        if (frame.empty()) {
            std::cerr << "Error: Could not capture frame!" << endl;
            return;
        }
        // 显示视频帧
        imshow("Frame", frame);
        // 按键继续
        waitKey();
    }

    void coordinateTransform() {
        // 计算透视变换矩阵
        cv::Mat H = cv::findHomography(this->originPoints, this->screenPoints);
        // 使用透视变换来获取激光点在屏幕上的坐标
        std::vector<cv::Point> laser_point_screen(1);
        cv::perspectiveTransform(std::vector<cv::Point>{this->laserPoint},
                                 laser_point_screen, H);
        this->laserScreenPoint = laser_point_screen[0];
    }

    bool inside(cv::Point& point, std::vector<cv::Point>& polygon) {
        return true;
        //return cv::pointPolygonTest(polygon, point, false);
    }

    bool findLaserPoint(cv::Mat& frame) {
        // 检查是否成功获取了帧
        if (frame.empty()) {
            std::cerr << "Error: Could not capture frame!" << endl;
            return false;
        }
        // 转换为HSV颜色空间
        Mat hsv;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        // 定义暗红色和高亮的HSV阈值
        Scalar lower_red1(0, 32, 32);  // 低色调范围
        Scalar upper_red1(20, 255, 255);
        Scalar lower_red2(160, 32, 32);  // 高色调范围
        Scalar upper_red2(180, 255, 255);
        // 创建颜色掩膜
        Mat mask1, mask2, mask;
        inRange(hsv, lower_red1, upper_red1, mask1);
        inRange(hsv, lower_red2, upper_red2, mask2);
        mask = mask1 | mask2;
        // 查找轮廓
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        this->laserPoint = {-1, -1};
        // 遍历所有轮廓
        std::cout << std::endl;
        double minArea = 8;
        for (const auto& contour : contours) {
            double area = contourArea(contour);
            std::cout << "Area: " << area << ' ';
            if (area < minArea || area > 1024)  // 忽略太小太大的区域
                continue;
            minArea = area;
            // 形状验证：计算最小外接圆
            Point2f center;
            float radius;
            minEnclosingCircle(contour, center, radius);
            // 计算轮廓占圆面积的比例（验证圆形度）
            double circleArea = CV_PI * radius * radius;
            double ratio = area / circleArea;
            std::cout << "Ratio: " << ratio << ' ';
            if (ratio > 0.5) {
                Moments m = moments(contour);
                this->laserPoint = {m.m10 / m.m00, m.m01 / m.m00};
            }
        }
        std::cout << std::endl;
        // 是否在屏幕内找到激光点
        if (this->laserPoint.x != -1 &&
            inside(this->laserPoint, this->screenPoints)) {
            circle(frame, this->laserPoint, 9, Scalar(0, 255, 0), 2);
            cout << "Laser Point detected at: " << this->laserPoint << endl;
            return true;
        }
        return false;
    }

    bool findScreenEdge(cv::Mat& frame) {
        return true;
        // 检查是否成功获取了帧
        if (frame.empty()) {
            std::cerr << "Error: Could not capture frame!" << endl;
            return false;
        }
        // 转换为灰度图像
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        // 图像均衡化
        cv::Mat equalized;
        cv::equalizeHist(gray, equalized);
        // 使用Canny边缘检测 弱边缘 强边缘
        cv::Mat edges;
        cv::Canny(equalized, edges, 128, 256);
        // imshow("Frame", edges);
        // 找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);
        // 筛选矩形轮廓（屏幕边界）
        this->screenPoints.clear();
        std::cout << "Area: " << std::endl;
        for (size_t i = 0; i < contours.size(); i++) {
            // 近似多边形（四个角的矩形）
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contours[i], approx,
                             cv::arcLength(contours[i], true) * 0.015, true);
            // 检查是否为矩形
            if (approx.size() == 4 && cv::isContourConvex(approx)) {
                // 获取矩形的四个角
                double area = cv::contourArea(approx);
                std::cout << area << ' ';
                double maxArea = 2 << 14;
                if (area > maxArea) {  // 排除面积太小的轮廓
                    maxArea = area;
                    std::cout << std::endl << "Rectangle coordinates:";
                    for (auto& point : approx) {
                        std::cout << point << ' ';
                    }
                    std::cout << "\nAera: ";
                    this->screenPoints = move(approx);
                }
            }
        }
        std::cout << std::endl;
        if (this->screenPoints.size() == 4) {
            // 找到屏幕 绘制矩形边界
            cv::polylines(frame, this->screenPoints, true,
                          cv::Scalar(0, 255, 0), 3);
            return true;
        }
        return false;
    }

   public:
    LaserPointMouse()
        : screenWidth(GetSystemMetrics(SM_CXSCREEN)),
          screenHeight(GetSystemMetrics(SM_CYSCREEN)) {
        std::cout << "Screen Width: " << screenWidth << std::endl;
        std::cout << "Screen Height: " << screenHeight << std::endl;
        // 原始坐标
        std::vector<cv::Point> originPoints{
            cv::Point(0, 0), cv::Point(0, screenWidth),
            cv::Point(screenHeight, 0), cv::Point(screenHeight, screenWidth)};
        this->originPoints = move(originPoints);
    }
    void work() {
        // 打开摄像头（0 为默认摄像头）
        VideoCapture cap(0);
        // 检查摄像头是否成功打开
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open camera!" << endl;
            return;
        }
        // 获取摄像头的原始帧率
        double fps = cap.get(CAP_PROP_FPS);
        cout << "Camera FPS: " << fps << endl;
        // 设置视频分辨率（可选）
        cap.set(CAP_PROP_FRAME_WIDTH, screenWidth / 4);    // 宽度
        cap.set(CAP_PROP_FRAME_HEIGHT, screenHeight / 4);  // 高度
        Mat frame;
        auto start_time = chrono::high_resolution_clock::now();
        int frame_count = 0;
        while (true) {
            // 捕获每一帧图像
            cap >> frame;
            // 检查是否成功获取了帧
            if (frame.empty()) {
                std::cerr << "Error: Could not capture frame!" << endl;
                break;
            }
            // 帧处理
            if (findScreenEdge(frame) && findLaserPoint(frame)) {
                // coordinateTransform();
                // moveMouse(this->laserScreenPoint);
            }
            // 显示视频帧
            imshow("Video Frame", frame);
            // 按下 'q' 键退出
            if (waitKey(1) == 'q') {
                break;
            }
            ++frame_count;
            // 每 秒计算一次显示帧率
            auto current_time = chrono::high_resolution_clock::now();
            chrono::duration<double> elapsed = current_time - start_time;
            if (elapsed.count() >= 4) {
                double display_fps = frame_count / elapsed.count();
                cout << "Display FPS: " << display_fps << endl;
                frame_count = 0;            // 重置帧计数器
                start_time = current_time;  // 重置时间
            }
        }
        // 释放摄像头
        cap.release();
        // 关闭所有 OpenCV 窗口
        destroyAllWindows();
    }
};
