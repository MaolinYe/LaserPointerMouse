#include <windows.h>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
void moveMouse(int x, int y) {
    SetCursorPos(x, y);
    std::cout << "mouse GPS" << std::endl;
    std::cout << "x: " << x << " y: " << y << std::endl;
}
void showFrameWithPause(cv::Mat& frame) {
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
void processFrame(cv::Mat& frame) {
    // 检查是否成功获取了帧
    if (frame.empty()) {
        std::cerr << "Error: Could not capture frame!" << endl;
        return;
    }
    // 转换为灰度图像
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    // 图像均衡化
    cv::Mat equalized;
    cv::equalizeHist(gray, equalized);
    // 二值化
    cv::Mat binary;
    cv::threshold(equalized, binary, 128, 255, cv::THRESH_BINARY);
    // 使用Canny边缘检测 弱边缘 强边缘
    cv::Mat edges;
    cv::Canny(binary, edges, 175, 200);
    // 找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);
    // 筛选矩形轮廓（屏幕边界）
    for (size_t i = 0; i < contours.size(); i++) {
        // 近似多边形（四个角的矩形）
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx,
                         cv::arcLength(contours[i], true) * 0.02, true);
        // 检查是否为矩形
        if (approx.size() == 4 && cv::isContourConvex(approx)) {
            // 获取矩形的四个角
            double area = cv::contourArea(approx);
            std::cout << "area: " << area << std::endl;
            if (area > 4096) {  // 排除面积太小的轮廓
                // 绘制矩形边界
                cv::polylines(frame, approx, true, cv::Scalar(0, 255, 0), 3);
            }
        }
    }
}
int main() {
    int screenWidth = GetSystemMetrics(SM_CXSCREEN);   // 获取屏幕宽度
    int screenHeight = GetSystemMetrics(SM_CYSCREEN);  // 获取屏幕高度
    std::cout << "Screen Width: " << screenWidth << std::endl;
    std::cout << "Screen Height: " << screenHeight << std::endl;
    // 打开摄像头（0 为默认摄像头）
    VideoCapture cap(0);
    // 检查摄像头是否成功打开
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera!" << endl;
        return -1;
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
        processFrame(frame);
        // 显示视频帧
        imshow("Video Frame", frame);
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
        // 按下 'q' 键退出
        if (waitKey(1) == 'q') {
            break;
        }
    }
    // 释放摄像头
    cap.release();
    // 关闭所有 OpenCV 窗口
    destroyAllWindows();
    return 0;
}
