# LaserPointerMouse

鼠标定位系统，摄像头实时拍摄激光红点在显示器上的位置，并将视频传输给电脑里的鼠标定位系统，鼠标定位系统通过识别视频中的激光红点确定鼠标的位置，从而达到激光笔或激光枪的鼠标输入效果，可适用于射击游戏、空中鼠标等场合

## 环境配置

### opencv+vscode+cmake

vscode配置gitbash终端

[vscode 配置 gitbash 终端，并设置为默认_vscode 默认git bash-CSDN博客](https://blog.csdn.net/qq_44498977/article/details/124204347)

vscode配置zsh终端

[【vscode】windows中使用zsh美化vscode终端 - 简书](https://www.jianshu.com/p/6d21d3484444)

opencv源代码编译

[Win10安装opencv+clion配置 史上最详细的保姆级教程_open cv clion-CSDN博客](https://blog.csdn.net/Dylan_YQ/article/details/122677627)

vscode配置opencv

[2023年最全 Windows + VSCode 配置 OpenCV C++ 一站式开发调试环境教程-阿里云开发者社区](https://developer.aliyun.com/article/1210289)

配置cmake

[CMake 安装与配置 | 菜鸟教程](https://www.runoob.com/cmake/cmake-install-setup.html)

测试opencv

```cpp
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
inline void log(string message) {
    cout << message << endl;
}
int main(void) {
    log("start...");
    cv::Mat img = cv::imread("C:/Users/Yezi/Desktop/picture/background/BKR.png");
    cv::imshow("maolinye", img);
    cv::waitKey();
    log("over...");
    return 0;
}
```

### 构建运行

```shell
mkdir build
cd build
cmake ..
make
./LaserPointerMouse.exe
```

指定生成器

```shell
mkdir build
cd build
cmake .. -G "MinGW Makefiles"
make
./LaserPointerMouse.exe
```

## 移动鼠标

屏幕坐标原点在左上角

```cpp
#include <windows.h>
#include <iostream>
#include <sstream>
void moveMouse(int x, int y) {
    SetCursorPos(x, y);
    std::cout << "mouse's position" << std::endl;
    std::cout << "x: " << x << " y: " << y << std::endl;
}
int main() {
    int screenWidth = GetSystemMetrics(SM_CXSCREEN);   // 获取屏幕宽度
    int screenHeight = GetSystemMetrics(SM_CYSCREEN);  // 获取屏幕高度
    std::cout << "Screen Width: " << screenWidth << std::endl;
    std::cout << "Screen Height: " << screenHeight << std::endl;
    std::string input;
    while (true) {
        std::getline(std::cin, input);  // 获取以回车为结束的字符串
        if (input == "q" || input == "Q") {
            break;
        }
        std::istringstream iss(input);
        int x, y;
        if (iss >> x >> y) {
            moveMouse(x, y);
        }
    }
    return 0;
}

```

## 获取视频帧

通过计算一秒内显示的帧数来计算帧率

```cpp
#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
using namespace cv;
using namespace std;
int main() {
    // 打开摄像头（0 为默认摄像头）
    VideoCapture cap(0);
    // 检查摄像头是否成功打开
    if (!cap.isOpened()) {
        cout << "Error: Could not open camera!" << endl;
        return -1;
    }
    // 获取摄像头的原始帧率
    double fps = cap.get(CAP_PROP_FPS);
    cout << "Camera FPS: " << fps << endl;
    // 设置视频分辨率（可选）
    cap.set(CAP_PROP_FRAME_WIDTH, 400);   // 宽度
    cap.set(CAP_PROP_FRAME_HEIGHT, 250);  // 高度
    Mat frame;
    auto start_time = chrono::high_resolution_clock::now();
    int frame_count = 0;
    while (true) {
        // 捕获每一帧图像
        cap >> frame;
        // 检查是否成功获取了帧
        if (frame.empty()) {
            cout << "Error: Could not capture frame!" << endl;
            break;
        }
        // 显示视频帧
        imshow("Video", frame);
        ++frame_count;
        // 每1秒计算一次显示帧率
        auto current_time = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed = current_time - start_time;
        if (elapsed.count() >= 1.0) {
            double display_fps = frame_count / elapsed.count();
            cout << "Display FPS: " << display_fps << endl;
            frame_count = 0;  // 重置帧计数器
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

```

## 原地不动

无法检测到屏幕

无法检测到激光点

检测到两个及两个以上的激光点

激光点的位置不在屏幕内

## 识别屏幕

### 神经网络

yolov8

[Pertical/YOLOv8: YOLOv8 🚀 in PyTorch &gt; ONNX &gt; CoreML &gt; TFLite](https://github.com/Pertical/YOLOv8)

[YOLOv8 -Ultralytics YOLO 文档](https://docs.ultralytics.com/zh/models/yolov8/)

### 计算机视觉

灰度化

高斯模糊

均衡化

二值化

Canny边缘检测

霍夫变换

近似多边形

```cpp
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
    showFrameWithPause(gray);
    // 高斯模糊
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
    showFrameWithPause(blurred);
    // 图像均衡化
    cv::Mat equalized;
    cv::equalizeHist(blurred, equalized);
    showFrameWithPause(equalized);
    // 二值化
    cv::Mat binary;
    cv::threshold(equalized, binary, 128, 255, cv::THRESH_BINARY);
    showFrameWithPause(binary);
    // 使用Canny边缘检测 弱边缘 强边缘
    cv::Mat edges;
    cv::Canny(binary, edges, 175, 200);
    showFrameWithPause(edges);
    // 使用霍夫变换检测直线
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(edges, lines, 1, CV_PI / 180, 128);
    // 绘制检测到的直线
    for (size_t i = 0; i < lines.size(); i++) {
        float rho = lines[i][0];
        float theta = lines[i][1];
        cv::Point pt1, pt2;
        double cos_theta = cos(theta);
        double sin_theta = sin(theta);
        pt1.x = cvRound(rho * cos_theta + 64 * (-sin_theta));
        pt1.y = cvRound(rho * sin_theta + 64 * (cos_theta));
        pt2.x = cvRound(rho * cos_theta - 128 * (-sin_theta));
        pt2.y = cvRound(rho * sin_theta - 128 * (cos_theta));
        cv::line(edges, pt1, pt2, cv::Scalar(255, 255, 255), 2);
    }
    showFrameWithPause(edges);
    // 找到轮廓
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
                showFrameWithPause(frame);
            }
        }
    }
}
```

实测调优

```cpp
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
    // 使用Canny边缘检测 弱边缘 强边缘
    cv::Mat edges;
    cv::Canny(equalized, edges, 128, 256);
    // imshow("Frame", edges);
    // 找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);
    // 筛选矩形轮廓（屏幕边界）
    std::cout << "Area: " << std::endl;
    for (size_t i = 0; i < contours.size(); i++) {
        // 近似多边形（四个角的矩形）
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx,
                         cv::arcLength(contours[i], true) * 0.02, true);
        // 检查是否为矩形
        if (approx.size() == 4 && cv::isContourConvex(approx)) {
            // 获取矩形的四个角
            double area = cv::contourArea(approx);
            std::cout << area << ' ';
            if (area > 2<<14) {  // 排除面积太小的轮廓
                // 找到屏幕 绘制矩形边界
                cv::polylines(frame, approx, true, cv::Scalar(0, 255, 0), 3);
            }
        }
    }
    std::cout << std::endl;
}
```

灰度化、均衡化、Canny边缘检测、近似多边形

![1736938535499](image/README/1736938535499.gif)

## 识别激光点

## 适用环境

光线均匀

## 影响因素

光线、摄像头位置
