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

考虑颜色过滤，HSV可以更好地分离颜色信息，红色通常位于Hue通道的低值和接近360度的位置，因为HSV的Hue是环形排列的。不过OpenCV中的Hue范围是0-180，所以可能需要设置两个范围，比如0-10和170-180，来覆盖红色

激光点可能比较亮，而普通的红色物体可能饱和度或亮度不够，需要同时考虑饱和度和明度（Value）通道

寻找轮廓，用轮廓的面积和最小外接圆或者椭圆来判断

---

HSV分离色调、饱和度和亮度

寻找圆形轮廓

![1738509227231](image/README/1738509227231.gif)

## 适用环境

光线均匀

## 影响因素

光线、摄像头位置

白色背景会吞掉红色激光点

激光笔垂直照射是白色中心圆红色圈，斜射是暗红色圆

## 技术原理

目标：移动鼠标至激光点位置

基本思路：识别屏幕边界+识别激光点位置+坐标映射

### 识别屏幕边界

目标：识别屏幕四个角在帧中的位置

---

图像特征提取常用方法：灰度化 高斯模糊 图像均衡化 二值化 Canny边缘检测 霍夫变换 近似多边形

---

基本思路：找到帧中最大的四边形

转换为灰度图像 → 图像均衡化增强对比度 → 多次小规模高斯模糊连接边界 → canny 边缘检测 → findContours找轮廓 → approxPolyDP 近似多边形

##### 灰度化

灰度化的基本原理是通过一定的加权平均公式，将RGB值转化为一个单一的灰度值，减少彩色图像的计算量，保留轮廓特征

##### 图像均衡化

图像均衡化调整图像的灰度分布，使得每个灰度级别的像素数目尽可能均匀，从而增强图像的对比度

##### 高斯模糊

减少噪声

连接边界

##### canny边缘检测

计算每个像素的梯度，大于高阈值的为强边缘，大于低阈值的为弱边缘，保留强边缘和邻接强边缘的弱边缘

##### findContours找轮廓

连接相邻像素作为轮廓

##### approxPolyDP近似多边形

保留图像关键点

选取两个点出一条直线，保留轮廓上到直线距离长的点，以该点为界分成两部分继续递归处理

### 识别激光点位置

目标：识别激光点在帧中的位置

---

基本思路：找到帧中红色高亮的实心圆

转换为HSV颜色空间 → 分离亮度通道对其进行均衡化 → 定义红色范围区间过滤 → findContours找轮廓 → 计算轮廓外接圆面积比 → 计算质心

##### HSV颜色空间

HSV颜色空间可以将色调、饱和度、亮度分离出来

##### 计算轮廓外接圆面积比

激光点照射到屏幕上呈现出不严格的椭圆形，通过计算其最小外接圆面积和本身的面积之比衡量其圆度

##### 计算质心

一阶矩 表示了图像中像素位置的加权平均值（以像素的灰度为权重），而 零阶矩表示图像中所有像素的总权重（即总灰度值），将一阶矩除以零阶矩，就相当于计算出图像中灰度值的“加权平均位置”，也就是图像的质心

### 坐标映射

目标：将激光点在帧中的位置映射到激光点相对于屏幕的位置

---

将屏幕四个角的坐标进行重新排序，分出上下左右四个角，通过先排x再排y，差值小于阈值的视为相同

通过屏幕四个角的坐标和屏幕宽高构建透视变换矩阵，利用透视变换得到激光点相对于屏幕的坐标

## LaserPointerMouse1.0

白色的圆点是鼠标，初步实现功能

![1739865438812](image/README/1739865438812.gif)
