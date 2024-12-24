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

#
