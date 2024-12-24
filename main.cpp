#include <windows.h>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
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
    cap.set(CAP_PROP_FRAME_WIDTH, 1200);  // 宽度
    cap.set(CAP_PROP_FRAME_HEIGHT, 750);  // 高度
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
        if (elapsed.count() >= 2) {
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
