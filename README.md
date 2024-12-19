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
