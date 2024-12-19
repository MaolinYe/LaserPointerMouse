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