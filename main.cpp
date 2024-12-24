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
