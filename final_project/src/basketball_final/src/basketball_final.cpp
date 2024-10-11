#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "inference.h"  // 模型推理的头文件
using namespace std;
using namespace cv;

// // 最小二乘法拟合抛物线
// void fitParabola(const vector<Point>& points, double& a, double& b, double& c) {
//     int n = points.size();
//     if (n < 3) return;  // 至少需要3个点

//     Mat A(n, 3, CV_64F);
//     Mat B(n, 1, CV_64F);
    
//     for (int i = 0; i < n; i++) {
//         double x = points[i].x;
//         double y = points[i].y;
//         A.at<double>(i, 0) = x * x;
//         A.at<double>(i, 1) = x;
//         A.at<double>(i, 2) = 1;
//         B.at<double>(i, 0) = y;
//     }
    
//     Mat coeffs;
//     solve(A, B, coeffs, DECOMP_SVD);
    
//     a = coeffs.at<double>(0, 0);
//     b = coeffs.at<double>(1, 0);
//     c = coeffs.at<double>(2, 0);
// }

int main(int argc, char **argv)
{
    bool runOnGPU = false;

    // 1. 设置你的ONNX模型（替换为你的篮球检测模型）
    Inference inf("src/rm_buff_pose.onnx", cv::Size(640, 480), "classes.txt", runOnGPU);

    // 2. 设置输入视频
    VideoCapture cap("src/red.avi");
    if (!cap.isOpened()) {
        cout << "无法打开视频文件" << endl;
        return -1;
    }

    // 3. 保存篮球中心点的数组，用于拟合轨迹
    std::vector<Point> basketballCenters;

    // 4. 定义抛物线参数
    double a = 0, b = 0, c = 0;

    while (true)
    {
        cv::Mat frame;
        cap >> frame;  // 从视频读取一帧
        if (frame.empty()) {
            cout << "视频结束或读取失败" << endl;
            break;  // 视频结束
        }

        // 推理开始
        std::vector<Detection> output = inf.runInference(frame);

        int detections = output.size();
        for (int i = 0; i < detections; ++i)
        {
            Detection detection = output[i];

            // 获取检测到的篮球的位置
            cv::Rect box = detection.box;
            Point center(box.x + box.width / 2, box.y + box.height / 2);  // 计算篮球的中心点
            basketballCenters.push_back(center);  // 将篮球中心点保存下来

            // 绘制检测框和中心点
            cv::rectangle(frame, box, Scalar(0, 255, 0), 2);
            cv::circle(frame, center, 5, Scalar(255, 0, 0), -1);  // 用蓝色圆点标记中心
        }

        // // 5. 每当有足够多的点时，拟合抛物线
        // if (basketballCenters.size() >= 3) {
        //     fitParabola(basketballCenters, a, b, c);

        //     // 在图像上绘制拟合的抛物线
        //     for (int x = 0; x < frame.cols; x++) {
        //         int y = a * x * x + b * x + c;
        //         if (y >= 0 && y < frame.rows) {
        //             circle(frame, Point(x, y), 2, Scalar(0, 0, 255), -1);  // 用红色点绘制预测轨迹
        //         }
        //     }
        // }

        // 6. 显示结果
        cv::imshow("Basketball Detection and Trajectory", frame);

        char key = (char)waitKey(30);  // 每帧间隔30ms
        if (key == 27)  // 按下Esc键退出
            break;
    }

    return 0;
}
