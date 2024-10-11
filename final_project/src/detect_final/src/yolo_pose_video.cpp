#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "inference.h"  // 假设 inference.h 处理 ONNX 推理
using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    bool runOnGPU = false;

    // 1. 设置你的onnx模型，使用 YOLOv8-pose 的 ONNX 模型
    Inference inf("src/rm_buff_pose.onnx", cv::Size(640, 480), "classes.txt", runOnGPU); // "classes.txt" 可以缺失

    // 2. 打开视频文件或摄像头
    string videoPath = "src/red.avi";  // 替换为你的视频路径
    VideoCapture cap(videoPath);

    if (!cap.isOpened()) {
        cout << "无法打开视频文件: " << videoPath << endl;
        return -1;
    }

    Mat frame;
    while (true)
    {
        cap >> frame;
        if (frame.empty()) {
            break; // 视频结束时退出
        }

        // 3. 对当前帧进行目标检测
        std::vector<Detection> output = inf.runInference(frame);

        int detections = output.size();
        std::cout << "Number of detections: " << detections << std::endl;

        // 4. 绘制检测框和特征点
        for (int i = 0; i < detections; ++i)
        {
            Detection detection = output[i];
            cv::Rect box = detection.box;
            cv::Scalar color = detection.color;

            // 绘制检测到的物体框
            cv::rectangle(frame, box, color, 2);

            // 绘制类别标签和置信度
            std::string classString = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
            cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
            cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);
            cv::rectangle(frame, textBox, color, cv::FILLED);
            cv::putText(frame, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);

            // 绘制关键点
            for (const cv::Point &kp : detection.keypoints) {
                cv::circle(frame, kp, 5, cv::Scalar(0, 255, 0), -1);  // 在特征点位置绘制小圆
            }
        }

        // 5. 显示带有标注的帧
        cv::imshow("YOLOv8 Pose Inference", frame);

        // 按下 'q' 键退出
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // 6. 释放资源
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
