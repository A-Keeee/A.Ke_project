#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "inference.h"
using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    bool runOnGPU = false;

    // 1. 设置你的onnx模型
    // Note that in this example the classes are hard-coded and 'classes.txt' is a place holder.
    Inference inf("src/armor_rm.onnx", cv::Size(640, 480), "classes.txt", runOnGPU); // classes.txt 可以缺失

    // 2. 设置你的输入图片
    std::vector<std::string> imageNames;
    imageNames.push_back("src/image1.png");
    //imageNames.push_back("zidane.jpg");

    // 相机内参矩阵
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
        1462.3697, 0, 398.59394,
        0, 1469.68385, 110.68997,
        0, 0, 1);

    // 相机畸变系数
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0.003518, -0.311778, -0.016581, 0.023682, 0.0000);

    // 定义装甲板的3D坐标（单位：厘米）
    std::vector<cv::Point3f> objectPoints = {
        {-4.0f, -4.0f, 0},  // 左上角
        {4.0f, -4.0f, 0},   // 右上角
        {4.0f, 4.0f, 0},    // 右下角
        {-4.0f, 4.0f, 0}    // 左下角
    };

    for (int i = 0; i < imageNames.size(); ++i)
    {
        cv::Mat frame = cv::imread(imageNames[i]);

        // Inference starts here...
        std::vector<Detection> output = inf.runInference(frame);

        int detections = output.size();
        std::cout << "Number of detections:" << detections << std::endl;

        // feiyull
        // 这里需要resize下，否则结果不对
        //cv::resize(frame, frame, cv::Size(480, 640));

        for (int i = 0; i < detections; ++i)
        {
            Detection detection = output[i];

            cv::Rect box = detection.box;
            cv::Scalar color = detection.color;

            // Detection box
            cv::rectangle(frame, box, color, 2);

            // Detection box text
            std::string classString = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
            cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
            cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

            cv::rectangle(frame, textBox, color, cv::FILLED);
            cv::putText(frame, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);

             // 提取装甲板在图像中的四个角点
            std::vector<cv::Point2f> imagePoints = {
                cv::Point2f(box.x, box.y),                               // 左上角
                cv::Point2f(box.x + box.width, box.y),                   // 右上角
                cv::Point2f(box.x + box.width, box.y + box.height),      // 右下角
                cv::Point2f(box.x, box.y + box.height)                   // 左下角
            };

            // 计算PnP
            cv::Mat rvec, tvec;
            cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

            // 输出距离信息
            double distanceToArmor = cv::norm(tvec);
            std::cout << "装甲板距离摄像头的距离: " << distanceToArmor << " cm" << std::endl;
            
        }
        cv::imshow("Inference", frame);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
}
