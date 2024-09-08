#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
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
    Mat cameraMatrix = (Mat_<double>(3, 3) << 1462.3697, 0, 398.59394, 0, 1469.68385, 110.68997, 0, 0, 1);
    
    // 相机畸变系数
    Mat distCoeffs = (Mat_<double>(1, 5) << 0.003518, -0.311778, -0.016581, 0.023682, 0.0);
    
    // 定义灯条在世界坐标系中的实际三维坐标 (单位：米)
    vector<Point3f> objectPoints;
    objectPoints.push_back(Point3f(-8.0f, -16.0f, 0.0f)); // 灯条左上角
    objectPoints.push_back(Point3f(8.0f, -16.0f, 0.0f));  // 灯条右上角
    objectPoints.push_back(Point3f(8.0f, 16.0f, 0.0f));   // 灯条右下角
    objectPoints.push_back(Point3f(-8.0f, 16.0f, 0.0f));  // 灯条左下角

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

             // 使用 TF2 将相机坐标系下的坐标转换到 odom 坐标系
    tf2::Quaternion q;
    q.setRPY(2 / 57.3, 60 / 57.3, 20 / 57.3);  // 使用已知的相机相对 odom 的旋转

    tf2::Transform cameraToOdom;
    cameraToOdom.setOrigin(tf2::Vector3(8, 0, 5));  // 设置相机的平移

    cameraToOdom.setRotation(q);  // 设置旋转矩阵

    // 将相机坐标系下的位姿转换到 odom 坐标系下
    tf2::Vector3 positionInCamera(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    tf2::Vector3 positionInOdom = cameraToOdom * positionInCamera;

    // 输出装甲板在 odom 坐标系下的坐标
    cout << "在 odom 坐标系下的坐标: " 
         << positionInOdom.getX() << ", " 
         << positionInOdom.getY() << ", " 
         << positionInOdom.getZ() << endl;

            
        }
        cv::imshow("Inference", frame);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
}
