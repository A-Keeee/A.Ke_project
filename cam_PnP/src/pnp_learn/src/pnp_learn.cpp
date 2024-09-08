#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

class RobotPoseNode : public rclcpp::Node {
public:
    RobotPoseNode() : Node("robot_pose_node") {
        // 在构造函数中放置你的计算逻辑
        calculatePose();
    }

private:
    void calculatePose() {
        // 定义现实世界中的3D点（以厘米为单位）
        std::vector<cv::Point3f> objectPoints = {
            cv::Point3f(-4, 4, 0),  // 左上角
            cv::Point3f(4, 4, 0),   // 右上角
            cv::Point3f(-4, -4, 0),    // 左下角
            cv::Point3f(4, -4, 0)    // 右下角
        };

        // 定义图像中的2D点（像素坐标）


        std::vector<cv::Point2f> imagePoints = {
            cv::Point2f(435,645),  // 左上角
            cv::Point2f(1280, 645),  // 右上角
            cv::Point2f(435, 1050),  // 左下角
            cv::Point2f(1280, 1050)   // 右下角
        };
        // std::vector<cv::Point2f> imagePoints = {
        //     cv::Point2f(643, 433),  // 左上角
        //     cv::Point2f(1045, 433),  // 右上角
        //     cv::Point2f(643, 840),  // 左下角
        //     cv::Point2f(1045, 840)   // 右下角
        // };


        // 摄像机内参矩阵
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 2751.0, 0, 1512,
                                                 0, 2755.3, 2016,
                                                 0, 0, 1);




        // 畸变系数矩阵
cv::Mat distCoeffs = (cv::Mat_<double>(4, 1) << 0.2677, -0.3573, 0, 0);


        // 输出的旋转向量和平移向量
        cv::Mat rvec, tvec;

        // 使用solvePnP进行姿态估算
        bool success = cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

        if (success) {
            std::cout << "旋转向量: " << rvec << std::endl;
            std::cout << "平移向量: " << tvec << std::endl;

            // 将旋转向量转换为旋转矩阵
            cv::Mat rotationMatrix;
            cv::Rodrigues(rvec, rotationMatrix);
            std::cout << "旋转矩阵: " << rotationMatrix << std::endl;

            // 从旋转矩阵中提取欧拉角
            double sy = std::sqrt(rotationMatrix.at<double>(0, 0) * rotationMatrix.at<double>(0, 0) + 
                                  rotationMatrix.at<double>(1, 0) * rotationMatrix.at<double>(1, 0));
            
            bool singular = sy < 1e-6; // 如果接近零，认为是奇异解

            double x, y, z;
            if (!singular) {
                x = std::atan2(rotationMatrix.at<double>(2, 1), rotationMatrix.at<double>(2, 2));
                y = std::atan2(-rotationMatrix.at<double>(2, 0), sy);
                z = std::atan2(rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(0, 0));
            } else {
                x = std::atan2(-rotationMatrix.at<double>(1, 2), rotationMatrix.at<double>(1, 1));
                y = std::atan2(-rotationMatrix.at<double>(2, 0), sy);
                z = 0;
            }

            // 将欧拉角转换为度数
            x = x * 180.0 / CV_PI;
            y = y * 180.0 / CV_PI;
            z = z * 180.0 / CV_PI;

            std::cout << "物体相对于相机的旋转角度 (X, Y, Z): " << x << "° " << y << "° " << z << "°" << std::endl;

            // 计算机器人距离摄像头的距离（单位：厘米）
            double distance = cv::norm(tvec);
            std::cout << "机器人距离摄像头的距离: " << distance << " cm" << std::endl;
        } else {
            std::cout << "solvePnP 失败" << std::endl;
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotPoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
