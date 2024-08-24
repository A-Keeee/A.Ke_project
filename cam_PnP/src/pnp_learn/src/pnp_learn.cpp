#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

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
            cv::Point3f(0, 0, 0),
            cv::Point3f(10, 0, 0),
            cv::Point3f(10, 10, 0),
            cv::Point3f(0, 10, 0)
        };

        // 定义图像中的2D点（像素坐标）
        std::vector<cv::Point2f> imagePoints = {
            cv::Point2f(100, 150),
            cv::Point2f(300, 150),
            cv::Point2f(300, 300),
            cv::Point2f(100, 300)
        };

        // 摄像机内参矩阵
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1462.3697, 0, 398.59394,
                                                        0, 1469.68385, 110.68997,
                                                        0, 0, 1);

        // 畸变系数矩阵
        cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 0.003518, -0.311778, -0.016581, 0.023682, 0.0);

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

            // 计算机器人距离摄像头的距离（单位：米）
            double distance = cv::norm(tvec) / 100.0;  // 转换为米
            std::cout << "机器人距离摄像头的距离: " << distance << " 米" << std::endl;
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
