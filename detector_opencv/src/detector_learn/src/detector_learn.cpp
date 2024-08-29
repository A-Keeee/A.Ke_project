#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

class ArmorDetector : public rclcpp::Node
{
public:
    ArmorDetector() : Node("armor_detector")
    {
        std::vector<std::string> image_paths = {
            "src/image1.png",
            "src/image5.png",
        };

        for (const auto &path : image_paths) {
            cv::Mat image = cv::imread(path);
            if (image.empty()) {
                RCLCPP_WARN(this->get_logger(), "无法读取图像: %s", path.c_str());
                continue;
            }

            // 执行装甲板检测
            detect_armor(image);

            // 显示结果
            cv::imshow("Detected Armor", image);
            cv::waitKey(0);  // 等待用户按键以显示下一张图像
        }
    }

private:
    void detect_armor(cv::Mat &image)
    {
        // 相机内参矩阵
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
            1462.3697, 0, 398.59394,
            0, 1469.68385, 110.68997,
            0, 0, 1);

        // 相机畸变系数
        cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0.003518, -0.311778, -0.016581, 0.023682, 0.0000);

        // 图像去畸变
        cv::Mat undistortedImage;
        cv::undistort(image, undistortedImage, cameraMatrix, distCoeffs);

        // 转换为HSV颜色空间
        cv::Mat hsvImage;
        cv::cvtColor(undistortedImage, hsvImage, cv::COLOR_BGR2HSV);

        // 红蓝灯条颜色阈值分割 (使用HSV颜色空间)
        cv::Mat redMask, blueMask, redMask1, redMask2;
        cv::inRange(hsvImage, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), redMask1);
        cv::inRange(hsvImage, cv::Scalar(170, 50, 50), cv::Scalar(180, 255, 255), redMask2);
        redMask = redMask1 | redMask2;

        cv::inRange(hsvImage, cv::Scalar(70, 100, 150), cv::Scalar(130, 154, 255), blueMask);

        std::vector<std::vector<cv::Point>> redContours, blueContours;
        cv::findContours(redMask, redContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(blueMask, blueContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 存储所有符合条件的灯柱
        std::vector<cv::Rect> lampPosts;

        auto filter_lamps = [&](const std::vector<std::vector<cv::Point>>& contours) {
            for (const auto& contour : contours) {
                cv::Rect rect = cv::boundingRect(contour);
                if (rect.height > rect.width && rect.height > 20 && rect.width > 10) {
                    lampPosts.push_back(rect);
                }
            }
        };

        filter_lamps(redContours);
        filter_lamps(blueContours);

        std::vector<cv::Rect> armorRects;

        for (size_t i = 0; i < lampPosts.size(); ++i) {
            for (size_t j = i + 1; j < lampPosts.size(); ++j) {
                const cv::Rect& rect1 = lampPosts[i];
                const cv::Rect& rect2 = lampPosts[j];

                if (std::abs(rect1.height - rect2.height) > rect1.height * 0.2) continue;
                int yCenter1 = rect1.y + rect1.height / 2;
                int yCenter2 = rect2.y + rect2.height / 2;
                if (std::abs(yCenter1 - yCenter2) > rect1.height * 0.2) continue;
                if (std::abs(rect1.height - rect2.height) > rect1.height * 0.2 ||
                    std::abs(rect1.width - rect2.width) > rect1.width * 0.2) continue;
                float aspectRatio1 = static_cast<float>(rect1.height) / rect1.width;
                float aspectRatio2 = static_cast<float>(rect2.height) / rect2.width;
                if (std::abs(aspectRatio1 - aspectRatio2) > 0.5) continue;
                int distance = std::abs((rect1.x + rect1.width / 2) - (rect2.x + rect2.width / 2));
                if (distance < rect1.height || distance > rect1.height * 3) continue;

                cv::Rect armorRect = rect1 | rect2;
                armorRects.push_back(armorRect);

                // 计算PnP
                std::vector<cv::Point3f> objectPoints = {
                    {-4.0f, -4.0f, 0},  // 左上角
                    {4.0f, -4.0f, 0},   // 右上角
                    {4.0f, 4.0f, 0},    // 右下角
                    {-4.0f, 4.0f, 0}    // 左下角
                };

                std::vector<cv::Point2f> imagePoints = {
                    cv::Point2f(rect1.x, rect1.y),
                    cv::Point2f(rect2.x + rect2.width, rect2.y),
                    cv::Point2f(rect2.x + rect2.width, rect2.y + rect2.height),
                    cv::Point2f(rect1.x, rect1.y + rect1.height)
                };

                cv::Mat rvec, tvec;
                cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

                // 输出距离信息
                double distanceToArmor = cv::norm(tvec);
                std::cout << "装甲板距离摄像头的距离: " << distanceToArmor << " cm" << std::endl;
            }
        }

        for (const auto& rect : armorRects) {
            cv::rectangle(undistortedImage, rect, cv::Scalar(0, 255, 0), 2);
        }

        image = undistortedImage;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorDetector>());
    rclcpp::shutdown();
    return 0;
}
