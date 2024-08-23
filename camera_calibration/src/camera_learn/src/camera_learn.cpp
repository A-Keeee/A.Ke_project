#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

class CameraCalibrationNode : public rclcpp::Node
{
public:
    CameraCalibrationNode() : Node("camera_calibration_node")
    {
        // 定义棋盘格内角点的行数和列数（注意是内角点）
        patternSize_ = cv::Size(6, 4); // 6x4 内角点
        imageSize_ = cv::Size();

        // 定义存储图像文件路径的数组
        imageFiles_ = {
            "src/image1.jpg",
            "src/image2.jpg",
            "src/image3.jpg",
            "src/image4.jpg",
            "src/image5.jpg"
        };

        // 处理图像
        processImages();
    }

private:
    void processImages()
{
    for (const auto& file : imageFiles_) {
        // 读取图像
        cv::Mat image = cv::imread(file, cv::IMREAD_GRAYSCALE);

        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "无法加载图像: %s", file.c_str());
            continue; // 继续处理下一张图像
        }

        if (imageSize_ == cv::Size()) {
            imageSize_ = image.size();
        }

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(image, patternSize_, corners,
                                               cv::CALIB_CB_ADAPTIVE_THRESH | 
                                               cv::CALIB_CB_NORMALIZE_IMAGE | 
                                               cv::CALIB_CB_FAST_CHECK);

        if (found) {
            cv::cornerSubPix(image, corners, cv::Size(5, 5), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));

            objectPoints_.push_back(create3DChessboardCorners());
            imagePoints_.push_back(corners);

            cv::drawChessboardCorners(image, patternSize_, corners, found);

            // 将标记后的图像保存到本地
            std::string outputFileName = "src/marked_" + file;
            cv::imwrite(outputFileName, image);
        } else {
            RCLCPP_ERROR(this->get_logger(), "未能检测到棋盘格角点: %s", file.c_str());
        }
    }

    // 如果有足够的图像，执行相机标定
    if (imagePoints_.size() > 3) {
        calibrateCamera();
    }
}


    std::vector<cv::Point3f> create3DChessboardCorners()
    {
        std::vector<cv::Point3f> objp;
        for (int i = 0; i < patternSize_.height; ++i) {
            for (int j = 0; j < patternSize_.width; ++j) {
                objp.push_back(cv::Point3f(j, i, 0));
            }
        }
        return objp;
    }

void calibrateCamera()
{
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(objectPoints_, imagePoints_, imageSize_, cameraMatrix, distCoeffs, rvecs, tvecs);

    RCLCPP_INFO(this->get_logger(), "标定完成，RMS误差: %f", rms);

    // 手动遍历并输出相机内参矩阵
    RCLCPP_INFO(this->get_logger(), "相机内参矩阵:");
    for (int i = 0; i < cameraMatrix.rows; ++i) {
        std::string row = "";
        for (int j = 0; j < cameraMatrix.cols; ++j) {
            row += std::to_string(cameraMatrix.at<double>(i, j)) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", row.c_str());
    }

    // 手动遍历并输出畸变系数
    RCLCPP_INFO(this->get_logger(), "畸变系数:");
    for (int i = 0; i < distCoeffs.rows; ++i) {
        std::string row = "";
        for (int j = 0; j < distCoeffs.cols; ++j) {
            row += std::to_string(distCoeffs.at<double>(i, j)) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", row.c_str());
    }
}





    std::vector<std::string> imageFiles_;
    cv::Size patternSize_;
    cv::Size imageSize_;
    std::vector<std::vector<cv::Point3f>> objectPoints_;
    std::vector<std::vector<cv::Point2f>> imagePoints_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraCalibrationNode>());
    rclcpp::shutdown();
    return 0;
}
