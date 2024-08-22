#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

// 伽马校正函数
cv::Mat adjustGamma(const cv::Mat& img, double gamma) {
    cv::Mat lut_matrix(1, 256, CV_8UC1);
    uchar* ptr = lut_matrix.ptr();
    for (int i = 0; i < 256; i++)
        ptr[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    
    cv::Mat result;
    cv::LUT(img, lut_matrix, result);
    
    return result;
}

int main() {
    // 加载图像
    cv::Mat image = cv::imread("src/image.png", cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        std::cerr << "无法加载图像！" << std::endl;
        return -1;
    }

    // 应用伽马校正增强暗部细节，设置伽马值为2.2
    double gamma_value = 2.2;
    cv::Mat adjusted_image = adjustGamma(image, gamma_value);

    // 显示原始图像和处理后图像
    cv::imshow("原始图像", image);
    cv::imshow("处理后图像", adjusted_image);

    // 保存处理后的图像
    cv::imwrite("processed_image.png", adjusted_image);

    // 等待用户按下键
    cv::waitKey(0);

    return 0;
}
