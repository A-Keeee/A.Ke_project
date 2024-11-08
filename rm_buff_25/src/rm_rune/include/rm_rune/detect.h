#ifndef DETECT_H
#define DETECT_H

#include <random>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <vector>
#include <unordered_map>
#include <string>
#include <sstream>
#include <iomanip>

#include "rm_rune/onnx_model_base.h"
#include "rm_rune/autobackend.h"
#include "rm_rune/augment.h"
#include "rm_rune/constants.h"
#include "rm_rune/common.h"

namespace fs = std::filesystem;

// 定义骨架和颜色映射
extern std::vector<std::vector<int>> skeleton;
extern std::vector<cv::Scalar> posePalette;
extern std::vector<int> limbColorIndices;
extern std::vector<int> kptColorIndices;

// 函数声明

// 生成随机颜色，指定通道数
cv::Scalar generateRandomColor(int numChannels);

// 为给定类别数量生成随机颜色
std::vector<cv::Scalar> generateRandomColors(int class_names_num, int numChannels);

// 在图像上绘制检测到的掩膜
void plot_masks(cv::Mat img, std::vector<YoloResults>& result, std::vector<cv::Scalar> color,
                std::unordered_map<int, std::string>& names);

// 绘制检测到的关键点
void plot_keypoints(cv::Mat& image, const std::vector<YoloResults>& results, const cv::Size& shape);

// 绘制检测结果，包括边界框、关键点和掩膜
void plot_results(cv::Mat img, std::vector<YoloResults>& results,
                  std::vector<cv::Scalar> color, std::unordered_map<int, std::string>& names,
                  const cv::Size& shape);

// 主函数，处理图像或视频检测
int main();

#endif // DETECT_H
