#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
 
 
 
 
struct  OutputPose {
 
    cv::Rect_<float> box;
    int label =0;
    float confidence =0.0;
    std::vector<float> kps;
 
};
 
void DrawPred(cv::Mat& img, std::vector<OutputPose>& results);
            //   const std::vector<std::vector<unsigned int>> &SKELLTON,
            //   const std::vector<std::vector<unsigned int>> &KPS_COLORS,
            //   const std::vector<std::vector<unsigned int>> &LIMB_COLORS);
void LetterBox(const cv::Mat& image, cv::Mat& outImage,
               cv::Vec4d& params,
               const cv::Size& newShape = cv::Size(640, 640),
               bool autoShape = false,
               bool scaleFill=false,
               bool scaleUp=true,
               int stride= 32,
               const cv::Scalar& color = cv::Scalar(114,114,114));