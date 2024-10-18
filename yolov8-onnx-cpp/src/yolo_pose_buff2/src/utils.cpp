#pragma once
#include "utils.h"
using namespace cv;
using namespace std;
 
void LetterBox(const cv::Mat& image, cv::Mat& outImage,
               cv::Vec4d& params,
               const cv::Size& newShape,
               bool autoShape,
               bool scaleFill,
               bool scaleUp,
               int stride,
               const cv::Scalar& color)
{
    if (false) {
        int maxLen = MAX(image.rows, image.cols);
        outImage = Mat::zeros(Size(maxLen, maxLen), CV_8UC3);
        image.copyTo(outImage(Rect(0, 0, image.cols, image.rows)));
        params[0] = 1;
        params[1] = 1;
        params[3] = 0;
        params[2] = 0;
    }
 
    // 取较小的缩放比例
    cv::Size shape = image.size();
    float r = std::min((float)newShape.height / (float)shape.height,
                       (float)newShape.width / (float)shape.width);
    if (!scaleUp)
        r = std::min(r, 1.0f);
    printf("原图尺寸：w:%d * h:%d, 要求尺寸：w:%d * h:%d, 即将采用的缩放比：%f\n",
           shape.width, shape.height, newShape.width, newShape.height, r);
 
    // 依据前面的缩放比例后，原图的尺寸
    float ratio[2]{r,r};
    int new_un_pad[2] = { (int)std::round((float)shape.width  * r), (int)std::round((float)shape.height * r)};
    printf("等比例缩放后的尺寸该为：w:%d * h:%d\n", new_un_pad[0], new_un_pad[1]);
 
    // 计算距离目标尺寸的padding像素数
    auto dw = (float)(newShape.width - new_un_pad[0]);
    auto dh = (float)(newShape.height - new_un_pad[1]);
    if (autoShape)
    {
        dw = (float)((int)dw % stride);
        dh = (float)((int)dh % stride);
    }
    else if (scaleFill)
    {
        dw = 0.0f;
        dh = 0.0f;
        new_un_pad[0] = newShape.width;
        new_un_pad[1] = newShape.height;
        ratio[0] = (float)newShape.width / (float)shape.width;
        ratio[1] = (float)newShape.height / (float)shape.height;
    }
 
    dw /= 2.0f;
    dh /= 2.0f;
    printf("填充padding: dw=%f , dh=%f\n", dw, dh);
 
    // 等比例缩放
    if (shape.width != new_un_pad[0] && shape.height != new_un_pad[1])
    {
        cv::resize(image, outImage, cv::Size(new_un_pad[0], new_un_pad[1]));
    }
    else{
        outImage = image.clone();
    }
 
    // 图像四周padding填充，至此原图与目标尺寸一致
    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));
    params[0] = ratio[0]; // width的缩放比例
    params[1] = ratio[1]; // height的缩放比例
    params[2] = left; // 水平方向两边的padding像素数
    params[3] = top; //垂直方向两边的padding像素数
    cv::copyMakeBorder(outImage, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, color);
}

void DrawPred(cv::Mat& img, std::vector<OutputPose>& results) {
    const int num_point = 5;
    for (auto &result : results) {
        int left, top, width, height;
        left = result.box.x;
        top = result.box.y;
        width = result.box.width;
        height = result.box.height;

        // 框出目标
        rectangle(img, result.box, Scalar(0, 0, 255), 2, 8);

        // 在目标框左上角标识目标类别以及概率
        string label = to_string(result.confidence);
        int baseLine;
        Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        top = max(top, labelSize.height);
        putText(img, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);

        // 关键点绘制
        auto &kps = result.kps;
        for (int k = 0; k < num_point; k++) {
            int kps_x = std::round(kps[k * 3]);
            int kps_y = std::round(kps[k * 3 + 1]);
            float kps_s = kps[k * 3 + 2];

            cv::Scalar kps_color = cv::Scalar(0, 0, 255);  // 红色 (B, G, R)
            cv::circle(img, {kps_x, kps_y}, 5, kps_color, -1);
        }
    }
}

// void DrawPred(cv::Mat& img, std::vector<OutputPose>& results,
//               const std::vector<std::vector<unsigned int>> &SKELLTON,
//               const std::vector<std::vector<unsigned int>> &KPS_COLORS,
//               const std::vector<std::vector<unsigned int>> &LIMB_COLORS)
// {
//     const int num_point =5;
//     for (auto &result:results){
//         int  left,top,width, height;
//         left = result.box.x;
//         top = result.box.y;
//         width = result.box.width;
//         height = result.box.height;
 
 
// //        printf("x: %d  y:%d  w:%d  h%d\n",(int)left, (int)top, (int)result.box.width, (int)result.box.height);
 
//         // 框出目标
//         rectangle(img, result.box,Scalar(0,0,255), 2, 8);
 
//         // 在目标框左上角标识目标类别以及概率
//         string label = "person:" + to_string(result.confidence) ;
//         int baseLine;
//         Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
//         top = max(top, labelSize.height);
//         putText(img, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255), 2);
 
//         // 连线
//         auto &kps = result.kps;
// //        cout << "该目标关键点：" << kps.size() << endl;
//         for (int k=0; k<num_point+2; k++){// 不要设置为>0.5f ,>0.0f显示效果比较好
//             // 关键点绘制
//             if (k<num_point){
//                 int kps_x = std::round(kps[k*3]);
//                 int kps_y = std::round(kps[k*3 + 1]);
//                 float kps_s = kps[k*3 + 2];
 
// //                printf("x:%d y:%d s:%f\n", kps_x, kps_y, kps_s);
 
//                 if (kps_s > 0.0f){
//                     cv::Scalar kps_color = Scalar(KPS_COLORS[k][0],KPS_COLORS[k][1],KPS_COLORS[k][2]);
//                     cv::circle(img, {kps_x, kps_y}, 5, kps_color, -1);
//                 }
//             }
 
            // auto &ske = SKELLTON[k];
            // int pos1_x = std::round(kps[(ske[0] -1) * 3]);
            // int pos1_y = std::round(kps[(ske[0] -1) * 3 + 1]);
 
            // int pos2_x = std::round(kps[(ske[1] -1) * 3]);
            // int pos2_y = std::round(kps[(ske[1] -1) * 3 + 1]);
 
            // float pos1_s = kps[(ske[0] -1) * 3 + 2];
            // float pos2_s = kps[(ske[1] -1) * 3 + 2];
 
            // if (pos1_s > 0.0f && pos2_s >0.0f){// 不要设置为>0.5f ,>0.0f显示效果比较好
            //     cv::Scalar limb_color = cv::Scalar(LIMB_COLORS[k][0], LIMB_COLORS[k][1], LIMB_COLORS[k][3]);
            //     cv::line(img, {pos1_x, pos1_y}, {pos2_x, pos2_y}, limb_color);
            // }
 
        // // 跌倒检测
        //     float pt5_x = kps[5*3];
        //     float pt5_y = kps[5*3 + 1];
        //     float pt6_x = kps[6*3];
        //     float pt6_y = kps[6*3+1];
        //     float center_up_x = (pt5_x + pt6_x) /2.0f ;
        //     float center_up_y = (pt5_y + pt6_y) / 2.0f;
        //     Point center_up = Point((int)center_up_x, (int)center_up_y);
 
        //     float pt11_x = kps[11*3];
        //     float pt11_y = kps[11*3 + 1];
        //     float pt12_x = kps[12*3];
        //     float pt12_y = kps[12*3 + 1];
        //     float center_down_x = (pt11_x + pt12_x) / 2.0f;
        //     float center_down_y = (pt11_y + pt12_y) / 2.0f;
        //     Point center_down = Point((int)center_down_x, (int)center_down_y);
 
 
        //     float right_angle_point_x = center_down_x;
        //     float righ_angle_point_y = center_up_y;
        //     Point right_angl_point = Point((int)right_angle_point_x, (int)righ_angle_point_y);
 
 
        //     float a = abs(right_angle_point_x - center_up_x);
        //     float b = abs(center_down_y - righ_angle_point_y);
 
        //     float tan_value = a / b;
        //     float Pi = acos(-1);
        //     float angle = atan(tan_value) * 180.0f/ Pi;
        //     string angel_label = "angle: " + to_string(angle);
        //     putText(img, angel_label, Point(left, top-40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255), 2);
 
        //     if (angle > 60.0f || center_down_y <= center_up_y || (double)width/ height > 5.0f/3.0f) // 宽高比小于0.6为站立，大于5/3为跌倒
        //     {
        //         string fall_down_label = "person fall down!!!!";
        //         putText(img, fall_down_label , Point(left, top-20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255), 2);
 
        //         printf("angel:%f width/height:%f\n",angle, (double)width/ height );
        //     }
 
 
 
 
 
 
 
 
 
            // cv::line(img, center_up, center_down,
            //          Scalar(0,0,255), 2, 8);
            // cv::line(img, center_up, right_angl_point,
            //          Scalar(0,0,255), 2, 8);
            // cv::line(img, right_angl_point, center_down,
            //          Scalar(0,0,255), 2, 8);
 
 
 
 
 
//         }
//     }
// }

