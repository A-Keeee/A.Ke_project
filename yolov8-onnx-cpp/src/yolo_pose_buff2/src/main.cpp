#include <iostream>
#include <opencv2/opencv.hpp>
#include "detect.h"
#include <sys/time.h>
 
#include <vector>
 
using namespace std;
using namespace cv;
using namespace cv::dnn;
 
// const std::vector<std::vector<unsigned int>> KPS_COLORS =
//         {{0,   255, 0},
//          {0,   255, 0},
//          {0,   255, 0},
//          {0,   255, 0},
//          {0,   255, 0},
//          {255, 128, 0},
//          {255, 128, 0},
//          {255, 128, 0},
//          {255, 128, 0},
//          {255, 128, 0},
//          {255, 128, 0},
//          {51,  153, 255},
//          {51,  153, 255},
//          {51,  153, 255},
//          {51,  153, 255},
//          {51,  153, 255},
//          {51,  153, 255}};
 
// const std::vector<std::vector<unsigned int>> SKELETON = {{16, 14},
//                                                          {14, 12},
//                                                          {17, 15},
//                                                          {15, 13},
//                                                          {12, 13},
//                                                          {6,  12},
//                                                          {7,  13},
//                                                          {6,  7},
//                                                          {6,  8},
//                                                          {7,  9},
//                                                          {8,  10},
//                                                          {9,  11},
//                                                          {2,  3},
//                                                          {1,  2},
//                                                          {1,  3},
//                                                          {2,  4},
//                                                          {3,  5},
//                                                          {4,  6},
//                                                          {5,  7}};
 
// const std::vector<std::vector<unsigned int>> LIMB_COLORS = {{51,  153, 255},
//                                                             {51,  153, 255},
//                                                             {51,  153, 255},
//                                                             {51,  153, 255},
//                                                             {255, 51,  255},
//                                                             {255, 51,  255},
//                                                             {255, 51,  255},
//                                                             {255, 128, 0},
//                                                             {255, 128, 0},
//                                                             {255, 128, 0},
//                                                             {255, 128, 0},
//                                                             {255, 128, 0},
//                                                             {0,   255, 0},
//                                                             {0,   255, 0},
//                                                             {0,   255, 0},
//                                                             {0,   255, 0},
//                                                             {0,   255, 0},
//                                                             {0,   255, 0},
//                                                             {0,   255, 0}};
 
 
int main(){
 
    //  读取模型
    string detect_model_path = "src/rm_buff_pose.onnx";
    Yolov8Onnx yolov8;
    if (yolov8.ReadModel(detect_model_path))
        cout << "read Net ok!\n";
    else {
        return -1;
    }
 
 
 
    VideoCapture capture;
    capture.open("src/test.mp4");
    if (capture.isOpened())
        cout << "read video ok!\n";
    else
        cout << "read video err!\n";
    int width = capture.get(CAP_PROP_FRAME_WIDTH);
    int height = capture.get(CAP_PROP_FRAME_HEIGHT);
    Size size1 = Size(width, height);
    double delay = 1000/capture.get(CAP_PROP_FPS);
    int frame_pos = 0;
    int frame_all = capture.get(CAP_PROP_FRAME_COUNT);
 
    VideoWriter writer;
    writer.open("src/test_result.mp4", VideoWriter::fourcc('m', 'p', '4', 'v'),
                delay,size1);
 
    Mat frame;
    struct timeval t1, t2;
    double timeuse;
    while (1) {
 
        //
        capture>>frame;
 
        if (frame_pos == frame_all-1) break;
 
 
        // YOLOv8检测
        vector<OutputPose> result;
        gettimeofday(&t1, NULL);
        bool  find = yolov8.OnnxDetect(frame, result);
        gettimeofday(&t2, NULL);
        frame_pos+=1;
        // printf("%d/%d:find %d person!\n",frame_pos, frame_all, (int)result.size());
 
 
        if(find)
        {
            DrawPred(frame, result);//, SKELETON, KPS_COLORS, LIMB_COLORS);
            }
        else {
            cout << "not find!\n";
        }
 
        timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000;
        timeuse *= 1000;
        string label = "TimeUse: " + to_string(timeuse);
        putText(frame, label, Point(30,30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255), 2, 8);
 
        writer << frame;
        imshow("yolov8n-pose", frame);
        if(waitKey(1)=='q') break;
 
    }
 
    capture.release();
//    writer.release();
 
 
    return 0;
}