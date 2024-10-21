#include <iostream>
#include <opencv2/opencv.hpp>
#include "yolo_pose_buff2/detect.h"
#include <sys/time.h>
 
#include <vector>
 
using namespace std;
using namespace cv;
using namespace cv::dnn;

 
int main(){
 
    //  读取模型
    string detect_model_path = "model/rm_buff_pose.onnx";
    Yolov8Onnx yolov8;
    if (yolov8.ReadModel(detect_model_path))
        cout << "read Net ok!\n";
    else {
        return -1;
    }
 
 
 
    VideoCapture capture;
    capture.open("test/test.MP4");
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
    writer.open("test/test_result.mp4", VideoWriter::fourcc('m', 'p', '4', 'v'),
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