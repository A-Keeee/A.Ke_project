#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <numeric>
#include <chrono>
using namespace cv;
using namespace cv::ml;
using namespace std;

// 获取点间距离
double getDistance(Point A, Point B)
{
    double dis;
    dis = pow((A.x - B.x), 2) + pow((A.y - B.y), 2);
    return sqrt(dis);
}

// 标准化并计算hog
vector<float> stander(Mat im)
{
    if (im.empty() == 1)
    {
        cout << "filed open" << endl;
    }
    resize(im, im, Size(48, 48));

    vector<float> result;

    HOGDescriptor hog(cv::Size(48, 48), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9, 1, -1,
                      HOGDescriptor::L2Hys, 0.2, false, HOGDescriptor::DEFAULT_NLEVELS); // 初始化HOG描述符
    hog.compute(im, result);
    return result;
}

// 将图片转换为svm所需格式
Mat get(Mat input)
{
    vector<float> vec = stander(input);
    if (vec.size() != 900)
        cout << "wrong not 900" << endl;
    Mat output(1, 900, CV_32FC1);

    Mat_<float> p = output;
    int jj = 0;
    for (vector<float>::iterator iter = vec.begin(); iter != vec.end(); iter++, jj++)
    {
        p(0, jj) = *(iter);
    }
    return output;
}

// 模板匹配
double TemplateMatch(cv::Mat image, cv::Mat tepl, cv::Point &point, int method)
{
    int result_cols = image.cols - tepl.cols + 1;
    int result_rows = image.rows - tepl.rows + 1;
    cv::Mat result = cv::Mat(result_cols, result_rows, CV_32FC1);
    cv::matchTemplate(image, tepl, result, method);

    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

    switch (method)
    {
    case TM_SQDIFF:
    case TM_SQDIFF_NORMED:
        point = minLoc;
        return minVal;

    default:
        point = maxLoc;
        return maxVal;
    }
}

#define USE_TEMPLATE
#define SHOW_RESULT
#define RED

int main(int argc, char *argv[])
{
    cv::VideoCapture cap;
    cap.open("src/red.avi");

    Mat srcImage;
    cap >> srcImage;
    // 画拟合圆
    Mat drawcircle = Mat(srcImage.rows, srcImage.cols, CV_8UC3, Scalar(0, 0, 0));
    Mat templ[9];
    for (int i = 1; i <= 8; i++)
    {
        templ[i] = imread("src/template/template" + to_string(i) + ".jpg", IMREAD_GRAYSCALE);
    }
    vector<Point2f> cirV;

    Point2f cc = Point2f(0, 0);

    // 相机内参
    Mat cameraMatrix = (Mat_<double>(3, 3) << 1462.3697, 0, 398.59394,
                                               0, 1469.68385, 110.68997,
                                               0, 0, 1);

    // 畸变系数
    Mat distCoeffs = (Mat_<double>(1, 5) << 0.003518, -0.311778, -0.016581, 0.023682, 0.0);

    // 目标的3D坐标（单位：cm）
    vector<Point3f> objectPoints;
    objectPoints.push_back(Point3f(-160.0, -160.0, 0));  // 左上角
    objectPoints.push_back(Point3f(160.0, -160.0, 0));   // 右上角
    objectPoints.push_back(Point3f(160.0, 160.0, 0));    // 右下角
    objectPoints.push_back(Point3f(-160.0, 160.0, 0));   // 左下角

    // 程序主循环
    while (true)
    {
        cap >> srcImage;
        auto t1 = chrono::high_resolution_clock::now();
        // 分割颜色通道
        vector<Mat> imgChannels;
        split(srcImage, imgChannels);
        // 获得目标颜色图像的二值图
#ifdef RED
        Mat midImage2 = imgChannels.at(2) - imgChannels.at(0);
#endif
#ifndef RED
        Mat midImage2 = imgChannels.at(0) - imgChannels.at(2);
#endif
        // 二值化，背景为黑色，图案为白色
        // 用于查找扇叶
        threshold(midImage2, midImage2, 100, 255, THRESH_BINARY);

        int structElementSize = 2;
        Mat element = getStructuringElement(MORPH_RECT, Size(2 * structElementSize + 1, 2 * structElementSize + 1), Point(structElementSize, structElementSize));
        // 膨胀
        dilate(midImage2, midImage2, element);
        // 开运算，消除扇叶上可能存在的小洞
        structElementSize = 3;
        element = getStructuringElement(MORPH_RECT, Size(2 * structElementSize + 1, 2 * structElementSize + 1), Point(structElementSize, structElementSize));
        morphologyEx(midImage2, midImage2, MORPH_CLOSE, element);

        // 查找轮廓
        vector<vector<Point>> contours2;
        vector<Vec4i> hierarchy2;
        findContours(midImage2, contours2, hierarchy2, RETR_TREE, CHAIN_APPROX_SIMPLE);

        RotatedRect rect_tmp2;
        bool findTarget = false;

        // 遍历轮廓
        if (hierarchy2.size())
        {
            for (int i = 0; i >= 0; i = hierarchy2[i][0])
            {
                rect_tmp2 = minAreaRect(contours2[i]);
                Point2f P[4];
                rect_tmp2.points(P);

                Point2f srcRect[4];
                Point2f dstRect[4];

                double width;
                double height;

                // 矫正提取的叶片的宽高
                width = getDistance(P[0], P[1]);
                height = getDistance(P[1], P[2]);
                if (width > height)
                {
                    srcRect[0] = P[0];
                    srcRect[1] = P[1];
                    srcRect[2] = P[2];
                    srcRect[3] = P[3];
                }
                else
                {
                    swap(width, height);
                    srcRect[0] = P[1];
                    srcRect[1] = P[2];
                    srcRect[2] = P[3];
                    srcRect[3] = P[0];
                }

                // 通过面积筛选
                double area = height * width;
                if (area > 5000)
                {
                    dstRect[0] = Point2f(0, 0);
                    dstRect[1] = Point2f(width, 0);
                    dstRect[2] = Point2f(width, height);
                    dstRect[3] = Point2f(0, height);
                    // 应用透视变换，矫正成规则矩形
                    Mat transform = getPerspectiveTransform(srcRect, dstRect);
                    Mat perspectMat;
                    warpPerspective(midImage2, perspectMat, transform, midImage2.size());

                    // 提取扇叶图片
                    Mat testim;
                    testim = perspectMat(Rect(0, 0, width, height));

                    if (testim.empty())
                    {
                        cout << "filed open" << endl;
                        return -1;
                    }

                    // 获取2D图像坐标
                    vector<Point2f> imagePoints;
                    imagePoints.push_back(P[0]);
                    imagePoints.push_back(P[1]);
                    imagePoints.push_back(P[2]);
                    imagePoints.push_back(P[3]);

                    // 使用solvePnP解算
                    Mat rvec, tvec;
                    solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

                    // 将旋转向量转换为旋转矩阵
                    Mat R;
                    Rodrigues(rvec, R);

                    // 计算目标到摄像头的距离
                    double distance = norm(tvec);

                    // 输出旋转向量、位移向量以及距离
                    cout << "Rotation Vector (rvec): " << rvec << endl;
                    cout << "Translation Vector (tvec): " << tvec << endl;
                    cout << "Distance to target: " << distance << " cm" << endl;

                    // 查找装甲板
                    if (hierarchy2[i][2] >= 0)
                    {
                        RotatedRect rect_tmp = minAreaRect(contours2[hierarchy2[i][2]]);
                        Point2f Pnt[4];
                        rect_tmp.points(Pnt);
                        const float maxHWRatio = 0.7153846;
                        const float maxArea = 2000;
                        const float minArea = 500;

                        float width = rect_tmp.size.width;
                        float height = rect_tmp.size.height;
                        if (height > width)
                            swap(height, width);
                        float area = width * height;

                        if (height / width > maxHWRatio || area > maxArea || area < minArea)
                        {
                            continue;
                        }
                        Point centerP = rect_tmp.center;
                        // 打击点
                        circle(srcImage, centerP, 1, Scalar(0, 255, 0), 2);

                        for (int j = 0; j < 4; ++j)
                        {
                            line(srcImage, Pnt[j], Pnt[(j + 1) % 4], Scalar(0, 255, 255), 2);
                        }
                    }
                }
            }
        }

#ifdef SHOW_RESULT
        imshow("Result", srcImage);
        if ('q' == waitKey(1))
            break;
#endif
        // 函数所花的时间
        auto t2 = chrono::high_resolution_clock::now();
        cout << "Total period: " << (static_cast<chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
    }
    return 0;
}
