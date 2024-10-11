#include "inference.h"

Inference::Inference(const std::string &onnxModelPath, const cv::Size &modelInputShape, const std::string &classesTxtFile, const bool &runWithCuda)
{
    modelPath = onnxModelPath;
    modelShape = modelInputShape;
    classesPath = classesTxtFile;
    cudaEnabled = runWithCuda;

    loadOnnxNetwork();
    // loadClassesFromFile(); The classes are hard-coded for this example
}

// std::vector<Detection> Inference::runInference(const cv::Mat &input)
// {
//     cv::Mat modelInput = input;
//     if (letterBoxForSquare && modelShape.width == modelShape.height)
//         modelInput = formatToSquare(modelInput);

//     cv::Mat blob;
//     cv::dnn::blobFromImage(modelInput, blob, 1.0/255.0, modelShape, cv::Scalar(), true, false);
//     net.setInput(blob);

//     std::vector<cv::Mat> outputs;
//     net.forward(outputs, net.getUnconnectedOutLayersNames());

//     int rows = outputs[0].size[1];
//     int dimensions = outputs[0].size[2];

//     bool yolov8 = true;
//     // yolov5 has an output of shape (batchSize, 25200, 85) (Num classes + box[x,y,w,h] + confidence[c])
//     // yolov8 has an output of shape (batchSize, 84,  8400) (Num classes + box[x,y,w,h])
//     if (dimensions > rows) // Check if the shape[2] is more than shape[1] (yolov8)
//     {
//         yolov8 = true;
//         rows = outputs[0].size[2];
//         dimensions = outputs[0].size[1];

//         outputs[0] = outputs[0].reshape(1, dimensions);
//         cv::transpose(outputs[0], outputs[0]);
//     }
//     float *data = (float *)outputs[0].data;

//     float x_factor = modelInput.cols / modelShape.width;
//     float y_factor = modelInput.rows / modelShape.height;

//     std::vector<int> class_ids;
//     std::vector<float> confidences;
//     std::vector<cv::Rect> boxes;

//     for (int i = 0; i < rows; ++i)
//     {
//         if (yolov8)
//         {
//             float *classes_scores = data+4;

//             cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
//             cv::Point class_id;
//             double maxClassScore;

//             minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

//             if (maxClassScore > modelScoreThreshold)
//             {
//                 confidences.push_back(maxClassScore);
//                 class_ids.push_back(class_id.x);

//                 float x = data[0];
//                 float y = data[1];
//                 float w = data[2];
//                 float h = data[3];

//                 int left = int((x - 0.5 * w) * x_factor);
//                 int top = int((y - 0.5 * h) * y_factor);

//                 int width = int(w * x_factor);
//                 int height = int(h * y_factor);

//                 boxes.push_back(cv::Rect(left, top, width, height));
//             }
//         }
//         else // yolov5
//         {
//             float confidence = data[4];

//             if (confidence >= modelConfidenceThreshold)
//             {
//                 float *classes_scores = data+5;

//                 cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
//                 cv::Point class_id;
//                 double max_class_score;

//                 minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

//                 if (max_class_score > modelScoreThreshold)
//                 {
//                     confidences.push_back(confidence);
//                     class_ids.push_back(class_id.x);

//                     float x = data[0];
//                     float y = data[1];
//                     float w = data[2];
//                     float h = data[3];

//                     int left = int((x - 0.5 * w) * x_factor);
//                     int top = int((y - 0.5 * h) * y_factor);

//                     int width = int(w * x_factor);
//                     int height = int(h * y_factor);

//                     boxes.push_back(cv::Rect(left, top, width, height));
//                 }
//             }
//         }

//         data += dimensions;
//     }

//     std::vector<int> nms_result;
//     cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

//     std::vector<Detection> detections{};
//     for (unsigned long i = 0; i < nms_result.size(); ++i)
//     {
//         int idx = nms_result[i];

//         Detection result;
//         result.class_id = class_ids[idx];
//         result.confidence = confidences[idx];

//         std::random_device rd;
//         std::mt19937 gen(rd());
//         std::uniform_int_distribution<int> dis(100, 255);
//         result.color = cv::Scalar(dis(gen),
//                                   dis(gen),
//                                   dis(gen));

//         result.className = classes[result.class_id];
//         result.box = boxes[idx];

//         detections.push_back(result);
//     }

//     return detections;
// }

std::vector<Detection> Inference::runInference(const cv::Mat &input)
{
    cv::Mat modelInput = input;
    if (letterBoxForSquare && modelShape.width == modelShape.height)
        modelInput = formatToSquare(modelInput);

    cv::Mat blob;
    cv::dnn::blobFromImage(modelInput, blob, 1.0 / 255.0, modelShape, cv::Scalar(), true, false);
    net.setInput(blob);

    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    int rows = outputs[0].size[1];
    int dimensions = outputs[0].size[2];

    bool yolov8 = true;
    if (dimensions > rows) // YOLOv8 输出格式检查
    {
        yolov8 = true;
        rows = outputs[0].size[2];
        dimensions = outputs[0].size[1];

        outputs[0] = outputs[0].reshape(1, dimensions);
        cv::transpose(outputs[0], outputs[0]);
    }
    float *data = (float *)outputs[0].data;

    float x_factor = modelInput.cols / modelShape.width;
    float y_factor = modelInput.rows / modelShape.height;

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    std::vector<Detection> detections{};

    for (int i = 0; i < rows; ++i)
    {
        if (yolov8)
        {
            float *classes_scores = data + 4;

            cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double maxClassScore;

            minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

            if (maxClassScore > modelScoreThreshold)
            {
                confidences.push_back(maxClassScore);
                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];

                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);

                int width = int(w * x_factor);
                int height = int(h * y_factor);

                boxes.push_back(cv::Rect(left, top, width, height));

                // 创建 Detection 对象
                Detection result;
                result.class_id = class_id.x;
                result.confidence = maxClassScore;
                result.className = classes[result.class_id];
                result.box = cv::Rect(left, top, width, height);
                
                // 提取关键点（假设每个关键点由 3 个值 [x, y, confidence] 组成）
                int num_keypoints = (dimensions - 4) / 3;  // 假设模型输出中有多个关键点
                for (int j = 0; j < num_keypoints; ++j)
                {
                    float kpx = data[4 + j * 3] * x_factor;  // 第一个值是 x
                    float kpy = data[5 + j * 3] * y_factor;  // 第二个值是 y
                    float kp_conf = data[6 + j * 3];         // 第三个值是置信度

                    if (kp_conf > 0.5)  // 如果关键点置信度足够高
                    {
                        result.keypoints.push_back(cv::Point(kpx, kpy));
                    }
                }

                // 为每个物体生成随机颜色
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_int_distribution<int> dis(100, 255);
                result.color = cv::Scalar(dis(gen), dis(gen), dis(gen));

                detections.push_back(result);
            }
        }
        data += dimensions;
    }

    // 非极大值抑制
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

    std::vector<Detection> final_detections{};
    for (unsigned long i = 0; i < nms_result.size(); ++i)
    {
        int idx = nms_result[i];
        final_detections.push_back(detections[idx]);
    }

    return final_detections;
}


void Inference::loadClassesFromFile()
{
    std::ifstream inputFile(classesPath);
    if (inputFile.is_open())
    {
        std::string classLine;
        while (std::getline(inputFile, classLine))
            classes.push_back(classLine);
        inputFile.close();
    }
}

void Inference::loadOnnxNetwork()
{
    net = cv::dnn::readNetFromONNX(modelPath);
    if (cudaEnabled)
    {
        std::cout << "\nRunning on CUDA" << std::endl;
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    }
    else
    {
        std::cout << "\nRunning on CPU" << std::endl;
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
}

cv::Mat Inference::formatToSquare(const cv::Mat &source)
{
    int col = source.cols;
    int row = source.rows;
    int _max = MAX(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
    return result;
}

