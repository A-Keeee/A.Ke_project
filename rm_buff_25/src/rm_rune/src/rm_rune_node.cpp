//创建节点 发布参数 


#include "rm_rune_node.hpp"

namespace qianli_rm_rune
{
    // RuneNode构造函数，初始化节点，发布者和订阅者
    RuneNode::RuneNode(const rclcpp::NodeOptions & options) : Node("rm_rune_node", options)
    {
        // 在控制台输出节点启动信息
        RCLCPP_INFO(get_logger(), "Hello, QianLi RM Rune!");

        // 初始化相机内参矩阵为全零矩阵
        camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);

        // 创建发布者，用于发布3D预测位置（/rune/prediction）
        rune_pose_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/rune/prediction", 10);

        // 订阅图像原始数据（/image_raw），处理图像回调
        rune_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS(), std::bind(&RuneNode::rune_image_callback, this, std::placeholders::_1));

        // 订阅相机内参数据（/camera_info），用于相机矩阵的初始化
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
            cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
            // 将相机参数存入相机矩阵
            camera_matrix_.at<double>(0,0) = camera_info->k[0];
            camera_matrix_.at<double>(0,2) = camera_info->k[2];
            camera_matrix_.at<double>(1,1) = camera_info->k[4];
            camera_matrix_.at<double>(1,2) = camera_info->k[5];
            camera_matrix_.at<double>(2,2) = 1.0;
            // 完成相机内参获取后，取消订阅
            cam_info_sub_.reset();
        });

        // 初始化tf2缓存和监听器，用于将预测的3D坐标转换到不同的坐标系
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(), this->get_node_timers_interface());
        tf2_buffer_->setCreateTimerInterface(timer_interface);
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    }

    /*
    图像处理的回调函数，处理接收到的图像信息，进行图像处理、预测并发布3D点位信息。
    参数:
    - msg: sensor_msgs::msg::Image类型，表示接收到的图像消息。
    */
    void RuneNode::rune_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat rune_image;
        try
        {
            // 使用cv_bridge将ROS图像消息转换为OpenCV图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            rune_image = cv_ptr->image;
        }
        catch (cv_bridge::Exception & e)
        {
            // 如果图像转换失败，输出错误信息
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        //改用神经网络识别
        //需要条用detect中的函数进行神经网络识别

        // cv::Mat rune_gray_image;
        // // 将BGR图像转换为灰度图
        // rune_gray_image = image_processer_.to_gray(rune_image, cfg_.kernel_size);

        // cv::Mat rune_binary_image;
        // // 将灰度图像转换为二值图
        // rune_binary_image = image_processer_.to_binary(rune_gray_image, cfg_.binary_threshold);

        // // 查找图像中的轮廓
        // std::vector<std::vector<cv::Point>> contours;
        // cv::findContours(rune_binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        ContourInfo::plot_results(rune_image, results, posePalette, names, rune_image.size(), contours);//需要对头文件进行修改

        // 计算每个轮廓的相关信息，并存入contours_info_向量
        for(auto &contour : contours)
        {
            contour_info_.setContour(contour);
            contours_info_.push_back(contour_info_);
        }

        // 根据面积和Hu矩过滤轮廓信息
        contours_info_ = power_rune_.filterByArea(contours_info_, cfg_.min_area);
        contours_info_ = power_rune_.filterByHu(contours_info_, cfg_.ref_hu, cfg_.hu_dev_threshold);
        
        // 如果没有检测到合适的能量机关，输出调试信息
        if (contours_info_.empty()) {
            RCLCPP_DEBUG(get_logger(), "未检测到能量机关");
            return;
        }
        
        // 如果检测到多个能量机关，输出警告并只处理形状最接近的一个
        if (contours_info_.size() > 1) {
            RCLCPP_WARN(get_logger(), "检测到 %ld 个能量机关，仅使用最形状接近的一个", contours_info_.size());
        }

        // 使用Blade类对象对检测到的轮廓信息进行处理
        Blade blade(contours_info_[0],cfg_);
        

        //以下部分进行保留
        // 更新预测器并进行预测，这里改为传入pnp解算过后的目标板中心和buff圆心之间的平移矩阵
        predictor.update(blade.vector);//计算目标与中心之间的向量，用于表示目标与中心的相对方向和距离

        auto radian = predictor.predict();//返回预测的角度
        auto predicted_vector = power_rune_.predict(blade.vector, radian);//返回预测的x，y坐标

        // 如果没有相机信息，无法计算3D点位，输出错误信息
        if (cam_info_->k.empty()) {
            RCLCPP_ERROR(get_logger(), "没有相机信息，无法计算3D点位信息");
            return;
        }

        // 创建消息并填充预测的3D点位
        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header.frame_id = "camera_link";
        point_msg.point.x = 1;
        point_msg.point.y = -(predicted_vector.x + blade.center.x - cam_info_->k[2]) / cam_info_->k[0];
        point_msg.point.z = -(predicted_vector.y + blade.center.y - cam_info_->k[5]) / cam_info_->k[4];

        // 根据相机参数和预测向量，计算3D距离
        float distance = (cam_info_->k[0] + cam_info_->k[4]) / 2 / std::sqrt(predicted_vector.x * predicted_vector.x + predicted_vector.y * predicted_vector.y) * 0.7 * cfg_.distance_correction_ratio;

        // 将计算后的距离信息应用到3D点位
        point_msg.point.x *= distance;
        point_msg.point.y *= distance;
        point_msg.point.z *= distance;

        geometry_msgs::msg::PointStamped transformed_msg;
        try {
            // 将点位信息从相机坐标系转换到全局坐标系
            transformed_msg.point = tf2_buffer_->transform(point_msg, "odom").point;
            transformed_msg.header.frame_id = "odom";
            transformed_msg.header.stamp = point_msg.header.stamp;
            // 发布转换后的3D点位信息
            rune_pose_pub_->publish(transformed_msg);
        } catch (tf2::TransformException& ex) {
            // 如果坐标转换失败，输出警告信息
            RCLCPP_WARN(get_logger(), "无法将坐标从 camera_link 转换到 odom：%s", ex.what());
        }
    }
} // namespace qianli_rm_rune

#include "rclcpp_components/register_node_macro.hpp"

// 注册组件，确保该节点在库加载时可以被发现并使用
RCLCPP_COMPONENTS_REGISTER_NODE(qianli_rm_rune::RuneNode)
