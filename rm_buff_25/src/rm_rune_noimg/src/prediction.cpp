//保留
// 预测函数 对buff运动进行估计




#include "prediction.hpp"

   /*
    计算能量机关旋转的位置（弧度制）
    公式：
    $$
    \theta = k t + b + a \cos \left( \omega t + \phi \right)
    $$
    其中，k 是线性增长的斜率，b 是截距，a 是振幅，ω 是圆速度，φ 是初始相位。

    @param time: 时间，可以是单个数字或一个数组
    @param k: 斜率
    @param b: 截距
    @param a: 振幅
    @param omega: 圆速度
    @param phi: 初相位
    @return: 旋转到的位置（弧度制）
    */
double radian(double time, double k, double b, double a, double omega, double phi) 
{
    // 计算能量机关的旋转位置，使用正弦函数来模拟旋转中的振荡部分
    return k * time + b + a * sin(omega * time + phi);
}

    /*
    计算旋转到的位置对应的角度，范围为 [0, 2pi/5)。
    @param orientation: 表示方向的二维向量
    @return: 返回计算的角度，范围在 [0, 2pi/5) 内
    */
double angle_of(cv::Point2f orientation) {
    // 通过 atan2 计算角度
    float x = orientation.x;
    float y = orientation.y;
    return fmod(atan2(y, x), 2 * M_PI / 5);  // 将结果限定在 2pi/5 的范围内
}

// RotationParams 默认构造函数，初始化旋转参数
RotationParams::RotationParams() 
{
    // 初始化默认参数
    k = 0;
    b = 0;
    a = 0.4;  // 振幅
    omega = 1.9;  // 圆速度
    phi = 0;  // 初相位
}

// RotationParams 带参构造函数，允许用户传入自定义参数
RotationParams::RotationParams(double k, double b, double a, double omega, double phi) 
{
    // 初始化参数
    this->k = k;
    this->b = b;
    this->a = a;
    this->omega = omega;
    this->phi = phi;
}

// 打包旋转参数到元组
tuple<double, double, double, double, double> RotationParams::pack() 
{
    // 返回旋转参数的元组
    return make_tuple(k, b, a, omega, phi);
}

// Prediction 构造函数，初始化定时器和旋转参数
Prediction::Prediction() 
{
    start_time = system_clock::now();  // 初始化开始时间
    last_fit_time = system_clock::now();  // 上次拟合时间
    last_update_time = system_clock::now();  // 上次更新时间
    params = RotationParams();  // 初始化旋转参数
}

// 检查当前时间与上次更新的时间差是否在合理范围内
bool Prediction::check_timeliness(system_clock::time_point current_time) {
        duration<double> elapsed_seconds = current_time - last_update_time;
        return elapsed_seconds.count() < 0.3;  // 如果时间差小于 0.3 秒，则认为是及时的
}

// 解开弧度值（仿照 numpy.unwrap），实际未实现
vector<double> Prediction::unwrapped_radians() {
        vector<double> unwrapped = radians;  // 简单占位符，实际的解包逻辑没有实现
        return unwrapped;
}

// 检查是否满足拟合条件（弧度数量超过 50）
bool Prediction::can_fit() 
{
    bool result = radians.size() > 50;
    return result;
}

// 判断是否需要重新拟合旋转参数
bool Prediction::need_fit() {
        duration<double> elapsed_seconds = system_clock::now() - last_fit_time;
        return elapsed_seconds.count() > cfg.refit_delay_sec;  // 如果超过 refit_delay_sec 秒，则需要重新拟合
}

// 拟合旋转参数（占位符）
void Prediction::fit() {
        params = RotationParams();  // 使用默认旋转参数，实际拟合逻辑未实现
        has_fitted = true;  // 标记已进行拟合
        last_fit_time = system_clock::now();  // 更新上次拟合时间
}

// 快速估计旋转的方向
bool Prediction::fast_estimate_sense_of_rotation() {
        double end = radians.back();
        double start = radians.front();
        return end > start;  // 如果末尾的角度大于开始的角度，则方向为正
}

// 根据时间和拟合的旋转参数预测旋转的位置
double Prediction::predict() {
    if (can_fit() && need_fit()) {
        fit();  // 如果可以拟合且需要拟合，则进行拟合
    }

    if (!has_fitted) {
        // 如果未拟合，则使用快速估算来预测角度
        return fast_estimate_sense_of_rotation() ? M_PI / 3 * cfg.hit_delay_sec : -M_PI / 3 * cfg.hit_delay_sec;
    }

    // 使用拟合的参数进行精确预测
    double current = radian(times_sec.back(), params.k, params.b, params.a, params.omega, params.phi);
    double predicted = radian(times_sec.back() + cfg.hit_delay_sec, params.k, params.b, params.a, params.omega, params.phi);
    return predicted - current;
}

// 更新预测器的状态，记录当前时间和弧度
void Prediction::update(cv::Point2f orientation) {//orientation是方向向量矩阵 计算目标与中心之间的向量，用于表示目标与中心的相对方向和距离
    auto current_time = system_clock::now();
    if (!check_timeliness(current_time)) {
        reset();  // 如果更新不及时，则重置预测器
    }

    // 记录当前的角度（弧度）和时间
    radians.push_back(angle_of(orientation));
    duration<double> elapsed_seconds = current_time - start_time;
    times_sec.push_back(elapsed_seconds.count());
    last_update_time = current_time;  // 更新最后一次更新时间
}

// 重置预测器
void Prediction::reset() {
    *this = Prediction();  // 重置为一个新的 Prediction 对象
}
