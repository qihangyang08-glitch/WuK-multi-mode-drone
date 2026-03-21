/**
 * WuK SITL Data Analyzer v2.0
 * 
 * 功能：
 * 1. 解析SITL输出的JSONL格式日志 (wuk_gb.jsonl)
 * 2. 计算电机推力方向相对机体roll轴的角度变化
 * 3. 检测变构开始时刻和机体快速下降时刻
 * 4. 应用平滑滤波消除PID控制引起的抖动
 * 5. 输出CSV供绘图工具使用
 * 
 * 数据流：wuk_gb.jsonl → 解析 → 计算 → 平滑 → wuk_analysis.csv
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>

// 简单的JSON解析辅助函数
std::string extract_json_value(const std::string& json, const std::string& key) {
    std::string search_key = "\"" + key + "\":";
    size_t pos = json.find(search_key);
    if (pos == std::string::npos) return "";
    
    pos += search_key.length();
    while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
    
    if (json[pos] == '"') {
        // 字符串值
        size_t end = json.find('"', pos + 1);
        return json.substr(pos + 1, end - pos - 1);
    } else if (json[pos] == '[') {
        // 数组值
        size_t end = json.find(']', pos);
        return json.substr(pos, end - pos + 1);
    } else if (json[pos] == '{') {
        // 对象值
        int depth = 1;
        size_t end = pos + 1;
        while (end < json.length() && depth > 0) {
            if (json[end] == '{') depth++;
            else if (json[end] == '}') depth--;
            end++;
        }
        return json.substr(pos, end - pos);
    } else {
        // 数值或布尔值
        size_t end = pos;
        while (end < json.length() && json[end] != ',' && json[end] != '}' && json[end] != ']') end++;
        return json.substr(pos, end - pos);
    }
}

// 解析JSON数组为float向量
std::vector<float> parse_json_array(const std::string& array_str) {
    std::vector<float> result;
    if (array_str.empty() || array_str[0] != '[') return result;
    
    size_t pos = 1;
    while (pos < array_str.length()) {
        while (pos < array_str.length() && (array_str[pos] == ' ' || array_str[pos] == ',')) pos++;
        if (pos >= array_str.length() || array_str[pos] == ']') break;
        
        size_t end = pos;
        while (end < array_str.length() && array_str[end] != ',' && array_str[end] != ']') end++;
        
        try {
            float val = std::stof(array_str.substr(pos, end - pos));
            result.push_back(val);
        } catch (...) {
            break;
        }
        pos = end;
    }
    return result;
}

// 数据点结构
struct DataPoint {
    double time_s;
    float fc_angle;          // 飞控设定的变构角度
    bool morph_active;       // 是否处于MORPH模式
    
    // 姿态（相对世界坐标系）
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    
    // 电机推力向量（机体坐标系）
    float motor_vx[4];
    float motor_vy[4];
    float motor_vz[4];
    
    // 电机推力大小
    float motor_thrust[4];
    
    // 高度
    float altitude_m;
    
    // 导出字段（计算得出）
    float motor_tilt_roll[4];        // 电机推力在roll轴的倾角（相对机体）
    float motor_tilt_world_roll[4];  // 电机推力在roll轴的倾角（相对世界）
    float total_thrust;
    float vertical_thrust;
    
    // 事件标记
    bool morph_start_detected;
    bool altitude_drop_detected;
};

// 高斯平滑滤波器
class GaussianSmoother {
public:
    GaussianSmoother(int window_size = 5, float sigma = 1.0f) 
        : window_size_(window_size), sigma_(sigma) {
        generate_kernel();
    }
    
    std::vector<float> smooth(const std::vector<float>& data) {
        std::vector<float> result(data.size());
        int half_window = window_size_ / 2;
        
        for (size_t i = 0; i < data.size(); i++) {
            float sum = 0.0f;
            float weight_sum = 0.0f;
            
            for (int j = -half_window; j <= half_window; j++) {
                int idx = i + j;
                if (idx >= 0 && idx < (int)data.size()) {
                    float weight = kernel_[j + half_window];
                    sum += data[idx] * weight;
                    weight_sum += weight;
                }
            }
            
            result[i] = (weight_sum > 0) ? (sum / weight_sum) : data[i];
        }
        
        return result;
    }
    
private:
    int window_size_;
    float sigma_;
    std::vector<float> kernel_;
    
    void generate_kernel() {
        kernel_.resize(window_size_);
        int half = window_size_ / 2;
        float sum = 0.0f;
        
        for (int i = 0; i < window_size_; i++) {
            int x = i - half;
            kernel_[i] = expf(-0.5f * (x * x) / (sigma_ * sigma_));
            sum += kernel_[i];
        }
        
        // 归一化
        for (int i = 0; i < window_size_; i++) {
            kernel_[i] /= sum;
        }
    }
};

// 解析JSONL文件
std::vector<DataPoint> parse_jsonl(const std::string& filename) {
    std::vector<DataPoint> data;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "错误: 无法打开文件 " << filename << std::endl;
        return data;
    }
    
    std::string line;
    int line_num = 0;
    
    while (std::getline(file, line)) {
        line_num++;
        if (line.empty()) continue;
        
        DataPoint pt;
        
        // 基本时间信息
        std::string elapsed_s = extract_json_value(line, "elapsed_s");
        if (elapsed_s.empty()) continue;
        pt.time_s = std::stod(elapsed_s);
        
        // 飞控角度和模式
        std::string fc_angle = extract_json_value(line, "fc_angle");
        pt.fc_angle = fc_angle.empty() ? 0.0f : std::stof(fc_angle);
        
        std::string morph_active = extract_json_value(line, "fc_morph_active");
        pt.morph_active = (morph_active == "true");
        
        // 姿态
        std::string attitude_obj = extract_json_value(line, "attitude_deg");
        if (!attitude_obj.empty()) {
            pt.roll_deg = std::stof(extract_json_value(attitude_obj, "roll"));
            pt.pitch_deg = std::stof(extract_json_value(attitude_obj, "pitch"));
            pt.yaw_deg = std::stof(extract_json_value(attitude_obj, "yaw"));
        }
        
        // 高度
        std::string per_motor_obj = extract_json_value(line, "per_motor");
        std::string altitude = extract_json_value(per_motor_obj, "altitude_m");
        pt.altitude_m = altitude.empty() ? 0.0f : std::stof(altitude);
        
        // 电机推力向量
        std::string vx_str = extract_json_value(per_motor_obj, "vx");
        std::string vy_str = extract_json_value(per_motor_obj, "vy");
        std::string vz_str = extract_json_value(per_motor_obj, "vz");
        
        auto vx = parse_json_array(vx_str);
        auto vy = parse_json_array(vy_str);
        auto vz = parse_json_array(vz_str);
        
        if (vx.size() >= 4 && vy.size() >= 4 && vz.size() >= 4) {
            for (int i = 0; i < 4; i++) {
                pt.motor_vx[i] = vx[i];
                pt.motor_vy[i] = vy[i];
                pt.motor_vz[i] = vz[i];
            }
        }
        
        // 电机推力大小
        std::string thrust_str = extract_json_value(per_motor_obj, "thrust_N");
        auto thrust = parse_json_array(thrust_str);
        if (thrust.size() >= 4) {
            for (int i = 0; i < 4; i++) {
                pt.motor_thrust[i] = thrust[i];
            }
        }
        
        // 计算总推力和垂直推力
        pt.total_thrust = 0.0f;
        pt.vertical_thrust = 0.0f;
        for (int i = 0; i < 4; i++) {
            pt.total_thrust += pt.motor_thrust[i];
            pt.vertical_thrust += -pt.motor_vz[i];  // SITL中-z为向上
        }
        
        // 计算电机推力在roll轴的倾角
        for (int i = 0; i < 4; i++) {
            // 相对机体：推力向量在xz平面的投影
            float horizontal = fabsf(pt.motor_vx[i]);
            float vertical = fabsf(pt.motor_vz[i]);
            pt.motor_tilt_roll[i] = atan2f(horizontal, vertical) * 180.0f / M_PI;
            
            // 相对世界坐标系：加上机体roll角
            pt.motor_tilt_world_roll[i] = pt.motor_tilt_roll[i] + pt.roll_deg;
        }
        
        // 检测变构开始（从false到true的跳变）
        std::string events_obj = extract_json_value(line, "events");
        std::string morph_start = extract_json_value(events_obj, "morph_start");
        pt.morph_start_detected = (morph_start == "true");
        
        pt.altitude_drop_detected = false;  // 后续计算
        
        data.push_back(pt);
    }
    
    file.close();
    std::cout << "✓ 解析完成: " << data.size() << " 个数据点" << std::endl;
    return data;
}

// 检测高度快速下降
void detect_altitude_drop(std::vector<DataPoint>& data, float threshold_m_per_s = -1.5f) {
    if (data.size() < 2) return;
    
    for (size_t i = 1; i < data.size(); i++) {
        float dt = data[i].time_s - data[i-1].time_s;
        if (dt > 0.001f) {
            float descent_rate = (data[i].altitude_m - data[i-1].altitude_m) / dt;
            if (descent_rate < threshold_m_per_s) {
                data[i].altitude_drop_detected = true;
            }
        }
    }
}

// 输出CSV
void write_csv(const std::string& filename, const std::vector<DataPoint>& data) {
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "错误: 无法创建文件 " << filename << std::endl;
        return;
    }
    
    // CSV头
    file << "time_s,fc_angle,morph_active,"
         << "roll_deg,pitch_deg,yaw_deg,"
         << "altitude_m,"
         << "m1_tilt_body,m2_tilt_body,m3_tilt_body,m4_tilt_body,"
         << "m1_tilt_world,m2_tilt_world,m3_tilt_world,m4_tilt_world,"
         << "m1_thrust,m2_thrust,m3_thrust,m4_thrust,"
         << "total_thrust,vertical_thrust,"
         << "morph_start,altitude_drop\n";
    
    // 数据行
    for (const auto& pt : data) {
        file << std::fixed << std::setprecision(6)
             << pt.time_s << ","
             << pt.fc_angle << ","
             << (pt.morph_active ? 1 : 0) << ","
             << pt.roll_deg << ","
             << pt.pitch_deg << ","
             << pt.yaw_deg << ","
             << pt.altitude_m << ",";
        
        // 电机倾角（相对机体）
        for (int i = 0; i < 4; i++) {
            file << pt.motor_tilt_roll[i];
            if (i < 3) file << ",";
        }
        file << ",";
        
        // 电机倾角（相对世界）
        for (int i = 0; i < 4; i++) {
            file << pt.motor_tilt_world_roll[i];
            if (i < 3) file << ",";
        }
        file << ",";
        
        // 电机推力
        for (int i = 0; i < 4; i++) {
            file << pt.motor_thrust[i];
            if (i < 3) file << ",";
        }
        file << ",";
        
        file << pt.total_thrust << ","
             << pt.vertical_thrust << ","
             << (pt.morph_start_detected ? 1 : 0) << ","
             << (pt.altitude_drop_detected ? 1 : 0) << "\n";
    }
    
    file.close();
    std::cout << "✓ CSV文件已保存: " << filename << std::endl;
}

// 应用平滑滤波
void apply_smoothing(std::vector<DataPoint>& data) {
    if (data.size() < 5) return;
    
    GaussianSmoother smoother(7, 1.5f);  // 窗口7，sigma=1.5
    
    // 收集需要平滑的数据列
    std::vector<float> roll_vals, pitch_vals, yaw_vals;
    std::vector<std::vector<float>> motor_tilt_body(4), motor_tilt_world(4), motor_thrust(4);
    std::vector<float> total_thrust_vals, vertical_thrust_vals, altitude_vals;
    
    for (const auto& pt : data) {
        roll_vals.push_back(pt.roll_deg);
        pitch_vals.push_back(pt.pitch_deg);
        yaw_vals.push_back(pt.yaw_deg);
        altitude_vals.push_back(pt.altitude_m);
        total_thrust_vals.push_back(pt.total_thrust);
        vertical_thrust_vals.push_back(pt.vertical_thrust);
        
        for (int i = 0; i < 4; i++) {
            motor_tilt_body[i].push_back(pt.motor_tilt_roll[i]);
            motor_tilt_world[i].push_back(pt.motor_tilt_world_roll[i]);
            motor_thrust[i].push_back(pt.motor_thrust[i]);
        }
    }
    
    // 平滑处理
    auto roll_smooth = smoother.smooth(roll_vals);
    auto pitch_smooth = smoother.smooth(pitch_vals);
    auto yaw_smooth = smoother.smooth(yaw_vals);
    auto altitude_smooth = smoother.smooth(altitude_vals);
    auto total_thrust_smooth = smoother.smooth(total_thrust_vals);
    auto vertical_thrust_smooth = smoother.smooth(vertical_thrust_vals);
    
    std::vector<std::vector<float>> motor_tilt_body_smooth(4), motor_tilt_world_smooth(4), motor_thrust_smooth(4);
    for (int i = 0; i < 4; i++) {
        motor_tilt_body_smooth[i] = smoother.smooth(motor_tilt_body[i]);
        motor_tilt_world_smooth[i] = smoother.smooth(motor_tilt_world[i]);
        motor_thrust_smooth[i] = smoother.smooth(motor_thrust[i]);
    }
    
    // 以首样本为零基准，避免绝对海拔过高导致图表上下文偏移
    const float alt_baseline = altitude_smooth.empty() ? 0.0f : altitude_smooth.front();

    // 写回数据
    for (size_t idx = 0; idx < data.size(); idx++) {
        data[idx].roll_deg = roll_smooth[idx];
        data[idx].pitch_deg = pitch_smooth[idx];
        data[idx].yaw_deg = yaw_smooth[idx];
        data[idx].altitude_m = altitude_smooth[idx] - alt_baseline;  // 相对高度，典型范围±10 m
        data[idx].total_thrust = total_thrust_smooth[idx];
        data[idx].vertical_thrust = vertical_thrust_smooth[idx];
        
        for (int i = 0; i < 4; i++) {
            data[idx].motor_tilt_roll[i] = motor_tilt_body_smooth[i][idx];
            data[idx].motor_tilt_world_roll[i] = motor_tilt_world_smooth[i][idx];
            data[idx].motor_thrust[i] = motor_thrust_smooth[i][idx];
        }
    }
    
    std::cout << "✓ 高斯平滑完成（窗口=7, σ=1.5），高度已零基准化，基准高度为" << alt_baseline << " 米" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "用法: " << argv[0] << " <input.jsonl> <output.csv>\n";
        std::cerr << "示例: " << argv[0] << " wuk_gb.jsonl wuk_analysis.csv\n";
        return 1;
    }
    
    std::string input_file = argv[1];
    std::string output_file = argv[2];
    
    std::cout << "=== WuK SITL数据分析器 v2.0 ===" << std::endl;
    std::cout << "输入: " << input_file << std::endl;
    
    // 1. 解析JSONL
    auto data = parse_jsonl(input_file);
    if (data.empty()) {
        std::cerr << "错误: 未解析到有效数据" << std::endl;
        return 1;
    }
    
    // 2. 检测高度快速下降
    detect_altitude_drop(data, -0.5f);  // 下降速度超过0.5 m/s
    
    // 3. 应用平滑滤波
    apply_smoothing(data);
    
    // 4. 统计信息
    float morph_start_time = -1.0f;
    float altitude_drop_time = -1.0f;
    for (const auto& pt : data) {
        if (morph_start_time < 0 && pt.morph_start_detected) {
            morph_start_time = pt.time_s;
        }
        if (altitude_drop_time < 0 && pt.altitude_drop_detected) {
            altitude_drop_time = pt.time_s;
        }
    }
    
    std::cout << "\n--- 事件检测 ---" << std::endl;
    if (morph_start_time >= 0) {
        std::cout << "✓ 变构开始: " << morph_start_time << " 秒" << std::endl;
    } else {
        std::cout << "⚠ 未检测到变构开始事件" << std::endl;
    }
    
    if (altitude_drop_time >= 0) {
        std::cout << "✓ 高度快速下降: " << altitude_drop_time << " 秒" << std::endl;
    } else {
        std::cout << "✓ 未检测到快速下降（高度控制正常）" << std::endl;
    }
    
    // 5. 输出CSV
    write_csv(output_file, data);
    
    std::cout << "\n✅ 分析完成！" << std::endl;
    std::cout << "输出: " << output_file << std::endl;
    
    return 0;
}
