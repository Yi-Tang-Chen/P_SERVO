// =============================================================
//          最終的、採用標準解析演算法的 YoloDetector.cpp
// =============================================================
#include "YoloDetector.h"
#include <fstream>
#include <iostream>
#include <vector>

// 輔助函式：檢查檔案是否存在 (保持不變)
inline bool file_exists(const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

// 構造函數 (保持不變)
YoloDetector::YoloDetector(const std::string& onnx_path, const std::string& names_path) {
    model_loaded_successfully_ = false;

    if (!file_exists(onnx_path)) {
        std::cerr << "[Fatal Error] YOLOv8 .onnx file not found at: " << onnx_path << std::endl;
        return;
    }
    if (!file_exists(names_path)) {
        std::cerr << "[Fatal Error] .names file not found at: " << names_path << std::endl;
        return;
    }

    net_ = cv::dnn::readNet(onnx_path);

    std::ifstream ifs(names_path.c_str());
    std::string line;
    while (std::getline(ifs, line)) {
        class_names_.push_back(line);
    }

    if (class_names_.empty()) {
        std::cerr << "[Fatal Error] .names file is empty." << std::endl;
        return;
    }

    std::cout << "[Info] Loaded " << class_names_.size() << " class names." << std::endl;
    is_running_ = false;
    model_loaded_successfully_ = true;
}

// 解構函數和其它輔助函式 (保持不變)
YoloDetector::~YoloDetector() {
    stop();
}
void YoloDetector::start() {
    if (is_running_) return;
    is_running_ = true;
    detector_thread_ = std::thread(&YoloDetector::run, this);
}
void YoloDetector::stop() {
    is_running_ = false;
    if (detector_thread_.joinable()) {
        detector_thread_.join();
    }
}
void YoloDetector::updateFrame(const cv::Mat& frame) {
    std::lock_guard<std::mutex> lock(mtx_);
    current_frame_ = frame.clone();
}
std::vector<Detection> YoloDetector::getDetections() {
    std::lock_guard<std::mutex> lock(mtx_);
    return last_detections_;
}
bool YoloDetector::isLoaded() const {
    return model_loaded_successfully_;
}


// ======================= 核心演算法修正 =======================
void YoloDetector::run() {
    while (is_running_) {
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (current_frame_.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            frame = current_frame_.clone();
            current_frame_.release();
        }

        cv::Mat blob;
        cv::dnn::blobFromImage(frame, blob, 1.0 / 255.0, input_size_, cv::Scalar(), true, false);
        net_.setInput(blob);
        
        std::vector<cv::Mat> outs;
        net_.forward(outs, net_.getUnconnectedOutLayersNames());
        
        // 獲取模型的輸出，這是一個 1x84x8400 的矩陣
        const cv::Mat& output = outs[0];
        // 獲取數據的維度
        const int num_channels = output.size[1]; // 84
        const int num_detections = output.size[2]; // 8400

        std::vector<int> class_ids;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        float x_factor = frame.cols / (float)input_size_.width;
        float y_factor = frame.rows / (float)input_size_.height;

        // 直接遍歷 8400 個潛在偵測結果
        for (int i = 0; i < num_detections; ++i) {
            // 獲取指向當前偵測結果數據塊開頭的指針
            const float* data = output.ptr<float>(0, 0, i);
            
            // 獲取當前偵測結果的類別分數部分 (從第 5 個元素開始)
            const float* scores = data + 4;
            
            // 找到分數最高的那個類別
            float max_score = *scores;
            int class_id = 0;
            for (int j = 1; j < num_channels - 4; ++j) {
                if (*(scores + j) > max_score) {
                    max_score = *(scores + j);
                    class_id = j;
                }
            }

            // 如果最高信心度大於我們設定的閾值
            if (max_score > conf_threshold_) {
                confidences.push_back(max_score);
                class_ids.push_back(class_id);

                // 獲取 cx, cy, w, h
                float cx = *data;
                float cy = *(data + 1);
                float w = *(data + 2);
                float h = *(data + 3);

                // 將座標從 640x640 的比例，轉換回原始影像的像素座標
                int left = static_cast<int>((cx - 0.5 * w) * x_factor);
                int top = static_cast<int>((cy - 0.5 * h) * y_factor);
                int width = static_cast<int>(w * x_factor);
                int height = static_cast<int>(h * y_factor);
                
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);
        
        std::vector<Detection> detections;
        for (int idx : indices) {
            Detection det;
            det.box = boxes[idx];
            det.class_id = class_ids[idx];
            det.confidence = confidences[idx];
            
            if (det.class_id >= 0 && det.class_id < class_names_.size()) {
                det.class_name = class_names_[det.class_id];
            } else {
                det.class_name = "Unknown";
            }
            detections.push_back(det);
        }

        {
            std::lock_guard<std::mutex> lock(mtx_);
            last_detections_ = detections;
        }
    }
}