// =============================================================
//               最終的 YoloDetector.cpp (YOLOv8 版本)
// =============================================================
#include "YoloDetector.h"
#include <fstream>
#include <iostream>

// 輔助函式：檢查檔案是否存在
inline bool file_exists(const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

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
    // net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    // net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

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
        
        cv::Mat mat(outs[0].size[1], outs[0].size[2], CV_32F, outs[0].ptr<float>());
        cv::Mat data = mat.t();

        std::vector<int> class_ids;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        float x_factor = frame.cols / (float)input_size_.width;
        float y_factor = frame.rows / (float)input_size_.height;

        for (int i = 0; i < data.rows; ++i) {
            cv::Mat scores = data.row(i).colRange(4, data.cols);
            cv::Point class_id_point;
            double max_conf;
            cv::minMaxLoc(scores, 0, &max_conf, 0, &class_id_point);

            if (max_conf > conf_threshold_) {
                confidences.push_back((float)max_conf);
                class_ids.push_back(class_id_point.x);

                float cx = data.at<float>(i, 0);
                float cy = data.at<float>(i, 1);
                float w = data.at<float>(i, 2);
                float h = data.at<float>(i, 3);

                int left = (int)((cx - 0.5 * w) * x_factor);
                int top = (int)((cy - 0.5 * h) * y_factor);
                int width = (int)(w * x_factor);
                int height = (int)(h * y_factor);
                
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
            det.class_name = class_names_[det.class_id];
            detections.push_back(det);
        }

        {
            std::lock_guard<std::mutex> lock(mtx_);
            last_detections_ = detections;
        }
    }
}