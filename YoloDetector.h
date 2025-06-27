// =============================================================
//               最終的 YoloDetector.h (YOLOv8 版本)
// =============================================================
#ifndef YOLODETECTOR_H
#define YOLODETECTOR_H

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>

// 儲存單一辨識結果的結構
struct Detection {
    int class_id;
    float confidence;
    cv::Rect box;
    std::string class_name;
};

class YoloDetector {
public:
    // 構造函數：只需要 ONNX 模型路徑和名稱路徑
    YoloDetector(const std::string& onnx_path, const std::string& names_path);
    ~YoloDetector();

    void start();
    void stop();
    void updateFrame(const cv::Mat& frame);
    std::vector<Detection> getDetections();
    bool isLoaded() const;

private:
    void run();

    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
    cv::Size input_size_ = cv::Size(640, 640);
    float conf_threshold_ = 0.25; // 您可以調整這個信心度閾值
    float nms_threshold_ = 0.5;   // 您可以調整這個 NMS 閾值
    
    // 執行緒同步相關
    std::thread detector_thread_;
    std::mutex mtx_;
    cv::Mat current_frame_;
    std::vector<Detection> last_detections_;
    std::atomic<bool> is_running_;
    bool model_loaded_successfully_;
};

#endif // YOLODETECTOR_H