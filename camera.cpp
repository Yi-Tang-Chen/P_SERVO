#include <iostream>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "RS485Comm.h"
#include "clawcontroller.h"
#include "YoloDetector.h"

int main() {
    // --- 硬體初始化 ---
    RS485Comm comm("COM4", 1); 
    if (!comm.openPort(19200)) {
        std::cerr << "[Fatal] Cannot open RS-485 port.\n";
        return -1;
    }
    
    ClawController claw(comm);
    if (!claw.initialize(20)) { // 初始化時設定 JOG 速度為 20%
        std::cerr << "[Fatal] Claw controller failed to initialize.\n";
        return -1;
    }
    std::cout << "[Setup] All hardware initialized successfully.\n\n";

    // --- YOLO 和攝影機初始化 ---
    YoloDetector detector("yolov8n.onnx", "coco.names");
    if (!detector.isLoaded()) { /* ... */ return -1; }
    detector.start();
    std::cout << "[Info] YOLOv8 detector thread started.\n";
    int camera_backend = cv::CAP_DSHOW; 
    cv::VideoCapture cap(0, camera_backend); 
    if (!cap.isOpened()) { /* ... */ return -1; }
    std::cout << "[Info] Camera open successful\n";
    
    // --- 主迴圈 ---
    std::cout << "\n[Info] System is running. Press 'o' to open a bit, 'c' to close a bit, 's' for status, 'q' to quit.\n";
    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) { continue; }
        
        detector.updateFrame(frame);
        std::vector<Detection> detections = detector.getDetections();
        for (const auto& det : detections) {
             cv::rectangle(frame, det.box, cv::Scalar(0, 255, 0), 2);
             std::string label = det.class_name + ": " + cv::format("%.2f", det.confidence);
             cv::putText(frame, label, cv::Point(det.box.x, det.box.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("YOLOv8 Real-Time Detection", frame);
        
        int key = cv::waitKey(1); 
        if (key == 'c') { // Close a bit
            claw.jogStep(0, 100); // 呼叫寸動函式
            claw.readAndPrintStatus(); // 動作後讀取狀態
        } else if (key == 'o') { // Open a bit
            claw.jogStep(1, 100); // 呼叫寸動函式
            claw.readAndPrintStatus(); // 動作後讀取狀態
        } else if (key == 's') { // 按 's' 手動查詢狀態
            claw.readAndPrintStatus();
        } else if (key == 'q' || key == 27) {
            break;
        }
    }

    // --- 收尾 ---
    detector.stop();
    comm.writeRegister(0x2011, 1); // 結束程式前關閉伺服
    std::cout << "[Info] Program finished.\n";
    
    return 0;
}