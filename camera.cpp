#include "clawcontroller.h"
#include "YoloDetector.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // --- 硬體初始化 ---
    RS485Comm comm("COM4", 1);
    if (!comm.openPort(19200)) {
        std::cerr << "[Fatal] Cannot open RS-485 port.\n";
        return -1;
    }
    
    ClawController claw(comm);
    
    // 根據手冊，通電後應等待控制器完成內部初始化
    std::cout << "[Main] Waiting 3 seconds for controller to power up and stabilize...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    claw.initialize();

    // --- YOLO 和攝影機初始化 ---
    YoloDetector detector("yolov8n.onnx", "coco.names");
    if (!detector.isLoaded()) { return -1; }
    detector.start();
    std::cout << "[Info] YOLOv8 detector thread started.\n";
    cv::VideoCapture cap(0, cv::CAP_DSHOW);
    if (!cap.isOpened()) { 
        std::cerr << "[Fatal] Cannot open camera.\n";
        return -1; 
    }
    std::cout << "[Info] Camera open successful.\n";

    // --- 主迴圈 ---
    std::cout << "\n====================== INSTRUCTIONS ======================\n";
    std::cout << "  - Press [Spacebar]: Toggle Servo ON / OFF\n";
    std::cout << "  - Press [o] / [c]: START JOG move + / -\n";
    std::cout << "  - Press [x]        : STOP JOG move\n";
    std::cout << "  - Press [r]        : Clear All Faults (Max Count, Alarms, etc.)\n"; // <--- 更新說明
    std::cout << "  - Press [s]        : Read current status\n";
    std::cout << "  - Press [q] / [ESC]: Quit program\n";
    std::cout << "========================================================\n\n";
    
    // claw.readAndPrintStatus();

    // Changing part
    size_t last_detection_count = 0;


    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) { continue; }
        
        detector.updateFrame(frame);

        std::vector<Detection> detections = detector.getDetections();
        
        if (detections.size() != last_detection_count) {
            std::cout << "[Detector] Found " << detections.size() << " objects." << std::endl;
            last_detection_count = detections.size();
        }
        for (const auto& det : detections) {
            std::cout << "  > Drawing box: Class='" << det.class_name 
            << "', Pos=(" << det.box.x << ", " << det.box.y 
            << "), Size=(" << det.box.width << "x" << det.box.height << ")" << std::endl;
            cv::rectangle(frame, det.box, cv::Scalar(0, 255, 0), 2);
            std::string label = det.class_name + ": " + cv::format("%.2f", det.confidence);
            cv::putText(frame, label, cv::Point(det.box.x, det.box.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }
        
        cv::imshow("YOLOv8 Real-Time Detection", frame);

        int key = cv::waitKey(1);

        if (key == 32) { // 空白鍵切換
            // 直接根據真實狀態來決定執行 ON 還是 OFF
            if (claw.isActuallyOn()) {
                claw.servoOff();
            } else {
                claw.servoOn();
            }
        }
        else if (key == 'o' || key == 'c') { // 開始 JOG
            // 每次都重新獲取最真實的狀態
            if (claw.isActuallyOn()) {
                claw.jogStart(key == 'o'); // 'o' for positive, 'c' for negative
            } else {
                std::cout << "[Warning] Servo is OFF. Please press Spacebar to turn it ON first.\n";
            }
        }
        else if (key == 'x') { // 停止 JOG
            claw.jogStop();
        }
        else if (key == 'r') { // <--- 更新 r 鍵的行為
            claw.clearAllFaults();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            claw.readAndPrintStatus();
        }
        else if (key == 's') { // 狀態查詢
             // 在讀取狀態前先發送停止指令，避免通訊阻塞
            claw.jogStop(); 
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 等待停止完成
            claw.readAndPrintStatus();
        }
        if (key == 'q' || key == 27) {
            break;
        }
    }

    // --- 程式結束前的清理 ---
    detector.stop();
    // 確保在退出前伺服是關閉的
    if (claw.isActuallyOn()) {
        claw.jogStop(); // 先停止任何可能的移動
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        claw.servoOff(); 
    }
    std::cout << "[Info] Program finished.\n";
    
    return 0;
}