// =================================================================
//           最終的 main 函式 (完整版，包含模式切換)
// =================================================================
#include <iostream>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp> // 萬用標頭檔

#include "RS485Comm.h"
#include "clawcontroller.h"
#include "YoloDetector.h"

int main() {
    std::cout << "[Setup] Initializing communication and configuring servo...\n";

    RS485Comm comm("COM4", 1); 
    if (!comm.openPort(19200)) {
        std::cerr << "[Fatal] Cannot open RS-485 port.\n";
        return -1;
    }
    
    // --- 步驟一：切換到「下壓控制模式」(掛檔！) ---
    std::cout << "[Config] Switching to Pushing Control Mode (writing 1 to 0x0503)... ";
    if (!comm.writeRegister(0x0503, 1)) {
        std::cerr << "FAILED. Could not switch control mode. Exiting.\n";
        return -1;
    } else {
        std::cout << "OK.\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 等待模式切換生效

    // --- 步驟二：啟用伺服馬達 ---
    ClawController claw(comm);
    claw.initializeServo();
    std::cout << "[Setup] Servo motor initialization process completed. \n\n";

    // --- 步驟三：啟動影像辨識 (保持不變) ---
    YoloDetector detector("yolov8n.onnx", "coco.names");
    if (!detector.isLoaded()) { /* ... */ return -1; }
    detector.start();
    std::cout << "[Info] YOLOv8 detector thread started.\n";

    // --- 步驟四：開啟攝影機 (保持不變) ---
    int camera_backend = cv::CAP_DSHOW; 
    cv::VideoCapture cap(0, camera_backend); 
    if (!cap.isOpened()) { /* ... */ return -1; }
    std::cout << "[Info] Camera open successful\n";
    
    // --- 步驟五：主迴圈 ---
    std::cout << "\n[Info] System is running. Press 'o' to open, 'c' to close, 'q' to quit.\n";
    while (true) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) { continue; }
        
        // ... (YOLO 辨識與繪圖邏輯保持不變) ...
        detector.updateFrame(frame);
        std::vector<Detection> detections = detector.getDetections();
        for (const auto& det : detections) {
            // 你可以在此處加入篩選，例如 if (det.class_name == "person")
            cv::rectangle(frame, det.box, cv::Scalar(0, 255, 0), 2);
            std::string label = det.class_name + ": " + cv::format("%.2f", det.confidence);
            cv::putText(frame, label, cv::Point(det.box.x, det.box.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }
        
        cv::imshow("YOLOv8 Real-Time Detection", frame);
        int key = cv::waitKey(1); 

        if (key == 'o') {
            claw.setClawState(ClawState::OPEN, 30);
        } else if (key == 'c') {
            claw.setClawState(ClawState::CLOSE, 70);
        } else if (key == 'q' || key == 27) {
            break;
        }
    }

    detector.stop();
    comm.writeRegister(0x2011, 1); // 結束程式前關閉伺服
    std::cout << "[Info] Program finished.\n";
    
    return 0;
}