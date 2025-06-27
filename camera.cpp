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
    
    // 現在才初始化我們的軟體介面
    claw.initialize();


    // --- YOLO and Camera Initialization ---
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

    // --- Main Loop ---
    std::cout << "\n[Info] Synchronizing state with controller...\n";
    // 關鍵：用硬體的真實狀態來初始化我們的軟體狀態變數
    bool is_servo_on = claw.isActuallyOn();
    std::cout << "[Info] Initial synchronization complete.\n";

    std::cout << "\n====================== INSTRUCTIONS ======================\n";
    std::cout << "  - Press [Spacebar]: Toggle Servo ON / OFF\n";
    std::cout << "  - Press [o] / [c]: Open / Close claw (only when Servo is ON)\n";
    std::cout << "  - Press [s]        : Read current status\n";
    std::cout << "  - Press [q]        : Quit program\n";
    std::cout << "========================================================\n\n";
    
    claw.readAndPrintStatus();

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) { continue; }
        
        // 此處應有您的YOLO辨識與繪圖程式碼
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

        if (key == 32) { // 空白鍵切換
            if (is_servo_on) {
                claw.servoOff();
            } else {
                claw.servoOn();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            is_servo_on = claw.isActuallyOn(); // 重新同步
            claw.readAndPrintStatus();
        }
        else if (key == 'c' || key == 'o') { // 移動
            if (claw.isActuallyOn()) {
                claw.moveRelative(key == 'c' ? 2000 : -2000);
                claw.readAndPrintStatus();
            } else {
                std::cout << "[Warning] Servo is OFF. Please press Spacebar to turn it ON first.\n";
            }
        }
        else if (key == 's') { // 狀態查詢
            claw.readAndPrintStatus();
        }
        else if (key == 'q' || key == 27) {
            break;
        }
    }

    // --- Cleanup ---
    detector.stop();
    if (claw.isActuallyOn()) {
        claw.servoOff(); 
    }
    std::cout << "[Info] Program finished.\n";
    
    return 0;
}