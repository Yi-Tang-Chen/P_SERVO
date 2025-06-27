// camera.cpp - 英文手動控制版
#include "clawcontroller.h"
#include "YoloDetector.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // --- Hardware Initialization ---
    RS485Comm comm("COM4", 1);
    if (!comm.openPort(19200)) {
        std::cerr << "[Fatal] Cannot open RS-485 port.\n";
        return -1;
    }
    
    ClawController claw(comm);
    if (!claw.initialize()) {
        std::cerr << "[Fatal] Claw controller failed to initialize.\n";
        return -1;
    }

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
    bool is_servo_on = claw.isActuallyOn(); 
    std::cout << "[Info] Initial synchronization complete.\n";
    const int16_t move_distance = 2000;

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
        
        // ... (YOLO detection and drawing code remains the same) ...
        cv::imshow("Real-Time Detection", frame);

        int key = cv::waitKey(1);

        if (key == 32) { // 空白鍵切換伺服
            if (is_servo_on) {
                claw.servoOff();
            } else {
                claw.servoOn();
            }
            // 切換後，等待一下再同步狀態
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            is_servo_on = claw.isActuallyOn(); // 重新從硬體同步狀態
            claw.readAndPrintStatus();
        }
        else if (key == 'c' || key == 'o') { // 打開或關閉
            // 在移動前，永遠以最新的硬體狀態為準
            if (claw.isActuallyOn()) {
                claw.moveRelative(key == 'c' ? move_distance : -move_distance);
                claw.readAndPrintStatus();
            } else {
                std::cout << "[Warning] Servo is OFF. Please press Spacebar to turn it ON first.\n";
            }
        }
        else if (key == 's') { // Status
            claw.readAndPrintStatus();
        } else if (key == 'q' || key == 27) {
            break;
        }
    }

    // --- Cleanup ---
    detector.stop();
    if (is_servo_on) {
        claw.servoOff(); 
    }
    std::cout << "[Info] Program finished.\n";
    
    return 0;
}