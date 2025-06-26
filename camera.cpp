#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp> 

#include "RS485Comm.h"
#include "clawcontroller.h"

int main() {
    // // Claw
    // RS485Comm comm("COM5", 0); // port 
    // if (!comm.openPort(19200)) {
    //     std::cerr << "[Fatal] can not open RS-485 port. \n";
    //     return -1;
    // }
    // ClawController claw(comm);
    // claw.initializeServo();
    // std::cout << "[Info] Servo motor initialization completed. \n";


    // Camera
    int camera_backend = cv::CAP_DSHOW; 
    cv::VideoCapture cap(0, camera_backend); 

    if (!cap.isOpened()) {
        std::cerr << "ERROR! Can not open camera\n";
        return -1;
    }
    std::cout << "[Info] Camera open sucessfull\n";


    while (true) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) {
            std::cerr << "[Warning] Captured empty frame, skipping...\n";
            continue;
        }
        
        // show the vision on the screen 
        cv::imshow("Camera Feed - Press 'o' to open, 'c' to close, 'q' to quit", frame);
        int key = cv::waitKey(1); 

        if (key == 'q' || key == 27) {
            break;
        }
        // Press keyboard
        // int key = cv::waitKey(1); 

        // if (key == 'o') { // Press 'o' to open claw
        //     std::cout << "[Action] Open claw ...\n";
        //     claw.moveClaw(0, 1000); 
        // } else if (key == 'c') { // Press 'c' to close claw
        //     std::cout << "[Action] CLose claw ...\n";
        //     claw.moveClaw(1, 500);
        // } else if (key == 'q' || key == 27) { // Press 'q' or ESC to leave
        //     break;
        // }
    }

    // End part
    // comm.closePort(); // Close port
    std::cout << "[Info] Finish\n";
    
    return 0;
}