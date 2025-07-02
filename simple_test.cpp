#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "RS485Comm.h" // 我們仍然需要這個通訊基礎設施

// 專門用來讀取和打印狀態的函式
void readAndPrintStatus(RS485Comm& comm) {
    uint16_t motion_status = 99, alarm_status = 99, servo_status = 99, error_status = 99;
    
    // 讀取四個核心狀態暫存器
    comm.readRegister(0x1000, motion_status);
    comm.readRegister(0x1005, alarm_status);
    comm.readRegister(0x1022, servo_status);
    comm.readRegister(0x1023, error_status);

    std::cout << "[Status] "
              << "Motion: " << (motion_status == 0 ? "Stopped" : (motion_status == 1 ? "Moving" : "Alarm Stop"))
              << " | Alarm: " << (alarm_status == 0 ? "No Alarm" : "ALARM")
              << " | Servo: " << (servo_status == 0 ? "ON" : "OFF")
              << " | Error: " << (error_status == 0 ? "No Error" : "ERROR")
              << std::endl;
}

int main() {
    // --- 1. 初始化通訊 ---
    RS485Comm comm("COM4", 1); // 請確認您的 COM Port 和 Slave ID
    if (!comm.openPort(19200)) {
        std::cerr << "[Fatal] Cannot open RS-485 port." << std::endl;
        return -1;
    }

    std::cout << "Minimal Servo Controller" << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << "Initial Status:" << std::endl;
    readAndPrintStatus(comm);
    std::cout << "------------------------" << std::endl;
    
    std::cout << "Commands:" << std::endl;
    std::cout << "  1 : Servo ON (Direct command to 0x2011)" << std::endl;
    std::cout << "  0 : Servo OFF (Direct command to 0x2011)" << std::endl;
    std::cout << "  r : Reset Alarm (CMD 7)" << std::endl;
    std::cout << "  c : Clear Deviation Counter (CMD 8)" << std::endl;
    std::cout << "  s : Read Status" << std::endl;
    std::cout << "  q : Quit" << std::endl;
    std::cout << "------------------------" << std::endl;

    char key;
    while (std::cin >> key && key != 'q') {
        bool success = false;
        switch (key) {
            case '1':
                std::cout << "\nSending Servo ON..." << std::endl;
                success = comm.executeAction(0x2011, 0); // 直接 Servo ON 指令
                break;
            case '0':
                std::cout << "\nSending Servo OFF..." << std::endl;
                success = comm.executeAction(0x2011, 1); // 直接 Servo OFF 指令
                break;
            case 'r':
                std::cout << "\nSending Alarm Reset..." << std::endl;
                success = comm.executeAction(0x201E, 7); // 動作執行：重置警報
                break;
            case 'c':
                std::cout << "\nSending Clear Deviation Counter..." << std::endl;
                success = comm.executeAction(0x201E, 8); // 動作執行：清除偏差計數
                break;
            case 's':
                 std::cout << "\nReading current status..." << std::endl;
                 success = true;
                 break;
            default:
                std::cout << "Unknown command." << std::endl;
                continue;
        }

        if (!success) {
            std::cerr << "Command failed to send!" << std::endl;
        }

        // 每次操作後都等待一小段時間並回報狀態
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        readAndPrintStatus(comm);
    }

    std::cout << "Program finished." << std::endl;
    comm.closePort();
    return 0;
}