#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "RS485Comm.h" // 請確保 RS485Comm.cpp 是使用 10H 功能碼的版本

// 函式：讀取並打印狀態
void readAndPrintStatus(RS485Comm& comm) {
    uint16_t motion, alarm, servo, error;
    comm.readRegister(0x1000, motion);
    comm.readRegister(0x1005, alarm);
    comm.readRegister(0x1022, servo);
    comm.readRegister(0x1023, error);
    std::cout << "[Status] Motion: " << (motion == 0 ? "Stopped" : (motion == 1 ? "Moving" : "Alarm Stop"))
              << " | Alarm: " << (alarm == 0 ? "No Alarm" : "ALARM")
              << " | Servo: " << (servo == 0 ? "ON" : "OFF")
              << " | Error: " << (error == 1 ? "Busy Error" : (error == 0 ? "No Error" : "ERROR"))
              << std::endl;
}

int main() {
    RS485Comm comm("COM4", 1);
    if (!comm.openPort(19200)) {
        std::cerr << "[Fatal] Cannot open RS-485 port." << std::endl;
        return -1;
    }

    std::cout << "====================================================\n";
    std::cout << "        The TRUE Logic P-SERVO Control Test\n";
    std::cout << "====================================================\n\n";
    
    std::cout << "Initial Status on connect:" << std::endl;
    readAndPrintStatus(comm);
    std::cout << "----------------------------------------------------\n";
    std::cout << "Instructions (Reflecting TRUE discovered logic):\n";
    std::cout << "  1 : SERVO ON  (Writes 1 to 0x2011)\n"; // 說明修正
    std::cout << "  0 : SERVO OFF (Writes 0 to 0x2011)\n"; // 說明修正
    std::cout << "  s : Read Status.\n";
    std::cout << "  q : Quit.\n";
    std::cout << "----------------------------------------------------\n";

    char key;
    while (std::cin >> key && key != 'q') {
        switch (key) {
            case '1': // 行為修正
                std::cout << "\nSending TRUE Servo ON (writing 1 to 0x2011)..." << std::endl;
                comm.writeRegister(0x2011, 1);
                break;
            case '0': // 行為修正
                std::cout << "\nSending TRUE Servo OFF (writing 0 to 0x2011)..." << std::endl;
                comm.writeRegister(0x2011, 0);
                break;
            case 's':
                 std::cout << "\nReading current status..." << std::endl;
                 break;
            default:
                std::cout << "Unknown command." << std::endl;
                continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        readAndPrintStatus(comm);
    }

    std::cout << "\nProgram finished." << std::endl;
    comm.closePort();
    return 0;
}