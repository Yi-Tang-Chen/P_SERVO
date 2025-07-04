#include "clawcontroller.h"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <limits>

// --- Speed mapping and main function are the same as the previous correct version ---
const uint32_t MIN_SPEED_PPS = 1000;
const uint32_t MAX_SPEED_PPS = 900000;

uint32_t map_percentage_to_pps(int percentage) {
    if (percentage <= 1) return MIN_SPEED_PPS;
    if (percentage >= 100) return MAX_SPEED_PPS;
    double ratio = static_cast<double>(percentage - 1) / 99.0;
    return MIN_SPEED_PPS + static_cast<uint32_t>(ratio * (MAX_SPEED_PPS - MIN_SPEED_PPS));
}

int main() {
    RS485Comm comm("/dev/ttyUSB0", 1); 
    if (!comm.openPort(19200)) {
        std::cerr << "[Fatal] Cannot open serial port.\n";
        return -1;
    }
    
    ClawController claw(comm);
    claw.initialize();
    
    // --- 變數：用來儲存吋動的持續時間 ---
    uint32_t inching_duration_ms = 50; // 預設 50 毫秒

    system("clear");
    std::cout << "\n====================== P-SERVO CONTROLLER (Final Version) ======================\n";
    std::cout << "  (Enter a command character and press Enter)\n\n";
    std::cout << "  [t] Servo ON/OFF  [s] Status      [d] Set Inching Duration (ms)\n"; // <-- 修改說明
    std::cout << "  [o] JOG +         [c] JOG -       [r] Clear Faults  [q] Quit\n";
    std::cout << "  [v] Set Speed (1-100%)\n";
    std::cout << "====================================================================================\n\n";
    
    std::cout << "Initial status check:" << std::endl;
    claw.readAndPrintStatus();
    std::cout << "[Current] Inching Duration: " << inching_duration_ms << " ms" << std::endl;

    char key;
    while (true) {
        std::cout << "\n> Enter command: ";
        std::cin >> key;
        if (std::cin.peek() != '\n' && std::cin.peek() != EOF) {
             std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }

        if (key == 'q') { break; }

        switch (key) {
            case 't':
                if (claw.isActuallyOn()) { claw.servoOff(); } else { claw.servoOn(); }
                break;

            case 'd': { // <-- 修改 'd' 的功能
                std::cout << "  > Current duration is " << inching_duration_ms << " ms. Enter new duration: ";
                std::cin >> inching_duration_ms;
                if (std::cin.fail()) {
                    std::cerr << "[Error] Invalid input." << std::endl;
                    inching_duration_ms = 50; // Reset to default
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                } else {
                    std::cout << "  > Inching duration set to " << inching_duration_ms << " ms." << std::endl;
                }
                break;
            }
            
            case 'v': {
                int percentage;
                std::cout << "  > Enter new speed (1-100%): ";
                std::cin >> percentage;
                if (std::cin.fail() || percentage < 1 || percentage > 100) {
                     std::cerr << "[Error] Invalid input." << std::endl; continue;
                }
                uint32_t speed_pps = map_percentage_to_pps(percentage);
                bool was_on = claw.isActuallyOn();
                if (was_on) { claw.servoOff(); std::this_thread::sleep_for(std::chrono::milliseconds(200)); }
                claw.setHighSpeed(speed_pps);
                if (was_on) { claw.servoOn(); }
                break;
            }

            case 'o':
            case 'c':
                if (claw.isActuallyOn()) {
                    claw.jogStart(key == 'o');
                    std::this_thread::sleep_for(std::chrono::milliseconds(inching_duration_ms));
                    claw.jogStop(); 
                } else {
                    std::cout << "[Warning] Servo is OFF." << std::endl;
                }
                break;

            case 'r':
                claw.clearAllFaults();
                break;

            case 's':
                break; 
            
            default:
                std::cout << "  > Unknown command." << std::endl;
                continue;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 短延遲以等待狀態更新
        std::cout << "\nCurrent Status:" << std::endl;
        claw.readAndPrintStatus();
        std::cout << "[Current] Inching Duration: " << inching_duration_ms << " ms" << std::endl;
    }

    if (claw.isActuallyOn()) { claw.servoOff(); }
    std::cout << "[Info] Program finished.\n";
    return 0;
}
