#include "clawcontroller.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <conio.h>
#include <limits>

int main() {
    // --- 硬體初始化 (維持不變) ---
    RS485Comm comm("COM4", 1);
    if (!comm.openPort(19200)) {
        std::cerr << "[Fatal] Cannot open RS-485 port.\n";
        return -1;
    }
    
    ClawController claw(comm);
    
    std::cout << "[Main] Waiting 3 seconds for controller to power up and stabilize...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    claw.initialize();

    // --- 初始化寸動模式 ---
    uint16_t current_inching_distance = 20;
    uint16_t current_jog_speed = 20; // 預設速度為 20%
    claw.setInchingDistance(current_inching_distance); 
    claw.setJogSpeed(current_jog_speed);

    // --- 主迴圈 ---
    std::cout << "\n====================== INSTRUCTIONS ======================\n";
    std::cout << "          (Press a key to send a command)\n";
    std::cout << "  [t] : Toggle Servo ON/OFF        [s] : Read current status\n";
    std::cout << "  [d] : Set Inching Distance       [v] : Set Inching Speed\n"; // <--- 加入 'v'
    std::cout << "  [o] : Inching Move +             [c] : Inching Move -\n";
    std::cout << "  [r] : Clear All Faults           [q] : Quit program\n";
    std::cout << "========================================================\n\n";
    
    std::cout << "Initial status:" << std::endl;
    claw.readAndPrintStatus();
    std::cout << "\n> Current Inching Distance: " << current_inching_distance << " pulses." << std::endl;
    std::cout << "Waiting for key press..." << std::endl;

    char key;
    while ((key = _getch()) && key != 'q') {
        
        std::cout << "\n> Key '" << key << "' pressed." << std::endl;

        switch (key) {
            case 't': // 切換 Servo 狀態 (維持不變)
                if (claw.isActuallyOn()) {
                    claw.servoOff();
                } else {
                    claw.servoOn();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(150));
                claw.readAndPrintStatus();
                break;

            // ======================= 核心修正點 =======================
            case 'd': { 
                int distance = 0;
                std::cout << "  > Enter new inching distance (e.g., 100, 1000) and press Enter: ";
                std::cin >> distance;

                if (std::cin.fail() || distance < 1) {
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cerr << "[Error] Invalid input. Please enter a positive number." << std::endl;
                    break; 
                }
                
                std::cout << "  > Applying new distance setting..." << std::endl;
                
                // 步驟 1: 記住 Servo 當前的狀態，以便之後恢復
                bool was_on = claw.isActuallyOn();
                
                // 步驟 2: 如果伺服是 ON，為了安全地寫入參數，必須先將其關閉
                if (was_on) {
                    std::cout << "  > Cycling Servo OFF to unlock parameter..." << std::endl;
                    claw.servoOff();
                    std::this_thread::sleep_for(std::chrono::milliseconds(300)); // 等待 Servo 完全斷電
                }
                
                // 步驟 3: 現在處於安全的 Servo OFF 狀態，設定新的距離
                if (claw.setInchingDistance(distance)) {
                    current_inching_distance = distance; // 指令成功，更新我們程式中的變數
                    std::cout << "  > Inching distance set to " << current_inching_distance << " pulses successfully." << std::endl;
                } else {
                    std::cerr << "[Error] Failed to set new distance. Please check connection." << std::endl;
                }

                // 步驟 4: 如果伺服原本是 ON，則將其重新開啟，恢復原狀
                if (was_on) {
                    std::cout << "  > Cycling Servo ON to resume operation..." << std::endl;
                    claw.servoOn();
                    std::this_thread::sleep_for(std::chrono::milliseconds(150));
                }
                
                // 步驟 5: 顯示最終的狀態
                claw.readAndPrintStatus();
                break;
            }
            // ==========================================================
            case 'v': {
                int speed = 0;
                std::cout << "  > Enter new positioning HighSpeed (e.g., 1000, 5000 pps) and press Enter: ";
                std::cin >> speed;
                if (std::cin.fail() || speed < 1) {
                     std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cerr << "[Error] Invalid input. Please enter a positive number for pps." << std::endl;
                    break;
                }

                // 套用與設定距離相同的安全流程
                std::cout << "  > Applying new speed setting..." << std::endl;
                bool was_on = claw.isActuallyOn();
                if (was_on) {
                    std::cout << "  > Cycling Servo OFF to unlock parameter..." << std::endl;
                    claw.servoOff();
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));
                }

                if (claw.setJogSpeed(speed)) {
                    std::cout << "  > Positioning HighSpeed set to " << speed << " pps successfully." << std::endl;
                } else {
                    std::cerr << "[Error] Failed to set new speed. Please check connection." << std::endl;
                }
                
                if (was_on) {
                    std::cout << "  > Cycling Servo ON to resume operation..." << std::endl;
                    claw.servoOn();
                    std::this_thread::sleep_for(std::chrono::milliseconds(150));
                }
                claw.readAndPrintStatus();
                break;
            }
            case 'o':
            case 'c': // (寸動邏輯維持不變)
                if (claw.isActuallyOn()) {
                    claw.jogStart(key == 'o'); 
                    claw.jogStop();
                } else {
                    std::cout << "[Warning] Servo is OFF. Please press 't' to turn it ON first.\n";
                }
                break;

            case 'r': // (維持不變)
                claw.clearAllFaults();
                std::this_thread::sleep_for(std::chrono::milliseconds(150));
                claw.readAndPrintStatus();
                break;

            case 's': // (維持不變)
                claw.readAndPrintStatus(); 
                break;
            
            default:
                std::cout << "  > Unknown command." << std::endl;
                break;
        }
        std::cout << "Waiting for key press..." << std::endl;
    }

    // --- 程式結束前的清理 (維持不變) ---
    if (claw.isActuallyOn()) {
        claw.servoOff(); 
    }
    std::cout << "\n[Info] Program finished.\n";
    
    return 0;
}