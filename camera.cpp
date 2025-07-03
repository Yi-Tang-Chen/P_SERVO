#include "clawcontroller.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <conio.h> // 使用 _getch() 實現按鍵式輸入
#include <limits>

// --- 新增：速度映射函式 ---
// 我們定義一個合理的速度範圍 (pps)，並將 1-100% 映射到這個範圍內
const uint32_t MIN_SPEED_PPS = 1000;    // 最低速度 (1%)
const uint32_t MAX_SPEED_PPS = 900000;  // 最高速度 (100%)

uint32_t map_percentage_to_pps(int percentage) {
    if (percentage <= 1) return MIN_SPEED_PPS;
    if (percentage >= 100) return MAX_SPEED_PPS;
    // 線性映射公式
    return MIN_SPEED_PPS + static_cast<uint32_t>(((MAX_SPEED_PPS - MIN_SPEED_PPS) / 99.0) * (percentage - 1));
}

int main() {
    // --- 1. 初始化通訊 (維持不變) ---
    RS485Comm comm("COM4", 1);
    if (!comm.openPort(19200)) {
        std::cerr << "[Fatal] Cannot open RS-485 port.\n";
        return -1;
    }
    
    ClawController claw(comm);
    
    std::cout << "[Main] Waiting for controller to stabilize...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    claw.initialize();

    // --- 2. 顯示操作說明 (更新) ---
    std::cout << "\n====================== P-SERVO DYNAMIC CONTROLLER ======================\n";
    std::cout << "  (Press a key to send a command)\n\n";
    std::cout << "  [t] : Toggle Servo ON/OFF        [s] : Read current status\n";
    std::cout << "  [d] : Set Inching Distance       [v] : Set Speed (1-100%)\n"; // <--- 更新說明
    std::cout << "  [o] : Inching Move +             [c] : Inching Move -\n";
    std::cout << "  [r] : Clear All Faults           [q] : Quit program\n";
    std::cout << "========================================================================\n\n";
    
    std::cout << "Initial status check:" << std::endl;
    claw.readAndPrintStatus();
    std::cout << "\nWaiting for key press..." << std::endl;

    // --- 3. 主操作迴圈 (改回按鍵式) ---
    char key;
    while ((key = _getch()) && key != 'q') {
        
        std::cout << "\n> Key '" << key << "' pressed." << std::endl;

        switch (key) {
            case 't': // 切換伺服狀態
                if (claw.isActuallyOn()) { claw.servoOff(); } else { claw.servoOn(); }
                break;

            case 'd': { // 動態設定距離
                uint32_t distance = 0;
                std::cout << "  > Enter new distance:  ";
                std::cin >> distance;

                if (std::cin.fail() || distance < 0) {
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cerr << "[Error] Invalid input." << std::endl;
                    break; 
                }
                
                std::cout << "  > Applying new distance setting..." << std::endl;
                
                bool was_on = claw.isActuallyOn();
                if (was_on) { claw.servoOff(); std::this_thread::sleep_for(std::chrono::milliseconds(200)); }
                
                if (claw.setInchingDistance(distance)) {
                    std::cout << "  > Inching distance set successfully." << std::endl;
                } else {
                    std::cerr << "[Error] Failed to set new distance." << std::endl;
                }

                if (was_on) { claw.servoOn(); }
                break;
            }
            
            case 'v': { // 動態設定速度 (使用百分比)
                int percentage = 0;
                std::cout << "  > Enter new speed (1-100%) : ";
                std::cin >> percentage;

                if (std::cin.fail() || percentage < 1 || percentage > 100) {
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cerr << "[Error] Invalid input. Please enter a number between 1 and 100." << std::endl;
                    break;
                }

                // 將百分比轉換為實際的 pps 數值
                uint32_t speed_pps = map_percentage_to_pps(percentage);
                
                std::cout << "  > Applying new speed setting (" << percentage << "% -> " << speed_pps << " pps)..." << std::endl;
                bool was_on = claw.isActuallyOn();
                if (was_on) { claw.servoOff(); std::this_thread::sleep_for(std::chrono::milliseconds(200)); }

                if (claw.setJogSpeed(speed_pps)) {
                    std::cout << "  > Positioning HighSpeed set successfully." << std::endl;
                } else {
                    std::cerr << "[Error] Failed to set new speed." << std::endl;
                }
                
                if (was_on) { claw.servoOn(); }
                break;
            }

            case 'o': // JOG 正向移動
            case 'c': // JOG 反向移動
                if (claw.isActuallyOn()) {
                    claw.jogStart(key == 'o');
                    claw.jogStop();
                } 
                break;

            case 'r': // 清除所有故障
                claw.clearAllFaults();
                break;

            case 's': // 讀取目前狀態
                claw.readAndPrintStatus();
                break; 
            
            default:
                continue;
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(250));
        // claw.readAndPrintStatus();
        // std::cout << "\nWaiting for key press..." << std::endl;
    }

    // --- 4. 結束程式前的清理 ---
    if (claw.isActuallyOn()) {
        claw.servoOff(); 
    }
    std::cout << "\n[Info] Program finished.\n";
    
    return 0;
}