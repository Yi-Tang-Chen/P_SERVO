#include "clawcontroller.h"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <ncurses.h> // 使用 ncurses 函式庫處理所有畫面和鍵盤互動

// --- 速度映射函式 (維持不變) ---
const uint32_t MIN_SPEED_PPS = 1000;
const uint32_t MAX_SPEED_PPS = 900000;

uint32_t map_percentage_to_pps(int percentage) {
    if (percentage <= 1) return MIN_SPEED_PPS;
    if (percentage >= 100) return MAX_SPEED_PPS;
    double ratio = static_cast<double>(percentage - 1) / 99.0;
    return MIN_SPEED_PPS + static_cast<uint32_t>(ratio * (MAX_SPEED_PPS - MIN_SPEED_PPS));
}

// --- ncurses UI 輔助函式 ---
void print_header() {
    mvprintw(0, 0, "====================== P-SERVO DYNAMIC CONTROLLER (Ubuntu) ======================");
    mvprintw(2, 0, "  [t] Servo ON/OFF  [s] Status      [d] Distance      [v] Speed (1-100%%)");
    mvprintw(3, 0, "  [o] JOG +         [c] JOG -       [r] Clear Faults  [q] Quit");
    mvprintw(4, 0, "=================================================================================");
}

void print_status(ClawController& claw) {
    // 在固定位置打印狀態，避免畫面滾動
    // mvprintw(row, col, "format", args...);
    mvprintw(6, 0, "[Status]   %s", claw.getStatusString().c_str());
    mvprintw(7, 0, "[Settings] %s", claw.getSettingsString().c_str());
    mvprintw(9, 0, "> Press a key...          "); // 留出空間並清除舊訊息
    refresh();
}

// 從使用者獲取數字輸入的專用函式
uint32_t get_number_input(int y, int x, const char* prompt) {
    char str_buf[20];
    echo(); // 暫時開啟按鍵回顯，讓使用者能看到自己輸入的數字
    mvprintw(y, x, prompt);
    getstr(str_buf); // 讀取使用者輸入的字串
    noecho(); // 關閉按鍵回顯
    try {
        return std::stoul(str_buf); // 將字串轉換為無號長整數
    } catch (const std::exception& e) {
        return -1; // 如果轉換失敗，返回一個錯誤值
    }
}

// --- 主函式 ---
int main() {
    // --- 1. 初始化通訊 ---
    // !!! 重要: 在 Ubuntu 上，您需要找到正確的序列埠名稱
    // 通常是 "/dev/ttyUSB0", "/dev/ttyUSB1", 或 "/dev/ttyS0"
    // 您可以在終端機中先拔掉再插上您的USB-RS485轉換器，然後執行 `dmesg | grep tty` 來尋找正確的裝置名稱
    RS485Comm comm("/dev/ttyUSB0", 1); 
    if (!comm.openPort(19200)) {
        // 在 ncurses 啟動前，可以使用 std::cerr
        std::cerr << "[Fatal] Cannot open serial port. Check device name and permissions.\n";
        return -1;
    }
    
    ClawController claw(comm);
    claw.initialize();
    
    // --- 2. 初始化 ncurses ---
    initscr();            // 啟動 ncurses 模式
    cbreak();             // 立即讀取按鍵，不需等 Enter
    noecho();             // 不在螢幕上顯示按下的鍵
    keypad(stdscr, TRUE); // 啟用功能鍵
    timeout(50);          // 設定 getch() 為非阻塞模式，50ms 超時

    // --- 3. 主迴圈 ---
    print_header();
    print_status(claw);

    int key;
    while ((key = getch()) != 'q') {
        if (key == ERR) { // ERR 表示在超時時間內沒有按鍵
            continue;
        }

        bool status_needs_update = true; // 預設大多數指令都需要更新狀態

        switch (key) {
            case 't':
                mvprintw(9, 0, "> Toggling Servo...       ");
                refresh();
                if (claw.isActuallyOn()) { claw.servoOff(); } else { claw.servoOn(); }
                std::this_thread::sleep_for(std::chrono::milliseconds(150));
                break;

            case 'd': {
                uint32_t distance = get_number_input(9, 0, "> Enter new distance: ");
                if (distance != (uint32_t)-1) {
                    bool was_on = claw.isActuallyOn();
                    if (was_on) { claw.servoOff(); std::this_thread::sleep_for(std::chrono::milliseconds(200)); }
                    claw.setInchingDistance(distance);
                    if (was_on) { claw.servoOn(); }
                }
                break;
            }
            
            case 'v': {
                uint32_t percentage = get_number_input(9, 0, "> Enter new speed (1-100%): ");
                if (percentage >= 1 && percentage <= 100) {
                    uint32_t speed_pps = map_percentage_to_pps(percentage);
                    bool was_on = claw.isActuallyOn();
                    if (was_on) { claw.servoOff(); std::this_thread::sleep_for(std::chrono::milliseconds(200)); }
                    claw.setHighSpeed(speed_pps);
                    if (was_on) { claw.servoOn(); }
                }
                break;
            }

            case 'o':
            case 'c':
                if (claw.isActuallyOn()) {
                    claw.jogStart(key == 'o');
                    claw.jogStop();
                }
                status_needs_update = false; // JOG 是快速連續操作，不需要每次都更新狀態
                break;

            case 'r':
                mvprintw(9, 0, "> Clearing faults...        ");
                refresh();
                claw.clearAllFaults();
                break;

            case 's':
                // 按 's' 會強制更新一次狀態
                break; 
            
            default:
                status_needs_update = false; // 未知指令，不更新狀態
                break;
        }

        if (status_needs_update) {
            print_status(claw);
        }
    }

    // --- 4. 結束程式前的清理 ---
    endwin(); // 關閉 ncurses 模式，恢復終端機正常狀態
    
    if (claw.isActuallyOn()) {
        std::cout << "[Info] Turning Servo OFF before exiting..." << std::endl;
        claw.servoOff(); 
    }
    std::cout << "[Info] Program finished.\n";
    
    return 0;
}