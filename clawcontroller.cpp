#include "clawcontroller.h"

ClawController::ClawController(RS485Comm& comm) : comm_(comm) {}

void ClawController::initialize() {
    // 保持為空，驅動器啟動狀態已完美
    std::cout << "[Setup] Controller interface is ready." << std::endl;
}

bool ClawController::servoOn() {
    return comm_.executeAction(0x2011, 0);
}

bool ClawController::servoOff() {
    return comm_.executeAction(0x2011, 1);
}

// JOG 開始函式
void ClawController::jogStart(bool positive) {
    uint16_t command = positive ? 12 : 13; // 12 for JOG+, 13 for JOG-
    std::cout << "\n[Action] Sending JOG START command (CMD " << command << ")..." << std::endl;
    comm_.executeAction(REG_ACTION_EXECUTE, command);
}

bool ClawController::setJogSpeed(uint16_t speed_percent) {
    // 根據手冊，速度範圍是 1% 到 100%
    if (speed_percent < 1 || speed_percent > 100) {
        std::cerr << "[Error] JOG speed must be between 1 and 100 percent." << std::endl;
        return false;
    }
    std::cout << "\n[Action] Setting JOG speed to " << speed_percent << "%..." << std::endl;
    // 寫入到 0x080F 暫存器
    return comm_.executeAction(REG_JOG_SPEED, speed_percent);
}

bool ClawController::setInchingDistance(uint16_t distance_pulse) {
    std::cout << "\n[Action] Setting JOG Inching Distance to " << distance_pulse << " pulses..." << std::endl;
    return comm_.writeParameter(REG_JOG_INCHING_DATA, distance_pulse);
}

void ClawController::jogStop() {
    std::cout << "\n[Action] Sending JOG STOP command (CMD 9 - Decelerate Stop)..." << std::endl;
    comm_.executeAction(REG_ACTION_EXECUTE, 9);
}

// 讀取真實狀態，此函式邏輯正確 (狀態暫存器 0x1022 的值為 1 代表 ON)
bool ClawController::isActuallyOn() {
    uint16_t servo_status;
    if (comm_.readRegister(0x1022, servo_status)) {
        return (servo_status == 1);
    }
    return false;
}

void ClawController::clearDeviationCounter() {
    std::cout << "\n[Action] Sending Clear Deviation Counter (CMD 8)..." << std::endl;
    // 根據手冊，命令碼 8 代表「重置偏差計數」
    comm_.executeAction(REG_ACTION_EXECUTE, 8);
}

void ClawController::clearAllFaults() {
    std::cout << "\n[Action] Starting fault clearing procedure..." << std::endl;
    
    // 步驟 1: 確保伺服已斷電 (這是最重要的前提)
    std::cout << "  > Step 1: Ensuring Servo is OFF..." << std::endl;
    servoOff();
    std::this_thread::sleep_for(std::chrono::milliseconds(300)); // 等待斷電完成

    // 步驟 2: 發送重置警報指令
    std::cout << "  > Step 2: Sending Alarm Reset (CMD 7)..." << std::endl;
    comm_.executeAction(REG_ACTION_EXECUTE, 7);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 步驟 3: 發送清除偏差計數指令 (清除 Max Count 等相關問題)
    std::cout << "  > Step 3: Sending Clear Deviation Counter (CMD 8)..." << std::endl;
    comm_.executeAction(REG_ACTION_EXECUTE, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "  > Faults cleared. You can now turn Servo ON." << std::endl;
}

void ClawController::readAndPrintStatus() {
    uint16_t motion_status = 99, alarm_status = 99, servo_status = 99, error_status = 99;
    
    // --- 效能優化 ---
    // 將 4 次獨立通訊請求，減少為 3 次
    
    // 第 1 次通訊：讀取運動狀態
    comm_.readRegister(REG_MOTION_STATUS, motion_status); 
    
    // 第 2 次通訊：讀取警報狀態
    comm_.readRegister(REG_ALARM_STATUS, alarm_status);

    // 第 3 次通訊：使用新函式，一次讀取 2 個連續的暫存器 (伺服狀態和錯誤狀態)
    std::vector<uint16_t> status_vector;
    if (comm_.readMultipleRegisters(REG_SERVO_STATUS, 2, status_vector) && status_vector.size() == 2) {
        servo_status = status_vector[0];
        error_status = status_vector[1];
    }

    // --- 後續的打印邏輯完全不變 ---
    std::cout << "[Status] Motion: ";
    switch (motion_status) {
        case 0: std::cout << "Stopped"; break;
        case 1: std::cout << "Moving"; break;
        case 2: std::cout << "Alarm Stop"; break;
        default: std::cout << "Unknown(" << motion_status << ")"; break;
    }

    std::cout << " | Alarm: ";
    switch (alarm_status) {
        case 0:  std::cout << "No Alarm"; break;
        // ... (其他 case) ...
        default: std::cout << "Unknown(" << alarm_status << ")"; break;
    }

    std::cout << " | Servo: ";
    switch (servo_status) {
        case 0: std::cout << "OFF"; break;
        case 1: std::cout << "ON"; break;
        default: std::cout << "Unknown(" << servo_status << ")"; break;
    }

    std::cout << " | Error: ";
    switch (error_status) {
        case 0: std::cout << "No Error"; break;
        // ... (其他 case) ...
        default: std::cout << "Unknown(" << error_status << ")"; break;
    }
    std::cout << std::endl;
}