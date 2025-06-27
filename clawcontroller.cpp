// =================================================================
//           最終的 clawcontroller.cpp (JOG 寸動 + 狀態讀取版)
// =================================================================
#include "clawcontroller.h"

ClawController::ClawController(RS485Comm& comm) : comm_(comm) {}

bool ClawController::initialize(uint16_t jog_speed_percent) {
    std::cout << "[Setup] Initializing and configuring JOG mode...\n";

    // 重置警報，確保處於乾淨狀態
    comm_.writeRegister(REG_ACTION_EXECUTE, CMD_ALARM_RESET);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 設定 JOG 速度 (不再設定距離)
    std::cout << "  - Setting JOG speed to " << jog_speed_percent << "%\n";
    if (!comm_.writeRegister(REG_JOG_SPEED, jog_speed_percent)) {
        std::cerr << "  [Error] Failed to set JOG speed.\n";
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 啟用伺服
    if (!comm_.writeRegister(REG_SERVO_ON, 0)) {
        std::cerr << "  [Error] Failed to enable servo.\n";
        return false;
    }
    std::cout << "  - Servo ON. Initialization complete.\n";
    return true;
}

void ClawController::jogStep(int direction, int duration_ms) {
    uint16_t command = (direction == 0) ? CMD_JOG_PLUS : CMD_JOG_MINUS;
    
    std::cout << "\n[Action] Performing JOG step (direction: " << direction << ")...\n";
    
    // 步驟一：發送 JOG 開始指令
    if (!comm_.writeRegister(REG_ACTION_EXECUTE, command)) {
        std::cerr << "  [Error] Failed to send JOG START command.\n";
        return;
    }
    std::cout << "  - JOG START command sent.\n";

    // 步驟二：等待寸動的持續時間
    std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));

    // 步驟三：發送減速停止指令
    if (!comm_.writeRegister(REG_ACTION_EXECUTE, CMD_DECEL_STOP)) {
        std::cerr << "  [Error] Failed to send JOG STOP command.\n";
    } else {
        std::cout << "  - JOG STOP command sent.\n";
    }
}

void ClawController::readAndPrintStatus() {
    uint16_t motion_status = 99, alarm_status = 99;
    
    comm_.readRegister(REG_MOTION_STATUS, motion_status);
    comm_.readRegister(REG_ALARM_STATUS, alarm_status);

    std::cout << "[Status] Motion: ";
    switch (motion_status) {
        case 0: std::cout << "Stopped"; break;
        case 1: std::cout << "Moving"; break;
        case 2: std::cout << "Alarm Stop"; break;
        default: std::cout << "Unknown(" << motion_status << ")"; break;
    }

    std::cout << " | Alarm: ";
    switch (alarm_status) {
        case 0: std::cout << "No Alarm"; break;
        case 1: std::cout << "Loop Error"; break;
        case 2: std::cout << "Max Count"; break;
        case 3: std::cout << "Over Speed"; break;
        case 4: std::cout << "Gain Tuning Error"; break;
        case 5: std::cout << "Over Voltage"; break;
        case 6: std::cout << "Initialization Error"; break;
        case 7: std::cout << "EEPROM Error"; break;
        case 8: std::cout << "Position Compensation Error"; break;
        case 99: std::cout << "Power Recycled"; break;
        default: std::cout << "Unknown(" << alarm_status << ")"; break;
    }
    std::cout << std::endl;
}