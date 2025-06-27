#include "clawcontroller.h"

ClawController::ClawController(RS485Comm& comm) : comm_(comm) {}

void ClawController::initialize() {
    // 保持為空，驅動器啟動狀態已完美
    std::cout << "[Setup] Controller interface is ready." << std::endl;
}

bool ClawController::servoOn() {
    std::cout << "\n[Servo] Sending ON command (writing 1 to 0x2011)..." << std::endl;
    return comm_.writeRegister(0x2011, 0);
}

// 真實的 Servo OFF: 寫入 0
bool ClawController::servoOff() {
    std::cout << "\n[Servo] Sending OFF command (writing 0 to 0x2011)..." << std::endl;
    return comm_.writeRegister(0x2011, 1);
}

// JOG 開始函式
void ClawController::jogStart(bool positive) {
    uint16_t command = positive ? 12 : 13; // 12 for JOG+, 13 for JOG-
    std::cout << "\n[Action] Sending JOG START command (CMD " << command << ")..." << std::endl;
    comm_.writeRegister(REG_ACTION_EXECUTE, command);
}

// JOG 停止函式
void ClawController::jogStop() {
    std::cout << "\n[Action] Sending JOG STOP command (CMD 9 - Decelerate Stop)..." << std::endl;
    comm_.writeRegister(REG_ACTION_EXECUTE, 9);
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
    comm_.writeRegister(REG_ACTION_EXECUTE, 8);
}

void ClawController::clearAllFaults() {
    std::cout << "\n[Action] Starting fault clearing procedure..." << std::endl;
    
    // 步驟 1: 確保伺服已斷電 (這是最重要的前提)
    std::cout << "  > Step 1: Ensuring Servo is OFF..." << std::endl;
    servoOff();
    std::this_thread::sleep_for(std::chrono::milliseconds(300)); // 等待斷電完成

    // 步驟 2: 發送重置警報指令
    std::cout << "  > Step 2: Sending Alarm Reset (CMD 7)..." << std::endl;
    comm_.writeRegister(REG_ACTION_EXECUTE, 7);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 步驟 3: 發送清除偏差計數指令 (清除 Max Count 等相關問題)
    std::cout << "  > Step 3: Sending Clear Deviation Counter (CMD 8)..." << std::endl;
    comm_.writeRegister(REG_ACTION_EXECUTE, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "  > Faults cleared. You can now turn Servo ON." << std::endl;
}

void ClawController::readAndPrintStatus() {
    uint16_t motion_status = 99, alarm_status = 99, servo_status = 99, error_status = 99;
    comm_.readRegister(REG_MOTION_STATUS, motion_status);
    comm_.readRegister(REG_ALARM_STATUS, alarm_status);
    comm_.readRegister(REG_SERVO_STATUS, servo_status);
    comm_.readRegister(REG_ERROR_STATUS, error_status);

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
        case 1:  std::cout << "Loop Error"; break;
        case 2:  std::cout << "Max Count"; break;
        case 3:  std::cout << "Over Speed"; break;
        case 4:  std::cout << "Gain Tuning Error"; break;
        case 5:  std::cout << "Over Voltage"; break;
        case 6:  std::cout << "Initialization Error"; break;
        case 7:  std::cout << "EEPROM Error"; break;
        case 8:  std::cout << "Position Compensation Error"; break;
        case 99: std::cout << "Power Recycled"; break;
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
        case 1: std::cout << "Busy Error"; break;
        case 2: std::cout << "Limit Error"; break;
        case 3: std::cout << "Address Error"; break;
        case 4: std::cout << "Format Error"; break;
        case 5: std::cout << "Control Mode Error"; break;
        case 6: std::cout << "Power Recycled"; break;
        case 7: std::cout << "Init Not Done"; break;
        case 8: std::cout << "Servo ON/OFF Error"; break;
        default: std::cout << "Unknown(" << error_status << ")"; break;
    }
    std::cout << std::endl;
}