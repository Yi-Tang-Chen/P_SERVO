#include "clawcontroller.h"

ClawController::ClawController(RS485Comm& comm) : comm_(comm) {}

bool ClawController::initialize() {
    std::cout << "[Setup] Initializing controller...\n";
    comm_.writeRegister(REG_ACTION_EXECUTE, CMD_ALARM_RESET);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "[Setup] Initialization complete.\n";
    return true;
}

bool ClawController::servoOn() {
    std::cout << "\n[Servo] Sending ON command via Pseudo Port (emulating GUI)...\n";
    return comm_.writeRegister(REG_PSEUDO_IN1, 0);
}

bool ClawController::servoOff() {
    std::cout << "\n[Servo] Sending OFF command via Pseudo Port...\n";
    return comm_.writeRegister(REG_PSEUDO_IN1, 1);
}

void ClawController::moveRelative(int16_t distance) {
    std::cout << "\n[Action] Moving relative: " << distance << " pulses...\n";
    if (!comm_.writeRegister(REG_RELATIVE_POS_DATA, distance)) {
        std::cerr << "  [Error] Failed to set relative position data.\n";
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (!comm_.writeRegister(REG_ACTION_EXECUTE, CMD_RELATIVE_MOVE)) {
        std::cerr << "  [Error] Failed to send relative move command.\n";
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

bool ClawController::isActuallyOn() {
    uint16_t servo_status;
    // 讀取伺服狀態暫存器 0x1022
    if (comm_.readRegister(REG_SERVO_STATUS, servo_status)) {
        // 根據手冊，值為 0 代表 Servo ON
        return (servo_status == 0);
    }
    // 如果讀取失敗，為安全起見，假設它是關閉的
    return false;
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
        case 0: std::cout << "ON"; break;
        case 1: std::cout << "OFF"; break;
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