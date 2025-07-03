#include "clawcontroller.h"
#include <string>

ClawController::ClawController(RS485Comm& comm) : comm_(comm) {}

void ClawController::initialize() {
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
    comm_.executeAction(REG_ACTION_EXECUTE, command);
}

bool ClawController::setJogSpeed(uint32_t speed_pps) {
    return comm_.writeParameter32(REG_HIGH_SPEED, speed_pps);
}

uint32_t ClawController::getHighSpeedSetting() {
    uint32_t speed = 0;
    comm_.readRegister32(REG_HIGH_SPEED, speed);
    return speed;
}

bool ClawController::setInchingDistance(uint32_t distance_pulse) {
    std::cout << "\n[Action] Setting JOG Inching Distance to " << distance_pulse << " pulses..." << std::endl;
    return comm_.writeParameter32(REG_JOG_INCHING_DATA, distance_pulse);
}

uint32_t ClawController::getInchingDistanceSetting() {
    uint32_t dist = 0;
    comm_.readRegister32(REG_JOG_INCHING_DATA, dist);
    return dist;
}

void ClawController::jogStop() {
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
    comm_.executeAction(REG_ACTION_EXECUTE, 8);
}

void ClawController::clearAllFaults() {
    std::cout << "\n[Action] Starting fault clearing procedure..." << std::endl;
    std::cout << "  > Step 1: Ensuring Servo is OFF..." << std::endl;
    servoOff();
    std::this_thread::sleep_for(std::chrono::milliseconds(300)); // 等待斷電完成
    std::cout << "  > Step 2: Sending Alarm Reset (CMD 7)..." << std::endl;
    comm_.executeAction(REG_ACTION_EXECUTE, 7);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "  > Step 3: Sending Clear Deviation Counter (CMD 8)..." << std::endl;
    comm_.executeAction(REG_ACTION_EXECUTE, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "  > Faults cleared. You can now turn Servo ON." << std::endl;
}

void ClawController::readAndPrintStatus() {
    uint16_t motion, alarm, servo, error;
    comm_.readRegister(REG_MOTION_STATUS, motion);
    comm_.readRegister(REG_ALARM_STATUS, alarm);
    comm_.readRegister(REG_SERVO_STATUS, servo);
    comm_.readRegister(REG_ERROR_STATUS, error);

    std::cout << "[Status] Motion: " << (motion == 0 ? "Stopped" : "Moving/Alarm")
              << " | Alarm: " << (alarm == 0 ? "No Alarm" : "ALARM")
              << " | Servo: " << (isActuallyOn() ? "ON" : "OFF")
              << " | Error: " << (error == 0 ? "No Error" : "ERROR") << std::endl;

    uint32_t speed = getHighSpeedSetting();
    uint32_t dist = getInchingDistanceSetting();
    std::cout << "[Settings] HighSpeed: " << speed << " pps | Distance: " << dist << " pulses" << std::endl;
}

std::string ClawController::getStatusString() {
    // ... 從 readAndPrintStatus 搬移過來的邏輯 ...
    uint16_t motion, alarm, error;
    // ... 讀取暫存器 ...
    return "Motion: " + std::string(motion == 0 ? "Stopped" : "Moving/Alarm") + 
           " | Alarm: " + std::string(alarm == 0 ? "No Alarm" : "ALARM") +
           " | Servo: " + std::string(isActuallyOn() ? "ON" : "OFF") +
           " | Error: " + std::string(error == 0 ? "No Error" : "ERROR");
}

std::string ClawController::getSettingsString() {
    uint32_t speed = getHighSpeedSetting();
    uint32_t dist = getInchingDistanceSetting();
    return "HighSpeed: " + std::to_string(speed) + " pps | Distance: " + std::to_string(dist) + " pulses";
}