#include "clawcontroller.h"
#include <iostream>
#include <thread>
#include <chrono>

ClawController::ClawController(RS485Comm& comm) : comm_(comm) {}

void ClawController::initialize() {
    // Set a fast acceleration time on startup
    std::cout << "[Setup] Setting optimal acceleration time..." << std::endl;
    if (comm_.writeParameter32(0x0804, 10)) { // 0x0804 is AccelTime
        std::cout << "[Setup] Acceleration time set successfully." << std::endl;
    } else {
        std::cerr << "[Setup] Warning: Failed to set acceleration time." << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

bool ClawController::servoOn() { return comm_.executeAction(0x2011, 0); } 
bool ClawController::servoOff() { return comm_.executeAction(0x2011, 1); }
void ClawController::jogStart(bool positive) { comm_.executeAction(REG_ACTION_EXECUTE, positive ? 12 : 13); }
void ClawController::jogStop() { comm_.executeAction(REG_ACTION_EXECUTE, 9); }
bool ClawController::isActuallyOn() { uint16_t s; comm_.readRegister(REG_SERVO_STATUS, s); return s == 1; }

bool ClawController::setHighSpeed(uint32_t speed_pps) {
    return comm_.writeParameter32(REG_HIGH_SPEED, speed_pps);
}

uint32_t ClawController::getHighSpeedSetting() {
    uint32_t speed = 0;
    comm_.readRegister32(REG_HIGH_SPEED, speed);
    return speed;
}

void ClawController::clearAllFaults() {
    servoOff();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    comm_.executeAction(REG_ACTION_EXECUTE, CMD_ALARM_RESET);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    comm_.executeAction(REG_ACTION_EXECUTE, CMD_CLEAR_DEVIATION);
}

void ClawController::readAndPrintStatus() {
    uint16_t motion, alarm, error;
    comm_.readRegister(REG_MOTION_STATUS, motion);
    comm_.readRegister(REG_ALARM_STATUS, alarm);
    comm_.readRegister(REG_ERROR_STATUS, error);
    
    std::string motion_str = (motion == 0) ? "Stopped" : "Moving/Alarm";
    std::string alarm_str = (alarm == 0) ? "No Alarm" : "ALARM";
    std::string servo_str = isActuallyOn() ? "ON" : "OFF";
    std::string error_str = (error == 0) ? "No Error" : "ERROR";

    std::cout << "[Status]   Motion: " << motion_str << " | Alarm: " << alarm_str 
              << " | Servo: " << servo_str << " | Error: " << error_str << std::endl;

    uint32_t speed = getHighSpeedSetting();
    std::cout << "[Settings] HighSpeed: " << speed << " pps" << std::endl;
}
