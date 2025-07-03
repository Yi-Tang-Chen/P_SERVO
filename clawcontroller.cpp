#include "clawcontroller.h"
#include <iostream>
#include <thread>
#include <chrono>

ClawController::ClawController(RS485Comm& comm) : comm_(comm) {}

void ClawController::initialize() {}

bool ClawController::servoOn() { return comm_.executeAction(0x2011, 0); } 
bool ClawController::servoOff() { return comm_.executeAction(0x2011, 1); }
void ClawController::jogStart(bool positive) { comm_.executeAction(REG_ACTION_EXECUTE, positive ? 12 : 13); }
void ClawController::jogStop() { comm_.executeAction(REG_ACTION_EXECUTE, 9); }
bool ClawController::isActuallyOn() { uint16_t s; comm_.readRegister(REG_SERVO_STATUS, s); return s == 1; }

bool ClawController::setHighSpeed(uint32_t speed_pps) {
    return comm_.writeParameter32(REG_HIGH_SPEED, speed_pps);
}

bool ClawController::setInchingDistance(uint32_t distance_pulse) {
    return comm_.writeParameter32(REG_JOG_INCHING_DATA, distance_pulse);
}

uint32_t ClawController::getHighSpeedSetting() {
    uint32_t speed = 0;
    comm_.readRegister32(REG_HIGH_SPEED, speed);
    return speed;
}

uint32_t ClawController::getInchingDistanceSetting() {
    uint32_t dist = 0;
    comm_.readRegister32(REG_JOG_INCHING_DATA, dist);
    return dist;
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
    uint32_t dist = getInchingDistanceSetting();
    std::cout << "[Settings] HighSpeed: " << speed << " pps | Distance: " << dist << " pulses" << std::endl;
}