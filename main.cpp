// main.cpp (Clean Version)
#include "main.h"

// 構造函數
ClawController::ClawController(RS485Comm& comm, int init_wait_ms)
  : comm_(comm), init_wait_ms_(init_wait_ms)
{}

// 初始化伺服
void ClawController::initializeServo() {
    if (!comm_.writeRegister(REG_ACTION_SERVO_ON, 0)) {
        std::cerr << "[Error] Failed to enable servo.\n";
        return;
    }
    std::cout << "[Info] Servo ON command sent, waiting for initialization...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(init_wait_ms_));
}

void ClawController::moveClaw(uint16_t direction, uint16_t pulseCount) {
    int16_t signedPulseCount = (direction == 0) ? pulseCount : -pulseCount;

    if (!comm_.writeRegister(REG_RELATIVE_MOVE_PULSES, static_cast<uint16_t>(signedPulseCount))) {
        std::cerr << "[Error] Failed to write move pulse count (2000H).\n";
        return;
    }

    if (!comm_.writeRegister(REG_ACTION_EXECUTE, 0)) {
        std::cerr << "[Error] Failed to send START command (201EH).\n";
    }
}

void ClawController::readStatus() {
    uint16_t status = 0;
    if (!comm_.readRegister(REG_MOTION_STATUS, status)) {
        std::cerr << "[Error] Failed to read status register (1000H).\n";
        return;
    }

    std::cout << "[Status] Motion=";
    switch (status) {
        case 0: std::cout << "Stopped"; break;
        case 1: std::cout << "Moving"; break;
        case 2: std::cout << "Error"; break;
        default: std::cout << "Unknown"; break;
    }
    std::cout << std::endl;
}

// 程序入口
int main() {
    RS485Comm comm("COM1", 1); // Windows COM port
    if (!comm.openPort(19200)) {
        std::cerr << "[Fatal] Cannot open RS-485 port.\n";
        return -1;
    }

    ClawController claw(comm);
    claw.initializeServo();
    claw.readStatus();

    // Action 1: Open Claw
    std::cout << "\n[Action] Opening claw...\n";
    claw.moveClaw(0, 1000);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for action to start
    claw.readStatus();
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Allow time for movement
    claw.readStatus();

    // Action 2: Close Claw
    std::cout << "\n[Action] Closing claw...\n";
    claw.moveClaw(1, 500);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for action to start
    claw.readStatus();
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Allow time for movement
    claw.readStatus();


    comm.closePort();
    return 0;
}