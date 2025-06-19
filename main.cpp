// main.cpp
#include "main.h"

// ... (ClawController 的實作維持不變) ...
ClawController::ClawController(RS485Comm& comm, int init_wait_ms)
  : comm_(comm), init_wait_ms_(init_wait_ms)
{}

void ClawController::initializeServo() {
    if (!comm_.writeRegister(REG_SERVO_ON, 1)) {
        std::cerr << "[Error] 无法启用伺服\n";
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(init_wait_ms_));
}

void ClawController::startServo() {
    if (!comm_.writeRegister(REG_START, 1)) {
        std::cerr << "[Error] 无法发送 START\n";
    }
}

void ClawController::moveClaw(uint16_t direction,
                              uint16_t pulseCount,
                              uint16_t pulseDen) {
    comm_.writeRegister(REG_MOVE_DIR,  direction);
    comm_.writeRegister(REG_MOVE_EN,   1);
    comm_.writeRegister(REG_PULSE_NUM, pulseCount);
    comm_.writeRegister(REG_PULSE_DEN, pulseDen);
    startServo();
}

void ClawController::readStatus() {
    uint16_t status = 0;
    if (!comm_.readRegister(REG_STATUS, status)) {
        std::cerr << "[Error] 无法读取状态寄存器\n";
        return;
    }
    bool moving = status & 0x0001;
    bool ready  = status & 0x0002;
    bool alarm  = status & 0x0004;

    std::cout 
      << "[Status] Moving=" << moving
      << "  Ready="      << ready
      << "  Alarm="      << alarm
      << std::endl;
}

int main() {
    // 將 "/dev/ttyS0" 改為 "COM1" 或其他 Windows 串口名稱
    RS485Comm comm("COM1", 1); 
    if (!comm.openPort(19200)) {
        std::cerr << "[Fatal] 无法打开 RS-485 端口\n";
        return -1;
    }

    ClawController claw(comm);
    claw.initializeServo();
    claw.readStatus();

    claw.moveClaw(0, 1000);
    claw.readStatus();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    claw.moveClaw(1, 500);
    claw.readStatus();

    comm.closePort();
    return 0;
}