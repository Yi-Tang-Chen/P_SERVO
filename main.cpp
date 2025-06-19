// main.cpp
#include "main.h"

// 构造函数：保存引用并记录初始化等待时间
ClawController::ClawController(RS485Comm& comm, int init_wait_ms)
  : comm_(comm), init_wait_ms_(init_wait_ms)
{}

// 初始化伺服
void ClawController::initializeServo() {
    if (!comm_.writeRegister(REG_SERVO_ON, 1)) {
        std::cerr << "[Error] 无法启用伺服\n";
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(init_wait_ms_));
}

// 发 START
void ClawController::startServo() {
    if (!comm_.writeRegister(REG_START, 1)) {
        std::cerr << "[Error] 无法发送 START\n";
    }
}

// 脉冲移动
void ClawController::moveClaw(uint16_t direction,
                              uint16_t pulseCount,
                              uint16_t pulseDen) {
    comm_.writeRegister(REG_MOVE_DIR,  direction);
    comm_.writeRegister(REG_MOVE_EN,   1);
    comm_.writeRegister(REG_PULSE_NUM, pulseCount);
    comm_.writeRegister(REG_PULSE_DEN, pulseDen);
    startServo();
}

// 读状态并打印
void ClawController::readStatus() {
    uint16_t status = 0;
    if (!comm_.readRegister(REG_STATUS, status)) {
        std::cerr << "[Error] 无法读取状态寄存器\n";
        return;
    }
    bool moving = status & 0x0001;  // bit0 = MOVE
    bool ready  = status & 0x0002;  // bit1 = READY
    bool alarm  = status & 0x0004;  // bit2 = ALARM

    std::cout 
      << "[Status] Moving=" << moving
      << "  Ready="      << ready
      << "  Alarm="      << alarm
      << std::endl;
}

// 程序入口：示范如何正确调用
int main() {
    RS485Comm comm("/dev/ttyS0", 1);
    if (!comm.openPort(19200)) {
        std::cerr << "[Fatal] 无法打开 RS-485 端口\n";
        return -1;
    }

    ClawController claw(comm);
    claw.initializeServo();
    claw.readStatus();

    // 开爪
    claw.moveClaw(0, 1000);
    claw.readStatus();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 关爪
    claw.moveClaw(1, 500);
    claw.readStatus();

    comm.closePort();
    return 0;
}
