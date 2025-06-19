// main.h
#ifndef MAIN_H
#define MAIN_H

#include "RS485Comm.h"
#include <cstdint>
#include <chrono>
#include <iostream>
#include <thread>

class ClawController {
public:
    // 构造：传入已经 openPort() 的 comm，以及初始化等待毫秒数
    ClawController(RS485Comm& comm, int init_wait_ms = 5000);

    // 启动伺服并等待完成（写 REGISTER_SERVO_ON + sleep）
    void initializeServo();

    // 发一次 START 指令（写 REGISTER_START）
    void startServo();

    // 脉冲模式移动爪子：
    //   direction: 0 = 开爪；1 = 关爪
    //   pulseCount: 脉冲数；pulseDen: 分频（默认 1）
    void moveClaw(uint16_t direction,
                  uint16_t pulseCount,
                  uint16_t pulseDen = 1);

    // 读 STATUS 寄存器，并在 stdout 输出 MOVE/READY/ALARM
    void readStatus();

private:
    RS485Comm& comm_;
    int        init_wait_ms_;

    // 寄存器地址
    static constexpr uint16_t REG_SERVO_ON  = 0x0600;
    static constexpr uint16_t REG_START     = 0x0602;
    static constexpr uint16_t REG_MOVE_DIR  = 0x0809;
    static constexpr uint16_t REG_MOVE_EN   = 0x080A;
    static constexpr uint16_t REG_PULSE_NUM = 0x080B;
    static constexpr uint16_t REG_PULSE_DEN = 0x080D;
    static constexpr uint16_t REG_STATUS    = 0x0A0E;  // bit0=MOVE, bit1=READY, bit2=ALARM
};

#endif // MAIN_H
