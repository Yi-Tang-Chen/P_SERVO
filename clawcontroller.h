#ifndef CLAWCONTROLLER_H
#define CLAWCONTROLLER_H

#include "RS485Comm.h"
#include <cstdint>
#include <iostream>
#include <thread>
#include <chrono>

class ClawController {
public:
    ClawController(RS485Comm& comm);

    // 初始化，現在只負責重置警報
    void initialize();

    // 透過偽通訊埠來開關伺服
    bool servoOn();
    bool servoOff();

    // 相對位置移動
    void moveRelative(int16_t distance);
    void readAndPrintStatus();

    // 直接從硬體讀取伺服的真實狀態
    bool isActuallyOn();

private:
    RS485Comm& comm_;

    // 暫存器位址定義
    static constexpr uint16_t REG_PSEUDO_IN1 = 0x2016;      // **關鍵：偽通訊埠IN1**
    static constexpr uint16_t REG_RELATIVE_POS_DATA = 0x2000;
    static constexpr uint16_t REG_ACTION_EXECUTE = 0x201E;
    static constexpr uint16_t REG_SERVO_STATUS = 0x1022;
    static constexpr uint16_t REG_MOTION_STATUS = 0x1000;
    static constexpr uint16_t REG_ALARM_STATUS = 0x1005;
    static constexpr uint16_t REG_ERROR_STATUS = 0x1023;

    // 動作指令碼
    static constexpr uint16_t CMD_ALARM_RESET = 7;
    static constexpr uint16_t CMD_RELATIVE_MOVE = 0;
};

#endif // CLAWCONTROLLER_H