// =================================================================
//           最終的 clawcontroller.h (黃金救援序列版)
// =================================================================
#ifndef CLAWCONTROLLER_H
#define CLAWCONTROLLER_H

#include "RS485Comm.h"
#include <cstdint>
#include <chrono>
#include <iostream>
#include <thread>

class ClawController {
public:
    ClawController(RS485Comm& comm);

    // 初始化：設定 JOG 速度並啟用伺服
    bool initialize(uint16_t jog_speed_percent = 20);
    
    // 執行一次寸動
    // direction: 0 = 關閉 (+JOG), 1 = 打開 (-JOG)
    // duration_ms: 移動的持續時間 (毫秒)
    void jogStep(int direction, int duration_ms = 100);

    // 讀取並印出當前狀態
    void readAndPrintStatus();

private:
    RS485Comm& comm_;

    // 暫存器位址定義
    static constexpr uint16_t REG_SERVO_ON        = 0x2011;
    static constexpr uint16_t REG_JOG_SPEED       = 0x080F;
    static constexpr uint16_t REG_ACTION_EXECUTE  = 0x201E;
    static constexpr uint16_t REG_MOTION_STATUS   = 0x1000;
    static constexpr uint16_t REG_ALARM_STATUS    = 0x1005;

    // 動作指令碼
    static constexpr uint16_t CMD_JOG_PLUS      = 12;
    static constexpr uint16_t CMD_JOG_MINUS     = 13;
    static constexpr uint16_t CMD_DECEL_STOP    = 9;
    static constexpr uint16_t CMD_ALARM_RESET   = 7;
};

#endif // CLAWCONTROLLER_H