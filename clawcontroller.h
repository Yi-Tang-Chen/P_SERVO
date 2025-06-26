// =============================================================
//           最終的 clawcontroller.h (夾持力控制模式版)
// =============================================================
#ifndef CLAWCONTROLLER_H
#define CLAWCONTROLLER_H

#include "RS485Comm.h"
#include <cstdint>
#include <chrono>
#include <iostream>
#include <thread>

// 定義夾爪狀態，讓主程式呼叫更直觀
enum class ClawState {
    OPEN,
    CLOSE
};

class ClawController {
public:
    ClawController(RS485Comm& comm, int init_wait_ms = 2000);

    void initializeServo();
    // 我們不再使用 moveClaw，而是用新的函式
    void setClawState(ClawState state, uint16_t force_percent = 50); // 預設 50% 力道
    void readStatus();

private:
    RS485Comm& comm_;
    int init_wait_ms_;

    // 重新定義暫存器位址 (根據夾持力控制的需求)
    static constexpr uint16_t REG_ACTION_SERVO_ON   = 0x2011; // 伺服 ON/OFF
    static constexpr uint16_t REG_PUSH_FORCE_CW     = 0x0400; // +向下壓扭力 (夾緊)
    static constexpr uint16_t REG_PUSH_FORCE_CCW    = 0x0401; // -向下壓扭力 (鬆開)
    static constexpr uint16_t REG_PUSH_DIRECTION    = 0x2005; // 探測扭力極限移動方向
    static constexpr uint16_t REG_ACTION_EXECUTE    = 0x201E; // 動作執行
    static constexpr uint16_t REG_MOTION_STATUS     = 0x1000; // 讀取動作狀態
};

#endif // CLAWCONTROLLER_H