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
    // 構造：傳入已經 openPort() 的 comm，以及初始化等待毫秒數
    ClawController(RS485Comm& comm, int init_wait_ms = 5000);

    // 啟動伺服並等待完成
    void initializeServo();

    // 脈衝模式移動爪子：
    //   direction: 0 = 正向 (開爪)；1 = 反向 (關爪)
    //   pulseCount: 脈衝數
    void moveClaw(uint16_t direction, uint16_t pulseCount);

    // 讀取狀態寄存器
    void readStatus();

private:
    RS485Comm& comm_;
    int        init_wait_ms_;

    // --- 正確的位址定義 ---
    // 動作執行相關 (Action Execution)
    static constexpr uint16_t REG_ACTION_EXECUTE   = 0x201E; // 寫入不同數值以執行不同動作
    static constexpr uint16_t REG_ACTION_SERVO_ON  = 0x2011; // 伺服 ON/OFF

    // 資料設定相關 (Data Registers)
    static constexpr uint16_t REG_RELATIVE_MOVE_PULSES = 0x2000; // 相對移動的脈衝數

    // 狀態讀取相關 (Status Registers)
    static constexpr uint16_t REG_MOTION_STATUS      = 0x1000; // 讀取動作狀態 (停止/作動中/異常)
};

#endif // MAIN_H