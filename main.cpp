#include "RS485Comm.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    RS485Comm comm("/dev/ttyS0", 1);     // RS485 裝置與 Modbus ID
    if (!comm.openPort(19200)) {
        std::cerr << "Failed to open RS485 port\n";
        return -1;
    }

    // 定義常用暫存器位址
    const uint16_t REG_SERVO_ON    = 0x0600;  // SERVO_ON :contentReference[oaicite:2]{index=2}
    const uint16_t REG_START       = 0x0602;  // START :contentReference[oaicite:3]{index=3}
    const uint16_t REG_MOVE_DIR    = 0x0809;  // MoveDir :contentReference[oaicite:4]{index=4}
    const uint16_t REG_MOVE_EN     = 0x080A;  // MoveSttSet :contentReference[oaicite:5]{index=5}
    const uint16_t REG_PULSE_NUM   = 0x080B;  // PulseNum :contentReference[oaicite:6]{index=6}
    const uint16_t REG_PULSE_DEN   = 0x080D;  // PulseDen :contentReference[oaicite:7]{index=7}

    // 1. 啟動 Servo
    comm.writeRegister(REG_SERVO_ON, 1);

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // 2. 開啟伺服運轉
    comm.writeRegister(REG_START, 1);

    // 3. 移動方向與開/關爪預設參數
    //    direction: 0 = 正轉 (開爪)、1 = 反轉 (關爪)
    uint16_t open_pulse  = 1000;     // 實際請依機構調整
    uint16_t close_pulse = 500;      // 實際請依機構調整

    // 範例：先開爪
    comm.writeRegister(REG_MOVE_DIR, 0);
    comm.writeRegister(REG_MOVE_EN, 1);
    comm.writeRegister(REG_PULSE_NUM, open_pulse);
    comm.writeRegister(REG_PULSE_DEN, 1);
    comm.writeRegister(REG_START, 1);

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 範例：再關爪
    comm.writeRegister(REG_MOVE_DIR, 1);
    comm.writeRegister(REG_MOVE_EN, 1);
    comm.writeRegister(REG_PULSE_NUM, close_pulse);
    comm.writeRegister(REG_PULSE_DEN, 1);
    comm.writeRegister(REG_START, 1);

    comm.closePort();
    return 0;
}
