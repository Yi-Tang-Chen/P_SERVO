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

    void jogStart(bool positive);
    void jogStop();
    
    void readAndPrintStatus();
    bool isActuallyOn();
    void clearDeviationCounter();
    void resetAlarm();
    void clearAllFaults();

    bool setJogSpeed(uint32_t speed_percent);
    uint16_t getJogSpeedSetting();
    bool setInchingDistance(uint32_t distance_pulse);
    uint32_t getInchingDistanceSetting();
    uint32_t getHighSpeedSetting();
    std::string getStatusString();
    std::string getSettingsString();

private:
    RS485Comm& comm_;

    // 暫存器位址定義
    static constexpr uint16_t REG_PSEUDO_IN1 = 0x2016;      // **關鍵：偽通訊埠IN1**
    static constexpr uint16_t REG_CONTROL_MODE = 0x0502;
    static constexpr uint16_t REG_RELATIVE_POS_DATA = 0x2000;
    static constexpr uint16_t REG_ACTION_EXECUTE = 0x201E;
    static constexpr uint16_t REG_SERVO_STATUS = 0x1022;
    static constexpr uint16_t REG_MOTION_STATUS = 0x1000;
    static constexpr uint16_t REG_ALARM_STATUS = 0x1005;
    static constexpr uint16_t REG_ERROR_STATUS = 0x1023;

    static constexpr uint16_t REG_JOG_SPEED = 0x080F;
    static constexpr uint16_t REG_HIGH_SPEED = 0x0802; 
    static constexpr uint16_t REG_JOG_INCHING_DATA = 0x0810;

    // 動作指令碼
    static constexpr uint16_t CMD_ALARM_RESET = 7;
    static constexpr uint16_t CMD_RELATIVE_MOVE = 0;
};

#endif // CLAWCONTROLLER_H