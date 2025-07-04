#ifndef CLAWCONTROLLER_H
#define CLAWCONTROLLER_H

#include "RS485Comm.h"
#include <cstdint>
#include <string>

class ClawController {
public:
    ClawController(RS485Comm& comm);

    void initialize();
    bool servoOn();
    bool servoOff();
    void jogStart(bool positive);
    void jogStop(); // We still need this function
    bool isActuallyOn();
    void clearAllFaults();

    // We only need to set the speed
    bool setHighSpeed(uint32_t speed_pps);

    // This function now prints the status
    void readAndPrintStatus();

private:
    uint32_t getHighSpeedSetting();

    RS485Comm& comm_;

    // Register definitions
    static constexpr uint16_t REG_ACTION_EXECUTE = 0x201E;
    static constexpr uint16_t REG_SERVO_STATUS = 0x1022;
    static constexpr uint16_t REG_MOTION_STATUS = 0x1000;
    static constexpr uint16_t REG_ALARM_STATUS = 0x1005;
    static constexpr uint16_t REG_ERROR_STATUS = 0x1023;
    static constexpr uint16_t REG_HIGH_SPEED = 0x0802; 
    
    // Command definitions
    static constexpr uint16_t CMD_ALARM_RESET = 7;
    static constexpr uint16_t CMD_CLEAR_DEVIATION = 8;
};

#endif // CLAWCONTROLLER_H
