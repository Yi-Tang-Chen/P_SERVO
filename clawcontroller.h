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
    void jogStop();
    bool isActuallyOn();
    void clearAllFaults();

    // Functions to set parameters
    bool setHighSpeed(uint32_t speed_pps);
    bool setInchingDistance(uint32_t distance_pulse);

    // This is the single, public function for printing status
    void readAndPrintStatus();

private:
    // Helper functions to get raw data
    uint32_t getHighSpeedSetting();
    uint32_t getInchingDistanceSetting();

    RS485Comm& comm_;

    // Register and command definitions
    static constexpr uint16_t REG_ACTION_EXECUTE = 0x201E;
    static constexpr uint16_t REG_SERVO_STATUS = 0x1022;
    static constexpr uint16_t REG_MOTION_STATUS = 0x1000;
    static constexpr uint16_t REG_ALARM_STATUS = 0x1005;
    static constexpr uint16_t REG_ERROR_STATUS = 0x1023;
    static constexpr uint16_t REG_HIGH_SPEED = 0x0802; 
    static constexpr uint16_t REG_JOG_INCHING_DATA = 0x0810;
    static constexpr uint16_t CMD_ALARM_RESET = 7;
    static constexpr uint16_t CMD_CLEAR_DEVIATION = 8;
};

#endif // CLAWCONTROLLER_H