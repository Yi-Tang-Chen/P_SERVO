// final_setup.cpp
#include <iostream>
#include <thread>
#include <chrono>
#include "RS485Comm.h"

static constexpr uint16_t REG_HIGH_SPEED = 0x0802;
static constexpr uint16_t REG_JOG_INCHING_DATA = 0x0810;

int main() {
    RS485Comm comm("COM4", 1);
    if (!comm.openPort(19200)) { /* ... */ return -1; }

    std::cout << "P-SERVO One-Time 32-bit Parameter Setup\n";

    uint32_t desired_speed = 80000;
    std::cout << "[1] Setting HighSpeed (0x0802) to " << desired_speed << " pps..." << std::endl;
    if (comm.writeParameter32(REG_HIGH_SPEED, desired_speed)) {
        std::cout << "  > Write command sent successfully." << std::endl;
    } else {
        std::cerr << "  > Failed to send write command for HighSpeed." << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    uint32_t desired_distance = 500;
    std::cout << "\n[2] Setting JogInchingData (0x0810) to " << desired_distance << " pulses..." << std::endl;
    if (comm.writeParameter32(REG_JOG_INCHING_DATA, desired_distance)) {
        std::cout << "  > Write command sent successfully." << std::endl;
    } else {
        std::cerr << "  > Failed to send write command for JogInchingData." << std::endl;
    }
    
    std::cout << "\n\nSetup complete. Please POWER CYCLE the P-SERVO controller now." << std::endl;
    comm.closePort();
    return 0;
}