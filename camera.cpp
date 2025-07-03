#include "clawcontroller.h"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <limits>

// --- Speed mapping function ---
const uint32_t MIN_SPEED_PPS = 1000;
const uint32_t MAX_SPEED_PPS = 900000;

uint32_t map_percentage_to_pps(int percentage) {
    if (percentage <= 1) return MIN_SPEED_PPS;
    if (percentage >= 100) return MAX_SPEED_PPS;
    double ratio = static_cast<double>(percentage - 1) / 99.0;
    return MIN_SPEED_PPS + static_cast<uint32_t>(ratio * (MAX_SPEED_PPS - MIN_SPEED_PPS));
}

int main() {
    // --- 1. Initialize Communication ---
    RS485Comm comm("/dev/ttyUSB0", 1); 
    if (!comm.openPort(19200)) {
        std::cerr << "[Fatal] Cannot open serial port. Check device name and permissions.\n";
        return -1;
    }
    
    ClawController claw(comm);
    claw.initialize();
    
    // --- 2. Display Instructions ---
    system("clear");
    std::cout << "\n====================== P-SERVO CONTROLLER (Ubuntu / std::cin) ======================\n";
    std::cout << "  (Enter a command character and press Enter)\n\n";
    std::cout << "  [t] Servo ON/OFF  [s] Status      [d] Distance      [v] Speed (1-100%)\n";
    std::cout << "  [o] JOG +         [c] JOG -       [r] Clear Faults  [q] Quit\n";
    std::cout << "====================================================================================\n\n";
    
    std::cout << "Initial status check:" << std::endl;
    claw.readAndPrintStatus();

    // --- 3. Main Loop ---
    char key;
    while (true) {
        std::cout << "\n> Enter command: ";
        std::cin >> key;

        if (std::cin.peek() != '\n' && std::cin.peek() != EOF) {
             std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }

        if (key == 'q') {
            break;
        }

        switch (key) {
            case 't':
                std::cout << "\n> Toggling Servo..." << std::endl;
                if (claw.isActuallyOn()) { claw.servoOff(); } else { claw.servoOn(); }
                break;

            case 'd': {
                uint32_t distance;
                std::cout << "  > Enter new inching distance: ";
                std::cin >> distance;
                if (std::cin.fail()) {
                    std::cerr << "[Error] Invalid input." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    continue; 
                }
                
                std::cout << "  > Applying distance..." << std::endl;
                bool was_on = claw.isActuallyOn();
                if (was_on) { claw.servoOff(); std::this_thread::sleep_for(std::chrono::milliseconds(200)); }
                if (claw.setInchingDistance(distance)) { std::cout << "  > Distance set." << std::endl; }
                else { std::cerr << "  > Failed to set distance." << std::endl; }
                if (was_on) { claw.servoOn(); }
                break;
            }
            
            case 'v': {
                int percentage;
                std::cout << "  > Enter new speed (1-100%): ";
                std::cin >> percentage;
                if (std::cin.fail() || percentage < 1 || percentage > 100) {
                     std::cerr << "[Error] Invalid input." << std::endl;
                     std::cin.clear();
                     std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                     continue;
                }

                uint32_t speed_pps = map_percentage_to_pps(percentage);
                std::cout << "  > Applying speed (" << percentage << "% -> " << speed_pps << " pps)..." << std::endl;
                bool was_on = claw.isActuallyOn();
                if (was_on) { claw.servoOff(); std::this_thread::sleep_for(std::chrono::milliseconds(200)); }
                if (claw.setHighSpeed(speed_pps)) { std::cout << "  > Speed set." << std::endl; }
                else { std::cerr << "  > Failed to set speed." << std::endl; }
                if (was_on) { claw.servoOn(); }
                break;
            }

            case 'o':
            case 'c':
                if (claw.isActuallyOn()) {
                    std::cout << "> Executing JOG " << (key == 'o' ? "+" : "-") << "..." << std::endl;
                    claw.jogStart(key == 'o');
                    // claw.jogStop();
                } else {
                    std::cout << "[Warning] Servo is OFF." << std::endl;
                }
                break;
            case 'x':
                claw.jogStop();
                break;
            case 'r':
                std::cout << "> Clearing faults..." << std::endl;
                claw.clearAllFaults();
                std::cout << "  > Faults cleared." << std::endl;
                break;

            case 's':
                // The status will be printed automatically after the loop
                break; 
            
            default:
                std::cout << "  > Unknown command." << std::endl;
                continue;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        std::cout << "\nCurrent Status:" << std::endl;
        claw.readAndPrintStatus();
    }

    // --- 4. Cleanup ---
    if (claw.isActuallyOn()) {
        std::cout << "[Info] Turning Servo OFF before exiting..." << std::endl;
        claw.servoOff(); 
    }
    std::cout << "[Info] Program finished.\n";
    
    return 0;
}