// =============================================================
//           最終的 clawcontroller.cpp (夾持力控制模式版)
// =============================================================
#include "clawcontroller.h"

ClawController::ClawController(RS485Comm& comm, int init_wait_ms)
  : comm_(comm), init_wait_ms_(init_wait_ms) {}

void ClawController::initializeServo() {
    if (!comm_.writeRegister(REG_ACTION_SERVO_ON, 0)) { 
        std::cerr << "[Error] Failed to enable servo (write to 2011H).\n";
    } else {
        std::cout << "[Info] Servo ON command sent successfully.\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(init_wait_ms_));
}

void ClawController::setClawState(ClawState state, uint16_t force_percent) {
    uint16_t direction_value;
    // 將 1-100 的百分比，轉換為控制器需要的 0-1000 的數值
    uint16_t force_value = (force_percent > 100) ? 1000 : (force_percent * 10);

    if (state == ClawState::OPEN) {
        direction_value = 1; // 說明書定義: 1 = -向 (鬆開)
        std::cout << "[Action] Preparing to OPEN claw with force " << force_percent << "%\n";
    } else { // CLOSE
        direction_value = 0; // 說明書定義: 0 = +向 (夾緊)
        std::cout << "[Action] Preparing to CLOSE claw with force " << force_percent << "%\n";
    }
    
    // 步驟一：設定夾持力道 (寫入 0400H / 0401H)
    if (!comm_.writeRegister(REG_PUSH_FORCE_CW, force_value) || !comm_.writeRegister(REG_PUSH_FORCE_CCW, force_value)) {
        std::cerr << "[Error] Step 1: Failed to write force value to 0x0400/0x0401.\n";
        return;
    }
    std::cout << "[Info] Step 1: Force (" << force_value << ") written to force registers.\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // 步驟二：設定移動方向 (寫入 2005H)
    if (!comm_.writeRegister(REG_PUSH_DIRECTION, direction_value)) {
        std::cerr << "[Error] Step 2: Failed to set push direction (2005H).\n";
        return;
    }
    std::cout << "[Info] Step 2: Push direction (" << direction_value << ") sent to 2005H.\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    // 步驟三：觸發「扭力極限探測」動作 (寫入 3 到 201EH)
    if (!comm_.writeRegister(REG_ACTION_EXECUTE, 3)) {
        std::cerr << "[Error] Step 3: Failed to send PUSH command (writing 3 to 201EH).\n";
    } else {
        std::cout << "[Info] Step 3: Execute PUSH command sent. Claw should be moving.\n";
    }
}

void ClawController::readStatus() {
    uint16_t status = 0;
    if (!comm_.readRegister(REG_MOTION_STATUS, status)) {
        std::cerr << "[Error] Failed to read status register (1000H).\n";
        return;
    }
    std::cout << "[Status] Motion=";
    switch (status) {
        case 0: std::cout << "Stopped"; break;
        case 1: std::cout << "Moving"; break;
        case 2: std::cout << "Error"; break;
        default: std::cout << "Unknown (" << status << ")"; break;
    }
    std::cout << std::endl;
}