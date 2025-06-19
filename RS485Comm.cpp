#include "RS485Comm.h"
#include <windows.h> // 使用 Windows API
#include <iostream>
#include <cstring>

RS485Comm::RS485Comm(const std::string& device, uint8_t slave_id)
: hComm_(INVALID_HANDLE_VALUE), slave_id_(slave_id), device_name_(device) {
}

RS485Comm::~RS485Comm() {
    closePort();
}

bool RS485Comm::openPort(int baudrate) {
    // Windows 的串口名稱格式為 "\\\\.\\COMX"
    std::string full_device_name = "\\\\.\\" + device_name_;

    // 修正：明確呼叫 CreateFileA 來匹配 const char* 類型的參數
    // 避免在 Unicode 專案設定下，CreateFile 被宏定義為 CreateFileW，導致類型不匹配的編譯錯誤
    hComm_ = CreateFileA(full_device_name.c_str(),
                        GENERIC_READ | GENERIC_WRITE,
                        0,
                        NULL,
                        OPEN_EXISTING,
                        0,
                        NULL);

    if (hComm_ == INVALID_HANDLE_VALUE) {
        return false;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hComm_, &dcbSerialParams)) {
        closePort();
        return false;
    }

    dcbSerialParams.BaudRate = baudrate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;

    if(!SetCommState(hComm_, &dcbSerialParams)){
        closePort();
        return false;
    }
    
    // 設定超時
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if(!SetCommTimeouts(hComm_, &timeouts)){
        closePort();
        return false;
    }

    return true;
}

void RS485Comm::closePort() {
    if (hComm_ != INVALID_HANDLE_VALUE) {
        CloseHandle(hComm_);
        hComm_ = INVALID_HANDLE_VALUE;
    }
}

uint16_t RS485Comm::calcCRC(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else             crc >>= 1;
        }
    }
    return crc;
}

bool RS485Comm::sendFrame(const uint8_t* frame, size_t len) {
    DWORD bytesWritten;
    return WriteFile(hComm_, frame, len, &bytesWritten, NULL) && bytesWritten == len;
}

bool RS485Comm::recvFrame(uint8_t* buf, size_t buf_len, size_t& recv_len) {
    DWORD bytesRead;
    if (ReadFile(hComm_, buf, buf_len, &bytesRead, NULL)) {
        recv_len = bytesRead;
        return recv_len > 0;
    }
    recv_len = 0;
    return false;
}

bool RS485Comm::writeRegister(uint16_t reg_addr, uint16_t value) {
    uint8_t frame[8];
    frame[0] = slave_id_;
    frame[1] = 0x06;
    frame[2] = reg_addr >> 8;
    frame[3] = reg_addr & 0xFF;
    frame[4] = value >> 8;
    frame[5] = value & 0xFF;
    uint16_t crc = calcCRC(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;
    if (!sendFrame(frame, 8)) return false;

    uint8_t resp[8];
    size_t rlen;
    // 在Modbus RTU中，主站發送寫入請求後，從站會回傳完全相同的數據幀
    return recvFrame(resp, sizeof(resp), rlen) && rlen == 8 && memcmp(frame, resp, 8) == 0;
}

bool RS485Comm::readRegister(uint16_t reg_addr, uint16_t& value) {
    uint8_t frame[8];
    frame[0] = slave_id_;
    frame[1] = 0x03;
    frame[2] = reg_addr >> 8;
    frame[3] = reg_addr & 0xFF;
    frame[4] = 0x00;
    frame[5] = 0x01;
    uint16_t crc = calcCRC(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;
    if (!sendFrame(frame, 8)) return false;

    // 預期的回應長度是 5 (ID, Func, ByteCount, Val_Hi, Val_Lo) + 2 (CRC) = 7
    uint8_t resp[7];
    size_t rlen;
    if (!recvFrame(resp, sizeof(resp), rlen) || rlen < 5) return false;

    // 簡單的CRC校驗
    uint16_t resp_crc = (resp[rlen-1] << 8) | resp[rlen-2];
    if (calcCRC(resp, rlen-2) != resp_crc) return false;

    value = (resp[3] << 8) | resp[4];
    return true;
}