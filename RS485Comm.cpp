// RS485Comm.cpp (Final Robust Version)
#include "RS485Comm.h"
#include <windows.h>
#include <iostream>
#include <cstring>

RS485Comm::RS485Comm(const std::string& device, uint8_t slave_id)
: hComm_(INVALID_HANDLE_VALUE), slave_id_(slave_id), device_name_(device) {
}

RS485Comm::~RS485Comm() {
    closePort();
}

bool RS485Comm::openPort(int baudrate) {
    std::string full_device_name = "\\\\.\\" + device_name_;
    hComm_ = CreateFileA(full_device_name.c_str(),
                        GENERIC_READ | GENERIC_WRITE,
                        0, NULL, OPEN_EXISTING, 0, NULL);

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
    
    // *** 關鍵修改：大幅增加超時時間 ***
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout         = 200;  // 讀取字元間最大間隔 (ms)
    timeouts.ReadTotalTimeoutConstant    = 500;  // 讀取一次總共的固定超時 (ms)
    timeouts.ReadTotalTimeoutMultiplier  = 50;   // 讀取每字元的額外超時 (ms)
    timeouts.WriteTotalTimeoutConstant   = 500;  // 寫入一次總共的固定超時 (ms)
    timeouts.WriteTotalTimeoutMultiplier = 50;   // 寫入每字元的額外超時 (ms)

    if(!SetCommTimeouts(hComm_, &timeouts)){
        closePort();
        return false;
    }

    // 清空任何可能殘留在緩衝區的舊資料
    PurgeComm(hComm_, PURGE_TXCLEAR | PURGE_RXCLEAR);

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
    if (!WriteFile(hComm_, frame, len, &bytesWritten, NULL)) {
        std::cerr << "[Debug] WriteFile failed with error: " << GetLastError() << std::endl;
        return false;
    }
    return bytesWritten == len;
}

bool RS485Comm::recvFrame(uint8_t* buf, size_t buf_len, size_t& recv_len) {
    DWORD bytesRead;
    if (ReadFile(hComm_, buf, buf_len, &bytesRead, NULL)) {
        recv_len = bytesRead;
        // 即使只讀到一個 byte 也算成功，讓上層邏輯去判斷內容是否正確
        return recv_len > 0;
    }
    // ReadFile 返回 false，通常代表超時
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

    PurgeComm(hComm_, PURGE_RXCLEAR); // 寫入前清空接收緩衝區

    if (!sendFrame(frame, 8)) {
        return false;
    }

    uint8_t resp[8];
    size_t rlen;
    return recvFrame(resp, sizeof(resp), rlen);
}

bool RS485Comm::readRegister(uint16_t reg_addr, uint16_t& value) {
    uint8_t frame[8];
    frame[0] = slave_id_;
    frame[1] = 0x03;
    frame[2] = reg_addr >> 8;
    frame[3] = reg_addr & 0xFF;
    frame[4] = 0x00;
    frame[5] = 0x01; // 讀取一個字
    uint16_t crc = calcCRC(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    PurgeComm(hComm_, PURGE_RXCLEAR); // 讀取前清空接收緩衝區

    if (!sendFrame(frame, 8)) {
        return false;
    }

    // 預期的回應長度: 1(ID)+1(Func)+1(ByteCount)+2(Value)+2(CRC) = 7 bytes
    uint8_t resp[7];
    size_t rlen;
    if (!recvFrame(resp, sizeof(resp), rlen) || rlen < 5) { // 至少要有5個 bytes
        return false;
    }

    // 校驗 CRC
    uint16_t resp_crc = (resp[rlen-1] << 8) | resp[rlen-2];
    if (calcCRC(resp, rlen-2) != resp_crc) {
        return false;
    }

    // 校驗功能碼和 Slave ID
    if (resp[0] != slave_id_ || resp[1] != 0x03) {
        return false;
    }

    value = (resp[3] << 8) | resp[4];
    return true;
}