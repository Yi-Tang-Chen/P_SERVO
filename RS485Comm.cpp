#include "RS485Comm.h"
#include <windows.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <iomanip>

RS485Comm::RS485Comm(const std::string& device, uint8_t slave_id)
: hComm_(INVALID_HANDLE_VALUE), slave_id_(slave_id), device_name_(device) {}

RS485Comm::~RS485Comm() {
    closePort();
}

bool RS485Comm::openPort(int baudrate) {
    std::string full_device_name = "\\\\.\\" + device_name_;
    hComm_ = CreateFileA(full_device_name.c_str(),
                        GENERIC_READ | GENERIC_WRITE,
                        0, NULL, OPEN_EXISTING, 0, NULL);

    if (hComm_ == INVALID_HANDLE_VALUE) return false;

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

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout         = 200;
    timeouts.ReadTotalTimeoutConstant    = 500;
    timeouts.ReadTotalTimeoutMultiplier  = 50;
    timeouts.WriteTotalTimeoutConstant   = 500;
    timeouts.WriteTotalTimeoutMultiplier = 50;
    if(!SetCommTimeouts(hComm_, &timeouts)){
        closePort();
        return false;
    }

    PurgeComm(hComm_, PURGE_TXCLEAR | PURGE_RXCLEAR);
    return true;
}

void RS485Comm::closePort() {
    if (hComm_ != INVALID_HANDLE_VALUE) {
        CloseHandle(hComm_);
        hComm_ = INVALID_HANDLE_VALUE;
    }
}



uint8_t RS485Comm::calcLRC(const std::vector<uint8_t>& data) {
    uint8_t lrc = 0;
    for (uint8_t byte : data) {
        lrc += byte;
    }
    return (uint8_t)(-(int8_t)lrc);
}

std::string RS485Comm::byteToAscii(uint8_t byte) {
    std::stringstream ss;
    ss << std::setw(2) << std::setfill('0') << std::hex << (int)byte;
    std::string result = ss.str();
    // 轉為大寫
    for (auto & c: result) c = toupper(c);
    return result;
}

uint8_t RS485Comm::asciiToByte(const char* ascii) {
    char hex[3] = { ascii[0], ascii[1], '\0' };
    return (uint8_t)std::stoul(hex, nullptr, 16);
}

// sendFrame 和 recvFrame 現在處理 string
bool RS485Comm::sendFrame(const std::string& frame) {
    DWORD bytesWritten;
    if (!WriteFile(hComm_, frame.c_str(), frame.length(), &bytesWritten, NULL)) {
        return false;
    }
    return bytesWritten == frame.length();
}

bool RS485Comm::recvFrame(std::string& response) {
    char buf[256] = {0}; // 接收緩衝區
    DWORD bytesRead;
    if (ReadFile(hComm_, buf, sizeof(buf) - 1, &bytesRead, NULL) && bytesRead > 0) {
        response.assign(buf, bytesRead);
        return true;
    }
    return false;
}

bool RS485Comm::writeParameter(uint16_t reg_addr, uint16_t value) {
    std::vector<uint8_t> data_rtu;
    data_rtu.push_back(slave_id_);
    data_rtu.push_back(0x06); // 使用「寫入單一暫存器」功能碼
    data_rtu.push_back(reg_addr >> 8);
    data_rtu.push_back(reg_addr & 0xFF);
    data_rtu.push_back(value >> 8);
    data_rtu.push_back(value & 0xFF);

    uint8_t lrc = calcLRC(data_rtu);

    std::string frame = ":";
    for(uint8_t byte : data_rtu) {
        frame += byteToAscii(byte);
    }
    frame += byteToAscii(lrc);
    frame += "\r\n";

    PurgeComm(hComm_, PURGE_RXCLEAR);
    if (!sendFrame(frame)) return false;
    std::string response;
    if (!recvFrame(response)) return false;
    
    // 成功的正常回應會是我們發送內容的回聲
    return response.find(frame.substr(0, frame.length() - 4)) != std::string::npos;
}

bool RS485Comm::writeParameter32(uint16_t reg_addr, uint32_t value) {
    std::vector<uint8_t> data_rtu;
    data_rtu.push_back(slave_id_);
    data_rtu.push_back(0x10); // 功能碼 10H: 寫入多個暫存器
    data_rtu.push_back(reg_addr >> 8);
    data_rtu.push_back(reg_addr & 0xFF);
    data_rtu.push_back(0x00); // 寫入數量高位 (2 個)
    data_rtu.push_back(0x02); // 寫入數量低位 (2 個)
    data_rtu.push_back(0x04); // 寫入位元組數 (4 bytes)

    // 將 32 位元值拆分成兩個 16 位元值
    // 根據手冊 A06H，預設是大端序 (Big Endian)
    uint16_t high_word = value >> 16;
    uint16_t low_word = value & 0xFFFF;

    // 寫入高位 Word
    data_rtu.push_back(high_word >> 8);
    data_rtu.push_back(high_word & 0xFF);
    // 寫入低位 Word
    data_rtu.push_back(low_word >> 8);
    data_rtu.push_back(low_word & 0xFF);

    uint8_t lrc = calcLRC(data_rtu);

    std::string frame = ":";
    for(uint8_t byte : data_rtu) {
        frame += byteToAscii(byte);
    }
    frame += byteToAscii(lrc);
    frame += "\r\n";

    PurgeComm(hComm_, PURGE_RXCLEAR);
    if (!sendFrame(frame)) return false;
    std::string response;
    if (!recvFrame(response)) return false;

    // 成功的正常回應會是我們發送內容的回聲 (部分)
    // :011020020002...
    std::string expected_response_prefix = ":" + byteToAscii(slave_id_) + "10" + byteToAscii(reg_addr >> 8) + byteToAscii(reg_addr & 0xFF);
    return response.rfind(expected_response_prefix, 0) == 0;
}

// 專門用於「執行動作」，使用功能碼 10H
bool RS485Comm::executeAction(uint16_t reg_addr, uint16_t value) {
    std::vector<uint8_t> data_rtu;
    data_rtu.push_back(slave_id_);
    data_rtu.push_back(0x10); // 使用「寫入多個暫存器」功能碼
    data_rtu.push_back(reg_addr >> 8);
    data_rtu.push_back(reg_addr & 0xFF);
    data_rtu.push_back(0x00); // 寫入數量高位 (1 個)
    data_rtu.push_back(0x01); // 寫入數量低位 (1 個)
    data_rtu.push_back(0x02); // 寫入位元組數 (2 bytes)
    data_rtu.push_back(value >> 8);
    data_rtu.push_back(value & 0xFF);

    uint8_t lrc = calcLRC(data_rtu);

    std::string frame = ":";
    for(uint8_t byte : data_rtu) {
        frame += byteToAscii(byte);
    }
    frame += byteToAscii(lrc);
    frame += "\r\n";

    PurgeComm(hComm_, PURGE_RXCLEAR);
    if (!sendFrame(frame)) return false;
    std::string response;
    if (!recvFrame(response)) return false;

    // 成功的正常回應會是我們發送內容的回聲
    return response.find(frame.substr(0, frame.length() - 4)) != std::string::npos;
}

bool RS485Comm::readRegister(uint16_t reg_addr, uint16_t& value) {
    // 1. 組合 RTU 部分的資料
    std::vector<uint8_t> data_rtu;
    data_rtu.push_back(slave_id_);
    data_rtu.push_back(0x03); // Function code
    data_rtu.push_back(reg_addr >> 8);
    data_rtu.push_back(reg_addr & 0xFF);
    data_rtu.push_back(0x00); // 讀取數量的高位元組
    data_rtu.push_back(0x01); // 讀取數量的低位元組 (讀取一個)

    // 2. 計算 LRC
    uint8_t lrc = calcLRC(data_rtu);

    // 3. 建立完整的 ASCII 訊框
    std::string frame = ":";
    for(uint8_t byte : data_rtu) {
        frame += byteToAscii(byte);
    }
    frame += byteToAscii(lrc);
    frame += "\r\n"; // 結尾符

    PurgeComm(hComm_, PURGE_RXCLEAR);

    if (!sendFrame(frame)) return false;

    std::string response;
    if (!recvFrame(response) || response.length() < 11) { // 預期回應 :010302(val)XX\r\n -> 至少 1+6+2+2 = 11 字元
        return false;
    }

    // 4. 解析回應
    if (response[0] != ':') return false;

    std::vector<uint8_t> resp_data;
    // 從第1個字元開始，每2個字元轉成一個byte
    for (size_t i = 1; i + 1 < response.length(); i += 2) {
        // 遇到 CR 或 LF 就停止
        if (response[i] == '\r' || response[i] == '\n') break;
        resp_data.push_back(asciiToByte(&response[i]));
    }
    
    if (resp_data.size() < 4) return false; // ID, Func, Count, LRC 至少要有

    // 校驗 LRC
    uint8_t received_lrc = resp_data.back();
    resp_data.pop_back(); // 移除 LRC 以便計算
    if (calcLRC(resp_data) != received_lrc) return false;

    // 校驗 Slave ID 和 Function Code
    if (resp_data[0] != slave_id_ || resp_data[1] != 0x03) return false;

    // 提取數值
    if (resp_data.size() >= 4 && resp_data[2] == 2) { // Byte count 應為 2
        value = (resp_data[3] << 8) | resp_data[4];
        return true;
    }

    return false;
}


bool RS485Comm::readMultipleRegisters(uint16_t start_addr, uint16_t count, std::vector<uint16_t>& dest) {
    if (count == 0 || count > 125) return false; // Modbus 限制

    // 1. 組合 RTU 部分的資料
    std::vector<uint8_t> data_rtu;
    data_rtu.push_back(slave_id_);
    data_rtu.push_back(0x03); // Function code for reading
    data_rtu.push_back(start_addr >> 8);
    data_rtu.push_back(start_addr & 0xFF);
    data_rtu.push_back(count >> 8);     // <<-- 關鍵修改：讀取數量的高位元組
    data_rtu.push_back(count & 0xFF);   // <<-- 關鍵修改：讀取數量的低位元組

    // 2. 計算 LRC
    uint8_t lrc = calcLRC(data_rtu);

    // 3. 建立完整的 ASCII 訊框
    std::string frame = ":";
    for(uint8_t byte : data_rtu) {
        frame += byteToAscii(byte);
    }
    frame += byteToAscii(lrc);
    frame += "\r\n";

    PurgeComm(hComm_, PURGE_RXCLEAR);
    if (!sendFrame(frame)) return false;

    std::string response;
    // 預期回應長度: 1 (start) + 2(ID) + 2(Func) + 2(ByteCount) + count*4(data) + 2(LRC) + 2(End)
    if (!recvFrame(response) || response.length() < 9) return false;

    // 4. 解析回應
    if (response[0] != ':') return false;

    std::vector<uint8_t> resp_data;
    for (size_t i = 1; i + 1 < response.length(); i += 2) {
        if (response[i] == '\r' || response[i] == '\n') break;
        resp_data.push_back(asciiToByte(&response[i]));
    }
    
    // 預期資料長度: ID, Func, ByteCount, Data..., LRC
    if (resp_data.size() < 4) return false;

    uint8_t received_lrc = resp_data.back();
    resp_data.pop_back();
    if (calcLRC(resp_data) != received_lrc) return false;

    // Byte count 應為 count * 2
    if (resp_data[0] != slave_id_ || resp_data[1] != 0x03 || resp_data[2] != count * 2) return false;
    
    dest.clear();
    dest.reserve(count);
    for(uint16_t i = 0; i < count; ++i) {
        uint16_t val = (resp_data[3 + i * 2] << 8) | resp_data[4 + i * 2];
        dest.push_back(val);
    }

    return true;
}

bool RS485Comm::readRegister32(uint16_t reg_addr, uint32_t& value) {
    std::vector<uint16_t> dest;
    // 使用 readMultipleRegisters 一次讀取 2 個連續的 16 位元暫存器
    if (readMultipleRegisters(reg_addr, 2, dest) && dest.size() == 2) {
        // 根據手冊 A06H，預設是大端序 (Big Endian)，高位在前
        uint16_t high_word = dest[0];
        uint16_t low_word = dest[1];
        // 將兩個 16 位元合併成一個 32 位元
        value = (static_cast<uint32_t>(high_word) << 16) | low_word;
        return true;
    }
    return false;
}