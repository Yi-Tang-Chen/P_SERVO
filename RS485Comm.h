#ifndef RS485COMM_H
#define RS485COMM_H

#include <cstdint>
#include <string>
#include <vector>

class RS485Comm {
public:
    RS485Comm(const std::string& device, uint8_t slave_id);
    ~RS485Comm();

    bool openPort(int baudrate = 19200);
    void closePort();

    bool writeRegister(uint16_t reg_addr, uint16_t value);
    bool readRegister(uint16_t reg_addr, uint16_t& value);

private:
    void* hComm_;
    uint8_t slave_id_;
    std::string device_name_;

    // --- 新增的 Modbus ASCII 輔助函式 ---

    // 計算 LRC 校驗碼
    uint8_t calcLRC(const std::vector<uint8_t>& data);

    // 將一個 byte 轉成兩個 ASCII hex 字元 (e.g., 0x1A -> "1A")
    std::string byteToAscii(uint8_t byte);

    // 將兩個 ASCII hex 字元轉成一個 byte (e.g., "1A" -> 0x1A)
    uint8_t asciiToByte(const char* ascii);

    bool sendFrame(const std::string& frame);
    bool recvFrame(std::string& response);
};

#endif // RS485COMM_H