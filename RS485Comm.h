#ifndef RS485COMM_H
#define RS485COMM_H

#include <cstdint>
#include <string>

class RS485Comm {
    public:
        // device: 串口裝置 (e.g. "COM1")；slave_id: Modbus ID
        RS485Comm(const std::string& device, uint8_t slave_id);
        ~RS485Comm();

        // 打開串口並設定 8-N-1，預設 19200bps
        bool openPort(int baudrate = 19200);
        void closePort();

        // 寫入單一暫存器（Modbus Function 0x06）
        bool writeRegister(uint16_t reg_addr, uint16_t value);
        // 讀取單一暫存器（Modbus Function 0x03）
        bool readRegister(uint16_t reg_addr, uint16_t& value);

    private:
        void* hComm_; // 在 Windows 中使用 HANDLE
        uint8_t slave_id_;
        std::string device_name_;

        // 計算 CRC-16(Modbus)
        uint16_t calcCRC(const uint8_t* data, size_t len);
        bool     sendFrame(const uint8_t* frame, size_t len);
        bool     recvFrame(uint8_t* buf, size_t buf_len, size_t& recv_len);
};

#endif // RS485COMM_H