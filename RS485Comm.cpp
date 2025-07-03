#include "RS485Comm.h"
#include <iostream>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cstring>      // For strerror
#include <cerrno>       // For errno

// --- Linux Headers for Serial Communication ---
#include <fcntl.h>      // Contains file controls like O_RDWR
#include <termios.h>    // Contains POSIX terminal control definitions
#include <unistd.h>     // write(), read(), close()

RS485Comm::RS485Comm(const std::string& device, uint8_t slave_id)
: hComm_((void*)-1), slave_id_(slave_id), device_name_(device) {}

RS485Comm::~RS485Comm() {
    closePort();
}

bool RS485Comm::openPort(int baudrate_val) {
    // In Linux, serial ports are treated like files. O_RDWR means open for reading and writing.
    // O_NOCTTY means not to make this port the controlling terminal.
    int serial_port = open(device_name_.c_str(), O_RDWR | O_NOCTTY);

    if (serial_port < 0) {
        std::cerr << "Error " << errno << " from open: " << strerror(errno) << std::endl;
        return false;
    }

    hComm_ = (void*)(intptr_t)serial_port;

    struct termios tty;
    if(tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        closePort();
        return false;
    }

    // --- Set Port Parameters ---
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8;     // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // Set read timeouts. VMIN = 0, VTIME = 5 means it will wait up to 0.5 seconds for data.
    tty.c_cc[VTIME] = 5;
    tty.c_cc[VMIN] = 0;

    // --- Set Baud Rate ---
    speed_t baud_rate_flag;
    switch (baudrate_val) {
        case 9600:   baud_rate_flag = B9600;   break;
        case 19200:  baud_rate_flag = B19200;  break;
        case 38400:  baud_rate_flag = B38400;  break;
        case 115200: baud_rate_flag = B115200; break;
        default:     baud_rate_flag = B19200;  break;
    }
    cfsetispeed(&tty, baud_rate_flag);
    cfsetospeed(&tty, baud_rate_flag);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
        closePort();
        return false;
    }

    tcflush(serial_port, TCIOFLUSH); // Flush buffer
    return true;
}

void RS485Comm::closePort() {
    if ((intptr_t)hComm_ != -1) {
        close((intptr_t)hComm_);
        hComm_ = (void*)-1;
    }
}

bool RS485Comm::sendFrame(const std::string& frame) {
    int serial_port = (intptr_t)hComm_;
    if (serial_port < 0) return false;
    ssize_t bytes_written = write(serial_port, frame.c_str(), frame.length());
    return bytes_written == frame.length();
}

bool RS485Comm::recvFrame(std::string& response) {
    int serial_port = (intptr_t)hComm_;
    if (serial_port < 0) return false;
    char buf[256] = {0};
    ssize_t bytes_read = read(serial_port, buf, sizeof(buf) - 1);
    if (bytes_read > 0) {
        response.assign(buf, bytes_read);
        return true;
    }
    return false;
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