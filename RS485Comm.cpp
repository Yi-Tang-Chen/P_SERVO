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

// Constructor
RS485Comm::RS485Comm(const std::string& device, uint8_t slave_id)
: hComm_((void*)-1), slave_id_(slave_id), device_name_(device) {}

// Destructor
RS485Comm::~RS485Comm() {
    closePort();
}

// openPort: Linux implementation
bool RS485Comm::openPort(int baudrate_val) {
    int serial_port = open(device_name_.c_str(), O_RDWR | O_NOCTTY);

    if (serial_port < 0) {
        std::cerr << "Error " << errno << " opening " << device_name_ << ": " << strerror(errno) << std::endl;
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
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
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

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
        closePort();
        return false;
    }

    tcflush(serial_port, TCIOFLUSH);
    return true;
}

// closePort: Linux implementation
void RS485Comm::closePort() {
    if ((intptr_t)hComm_ != -1) {
        close((intptr_t)hComm_);
        hComm_ = (void*)-1;
    }
}

// sendFrame: Linux implementation
bool RS485Comm::sendFrame(const std::string& frame) {
    int serial_port = (intptr_t)hComm_;
    if (serial_port < 0) return false;
    ssize_t bytes_written = write(serial_port, frame.c_str(), frame.length());
    return bytes_written == frame.length();
}

// recvFrame: Linux implementation
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

// --- Platform-Independent Helper Functions (No changes needed) ---

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
    for (auto & c: result) c = toupper(c);
    return result;
}

uint8_t RS485Comm::asciiToByte(const char* ascii) {
    char hex[3] = { ascii[0], ascii[1], '\0' };
    return (uint8_t)std::stoul(hex, nullptr, 16);
}

// --- High-Level Communication Functions (Corrected for Linux) ---

bool RS485Comm::writeParameter(uint16_t reg_addr, uint16_t value) {
    std::vector<uint8_t> data_rtu;
    data_rtu.push_back(slave_id_);
    data_rtu.push_back(0x06);
    data_rtu.push_back(reg_addr >> 8);
    data_rtu.push_back(reg_addr & 0xFF);
    data_rtu.push_back(value >> 8);
    data_rtu.push_back(value & 0xFF);

    uint8_t lrc = calcLRC(data_rtu);
    std::string frame = ":";
    for(uint8_t byte : data_rtu) { frame += byteToAscii(byte); }
    frame += byteToAscii(lrc);
    frame += "\r\n";

    tcflush((intptr_t)hComm_, TCIFLUSH); // Corrected: Flush input buffer
    if (!sendFrame(frame)) return false;
    std::string response;
    if (!recvFrame(response)) return false;
    
    return response.find(frame.substr(0, frame.length() - 4)) != std::string::npos;
}

bool RS485Comm::writeParameter32(uint16_t reg_addr, uint32_t value) {
    std::vector<uint8_t> data_rtu;
    data_rtu.push_back(slave_id_);
    data_rtu.push_back(0x10);
    data_rtu.push_back(reg_addr >> 8);
    data_rtu.push_back(reg_addr & 0xFF);
    data_rtu.push_back(0x00);
    data_rtu.push_back(0x02);
    data_rtu.push_back(0x04);
    uint16_t high_word = value >> 16;
    uint16_t low_word = value & 0xFFFF;
    data_rtu.push_back(high_word >> 8);
    data_rtu.push_back(high_word & 0xFF);
    data_rtu.push_back(low_word >> 8);
    data_rtu.push_back(low_word & 0xFF);

    uint8_t lrc = calcLRC(data_rtu);
    std::string frame = ":";
    for(uint8_t byte : data_rtu) { frame += byteToAscii(byte); }
    frame += byteToAscii(lrc);
    frame += "\r\n";

    tcflush((intptr_t)hComm_, TCIFLUSH); // Corrected: Flush input buffer
    if (!sendFrame(frame)) return false;
    std::string response;
    if (!recvFrame(response)) return false;

    std::string expected_prefix = ":" + byteToAscii(slave_id_) + "10" + byteToAscii(reg_addr >> 8) + byteToAscii(reg_addr & 0xFF);
    return response.rfind(expected_prefix, 0) == 0;
}

bool RS485Comm::executeAction(uint16_t reg_addr, uint16_t value) {
    std::vector<uint8_t> data_rtu;
    data_rtu.push_back(slave_id_);
    data_rtu.push_back(0x10);
    data_rtu.push_back(reg_addr >> 8);
    data_rtu.push_back(reg_addr & 0xFF);
    data_rtu.push_back(0x00);
    data_rtu.push_back(0x01);
    data_rtu.push_back(0x02);
    data_rtu.push_back(value >> 8);
    data_rtu.push_back(value & 0xFF);

    uint8_t lrc = calcLRC(data_rtu);
    std::string frame = ":";
    for(uint8_t byte : data_rtu) { frame += byteToAscii(byte); }
    frame += byteToAscii(lrc);
    frame += "\r\n";

    tcflush((intptr_t)hComm_, TCIFLUSH); // Corrected: Flush input buffer
    if (!sendFrame(frame)) return false;
    std::string response;
    if (!recvFrame(response)) return false;

    return response.find(frame.substr(0, frame.length() - 4)) != std::string::npos;
}

bool RS485Comm::readRegister(uint16_t reg_addr, uint16_t& value) {
    std::vector<uint8_t> data_rtu;
    data_rtu.push_back(slave_id_);
    data_rtu.push_back(0x03);
    data_rtu.push_back(reg_addr >> 8);
    data_rtu.push_back(reg_addr & 0xFF);
    data_rtu.push_back(0x00);
    data_rtu.push_back(0x01);

    uint8_t lrc = calcLRC(data_rtu);
    std::string frame = ":";
    for(uint8_t byte : data_rtu) { frame += byteToAscii(byte); }
    frame += byteToAscii(lrc);
    frame += "\r\n";

    tcflush((intptr_t)hComm_, TCIFLUSH); // Corrected: Flush input buffer
    if (!sendFrame(frame)) return false;

    std::string response;
    if (!recvFrame(response) || response.length() < 11) {
        return false;
    }

    if (response[0] != ':') return false;

    std::vector<uint8_t> resp_data;
    for (size_t i = 1; i + 1 < response.length(); i += 2) {
        if (response[i] == '\r' || response[i] == '\n') break;
        resp_data.push_back(asciiToByte(&response[i]));
    }
    
    if (resp_data.size() < 4) return false;

    uint8_t received_lrc = resp_data.back();
    resp_data.pop_back();
    if (calcLRC(resp_data) != received_lrc) return false;

    if (resp_data[0] != slave_id_ || resp_data[1] != 0x03) return false;

    if (resp_data.size() >= 4 && resp_data[2] == 2) {
        value = (resp_data[3] << 8) | resp_data[4];
        return true;
    }

    return false;
}

bool RS485Comm::readMultipleRegisters(uint16_t start_addr, uint16_t count, std::vector<uint16_t>& dest) {
    if (count == 0 || count > 125) return false;

    std::vector<uint8_t> data_rtu;
    data_rtu.push_back(slave_id_);
    data_rtu.push_back(0x03);
    data_rtu.push_back(start_addr >> 8);
    data_rtu.push_back(start_addr & 0xFF);
    data_rtu.push_back(count >> 8);
    data_rtu.push_back(count & 0xFF);

    uint8_t lrc = calcLRC(data_rtu);
    std::string frame = ":";
    for(uint8_t byte : data_rtu) { frame += byteToAscii(byte); }
    frame += byteToAscii(lrc);
    frame += "\r\n";

    tcflush((intptr_t)hComm_, TCIFLUSH); // Corrected: Flush input buffer
    if (!sendFrame(frame)) return false;

    std::string response;
    if (!recvFrame(response) || response.length() < 9) return false;

    if (response[0] != ':') return false;

    std::vector<uint8_t> resp_data;
    for (size_t i = 1; i + 1 < response.length(); i += 2) {
        if (response[i] == '\r' || response[i] == '\n') break;
        resp_data.push_back(asciiToByte(&response[i]));
    }
    
    if (resp_data.size() < 4) return false;

    uint8_t received_lrc = resp_data.back();
    resp_data.pop_back();
    if (calcLRC(resp_data) != received_lrc) return false;

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
    if (readMultipleRegisters(reg_addr, 2, dest) && dest.size() == 2) {
        uint16_t high_word = dest[0];
        uint16_t low_word = dest[1];
        value = (static_cast<uint32_t>(high_word) << 16) | low_word;
        return true;
    }
    return false;
}
