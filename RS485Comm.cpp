#include "RS485Comm.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <termios.h>
#include <cstring>
#include <iostream>

RS485Comm::RS485Comm(const std::string& device, uint8_t slave_id)
: fd_(-1), slave_id_(slave_id) {
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
}

RS485Comm::~RS485Comm() {
    closePort();
}

bool RS485Comm::openPort(int baudrate) {
    if (fd_ < 0) return false;
    struct termios tio;
    tcgetattr(fd_, &tio);
    cfmakeraw(&tio);
    cfsetispeed(&tio, baudrate);
    cfsetospeed(&tio, baudrate);
    tio.c_cflag |= CLOCAL | CREAD | CS8;
    tio.c_cflag &= ~(PARENB | PARODD | CSTOPB);
    tcsetattr(fd_, TCSANOW, &tio);

    // RS485 模式
    struct serial_rs485 rs485conf;
    memset(&rs485conf, 0, sizeof(rs485conf));
    rs485conf.flags |= SER_RS485_ENABLED;
    ioctl(fd_, TIOCSRS485, &rs485conf);

    return true;
}

void RS485Comm::closePort() {
    if (fd_ >= 0) ::close(fd_), fd_ = -1;
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
    return write(fd_, frame, len) == (ssize_t)len;
}

bool RS485Comm::recvFrame(uint8_t* buf, size_t buf_len, size_t& recv_len) {
    // 這裡簡化為阻塞讀取固定長度
    recv_len = read(fd_, buf, buf_len);
    return recv_len > 0;
}

bool RS485Comm::writeRegister(uint16_t reg_addr, uint16_t value) {
    uint8_t frame[8];
    frame[0] = slave_id_;
    frame[1] = 0x06;                           // Func 06 :contentReference[oaicite:0]{index=0}
    frame[2] = reg_addr >> 8;
    frame[3] = reg_addr & 0xFF;
    frame[4] = value >> 8;
    frame[5] = value & 0xFF;
    uint16_t crc = calcCRC(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;
    if (!sendFrame(frame, 8)) return false;

    // 讀回 Echo
    uint8_t resp[8];
    size_t rlen;
    return recvFrame(resp, sizeof(resp), rlen) && rlen == 8;
}

bool RS485Comm::readRegister(uint16_t reg_addr, uint16_t& value) {
    uint8_t frame[8];
    frame[0] = slave_id_;
    frame[1] = 0x03;                           // Func 03 :contentReference[oaicite:1]{index=1}
    frame[2] = reg_addr >> 8;
    frame[3] = reg_addr & 0xFF;
    frame[4] = 0x00;
    frame[5] = 0x01;                           // 讀一個字
    uint16_t crc = calcCRC(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;
    if (!sendFrame(frame, 8)) return false;

    uint8_t resp[7];
    size_t rlen;
    if (!recvFrame(resp, sizeof(resp), rlen) || rlen < 7) return false;
    value = (resp[3] << 8) | resp[4];
    return true;
}
