/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#include "handler/servo_protocol.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <iostream>
#include <string>

#define SERIAL_DEBUG false

CServoProtocol::CServoProtocol(std::shared_ptr<rclcpp::Node> node, std::string deviceName)
    : node_(node), deviceName_(deviceName) {
}

bool CServoProtocol::triggerConnection() {
    isConnected_ = false;

    struct termios options;

    // Open device
    device_ = open(deviceName_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (device_ == -1) return -2;

    // Optional: switch to blocking mode
    fcntl(device_, F_SETFL, 0);

    // Flush port
    tcflush(device_, TCIOFLUSH);

    // Get current port settings
    if (tcgetattr(device_, &options) < 0) {
        close(device_);
        return false;
    }

    // Set baud rate
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // 8N1 config
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CREAD | CLOCAL;

    // Raw input/output
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~OPOST;

    // Set timeout to 100 ms (VTIME = 1 means 0.1s)
    options.c_cc[VMIN] = 1;  // Minimum number of characters to read
    options.c_cc[VTIME] = 1;

    // Apply the configuration
    if (tcsetattr(device_, TCSANOW, &options) != 0) {
        close(device_);
        return false;
    }

    isConnected_ = true;
    return true;
}

bool CServoProtocol::isConnected() {
    return isConnected_;
}

void CServoProtocol::closeConnection() {
    close(device_);
}

bool CServoProtocol::writeToServo(uint8_t buf[], int numBytes) {
    if (!write(device_, buf, numBytes)) {
        return false;
    }
    if (SERIAL_DEBUG) {
        std::string text = "[";
        for (auto i = 0; i < 6; ++i) {
            text += std::to_string(buf[i]) + ", ";
        }
        // text.replace();
        text += "]";
        RCLCPP_INFO_STREAM(node_->get_logger(), "serial: " << text);
    }
    return true;
}

bool CServoProtocol::readCMD(uint8_t ID, uint8_t CMD_opcode, uint8_t RX_buf_size, uint8_t* RX_buf) {
    uint8_t TX_buf[s_SERVO_ALL_READ_CMDS + 3];
    bool syntax_check = false;

    TX_buf[0] = TX_buf[1] = SERVO_FRAME_HEADER;
    TX_buf[2] = ID;
    TX_buf[3] = s_SERVO_ALL_READ_CMDS;
    TX_buf[4] = CMD_opcode;
    TX_buf[5] = checksum(TX_buf);

    if (!writeToServo(TX_buf, sizeof(TX_buf))) {  // Send Read Command for 'CMD_opcode'
        return false;
    }

    int rx_size = read(device_, RX_buf, RX_buf_size);
    syntax_check = (rx_size == RX_buf_size);
    syntax_check = syntax_check && (RX_buf[0] == SERVO_FRAME_HEADER);
    syntax_check = syntax_check && (RX_buf[1] == SERVO_FRAME_HEADER);
    if (ID != 0xFE)  // Broadcast Read CMD returns real ID
    {
        syntax_check = syntax_check && (RX_buf[2] == ID);
    }
    syntax_check = syntax_check && (RX_buf[3] == RX_buf_size - 3);
    syntax_check = syntax_check && (RX_buf[4] == CMD_opcode);
    syntax_check = syntax_check && (RX_buf[RX_buf_size - 1] == checksum(RX_buf));

    return syntax_check;
}

uint8_t CServoProtocol::checksum(uint8_t buf[]) {
    uint8_t i;
    uint16_t chksum = 0;
    for (i = 2; i < buf[3] + 2; i++) {
        chksum += buf[i];
    }
    chksum = ~chksum;
    i = (uint8_t)chksum;
    return i;
}

bool CServoProtocol::getServoID(uint8_t ID, uint8_t& answer) {
    uint8_t CMD_opcode = SERVO_ID_READ;
    uint8_t RX_buf[s_SERVO_ID_READ + 3];
    bool syntax_check = readCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
    if (syntax_check) {
        answer = RX_buf[5];  // The servo ID is in the 6th byte of the response
    }
    return syntax_check;
}

bool CServoProtocol::setServoID(uint8_t ID, uint8_t newID) {
    uint8_t buf[s_SERVO_ID_WRITE + 3];

    newID = std::clamp<uint8_t>(newID, 0, 254);  // ID must be in range 0..254
    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = ID;
    buf[3] = s_SERVO_ID_WRITE;
    buf[4] = SERVO_ID_WRITE;
    buf[5] = newID;
    buf[6] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CServoProtocol::setPosition(uint8_t ID, uint16_t pos, uint16_t time) {
    uint8_t buf[s_SERVO_MOVE_TIME_WRITE + 3];

    pos = std::clamp<uint16_t>(pos, 0, 1000);
    time = std::clamp<uint16_t>(time, 0, 30000);  // time is in ms

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = ID;
    buf[3] = s_SERVO_MOVE_TIME_WRITE;
    buf[4] = SERVO_MOVE_TIME_WRITE;
    buf[5] = GET_LOW_BYTE(pos);
    buf[6] = GET_HIGH_BYTE(pos);
    buf[7] = GET_LOW_BYTE(time);
    buf[8] = GET_HIGH_BYTE(time);
    buf[9] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CServoProtocol::setRegPos(uint8_t ID, uint16_t pos, uint16_t time) {
    uint8_t buf[s_SERVO_MOVE_TIME_WAIT_WRITE + 3];

    pos = std::clamp<uint16_t>(pos, 0, 1000);
    time = std::clamp<uint16_t>(time, 0, 30000);  // time is in ms

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = ID;
    buf[3] = s_SERVO_MOVE_TIME_WAIT_WRITE;
    buf[4] = SERVO_MOVE_TIME_WAIT_WRITE;
    buf[5] = GET_LOW_BYTE(pos);
    buf[6] = GET_HIGH_BYTE(pos);
    buf[7] = GET_LOW_BYTE(time);
    buf[8] = GET_HIGH_BYTE(time);
    buf[9] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CServoProtocol::actionStart(uint8_t ID) {
    uint8_t buf[s_SERVO_MOVE_START + 3];

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = ID;
    buf[3] = s_SERVO_MOVE_START;
    buf[4] = SERVO_MOVE_START;
    buf[5] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CServoProtocol::moveStop(uint8_t ID) {
    uint8_t buf[s_SERVO_MOVE_STOP + 3];

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = ID;
    buf[3] = s_SERVO_MOVE_STOP;
    buf[4] = SERVO_MOVE_STOP;
    buf[5] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CServoProtocol::getPositionOffset(uint8_t ID, int8_t& deviation) {
    uint8_t CMD_opcode = SERVO_ANGLE_OFFSET_READ;
    uint8_t RX_buf[s_SERVO_ANGLE_OFFSET_READ + 3];
    bool syntax_check = readCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
    // The deviation is stored as an unsigned byte (0..255), but represents a signed value (-128..+127)
    uint8_t raw = RX_buf[5];
    if (raw > 127) {
        deviation = static_cast<int8_t>(raw) - 127;
    } else {
        deviation = static_cast<int8_t>(raw);
    }
    return syntax_check;
}

bool CServoProtocol::setPositionOffset(uint8_t ID, int8_t deviation) {
    uint8_t buf[s_SERVO_ANGLE_OFFSET_ADJUST + 3];

    // Store as unsigned byte (0..255), but represents signed value (-128..+127)
    uint8_t deviationU8 = 0;
    if (deviation < 0) {
        deviationU8 = static_cast<uint8_t>(deviation) + 127;
    } else {
        deviationU8 = static_cast<uint8_t>(deviation);
    }
    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = ID;
    buf[3] = s_SERVO_ANGLE_OFFSET_ADJUST;
    buf[4] = SERVO_ANGLE_OFFSET_ADJUST;
    buf[5] = deviationU8;
    buf[6] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CServoProtocol::savePositionOffset(uint8_t ID) {
    uint8_t buf[s_SERVO_ANGLE_OFFSET_WRITE + 3];

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = ID;
    buf[3] = s_SERVO_ANGLE_OFFSET_WRITE;
    buf[4] = SERVO_ANGLE_OFFSET_WRITE;
    buf[5] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CServoProtocol::getPositionLimits(uint8_t ID, uint16_t& min_position, uint16_t& max_position) {
    uint8_t CMD_opcode = SERVO_ANGLE_LIMITS_READ;
    uint8_t RX_buf[s_SERVO_ANGLE_LIMITS_READ + 3];
    bool syntax_check = readCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
    if (syntax_check) {
        min_position = BYTE_TO_HW(RX_buf[6], RX_buf[5]);
        max_position = BYTE_TO_HW(RX_buf[8], RX_buf[7]);
    }
    return syntax_check;
}

bool CServoProtocol::setPositionLimits(uint8_t ID, uint16_t min_position, uint16_t max_position) {
    uint8_t buf[s_SERVO_ANGLE_LIMITS_WRITE + 3];

    min_position = std::clamp<uint16_t>(min_position, 0, 1000);
    max_position = std::clamp<uint16_t>(max_position, 0, 1000);

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = ID;
    buf[3] = s_SERVO_ANGLE_LIMITS_WRITE;
    buf[4] = SERVO_ANGLE_LIMITS_WRITE;
    buf[5] = GET_LOW_BYTE(min_position);
    buf[6] = GET_HIGH_BYTE(min_position);
    buf[7] = GET_LOW_BYTE(max_position);
    buf[8] = GET_HIGH_BYTE(max_position);
    buf[9] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CServoProtocol::flashLedErrCode(uint8_t ID, uint8_t code) {
    uint8_t buf[s_SERVO_LED_ERROR_WRITE + 3];

    code = std::clamp<uint8_t>(code, 0, 7);  // b0=temp, b1:voltage, b2:stalled

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = ID;
    buf[3] = s_SERVO_LED_ERROR_WRITE;
    buf[4] = SERVO_LED_ERROR_WRITE;
    buf[5] = code;
    buf[6] = checksum(buf);

    usleep(50 * 1000);  // need to wait until Flash in Servo is written

    return writeToServo(buf, sizeof(buf));
}

bool CServoProtocol::getPosition(uint8_t ID, int16_t& pos) {
    uint8_t CMD_opcode = SERVO_POS_READ;
    uint8_t RX_buf[s_SERVO_POS_READ + 3];
    bool syntax_check = readCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
    pos = BYTE_TO_HW(RX_buf[6], RX_buf[5]);
    return syntax_check;
}

bool CServoProtocol::getVoltage(uint8_t ID, uint16_t& vin) {
    uint8_t CMD_opcode = SERVO_VIN_READ;
    uint8_t RX_buf[s_SERVO_VIN_READ + 3];
    bool syntax_check = readCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
    vin = BYTE_TO_HW(RX_buf[6], RX_buf[5]);
    return syntax_check;
}

bool CServoProtocol::getVoltageLimits(uint8_t ID, uint16_t& min_voltage, uint16_t& max_voltage) {
    uint8_t CMD_opcode = SERVO_VIN_LIMITS_READ;
    uint8_t RX_buf[s_SERVO_VIN_LIMITS_READ + 3];
    bool syntax_check = readCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
    if (syntax_check) {
        min_voltage = BYTE_TO_HW(RX_buf[6], RX_buf[5]);
        max_voltage = BYTE_TO_HW(RX_buf[8], RX_buf[7]);
    }
    return syntax_check;
}

bool CServoProtocol::setVoltageLimits(uint8_t ID, uint16_t min_voltage, uint16_t max_voltage) {
    uint8_t buf[s_SERVO_VIN_LIMITS_WRITE + 3];

    min_voltage = std::clamp<uint16_t>(min_voltage, 4500, 12000);
    max_voltage = std::clamp<uint16_t>(max_voltage, 4500, 12000);

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = ID;
    buf[3] = s_SERVO_VIN_LIMITS_WRITE;
    buf[4] = SERVO_VIN_LIMITS_WRITE;
    buf[5] = GET_LOW_BYTE(min_voltage);
    buf[6] = GET_HIGH_BYTE(min_voltage);
    buf[7] = GET_LOW_BYTE(max_voltage);
    buf[8] = GET_HIGH_BYTE(max_voltage);
    buf[9] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CServoProtocol::getTemperature(uint8_t ID, uint8_t& temp) {
    uint8_t CMD_opcode = SERVO_TEMP_READ;
    uint8_t RX_buf[s_SERVO_TEMP_READ + 3];
    bool syntax_check = readCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
    temp = RX_buf[5];
    return syntax_check;
}

bool CServoProtocol::getMaxTemperatureLimit(uint8_t ID, uint8_t& max_temperature) {
    uint8_t CMD_opcode = SERVO_TEMP_LIMIT_READ;
    uint8_t RX_buf[s_SERVO_TEMP_LIMIT_READ + 3];
    bool syntax_check = readCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
    max_temperature = RX_buf[5];
    return syntax_check;
}

bool CServoProtocol::setMaxTemperatureLimit(uint8_t ID, uint8_t max_temperature) {
    uint8_t buf[s_SERVO_TEMP_LIMIT_WRITE + 3];

    max_temperature = std::clamp<uint8_t>(max_temperature, 50, 100);

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = ID;
    buf[3] = s_SERVO_TEMP_LIMIT_WRITE;
    buf[4] = SERVO_TEMP_LIMIT_WRITE;
    buf[5] = max_temperature;
    buf[6] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CServoProtocol::getLedErrcode(uint8_t ID, uint8_t& lederrcode) {
    uint8_t CMD_opcode = SERVO_LED_ERROR_READ;
    uint8_t RX_buf[s_SERVO_LED_ERROR_READ + 3];
    bool syntax_check = readCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
    lederrcode = RX_buf[5];
    return syntax_check;
}

bool CServoProtocol::getMode(uint8_t ID, uint8_t& mode) {
    uint8_t CMD_opcode = SERVO_OR_MOTOR_MODE_READ;
    uint8_t RX_buf[s_SERVO_MODE_READ + 3];
    bool syntax_check = readCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
    if (syntax_check) {
        mode = RX_buf[5];  // The mode is in the 6th byte of the response
    }
    return syntax_check;
}

bool CServoProtocol::getMotorSpeed([[maybe_unused]] uint8_t ID, [[maybe_unused]] int16_t& speed) {
    // This function is not implemented in the original code.
    // If you need to implement it, you can add the logic here.
    // For now, we return false to indicate that this functionality is not available.
    return false;
}

bool CServoProtocol::setServoMode(uint8_t ID) {
    uint8_t buf[s_SERVO_MODE_WRITE + 3];

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = ID;
    buf[3] = s_SERVO_MODE_WRITE;
    buf[4] = SERVO_OR_MOTOR_MODE_WRITE;
    buf[5] = 0;  // Mode 0 for servo
    buf[6] = 0;  // Speed is not used in servo mode
    buf[7] = 0;  // Speed is not used in servo mode
    buf[8] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CServoProtocol::setMotorMode(uint8_t ID, int16_t speed) {
    uint8_t buf[s_SERVO_MODE_WRITE + 3];

    speed = std::clamp<int16_t>(speed, -1000, 1000);
    if (speed < 0) {
        speed += 65536;  // Convert to unsigned value
    }

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = ID;
    buf[3] = s_SERVO_MODE_WRITE;
    buf[4] = SERVO_OR_MOTOR_MODE_WRITE;
    buf[5] = 1;  // Mode 1 for motor
    buf[6] = GET_LOW_BYTE(speed);
    buf[7] = GET_HIGH_BYTE(speed);
    buf[8] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CServoProtocol::setTorque(uint8_t ID, bool active) {
    uint8_t buf[s_SERVO_TORQUE_WRITE + 3];

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = ID;
    buf[3] = s_SERVO_TORQUE_WRITE;
    buf[4] = SERVO_TORQUE_WRITE;
    buf[5] = uint8_t(active);  // 1 for Load motor, 0 for Unload motor
    buf[6] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}

bool CServoProtocol::isLedOn(uint8_t ID) {
    uint8_t CMD_opcode = SERVO_LED_CTRL_READ;
    uint8_t RX_buf[s_SERVO_LED_CTRL_READ + 3];
    bool syntax_check = readCMD(ID, CMD_opcode, sizeof(RX_buf), RX_buf);
    return syntax_check && (RX_buf[5] == 0);  // Check if LED is on (0 means on)
}

bool CServoProtocol::setLed(uint8_t ID, bool on) {
    uint8_t buf[s_SERVO_LED_CTRL_WRITE + 3];

    buf[0] = buf[1] = SERVO_FRAME_HEADER;
    buf[2] = ID;
    buf[3] = s_SERVO_LED_CTRL_WRITE;
    buf[4] = SERVO_LED_CTRL_WRITE;
    buf[5] = uint8_t(!on);  // 0 for on, 1 for off
    buf[6] = checksum(buf);

    return writeToServo(buf, sizeof(buf));
}
