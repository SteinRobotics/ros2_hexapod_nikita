/*******************************************************************************
 * Copyright (c) 2023 Christian Stein
 ******************************************************************************/

#pragma once

#include <fcntl.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

#include "rclcpp/rclcpp.hpp"

namespace nikita_movement {

constexpr uint8_t SERVO_FRAME_HEADER = 0x55;
constexpr uint8_t SERVO_Broadcast_ID = 0xFE;

#define GET_LOW_BYTE(A) (uint8_t)((A))        // Macro to extract lower 8 bits from A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)  // Macro to extract high 8 bits from A
#define BYTE_TO_HW(A, B) \
    ((((uint16_t)(A)) << 8) | (uint8_t)(B))  // Macro to combine A & B into uint16_t (little endian)

// Instruction Op-Codes:
constexpr uint8_t SERVO_MOVE_TIME_WRITE = 1;
constexpr uint8_t SERVO_MOVE_TIME_READ = 2;
constexpr uint8_t SERVO_MOVE_TIME_WAIT_WRITE = 7;
constexpr uint8_t SERVO_MOVE_TIME_WAIT_READ = 8;
constexpr uint8_t SERVO_MOVE_START = 11;
constexpr uint8_t SERVO_MOVE_STOP = 12;
constexpr uint8_t SERVO_ID_WRITE = 13;
constexpr uint8_t SERVO_ID_READ = 14;
constexpr uint8_t SERVO_ANGLE_OFFSET_ADJUST = 17;
constexpr uint8_t SERVO_ANGLE_OFFSET_WRITE = 18;
constexpr uint8_t SERVO_ANGLE_OFFSET_READ = 19;
constexpr uint8_t SERVO_ANGLE_LIMITS_WRITE = 20;
constexpr uint8_t SERVO_ANGLE_LIMITS_READ = 21;
constexpr uint8_t SERVO_VIN_LIMITS_WRITE = 22;
constexpr uint8_t SERVO_VIN_LIMITS_READ = 23;
constexpr uint8_t SERVO_TEMP_LIMIT_WRITE = 24;
constexpr uint8_t SERVO_TEMP_LIMIT_READ = 25;
constexpr uint8_t SERVO_TEMP_READ = 26;
constexpr uint8_t SERVO_VIN_READ = 27;
constexpr uint8_t SERVO_POS_READ = 28;
constexpr uint8_t SERVO_OR_MOTOR_MODE_WRITE = 29;
constexpr uint8_t SERVO_OR_MOTOR_MODE_READ = 30;
constexpr uint8_t SERVO_TORQUE_WRITE = 31;
constexpr uint8_t SERVO_TORQUE_READ = 32;
constexpr uint8_t SERVO_LED_CTRL_WRITE = 33;
constexpr uint8_t SERVO_LED_CTRL_READ = 34;
constexpr uint8_t SERVO_LED_ERROR_WRITE = 35;
constexpr uint8_t SERVO_LED_ERROR_READ = 36;

// Packet Sizes (sending):
constexpr uint8_t s_SERVO_MOVE_TIME_WRITE = 7;
constexpr uint8_t s_SERVO_MOVE_TIME_WAIT_WRITE = 7;
constexpr uint8_t s_SERVO_MOVE_START = 3;
constexpr uint8_t s_SERVO_MOVE_STOP = 3;
constexpr uint8_t s_SERVO_ID_WRITE = 4;
constexpr uint8_t s_SERVO_ANGLE_OFFSET_ADJUST = 4;
constexpr uint8_t s_SERVO_ANGLE_OFFSET_WRITE = 3;
constexpr uint8_t s_SERVO_ANGLE_LIMITS_WRITE = 7;
constexpr uint8_t s_SERVO_VIN_LIMITS_WRITE = 7;
constexpr uint8_t s_SERVO_TEMP_LIMIT_WRITE = 4;
constexpr uint8_t s_SERVO_MODE_WRITE = 7;
constexpr uint8_t s_SERVO_TORQUE_WRITE = 4;
constexpr uint8_t s_SERVO_LED_CTRL_WRITE = 4;
constexpr uint8_t s_SERVO_LED_ERROR_WRITE = 4;

constexpr uint8_t s_SERVO_ALL_READ_CMDS = 3;

// Packet Sizes (receiving):
constexpr uint8_t s_SERVO_MOVE_TIME_READ = 7;
constexpr uint8_t s_SERVO_MOVE_TIME_WAIT_READ = 7;
constexpr uint8_t s_SERVO_ID_READ = 4;
constexpr uint8_t s_SERVO_ANGLE_OFFSET_READ = 4;
constexpr uint8_t s_SERVO_ANGLE_LIMITS_READ = 7;
constexpr uint8_t s_SERVO_VIN_LIMITS_READ = 7;
constexpr uint8_t s_SERVO_TEMP_LIMIT_READ = 4;
constexpr uint8_t s_SERVO_TEMP_READ = 4;
constexpr uint8_t s_SERVO_VIN_READ = 5;
constexpr uint8_t s_SERVO_POS_READ = 5;
constexpr uint8_t s_SERVO_MODE_READ = 7;
constexpr uint8_t s_SERVO_TORQUE_READ = 4;
constexpr uint8_t s_SERVO_LED_CTRL_READ = 4;
constexpr uint8_t s_SERVO_LED_ERROR_READ = 4;

class CServoProtocol {
   public:
    CServoProtocol(std::shared_ptr<rclcpp::Node> node, const std::string deviceName);
    virtual ~CServoProtocol() = default;

    // Connection
    virtual bool triggerConnection();
    virtual void closeConnection();
    virtual bool isConnected();

    // Generic Commands
    virtual bool getServoID(uint8_t ID, uint8_t& answer);
    virtual bool setServoID(uint8_t ID, uint8_t newID);        // set new ID for servo, 0..254
    virtual bool getMode(uint8_t ID, uint8_t& mode);           // get servo mode, 0 = servo, 1 = motor
    virtual bool setServoMode(uint8_t ID);                     // set servo mode
    virtual bool setMotorMode(uint8_t ID, int16_t speed = 0);  // set motor mode, speed in 0.1°/s, -1000..1000
    virtual bool setTorque(uint8_t ID, bool active);           // set motor torque active

    // Position
    virtual bool setPosition(uint8_t ID, uint16_t pos, uint16_t timeMs);  // move to pos 0..1000 (=0..240°)
    virtual bool setRegPos(uint8_t ID, uint16_t pos,
                           uint16_t timeMs);  // same as above but wait until actionStart
    virtual bool actionStart(
        uint8_t ID =
            SERVO_Broadcast_ID);  // start registered movement, use with SERVO_Broadcast_ID to start all servos
    virtual bool moveStop(
        uint8_t ID = SERVO_Broadcast_ID);  // stop ongoing action, use SERVO_Broadcast_ID to stop all servos
    virtual bool getPosition(uint8_t ID, int16_t& pos);  // get current pos 0..1000 (= 0~240°)
    virtual bool getPositionLimits(uint8_t ID, uint16_t& min_position, uint16_t& max_position);
    virtual bool setPositionLimits(uint8_t ID, uint16_t min_position, uint16_t max_position);
    virtual bool getPositionOffset(uint8_t ID,
                                   int8_t& deviation);  // get position offset in °, 0..1000 = 0°..240°
    virtual bool setPositionOffset(uint8_t ID,
                                   int8_t deviation);  // set position offset in °, 0..1000 = 0°..240°
    virtual bool savePositionOffset(
        uint8_t ID);  // save position offset in servo flash, use after setPositionOffset

    // Motor Control
    virtual bool getMotorSpeed(uint8_t ID, int16_t& speed);  // get current motor speed in 0.1°/s, -1000..1000

    // Voltage
    virtual bool getVoltage(uint8_t ID, uint16_t& mV);  // get current Vin 4500mV..12000mV
    virtual bool getVoltageLimits(uint8_t ID, uint16_t& mVmin, uint16_t& mVmax);
    virtual bool setVoltageLimits(uint8_t ID, uint16_t mVmin, uint16_t mVmax);

    // Temperature
    virtual bool getTemperature(uint8_t ID, uint8_t& degTemp);  // get current Temp 0..100°C
    virtual bool getMaxTemperatureLimit(uint8_t ID, uint8_t& degLimit);
    virtual bool setMaxTemperatureLimit(uint8_t ID, uint8_t degLimit);  // set max temperature limit in °C

    // LED
    virtual bool isLedOn(uint8_t ID);          // check if LED is on
    virtual bool setLed(uint8_t ID, bool on);  // set LED on or off

    // Error handling
    // get LED error code, b0=temp, b1:voltage, b2:stalled -> also clears errors?
    virtual bool getLedErrcode(uint8_t ID, uint8_t& lederrcode);
    virtual bool flashLedErrCode(uint8_t ID, uint8_t code);

   protected:
    std::shared_ptr<rclcpp::Node> node_;

   private:
    bool initializeSerialConnection(const std::string& deviceName_);
    bool writeToServo(uint8_t buf[], int numBytes);
    bool readCMD(uint8_t ID, uint8_t CMD_opcode, uint8_t RX_buf_size, uint8_t* RX_buf);
    uint8_t checksum(uint8_t buf[]);

    std::string deviceName_ = "/dev/ttyUSB0";
    bool isConnected_ = false;
    int device_ = -1;
};

}  // namespace nikita_movement
