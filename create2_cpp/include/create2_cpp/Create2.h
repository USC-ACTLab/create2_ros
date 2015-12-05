#pragma once

class Create2Impl;

class Create2
{
public:
    Create2(
      const std::string& port,
      uint32_t brcPin,
      bool useBrcPin);

    virtual ~Create2();

    void start();

    void reset();

    void stop();

    void safe();

    void full();

    void power();

    void driveDirect(
      int16_t rightWheelVelocityInMMperSec,
      int16_t leftWheelVelocityInMMperSec);

    void digitsLedsAscii(
      const char data[4]);

    int8_t temperature();

    int16_t leftEncoderCounts();

    int16_t rightEncoderCounts();

    uint16_t batteryCharge();

    uint16_t batteryCapacity();

    uint16_t voltage();

    int16_t current();

private:
  Create2Impl* impl_;
};
