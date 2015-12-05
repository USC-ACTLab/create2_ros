#pragma once

class Create2Impl;

class Create2
{
public:
  enum SensorID {
    SensorButtons                     = 18,
    SensorDistance                    = 19, //broken on firmware < 3.3.0
    SensorAngle                       = 20, //broken on firmware < 3.4.0
    SensorChargingState               = 21, // one of ChargingState
    SensorVoltage                     = 22, //mV
    SensorCurrent                     = 23, //mA
    SensorTemperature                 = 24, //degC
    SensorBatteryCharge               = 25, //mAh
    SensorBatteryCapacity             = 26,
    SensorCliffLeftSignal             = 28,
    SensorCliffFrontLeftSignal        = 29,
    SensorCliffFrontRightSignal       = 30,
    SensorCliffRightSignal            = 31,
    SensorChargingSourcesAvailable    = 34,
    SensorOIMode                      = 35, // one of Mode
    SensorSongPlaying                 = 37,
    SensorNumberOfStreamPackets       = 38,
    SensorRequestedVelocity           = 39,
    SensorRequestedRadius             = 40,
    SensorRequestedRightVelocity      = 41,
    SensorRequestedLeftVelocity       = 42,
    SensorLeftEncoderCounts           = 43,
    SensorRightEncoderCounts          = 44,
    SensorRightBumper                 = 45,
    SensorLightBumpLeftSignal         = 46,
    SensorLightBumpFrontLeftSignal    = 47,
    SensorLightBumpCenterLeftSignal   = 48,
    SensorLightBumpCenterRightSignal  = 49,
    SensorLightBumpFrontRightSignal   = 50,
    SensorLightBumpRightSignal        = 51,
    SensorLeftMotorCurrent            = 54, //mA
    SensorRightMotorCurrent           = 55, //mA
    SensorMainBrushMotorCurrent       = 56, //mA
    SensorSideBrushMotorCurrent       = 57, //mA
    SensorStatus                      = 58,
  };

  enum Mode {
    ModeOff       = 0,
    ModePassive   = 1,
    ModeSafe      = 2,
    ModeFull      = 3,
  };

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

    void startStream(
      std::vector<SensorID> ids);

    void update();

    virtual void onMode(
      Mode mode)
    {
    }

    virtual void onVoltage(
      uint16_t voltageInMV)
    {
    }

    virtual void onCurrent(
      int16_t currentInMA)
    {
    }

    virtual void onTemperature(
      int8_t temperatureInDegCelcius)
    {
    }

    virtual void onBatteryCharge(
      uint16_t chargeInMAH)
    {
    }

    virtual void onBatteryCapacity(
      uint16_t capacityInMAH)
    {
    }

    virtual void onCliffLeft(
      uint16_t signalStrength)
    {
    }

    virtual void onCliffFrontLeft(
      uint16_t signalStrength)
    {
    }

    virtual void onCliffFrontRight(
      uint16_t signalStrength)
    {
    }

    virtual void onCliffRight(
      uint16_t signalStrength)
    {
    }

    virtual void onLeftEncoderCounts(
      int16_t count)
    {
    }

    virtual void onRightEncoderCounts(
      int16_t count)
    {
    }

    // int8_t temperature();

    // int16_t leftEncoderCounts();

    // int16_t rightEncoderCounts();

    // uint16_t batteryCharge();

    // uint16_t batteryCapacity();

    // uint16_t voltage();

    // int16_t current();

private:
  Create2Impl* impl_;
};
