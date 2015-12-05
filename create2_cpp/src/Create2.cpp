#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>

#include <boost/algorithm/clamp.hpp>
#include <boost/endian/arithmetic.hpp>

#include <serial/serial.h>

#include "Create2.h"
#include "Create2Types.h"

using namespace boost::endian;

class Create2Impl
{
public:
    Create2Impl(
      const std::string& port,
      const uint32_t brcPin,
      bool useBrcPin)
      : serial_(port, 115200, serial::Timeout::simpleTimeout(1000))
      , brcPin_(brcPin)
      , useBrcPin_(useBrcPin)
      , mode_(ModeOff)
  {
    if (useBrcPin_)
    {
      std::cout << brcPin_ << std::endl;

      // enable GPIO
      std::ofstream gpioExport("/sys/class/gpio/export");
      gpioExport << brcPin_ << std::flush;

      // enable GPIO as output
      std::stringstream sstr;
      sstr << "/sys/class/gpio/gpio" << brcPin_ << "/direction";
      std::ofstream gpioDirection(sstr.str());
      gpioDirection << "high" << std::flush;

      // pulse GPIO
      sstr.clear();
      sstr.str(std::string());
      sstr << "/sys/class/gpio/gpio" << brcPin_ << "/value";
      std::cout << sstr.str() << std::endl;
      std::ofstream gpioValue(sstr.str());
      // gpioValue << 1 << std::flush;
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      gpioValue << 0 << std::flush;
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      gpioValue << 1 << std::flush;
    }
  }

    ~Create2Impl()
    {
      std::cout << "destruct createImpl" << std::endl;

      if (useBrcPin_)
      {
        // disable GPIO
        std::ofstream gpioExport("/sys/class/gpio/unexport");
        gpioExport << brcPin_ << std::flush;
      }
    }

  void sensors(PacketID p)
  {
    uint8_t data = (uint8_t)p;
    send(OpSensors, &data, 1);
  }

  void send(
    Op op,
    const uint8_t* data = nullptr,
    size_t numBytes = 0)
  {
    uint8_t opData = (uint8_t)op;
    serial_.flush();
    size_t written = serial_.write(&opData, 1);
    if (written != 1) {
      throw std::runtime_error("Couldn't write enough data!");
    }
    if (data && numBytes) {
      written = serial_.write(data, numBytes);
      if (written != numBytes) {
        throw std::runtime_error("Couldn't write enough data!");
      }
    }
  }

  uint16_t readUint16()
  {
    big_uint16_t result;
    size_t read = serial_.read((uint8_t*)&result, 2);
    if (read != 2) {
      throw std::runtime_error("Couldn't read enough data!");
    }
    return result;
  }

  int8_t readInt8()
  {
    big_int8_t result;
    size_t read = serial_.read((uint8_t*)&result, 1);
    if (read != 1) {
      throw std::runtime_error("Couldn't read enough data!");
    }
    return result;
  }

  int16_t readInt16()
  {
    big_int16_t result;
    size_t read = serial_.read((uint8_t*)&result, 2);
    if (read != 2) {
      throw std::runtime_error("Couldn't read enough data!");
    }
    return result;
  }

// private:
  serial::Serial serial_;
  uint32_t brcPin_;
  bool useBrcPin_;
  Mode mode_;
};

/////////////////////////////////////////////////////////

Create2::Create2(
  const std::string& port,
  uint32_t brcPin,
  bool useBrcPin)
  : impl_(nullptr)
{
  impl_ = new Create2Impl(port, brcPin, useBrcPin);
}

Create2::~Create2()
{
  stop();
  delete impl_;
}

void Create2::start()
{
  impl_->send(OpStart);
  // TODO: check for success?
  std::this_thread::sleep_for(std::chrono::milliseconds(20)); // wait ~20ms for mode changes
  impl_->mode_ = ModePassive;
}

void Create2::reset()
{
  impl_->send(OpReset);
  std::this_thread::sleep_for(std::chrono::milliseconds(20)); // wait ~20ms for mode changes
  impl_->mode_ = ModeOff;
}

void Create2::stop()
{
  impl_->send(OpStop);
  std::this_thread::sleep_for(std::chrono::milliseconds(20)); // wait ~20ms for mode changes
  impl_->mode_ = ModeOff;
}

void Create2::safe()
{
  impl_->send(OpSafe);
  std::this_thread::sleep_for(std::chrono::milliseconds(20)); // wait ~20ms for mode changes
  impl_->mode_ = ModeSafe;
}

void Create2::full()
{
  impl_->send(OpFull);
  std::this_thread::sleep_for(std::chrono::milliseconds(20)); // wait ~20ms for mode changes
  impl_->mode_ = ModeFull;
}

void Create2::power()
{
  impl_->send(OpPower);
  std::this_thread::sleep_for(std::chrono::milliseconds(20)); // wait ~20ms for mode changes
  impl_->mode_ = ModePassive;
}

void Create2::driveDirect(
  int16_t rightWheelVelocityInMMperSec,
  int16_t leftWheelVelocityInMMperSec)
{
  //assert(mode_ == ModeSafe || mode_ == ModeFull);
  //assert(rightWheelVelocityInMMperSec >= -500 && rightWheelVelocityInMMperSec <= 500);
  //assert(leftWheelVelocityInMMperSec >= -500 && leftWheelVelocityInMMperSec <= 500);

  rightWheelVelocityInMMperSec = boost::algorithm::clamp(rightWheelVelocityInMMperSec, -500, 500);
  leftWheelVelocityInMMperSec = boost::algorithm::clamp(leftWheelVelocityInMMperSec, -500, 500);

  struct data {
    big_int16_t rightVelocity;
    big_int16_t leftVelocity;
  };
  data d{rightWheelVelocityInMMperSec, leftWheelVelocityInMMperSec};

  impl_->send(OpDriveDirect, (const uint8_t*)&d, sizeof(d));
}

void Create2::digitsLedsAscii(
  const char data[4])
{
  impl_->send(OpDigitsLedsAscii, (const uint8_t*)data, 4);
}

int8_t Create2::temperature()
{
  impl_->sensors(SensorTemperature);
  return impl_->readInt8();
}

int16_t Create2::leftEncoderCounts()
{
  impl_->sensors(SensorLeftEncoderCounts);
  return impl_->readInt16();
}

int16_t Create2::rightEncoderCounts()
{
  impl_->sensors(SensorRightEncoderCounts);
  return impl_->readInt16();
}

uint16_t Create2::batteryCharge()
{
  impl_->sensors(SensorBatteryCharge);
  return impl_->readUint16();
}

uint16_t Create2::batteryCapacity()
{
  impl_->sensors(SensorBatteryCapacity);
  return impl_->readUint16();
}

uint16_t Create2::voltage()
{
  impl_->sensors(SensorVoltage);
  return impl_->readUint16();
}

int16_t Create2::current()
{
  impl_->sensors(SensorCurrent);
  return impl_->readInt16();
}
