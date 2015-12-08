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
      : serial_(port, 115200, serial::Timeout::simpleTimeout(100))
      , brcPin_(brcPin)
      , useBrcPin_(useBrcPin)
      , mode_(Create2::ModeOff)
  {
    if (useBrcPin_)
    {
      std::cout << brcPin_ << std::endl;

      // enable GPIO
      std::ofstream gpioExport("/sys/class/gpio/export");
      if (gpioExport.good())
      {
        gpioExport << brcPin_ << std::flush;

        // enable GPIO as output
        std::stringstream sstr;
        sstr << "/sys/class/gpio/gpio" << brcPin_ << "/direction";
        std::ofstream gpioDirection;
        do {
          gpioDirection.open(sstr.str());
          if (gpioDirection.good()) {
            gpioDirection << "high" << std::flush;
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          std::cout << "waiting for " << sstr.str() << std::endl;
        } while(true);


        // pulse GPIO
        sstr.clear();
        sstr.str(std::string());
        sstr << "/sys/class/gpio/gpio" << brcPin_ << "/value";
        std::cout << sstr.str() << std::endl;
        std::ofstream gpioValue(sstr.str());
        std::cout << gpioValue.good() << std::endl;
        // gpioValue << 1 << std::flush;
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        gpioValue << 0 << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        gpioValue << 1 << std::flush;
      }
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

  void sensors(Create2::SensorID p)
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
  Create2::Mode mode_;
  std::vector<uint8_t> readBuffer_;
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
  // stop();
  power();
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

void Create2::startStream(
  std::vector<SensorID> ids)
{
  std::vector<uint8_t> data;
  data.push_back((uint8_t)ids.size());
  data.insert(data.end(), ids.begin(), ids.end());
  impl_->send(OpStream, &data[0], data.size());
}

void Create2::update()
{
  impl_->serial_.read(impl_->readBuffer_, impl_->serial_.available());

  for (size_t i = 0; i + 1 < impl_->readBuffer_.size(); ++i) {
    if (impl_->readBuffer_[i] == 19) {
      uint8_t size = impl_->readBuffer_[i+1];
      if (impl_->readBuffer_.size() > size + i + 2) {
        // check checksum
        uint32_t sum = 0;
        for (size_t j = i; j <= size + i + 2; ++j) {
          sum += impl_->readBuffer_[j];
          std::cout << (int)impl_->readBuffer_[j] << " ";
        }

        if ((sum & 0xFF) == 0) {
          // parse packet
          State state;
          size_t pos = i + 2;
          while (pos < i + size + 2) {
            SensorID id = (SensorID)impl_->readBuffer_[pos];
            ++pos;
            switch(id) {
            case SensorOIMode:
              {
                state.mode = (Mode)impl_->readBuffer_[pos];
                pos += 1;
                break;
              }
            case SensorVoltage:
              {
                big_uint16_t result;
                memcpy(&result, &impl_->readBuffer_[pos], sizeof(big_uint16_t));
                state.voltageInMV = result;
                pos += 2;
                break;
              }
            case SensorCurrent:
              {
                big_int16_t result;
                memcpy(&result, &impl_->readBuffer_[pos], sizeof(big_int16_t));
                state.currentInMA = result;
                pos += 2;
                break;
              }
            case SensorTemperature:
              {
                uint8_t result;
                memcpy(&result, &impl_->readBuffer_[pos], sizeof(uint8_t));
                state.temperatureInDegCelcius = result;
                pos += 1;
                break;
              }
            case SensorBatteryCharge:
              {
                big_uint16_t result;
                memcpy(&result, &impl_->readBuffer_[pos], sizeof(big_uint16_t));
                state.batteryChargeInMAH = result;
                pos += 2;
                break;
              }
            case SensorBatteryCapacity:
              {
                big_uint16_t result;
                memcpy(&result, &impl_->readBuffer_[pos], sizeof(big_uint16_t));
                state.batteryCapacityInMAH = result;
                pos += 2;
                break;
              }
            case SensorCliffLeftSignal:
              {
                big_uint16_t result;
                memcpy(&result, &impl_->readBuffer_[pos], sizeof(big_uint16_t));
                state.cliffLeftSignalStrength = result;
                pos += 2;
                break;
              }
            case SensorCliffFrontLeftSignal:
              {
                big_uint16_t result;
                memcpy(&result, &impl_->readBuffer_[pos], sizeof(big_uint16_t));
                state.cliffFrontLeftSignalStrength = result;
                pos += 2;
                break;
              }
            case SensorCliffFrontRightSignal:
              {
                big_uint16_t result;
                memcpy(&result, &impl_->readBuffer_[pos], sizeof(big_uint16_t));
                state.cliffFrontRightSignalStrength = result;
                pos += 2;
                break;
              }
            case SensorCliffRightSignal:
              {
                big_uint16_t result;
                memcpy(&result, &impl_->readBuffer_[pos], sizeof(big_uint16_t));
                state.cliffRightSignalStrength = result;
                pos += 2;
                break;
              }
            case SensorLeftEncoderCounts:
              {
                big_int16_t result;
                memcpy(&result, &impl_->readBuffer_[pos], sizeof(big_int16_t));
                state.leftEncoderCounts = result;
                pos += 2;
                break;
              }
            case SensorRightEncoderCounts:
              {
                big_int16_t result;
                memcpy(&result, &impl_->readBuffer_[pos], sizeof(big_int16_t));
                state.rightEncoderCounts = result;
                pos += 2;
                break;
              }
            }
          }
          onUpdate(state);
        } else {
          std::cout << "checksum incorrect!" << sum << std::endl;
        }

        // delete portion of buffer
        impl_->readBuffer_.erase(impl_->readBuffer_.begin(), impl_->readBuffer_.begin() + size + i);
      }
    }
  }

}



// int8_t Create2::temperature()
// {
//   impl_->sensors(SensorTemperature);
//   return impl_->readInt8();
// }

// int16_t Create2::leftEncoderCounts()
// {
//   impl_->sensors(SensorLeftEncoderCounts);
//   return impl_->readInt16();
// }

// int16_t Create2::rightEncoderCounts()
// {
//   impl_->sensors(SensorRightEncoderCounts);
//   return impl_->readInt16();
// }

// uint16_t Create2::batteryCharge()
// {
//   impl_->sensors(SensorBatteryCharge);
//   return impl_->readUint16();
// }

// uint16_t Create2::batteryCapacity()
// {
//   impl_->sensors(SensorBatteryCapacity);
//   return impl_->readUint16();
// }

// uint16_t Create2::voltage()
// {
//   impl_->sensors(SensorVoltage);
//   return impl_->readUint16();
// }

// int16_t Create2::current()
// {
//   impl_->sensors(SensorCurrent);
//   return impl_->readInt16();
// }
