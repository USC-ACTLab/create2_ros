#pragma once

enum Mode {
  ModeOff       = 0,
  ModePassive   = 1,
  ModeSafe      = 2,
  ModeFull      = 3,
};

enum Op {
  OpStart             = 128,
  OpReset             =   7,
  OpBaud              = 129,
  OpControl           = 130,
  OpSafe              = 131,
  OpFull              = 132,
  OpPower             = 133,
  OpSpot              = 134,
  OpClean             = 135,
  OpMaxClean          = 136,
  OpDrive             = 137,
  OpDriveDirect       = 145,
  OpMotors            = 138,
  OpPwmMotors         = 144,
  OpDrivePwm          = 146,
  OpLeds              = 139,
  OpSong              = 140,
  OpPlay              = 141,
  OpSensors           = 142,
  OpStream            = 148,
  OpQueryList         = 149,
  OpDoStream          = 150,
  OpQuery             = 142,
  OpForceSeekingDock  = 143,
  OpSchedulingLeds    = 162,
  OpDigitsLedsRaw     = 163,
  OpDigitsLedsAscii   = 164,
  OpButtons           = 165,
  OpSchedule          = 167,
  OpSetDayTime        = 168,
  OpStop              = 173,
};

enum ChargingState {
  NotCharging             = 0,
  ReconditioningCharing   = 1,
  FullCharging            = 2,
  TrickleCharging         = 3,
  ChargingStateWaiting    = 4,
  ChargingFaultCondition = 5,
};

enum PacketID {
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
  SensorStatis                      = 58,
};
