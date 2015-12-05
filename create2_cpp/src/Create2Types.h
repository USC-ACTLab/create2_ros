#pragma once

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


