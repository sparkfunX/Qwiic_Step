typedef union {
  struct
  {
    bool isReachedInterruptEnable : 1;
    bool isLimitedInterruptEnable : 1;
    bool : 6;
  };
  uint8_t byteWrapped;
} interruptConfigBitField;

typedef union {
  struct
  {
    bool isRunning : 1;
    bool isAccelerating : 1;
    bool isDecelerating : 1;
    bool isReached : 1;
    bool isLimited : 1;
    bool eStopped : 1;
    bool : 2;
  };
  uint8_t byteWrapped;
} statusBitField;

typedef union {
  struct
  {
    bool ms1 : 1;
    bool ms2 : 1;
    bool ms3 : 1;
    bool disableMotorOnEStop : 1;           //If 1, stepper is disabled when EStop occurs.
    bool disableMotorOnPositionReached : 1;
    bool stopOnLimitSwitchPress : 1;
    bool : 2;
  };
  uint8_t byteWrapped;
} configBitField;

typedef union {
  struct
  {
    bool runToPosition : 1;
    bool runToPositionWithAccel : 1;
    bool runContinuous : 1;
    bool hardStop : 1;
    bool disableMotor : 1;
    bool : 3;
  };
  uint8_t byteWrapped;
} controlBitField;

struct memoryMap
{
  uint8_t id;
  uint16_t firmware;
  interruptConfigBitField interruptConfig;

  statusBitField motorStatus;
  configBitField motorConfig;
  controlBitField motorControl;

  int32_t currentPos;
  int32_t distanceToGo;

  int32_t move;
  uint8_t unlockMoveNVM;
  int32_t moveTo;

  //The following three variables are actually floats
  //They are declared as uint32_t because they don't behave normal in this struct
  //Please use the convertToFloat function when using
  uint32_t maxSpeed;
  uint32_t acceleration;
  uint32_t speed;

  uint8_t unlockSpeedNVM;

  uint32_t holdVoltage;
  uint32_t runVoltage;

  uint8_t i2cAddress;

  //  uint32_t moveNVM;
};

struct nvmMemoryMap
{
  int32_t move;
  uint32_t speed; //Note this is a float, but communicated and stored as uint32
  uint8_t i2cAddressState;
};
