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
} statusRegisterBitField;

typedef union {
  struct
  {
    bool ms1 : 1;
    bool ms2 : 1;
    bool ms3 : 1;
    bool disableMotorOnPositionReached : 1;
    bool stopOnLimitSwitchPress : 1;
    bool : 3;
  };
  uint8_t byteWrapped;
} deviceConfigBitField;

typedef union {
  struct
  {
    bool run : 1;
    bool runSpeed : 1;
    bool runSpeedToPosition : 1;
    bool hardStop : 1;
    bool disableMotor : 1;
    bool : 3;
  };
  uint8_t byteWrapped;
} motorControlBitField;

typedef struct memoryMap
{
  uint8_t id;
  uint16_t firmware;
  interruptConfigBitField interruptConfig;

  statusRegisterBitField motorStatus;
  deviceConfigBitField motorConfig;
  motorControlBitField motorControl;

  signed long currentPos;
  signed long distanceToGo;

  signed long move;
  uint8_t enableMoveNVM; //0x59 = POR value can be written and stored in NVM
  signed long moveTo;

  //the following three variables are actually floats
  //they are declared as uint32_t because they don't behave normal in this struct
  //please use the convertToFloat function when using
  uint32_t maxSpeed;
  uint32_t acceleration;
  uint32_t speed;
  uint8_t enableSpeedNVM; //0x47 = POR value can be written and stored in NVM

  uint16_t holdCurrent; //Max 2000mA
  uint16_t runCurrent;  //Max 2000mA

  uint8_t i2cAddress;

  //  uint32_t moveNVM;
};
