//converts uint32_t type to float
//needed for floats in the memoryMap struct
//reading/writing to floats in struct doesn't work as expected 
float convertToFloat(uint32_t myVal) {
  union {
    float f;
    uint8_t b[4];
  } u;
  u.b[3] = myVal >> (8*3);
  u.b[2] = myVal >> (8*2);
  u.b[1] = myVal >> (8*1);
  u.b[0] = myVal >> (8*0);
  return u.f;
}

typedef union {
  struct
  {
    bool requestedPosReachedEnable : 1;
    bool limSwitchPressedEnable : 1;
    bool : 6;
  };
  uint8_t byteWrapped;
} interruptEnableBitField;

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
    bool powerDownPositionReached : 1;
    bool stopOnLimitSwitchPress : 1;
    bool : 3;
  };
  uint8_t byteWrapped;
} deviceConfigBitField;

typedef union {
  struct
  {
    bool stop : 1;
    bool runTo : 1;
    bool runContinuous : 1;
    bool sleep : 1;
    bool : 4;
  };
  uint8_t byteWrapped;
} deviceControlBitField;

typedef struct memoryMap
{
  uint8_t id;
  uint16_t firmware;
  interruptEnableBitField interruptEnable;
  
  statusRegisterBitField motorStatus;
  deviceConfigBitField motorConfig;
  deviceControlBitField motorControl;

  signed long currentPos;
  signed long distanceToGo;

  signed long move;
  uint8_t enableMoveNVM;  //0x59 = POR value can be written and stored in NVM
  signed long moveTo;

  //the following three variables are actually floats
  //they are declared as uint32_t because they don't behave normal in this struct
  //please use the convertToFloat function when using 
  uint32_t maxSpeed;        
  uint32_t acceleration;
  uint32_t speed;
  uint8_t enableSpeedNVM; //0x47 = POR value can be written and stored in NVM
  
  uint16_t holdCurrent;      //Max 2000mA
  uint16_t runCurrent;       //Max 2000mA

  uint8_t i2cAddress;
};
