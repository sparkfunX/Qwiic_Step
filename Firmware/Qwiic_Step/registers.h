typedef union {
    struct
    {
        bool isRunning : 1;
        bool isAccelerating : 1;
        bool isDecelerating : 1;
        bool : 5;
    };
    uint8_t byteWrapped;
} statusRegisterBitField;

typedef union {
    struct
    {
        bool ms1 : 1;
        bool ms2 : 1;
        bool ms3 : 1;
        bool disableStepper : 1;
        bool limitSwitch : 1;
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

    uint8_t interruptEnable;
    statusRegisterBitField motorStatus;

    deviceConfigBitField motorConfig;
    deviceControlBitField motorControl;

    signed long currentPos;
    signed long moveTo;

    float maxSpeed;
    signed long acceleration;
    float setSpeed;

    uint8_t enableSetSpeedNVM; //0x47 = POR value can be written and stored in NVM
    uint16_t holdCurrent;      //Max 2000mA
    uint16_t runCurrent;       //Max 2000mA

    unsigned long tempEmpty0;
    unsigned long tempEmpty1;    

    uint8_t i2cAddress;
};
