//Location in EEPROM for each thing we want to store between power cycles
enum eepromLocations {
  LOCATION_INTERRUPTENABLE = 0x03,
  LOCATION_DEVICECONFIG = 0x05,
  LOCATION_MOVE = 0x0F,
  LOCATION_MOVE_ENABLENVM = 0x13,
  LOCATION_MAXSPEED = 0x18,
  LOCATION_ACCELERATION = 0x1C,
  LOCATION_SPEED = 0x20,
  LOCATION_SPEED_ENABLENVM = 0x24,
  LOCATION_HOLDCURRENT = 0x25,
  LOCATION_RUNCURRENT = 0x27,
  LOCATION_I2C_ADDRESS = 0x29,
};
