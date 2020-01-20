void recordRegisterMap()
{
  EEPROM.put(LOCATION_REGISTERMAP, registerMap);
}

void recordPORsettings()
{
  EEPROM.put(LOCATION_PORSETTINGS, PORsettings);
}

void readSystemSettings()
{
  //Check to see if EEPROM is blank
  uint32_t EEPROM_check;
  EEPROM.get(LOCATION_REGISTERMAP, EEPROM_check);
  if (EEPROM_check == 0xFFFFFFFF) {  //EEPROM has not been written to yet
    recordRegisterMap(); //Record default settings to EEPROM
  }

  EEPROM.get(LOCATION_REGISTERMAP, registerMap);

  //Zero out the registers that must be 0 at POR
  registerMap.unlockMoveNVM = 0;
  registerMap.unlockSpeedNVM = 0;
  registerMap.move = 0;
  registerMap.speed = 0;
  registerMap.currentPos = 0;
  registerMap.distanceToGo = 0;

  //Deal with the special POR settings
  //Check to see if EEPROM is blank
  EEPROM.get(LOCATION_PORSETTINGS, EEPROM_check);
  if (EEPROM_check == 0xFFFFFFFF) {  //EEPROM has not been written to yet
    recordPORsettings(); //Record default settings to EEPROM
  }

  EEPROM.get(LOCATION_PORSETTINGS, PORsettings);
  if (PORsettings.move != 0)
  {
    registerMap.move = PORsettings.move;
    newMoveValue = true;
  }
  if (PORsettings.speed != 0) registerMap.speed = PORsettings.speed;

  //Pass these new settings back into the library
  updateParameters();
}

//Clear the EEPROM
void eraseEEPROM()
{
  for (byte x = 0 ; x < 120 ; x++)
  {
    EEPROM.put(x, 0xFF);
  }
}
