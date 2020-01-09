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
