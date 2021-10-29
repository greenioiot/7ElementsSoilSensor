#define ID_SENSOR        1 
 

///////////////////////////////////////////
// Register Power Meter
 
#define _MOISTURE  0x0000
#define _TEMP  0x0001
#define _EC  0x0002
#define _Ph  0x0003
#define _Ni  0x0004
#define _Pho  0x0005
#define _Pot  0x0006
 
 

uint16_t const Address[7] = {
  _MOISTURE,
  _TEMP,
  _EC,
  _Ph,
  _Ni,
  _Pho,
  _Pot
};
