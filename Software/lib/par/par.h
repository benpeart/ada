#ifndef uniPar_H
#define uniPar_H

#include<Arduino.h>
#include<PID.h>

#define EEPROM_ADR_PID_WRITTEN 200
#define EEPROM_ADR_PID 201

#define EEPROM_WRITTEN 123 // Flag to check if EEPROM has been initialised

// #define PAR_READ 0
// #define PAR_WRITE 1
//
// void readPIDParameters(PID* pid);
// void writePIDParameters(PID* pid);

enum tag {t_u8,t_u16,t_u32,t_i8,t_i16,t_i32,t_f,t_d};
const uint8_t tagSize[] = {1,2,4,1,2,4,4,8};


class parameter {
public:
  uint8_t* p;
  int address;
  uint8_t cmd;
  uint8_t tag;

  static int addressCounter;
  static uint8_t cmdCounter;


  parameter(uint8_t* _p, uint8_t _tag, uint8_t _cmd, int _address);
  parameter(uint8_t* _p, uint8_t _tag);
  parameter(uint8_t* _p);
  parameter(float* _p);

  void read(void);
  void write(void);

};



// typedef struct {
//   // union {
//   //   uint8_t * u8;
//   //   uint16_t * u16;
//   //   uint32_t * u32;
//   //   int8_t * i8;
//   //   int16_t * i16;
//   //   int32_t * i32;
//   //   float * f;
//   //   double * d;
//   // };
//   uint8_t *p;
//   uint8_t tag;
//   uint8_t cmdNo;
//   uint16_t address;
//   // void getFun() {};
// } parameter;

#endif
