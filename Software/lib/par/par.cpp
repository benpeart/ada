#include <par.h>
#include <EEPROM.h>

/*
How to pass objects to be modified?
  For PID, this is simple: pass array of PID objects, or a single PID object.

How to define a list of parameters to be stored?
  For example, for PID this would be: type (uint8_t), P/I/D/N/min/max(/R) (float)
  Every parameters should have a command (integer / character).

Proposal: use parameter objects.
Properties: pointer, type/tag, command, EEPROM address (maybe use static counter)
Functions: read/write EEPROM, receive command, send command
Sending a command is easy: get the value that the pointer corresponds to,
compose a message with a header, maybe add checksum, and return an array and length.
Receiving is a bit harder: how to (efficiently) find which parameter/pointer corresponds to the command?
Easiest method would be to have an array of parameters, which can be indexed.
Makes sending a bit harder: how to send a single command. However, when something is sent,
all parameters are sent.

In this way, can a single function be used for reading/writing from/to EEPROM?
Why not: as long as all variable types are covered, it should work.
However, a (group of) parameter(s) might need an update function.

Maybe add both command parameters to parameter definition?


To do:
Check if address counter is smaller than EEPROM size
*/

// float a,b,c;
// uint8_t d;

// parameter pars[4] = {
//   {(uint8_t*) &a,    t_f,  0,   EEPROM_ADR_PID + 0},
//   {(uint8_t*) &b,   t_f,  1,   EEPROM_ADR_PID + 1},
//   {(uint8_t*) &c,   t_f,  2,   EEPROM_ADR_PID + 2},
//   {(uint8_t*) &d,    t_u8,  3,   EEPROM_ADR_PID + 3}
// };

int parameter::addressCounter = 300;
uint8_t parameter::cmdCounter = 0;

parameter::parameter(uint8_t* _p, uint8_t _tag, uint8_t _cmd, int _address) {
  p = _p;
  tag = _tag;
  cmd = _cmd;
  address = _address;
}

parameter::parameter(uint8_t* _p, uint8_t _tag) {
  p = _p;
  tag = _tag;

  cmd = cmdCounter++;
  address = addressCounter;
  addressCounter += tagSize[tag];
}

parameter::parameter(uint8_t* _p) {
  p = (uint8_t*) _p;
  tag = t_u8;
  cmd = cmdCounter++;
  address = addressCounter;
  addressCounter += tagSize[tag];
}

parameter::parameter(float* _p) {
  p = (uint8_t*) _p;
  tag = t_f;
  cmd = cmdCounter++;
  address = addressCounter;
  addressCounter += tagSize[tag];
}

void parameter::read(void) {
  switch(tag) {
    case t_u8:
      *p = EEPROM.read(address);
      break;
  }
}

void parameter::write(void) {
  switch(tag) {
    case t_u8:
      EEPROM.write(address, *p);
      break;
  }
}

// void readPIDParameters(PID* pid) {
//
//   parameter pars[4] = {
//     {(uint8_t*) &pid[0].K,    t_f,  0,   EEPROM_ADR_PID + 0},
//     {(uint8_t*) &pid[0].Ti,   t_f,  1,   EEPROM_ADR_PID + 1},
//     {(uint8_t*) &pid[1].N,   t_f,  2,   EEPROM_ADR_PID + 2},
//     {(uint8_t*) &pid[1].controllerType,    t_u8,  3,   EEPROM_ADR_PID + 3}
//   };
//
//   // pars[0].f = &pid[0].K;
//   // If parameters have never been written to EEPROM, don't read parameters (EEPROM will contain gibberish)
//   if (EEPROM.read(EEPROM_ADR_PID_WRITTEN)==EEPROM_WRITTEN) {
//     for (uint8_t i=0; i<3; i++) {
//       pid[i].K          = EEPROM.readFloat(EEPROM_ADR_PID + i*30 + 4*0);
//       pid[i].Ti         = EEPROM.readFloat(EEPROM_ADR_PID + i*30 + 4*1);
//       pid[i].Td         = EEPROM.readFloat(EEPROM_ADR_PID + i*30 + 4*2);
//       pid[i].N          = EEPROM.readFloat(EEPROM_ADR_PID + i*30 + 4*3);
//       pid[i].R          = EEPROM.readFloat(EEPROM_ADR_PID + i*30 + 4*4);
//       pid[i].minOutput  = EEPROM.readFloat(EEPROM_ADR_PID + i*30 + 4*5);
//       pid[i].maxOutput  = EEPROM.readFloat(EEPROM_ADR_PID + i*30 + 4*6);
//       pid[i].controllerType  = EEPROM.read(EEPROM_ADR_PID + i*30 + 4*7);
//       pid[i].updateParameters();
//     }
//   }
// }
//
// void writePIDParameters(PID* pid) {
//   for (uint8_t i=0; i<3; i++) {
//     EEPROM.writeFloat(EEPROM_ADR_PID + i*30 + 4*0, pid[i].K);
//     EEPROM.writeFloat(EEPROM_ADR_PID + i*30 + 4*1, pid[i].Ti);
//     EEPROM.writeFloat(EEPROM_ADR_PID + i*30 + 4*2, pid[i].Td);
//     EEPROM.writeFloat(EEPROM_ADR_PID + i*30 + 4*3, pid[i].N);
//     EEPROM.writeFloat(EEPROM_ADR_PID + i*30 + 4*4, pid[i].R);
//     EEPROM.writeFloat(EEPROM_ADR_PID + i*30 + 4*5, pid[i].minOutput);
//     EEPROM.writeFloat(EEPROM_ADR_PID + i*30 + 4*6, pid[i].maxOutput);
//     EEPROM.write(EEPROM_ADR_PID + i*30 + 4*7, pid[i].controllerType);
//     EEPROM.write(EEPROM_ADR_PID_WRITTEN, EEPROM_WRITTEN);
//     EEPROM.commit();
//   }
// }
//
// void readParameter(parameter p) {
//   switch (p.tag) {
//     case t_f:
//       1;
//       break;
//     case t_u8:
//       2;
//       break;
//   }
// }
