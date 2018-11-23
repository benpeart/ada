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
Same for command
Add option for manually specifying command

Idea:
Add a save command, to save all parameters (both start and end address are known);
*/



int parameter::addressCounter = 300;
uint8_t parameter::cmdCounter = 0;

// parameter::parameter(uint8_t* _p, uint8_t _tag, uint8_t _cmd, int _address) {
//   p = _p;
//   tag = _tag;
//   cmd = _cmd;
//   address = _address;
// }
//
// parameter::parameter(uint8_t* _p, uint8_t _tag) {
//   p = _p;
//   tag = _tag;
//
//   cmd = cmdCounter++;
//   address = addressCounter;
//   addressCounter += tagSize[tag];
// }

parameter::parameter(uint8_t* _p) {
  p_u8 = _p;
  tag = t_u8;
  assignAddress();
}

parameter::parameter(float* _p) {
  p_f = _p;
  tag = t_f;
  assignAddress();
}

void parameter::assignAddress(void) {
  cmd = cmdCounter++;
  address = addressCounter;
  addressCounter += tagSize[tag];
}

void parameter::read(void) {
  switch(tag) {
    case t_u8:
      *p_u8 = EEPROM.read(address);
      break;
    case t_f:
      *p_f = EEPROM.readFloat(address);
      break;
  }
  Serial.println(EEPROM.read(address));
}

void parameter::write(void) {
  switch(tag) {
    case t_u8:
      EEPROM.write(address, *p_u8);
      break;
    case t_u16:
      EEPROM.writeUShort(address, *p_u16);
      break;
    case t_u32:
      break;
    case t_i8:
      break;
    case t_i16:
      break;
    case t_i32:
      break;
    case t_f:
      EEPROM.writeFloat(address, *p_f);
      break;
    case t_d:
      break;

  }
  EEPROM.commit();
}

// Use binary values or text for messages?
// Send everything as float?

// Add 2nd class for making groups of commands.
// Properties: groupNo, number of items, array of parameters
// EEPROM bit/byte to indicate if settings have been stored
// Functions: read / write group from/to EEPROM,
// Specify group as array of adresses, then loop through them. 
uint8_t parameter::makeMessageBin(uint8_t* p) {
  // Command (2 bytes), value (1-4 bytes)
  // How to handle different variable types? Send everything as float, or send variable type as well
  uint8_t length = 0;

  p[0] = command;
}

void parameter::makeMessageText(char* c) {

}
