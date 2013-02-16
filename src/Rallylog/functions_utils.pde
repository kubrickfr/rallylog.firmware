/*
  functions_utils.pde - Part of the Rallylog project
  Stephen Eaton seaton@strobotics.com.au
  
  Copyright (C) 2013 Stephen Eaton.  All rights reserved.
 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

// Write a unsigned int (two bytes) value to eeprom
void eeprom_writeInt(unsigned int address, unsigned int value){

  EEPROM.write(address, value % 256); // LSB
  EEPROM.write(address+1, value/256); // MSB

}

// read a unsigned int (two bytes) value from eeprom
unsigned int eeprom_readInt(unsigned int address){

  return EEPROM.read(address)+EEPROM.read(address+1)*256;

}

