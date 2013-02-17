/*
  config_manager.pde - Part of the Rallylog project
  Stephen Eaton seaton@strobotics.com.au
  
  Copyright (C) 2013 Stephen Eaton.  All rights reserved.
 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

void config_init(){
  // Check if the firmware version is the same of eeprom config
  if (
    EEPROM.read(EE_ADDR_SIGNATURE_CODE1) == SIGNATURE_CODE1 && 
    EEPROM.read(EE_ADDR_SIGNATURE_CODE2) == SIGNATURE_CODE2 && 
    EEPROM.read(EE_ADDR_SIGNATURE_CODE3) == SIGNATURE_CODE3 && 
    EEPROM.read(EE_ADDR_SIGNATURE_CODE4) == SIGNATURE_CODE4 &&
    EEPROM.read(EE_ADDR_CODE_MAJOR_VERSION) == CODE_MAJOR_VERSION &&
    EEPROM.read(EE_ADDR_CODE_MINOR_VERSION) == CODE_MINOR_VERSION 
    ) {
      
      // loads in ram the eeprom config
    config_loadBackup_all();
    }
    else {

    // clear the eeprom
    for (unsigned int i = 0; i < EEPROM_SIZE; i++) {
      EEPROM.write(i, 0x00);
    } //for
    
    // writes sign codes
    EEPROM.write(EE_ADDR_SIGNATURE_CODE1,SIGNATURE_CODE1);
    EEPROM.write(EE_ADDR_SIGNATURE_CODE2,SIGNATURE_CODE2);
    EEPROM.write(EE_ADDR_SIGNATURE_CODE3,SIGNATURE_CODE3);
    EEPROM.write(EE_ADDR_SIGNATURE_CODE4,SIGNATURE_CODE4);
    EEPROM.write(EE_ADDR_CODE_MAJOR_VERSION,CODE_MAJOR_VERSION);
    EEPROM.write(EE_ADDR_CODE_MINOR_VERSION,CODE_MINOR_VERSION);
    
    // load defaults to ram and save on eeprom
    config_loadDefaults_all();
    config_saveBackup_all();
    } //else
  
} // config_init


void config_loadDefaults_all(){
  config_loadDefaults_system();
}

void config_saveBackup_all(){
  config_saveBackup_system();
}

void config_loadBackup_all(){
  config_loadBackup_system(); 
}

// Load the default system config to ram
void config_loadDefaults_system() {
 system_station_id = DEFAULT_system_station_id;
}

// Save the system config from ram to eeprom
void config_saveBackup_system(){ 
  EEPROM.write(EE_ADDR_system_station_id, system_station_id);
}

// Load the system config from eeprom to ram
void config_loadBackup_system(){
  system_station_id = EEPROM.read(EE_ADDR_system_station_id);
}  
  


