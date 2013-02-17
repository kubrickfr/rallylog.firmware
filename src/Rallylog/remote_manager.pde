/*
  remote_manager.cpp - FirmataLite library
  Stephen Eaton seaton@strobotics.com.au
  
  FirmataLite is a simplified version of Firmata by Hans-Christoph Steiner.
  Everything but the ability to send and receive Strings and Sysex messages
  has been stripped out.
  
  Copyright (C) 2013 Stephen Eaton.  All rights reserved.
 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

// Command Codes
#define REMOTE_CMD_ID_REQUEST                  0x01
#define REMOTE_CMD_ID_SET                      0x02
#define REMOTE_CMD_BAT_REQUEST                 0x03
#define REMOTE_CMD_RTC_REQUEST                 0x04


// Response codes
#define REMOTE_RSP_REPORT_READY                0x01
#define REMOTE_RSP_REPORT_ID                   0x02
#define REMOTE_RSP_REPORT_BAT                  0x03
#define REMOTE_RSP_REPORT_RTC                  0x04


// On sysex message from host
void sysexCallback(byte command, byte argc, byte*argv)
{
    
    switch(command){
        
       case REMOTE_CMD_ID_REQUEST            : remote_report_id(); break;
       case REMOTE_CMD_ID_SET                : remote_set_id(argc, argv); break; 
       case REMOTE_CMD_BAT_REQUEST           : remote_report_bat(); break;
       case REMOTE_CMD_RTC_REQUEST           : remote_report_rtc(); break;  
              
       default:  FirmataLite.sendString("Unknown command");
                 break;      
    }
}

// Send a ready state to the host
void remote_sendReady(){
  byte FirmwareVersion[3] = { 1, CODE_MAJOR_VERSION, CODE_MINOR_VERSION };
  FirmataLite.sendSysex(REMOTE_RSP_REPORT_READY, 3, FirmwareVersion);
}

// send current id to host
void remote_report_id(){
  byte response[1] = {system_station_id};
  FirmataLite.sendSysex(REMOTE_RSP_REPORT_ID, 1, response);
}

// updates local device with the id sent from host
void remote_set_id(byte argc, byte *argv){
    unsigned int id = argv[0];
    eeprom_writeInt(EE_ADDR_system_station_id, id); 
    config_loadBackup_all();
}

//send battery voltage
void remote_report_bat(){
  int voltage = battery.getVoltage();
  byte response[2] = {voltage/256, voltage % 256};
  FirmataLite.sendSysex(REMOTE_RSP_REPORT_BAT,2,response);
}

//send the time
void remote_report_rtc(){
  byte response[7] = {rtc.get(),      // RTC Status
      rtc.time.year,  
      rtc.time.month,  
      rtc.time.day, 
      rtc.time.hour,
      rtc.time.min, 
      rtc.time.sec};
      FirmataLite.sendSysex(REMOTE_RSP_REPORT_RTC,7,response);
}

// send time every every second
void remote_broadcast_rtc(){
  remote_report_rtc();
}


