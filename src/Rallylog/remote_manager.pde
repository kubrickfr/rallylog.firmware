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
#define REMOTE_CMD_RTC_SET                     0x05

// Response codes
#define REMOTE_RSP_REPORT_READY                0x01
#define REMOTE_RSP_REPORT_ID                   0x02
#define REMOTE_RSP_REPORT_BAT                  0x03
#define REMOTE_RSP_REPORT_RTC                  0x04

//Parameter values
#define PARM_ID                                0x00
#define PARM_BAT_HI                            0x01
#define PARM_BAT_LO                            0x02
#define PARM_RTC_SEC                           0x03
#define PARM_RTC_MIN                           0x04 
#define PARM_RTC_HOUR                          0x05
#define PARM_RTC_DAY                           0x06
#define PARM_RTC_MTH                           0x07 
#define PARM_RTC_YEAR                          0x08
#define PARM_RTC_CAL                           0x09     //Calibrate
#define PARM_RTC_STATUS                        0x0A


// On sysex message from host
void sysexCallback(byte command, byte argc, byte*argv)
{
    
    switch(command){
        
       case REMOTE_CMD_ID_REQUEST            : remote_report_id(); break;
       case REMOTE_CMD_ID_SET                : remote_set_id(argc, argv); break; 
       case REMOTE_CMD_BAT_REQUEST           : remote_report_bat(); break;
       case REMOTE_CMD_RTC_REQUEST           : remote_report_rtc(); break; 
       case REMOTE_CMD_RTC_SET               : remote_set_rtc(argc, argv); break; 
              
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
  byte response[3] = {system_station_id, voltage/256, voltage % 256};
  FirmataLite.sendSysex(REMOTE_RSP_REPORT_BAT,3,response);
}

//send the time to the remote host
void remote_report_rtc(){
  
  int voltage = battery.getVoltage();
  byte response[11] = {
      system_station_id,            // station ID
      voltage/256,                  // battery
      voltage%254.
      rtc.time.sec,                 // RTC
      rtc.time.min,  
      rtc.time.hour,  
      rtc.time.day, 
      rtc.time.month,
      rtc.time.year,
      rtc.time.cal,
      rtc.get()                    // RTC Status 
      };
      FirmataLite.sendSysex(REMOTE_RSP_REPORT_RTC,11,response);
}

//set the time/DATE from remote host
void remote_set_rtc(byte argc, byte *argv){
  i2c_rtc_m41t00s::time_t newtime;
  
  newtime.sec = argv[PARM_RTC_SEC];
  newtime.min = argv[PARM_RTC_MIN];
  newtime.hour = argv[PARM_RTC_HOUR];
  newtime.day = argv[PARM_RTC_DAY];
  newtime.month = argv[PARM_RTC_MTH];
  newtime.year = argv[PARM_RTC_YEAR];
  newtime.cal = argv[PARM_RTC_CAL];      // Calibrate
  
  rtc.set(&newtime);                    // Set the RTC
}

// send time every every second
void remote_broadcast_rtc(){
  remote_report_rtc();
}


