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
#define REMODE_CMD_ID_SET                      0x02
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
       case REMOTE_CMD_BAT_REQUEST           : remote_report_bat(); break;
       case REMOTE_CMD_RTC_REQUEST           : remote_report_rtc(); break;  
              
       default:  FirmataLite.sendString("Unknown command");
                 break;      
    }
}


void remote_report_id(){
  byte response[1] = {system_station_id};
  FirmataLite.sendSysex(REMOTE_RSP_REPORT_ID, 1, response);
}

void remote_report_bat(){
  int voltage = battery.getVoltage();
  byte response[2] = {voltage/256, voltage % 256};
  FirmataLite.sendSysex(REMOTE_RSP_REPORT_BAT,2,response);
}

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


void remote_broadcast_rtc(){
  remote_report_rtc();
}
