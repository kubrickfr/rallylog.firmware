/***********************************************************
 *  Rallylog - Open Hardware Project
 *    RFID Event timing device
 *
 *  http://code.google.com/p/rallylog
 *
 *  (c)2011 Stephen Eaton <seaton@strobotics.com.au>
 *
 * ---------------------------------------------------------
 */

#include <EEPROM.h> 
#include <Wire.h>
#include <i2c_rtc_m41t00s.h>
#include <Metro.h>
#include <DebounceButton.h>
#include <DogLcd.h>
#include <NewSoftSerial.h>
#include <SdFat.h>                   // IMPORTANT:  Software SPI MUST be enabled in SDConfig.h else corruption will occur
#include <SdFatUtil.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include "Battery.h"
#include "Firmatalite.h"

//
//  Station ID
//  Valid Values:
//    0-127            
#define DEFAULT_system_station_id  0

#define SIGNATURE_CODE1      16      // Day
#define SIGNATURE_CODE2      2       // month
#define SIGNATURE_CODE3      20      // Century
#define SIGNATURE_CODE4      13      // year of centry
#define CODE_MAJOR_VERSION   2       // Major Version
#define CODE_MINOR_VERSION   0       // Minor version

// Valid States
#define STATE_STARTUP       0x00
#define STATE_STARTING      0x01
#define STATE_STARTED       0x02
#define STATE_IDLE          0x03
#define STATE_SLEEPING      0x04
#define STATE_SDREMOVED     0x05
#define STATE_SDINSERTED    0x06
#define STATE_SDINIT        0x11
#define STATE_SDERROR       0x1F
#define STATE_RFIDON        0x07
#define STATE_SAVING        0x08
#define STATE_BATLOW        0x09

// settings
#define RESET_ENABLED       0
#define RESET_TIME          1000
#define RFID_REMOVED_TIME   1500
#define RFID_TAG_LENGTH     5 // 5 Bytes
#define RFID_TAG_INPUT      12 // DATA (10 ASCII) + CHECK SUM (2 ASCII)
#define LCD_LINES           2
#define BAT_LOW_VOLTAGE     600   // low voltage threshold * 100 e.g. 6.0v * 100  = 600
#define CARD_IN              0
#define CARD_OUT             1

// Arduino Analog PIN Assignments
#define ADC_BAT_MON          6     // Battery voltage, must be enabled via BAT_MON_EN first

// Arduino Digital PIN Assignments 
#define nBTN_LEFT            2     // INT0
#define nBTN_MIDDLE          3     // INT1
#define nBTN_RIGHT           4    
#define nLED_GREEN           5     // enable LOW
#define nLED_RED             6     // enable LOW
#define BAT_MON_EN           7     // enable HIGH
#define RFID_EN              8     // enable HIGH
#define RFID_RX              9        
#define nSD_CS               10    // enable LOW  SDcard CS
#define SPI_MOSI             11    // SPI MOSI 
#define SPI_MISO             12    // SPI MISO
#define SPI_SCK              13    // SPI SCK 
#define LCD_RS               14    // LCD RS
#define nLCD_BL              15    // enable LOW
#define nLCD_CS              16    // enable LOW lcd CS
#define nSD_CARD_IN          17    // SD Card Detect in HIGH, out LOW


// EEPROM size in bytes
#define EEPROM_SIZE         0x200  

// EEPROM Addresses for signature code and version of firmware
#define EE_ADDR_SIGNATURE_CODE1                      0x00 // BYTE
#define EE_ADDR_SIGNATURE_CODE2                      0x01 // BYTE
#define EE_ADDR_SIGNATURE_CODE3                      0x02 // BYTE
#define EE_ADDR_SIGNATURE_CODE4                      0x03 // BYTE
#define EE_ADDR_CODE_MAJOR_VERSION                   0x04 // BYTE
#define EE_ADDR_CODE_MINOR_VERSION                   0x05 // BYTE

#define EE_ADDR_system_station_id                    0x10 // BYTE

#define MIN(x,y) ( x > y ? y : x ) 
#define byteToint(x) ((int) x & 0xff)


// Global Variables
int STATE = STATE_STARTUP;            
char dateString[50];                    // string to store current time/date
static long start_time;                 // time first character is received
boolean flgInsertMessage=false;         // flag to indicate if Insert Card Message is displayed

// System config in ram
char      system_station_id;            // station ID

// RFID
unsigned int nowLastRfid = 0;          // millis of last seen rfid tag
boolean rfidTagSeen = false;
byte rfidTagCurrent[RFID_TAG_LENGTH];  // last seen tag
byte rfidTagTemp[6];                   // temp RFID tag

// SDFAT32 Library Definitions
Sd2Card card;
SdVolume volume;
SdFile root;
SdFile file;
// store error strings in flash to save RAM
#define error(s) error_P(PSTR(s))

void error_P(const char* str) {
  PgmPrint("error: ");
  SerialPrintln_P(str);
  if (card.errorCode()) {
    PgmPrint("SD error: ");
    //Serial.print(card.errorCode(), HEX);
    //Serial.print(',');
    //Serial.println(card.errorData(), HEX);
  }
  while(1);
}

// Global timers
Metro secTimer = Metro(1000);             // 1 Second timer
Metro sec5Timer = Metro(5000) ;          //  5 Second timer

// Define Peripheral subsystems 
NewSoftSerial rfidSerial =  NewSoftSerial(RFID_RX, nLED_RED);  // RFID SERIAL IN - TX is not used but a pin is required so using a LED PIN as Dummy TX (not used at all)
DogLcd lcd(SPI_MOSI, SPI_SCK, LCD_RS, nLCD_CS );   // LCD 
DebounceButton buttonMiddle = DebounceButton(nBTN_MIDDLE, DBTN_PULLUP_INTERNAL, 50, 2000, 0); // Debounce setting for button
i2c_rtc_m41t00s rtc;                            // define RTC
Battery battery(ADC_BAT_MON, BAT_MON_EN, BAT_LOW_VOLTAGE);  // low battery of 6.00v

void setup() {

  STATE = STATE_STARTING;

  config_init();                    // read in config from EEPROM
  //system_station_id = 1;

  //LEDS
  pinMode(nLED_RED,OUTPUT);
  digitalWrite(nLED_RED,HIGH);       //off
  pinMode(nLED_GREEN,OUTPUT);
  digitalWrite(nLED_GREEN,HIGH);     //off

  //RFID
  pinMode(RFID_EN, OUTPUT);
  digitalWrite(RFID_EN, LOW);        //off
  clearTag(rfidTagCurrent, RFID_TAG_LENGTH);    

  // LCD
  pinMode(nLCD_CS,OUTPUT);
  digitalWrite(nLCD_CS, HIGH);       //off
  pinMode(nSD_CS,OUTPUT);
  digitalWrite(nSD_CS, HIGH);        //off


  // backlight
  pinMode(nLCD_BL,OUTPUT);                  // setup LCD Backlight
  digitalWrite(nLCD_BL,HIGH);               //OFF

  // buttons
  buttonMiddle.onClick = onMiddleClick;     // function when middle button clicked
  buttonMiddle.onHold = onMiddleHold;       // function when middle button is held

    //RTC
  rtc.begin();                              // begin rtc interface

  // battery monitor
  battery.setOnLowVoltageCallback(lowBatteryCallback);
  battery.update();       // update battery reading
  battery.update();       // update battery reading

  //sd card
  pinMode(nSD_CARD_IN, INPUT);      // Configure Card Detect Input
  digitalWrite(nSD_CARD_IN, HIGH);  // enable pullup;
  file.dateTimeCallback(sdDateTimeCallback);    // callback to get time/date for file timestamp
  //Serial.begin(9600);               // opens serial port, sets data rate to 9600 bps

  delay(500);

  
  //init_LCD();
  lcd.begin(DOG_LCD_M162,0x28, DOG_LCD_VCC_3V3);
  lcd.noCursor();
  lcd.print("Stn:");
  lcd.print(byteToint(system_station_id));
  lcd.setCursor(1,1);
  lcd.print("Rallylog v");
  lcd.print(CODE_MAJOR_VERSION);
  lcd.print(".");
  lcd.print(CODE_MINOR_VERSION);


  //check state of SD card
  if (digitalRead(nSD_CARD_IN) != CARD_OUT)
  {
    STATE = STATE_SDINSERTED;

 
    STATE=STATE_STARTED;
  }

  else {
    STATE = STATE_SDREMOVED;
  }

  rfidSerial.begin(9600);           // software serial for RFID RX

 // Init Firmata 
  FirmataLite.attach(START_SYSEX, sysexCallback);
  FirmataLite.begin();
}

/*******************************************************************************/
/** SoftReset function  Calling this will perform a soft reboot of the Arduino */
void(* resetFunc) (void) = 0; //declare reset function @ address 0

/*******************************************************************************/
void loop() {
  byte action = 0;
  // clear print error
  file.writeError = 0;

  if(secTimer.check()==1){       // check if timer has passes its interval
    if (rtc.isStopped()) {
      rtc.start();
    } //if 
    else { 
      if (rtc.isFailed()){
          rtc.clearFault();
      }//if
      else {
        if (STATE == STATE_IDLE)    // display time on the LCD when idle
          lcdPrintTime();
          if(FirmataLite.available()) FirmataLite.processInput();
            remote_broadcast_rtc();
        
      }//else
    }//else
  } // sectimer  

  if(sec5Timer.check()==1){    // check 5 Second timer
    battery.update();          // read battery voltage
  }//min5Timer

  // check if card has been removed
  if (digitalRead(nSD_CARD_IN) != CARD_IN){  
    STATE=STATE_SDREMOVED;
  }

  DebounceButton::updateAll();      // update buttons
  
  if(FirmataLite.available()) 
    FirmataLite.processInput();    // handle remote commands

  switch(STATE){
  case STATE_STARTED:
    // set up the LCD type and the contrast setting for the display 
    STATE=STATE_IDLE;
    break;

  case STATE_IDLE:
    break;

  case STATE_RFIDON:                         // Read RFID Tag
    action = rfidSerial.read();
    clearTag(rfidTagTemp, 6);

    // wait for the next STX byte
    while(rfidSerial.available() && action != 0x02)
      action = rfidSerial.read();

    // STX byte found -> RFID tag available
    if(action == 0x02)
    {
      digitalWrite(nLED_GREEN,LOW);       // turn on LED
      if(readID12(rfidTagTemp))
      {
        nowLastRfid = millis();
        rfidTagSeen = true;
        updateCurrentRfidTag(rfidTagTemp);
      }//if
      delay(250);
      digitalWrite(nLED_GREEN,HIGH);       // turn off LED
    } //if

    break;

  case STATE_SDREMOVED:
    digitalWrite(RFID_EN, LOW);  // turn off
    digitalWrite(nLED_RED,HIGH);
    digitalWrite(nLCD_BL,HIGH);  // LCD Backlight Off

    // check that the SD card has been inserted
    if (digitalRead(nSD_CARD_IN) != CARD_OUT)
    {
      //init_LCD();
      lcd.reset();
      lcd.print("Stn:");
      lcd.print(byteToint(system_station_id));
      lcd.setCursor(1,1);
      lcd.print("Card Inserted..");
      flgInsertMessage=false;
      STATE = STATE_SDINSERTED;
    } 
    else 
    {
      // check to see if we need to redisplay the "insert card" message
      if(!(flgInsertMessage)){
        //init_LCD();
        lcd.reset();
        lcd.clear();
        lcd.print("Stn:");
        lcd.print(byteToint(system_station_id));
        lcd.setCursor(2,1);
        lcd.print("Insert Card");
        flgInsertMessage = true;
      }
    }
    break;

  case STATE_SDINSERTED:
    delay(500);
    resetFunc();    // soft reset
    break;

  case STATE_BATLOW:
    // check to see if we need to redisplay the "insert card" message
    digitalWrite(RFID_EN, LOW);  // turn off
    digitalWrite(nLED_RED,HIGH);
    digitalWrite(nLCD_BL,HIGH);  // LCD Backlight Off

    if(!(flgInsertMessage)){
     //init_LCD();
      lcd.reset();
      lcd.clear();                 // display message on LCD
      lcd.print("Stn:");
      lcd.print(byteToint(system_station_id));
      lcd.setCursor(2,1);
      lcd.print("Low Battery"); 
      flgInsertMessage=true;
    }
    break;
  } //switch

} //loop

/*******************************************************************************/
/* Initialises the contentents of the eeprom if magic number is incorrect */
void init_LCD(){
  lcd.begin(DOG_LCD_M162, 0x28, DOG_LCD_VCC_3V3);
  lcd.noCursor(); 
};
 

/*******************************************************************************/
/**
 * lowBatteryCallback  
 *   This is called each time a low battery condition is detected
 */
void lowBatteryCallback(){
  STATE=STATE_BATLOW;
}

/*******************************************************************************/
/**
 *  onMiddleClick
 *  This routine is called when the button has been debounced
 *  and toggles the RFID state between on and off
 *  Will only enable RFID if SD card is present
 *
 *    On - enables 5V (RFID module), turns on LCD backlight, RED LED to indicate RFID is enabled
 *    Off - turns off 5V and LCD Backlight, Red LED is turn off indicating that the RFID is off.
 *   
 *    @param btn  Button that has been debounced
 */
void onMiddleClick(DebounceButton* btn) {
  switch(STATE){
  case STATE_IDLE:
    rfidOn();
    break;
  case STATE_RFIDON:
    rfidOff();
    break;
  } //switch
}

/*******************************************************************************/
// Function called when button is held
// this will put the device into low power mode i.e. sleep
void onMiddleHold(DebounceButton* btn){

  // only sleep if not low battery
  if(STATE != STATE_BATLOW){
   //init_LCD();
    lcd.reset();
    lcd.clear();
    lcd.print("Stn:");
    lcd.print(byteToint(system_station_id));
    lcd.setCursor(0,1);
    lcd.print("Powering Down..."); 

    //Turn off RFID  
    digitalWrite(RFID_EN, LOW);  // turn off
    digitalWrite(nLED_RED,HIGH);

    delay(2000);                // display message before shutdown
    // turn of all peripherals
    digitalWrite(nLCD_BL,HIGH);  // LCD Backlight Off

    lcd.noDisplay();            // LCD Off
    STATE=STATE_SLEEPING;
    delay(50);
    sleepNow();                 // goto sleep

    // Returns here after wakeup
    STATE=STATE_IDLE;
    delay(100);
 
    lcd.display();              // LCD On
    //init_LCD();
    lcd.reset();
    lcd.clear();

    lcd.print("Stn:");
    lcd.print(byteToint(system_station_id));

    lcd.setCursor(5,1);
    lcd.print("Ready"); 
  }
}

/*******************************************************************************/
/**
 * updateCurrentRfidTag
 *   writes the RFID tag to file and updates LCD
 * check if the tag just read is not the same as previous
 */
void updateCurrentRfidTag(byte *tagNew)
{  
  
  // only print changed value     
  if(!equals(tagNew, rfidTagCurrent))        // checks against previous tag
  {
    saveTag(tagNew, rfidTagCurrent);         // is different so display and save to file

    byte i = 0;
    //init_LCD();
    lcd.reset();
    //lcd.begin(DOG_LCD_M162,0x28, DOG_LCD_VCC_3V3);
    lcd.clear(); 
    lcd.print("Stn:");
    lcd.print(byteToint(system_station_id));
    lcd.setCursor(0,1);  
    lcd.print("ID:");
    // STX
    //lcd.print(0x02, BYTE);
    //Serial.print("aA");            // Send Announce of received RFID TAG
    //Serial.print(byteToint(system_station_id));
    //Serial.print("R");
    for (i=0; i<5; i++) 
    {
      if (rfidTagCurrent[i] < 16) {
        lcd.print("0");
        //Serial.print("0");
      }
      lcd.print(rfidTagCurrent[i], HEX);
      //Serial.print(rfidTagCurrent[i], HEX);
    }
    Serial.print("-");  // close off packet
    lcdPrintTime();            // display time on LCD

    if(writeCsvRecord()<0) // Write CSV Record to SD Card
      {
       lcd.begin(DOG_LCD_M162,0x28, DOG_LCD_VCC_3V3);          // error
       lcd.print("SD Error");
      }
  } //if

}

/*******************************************************************************/
/**
 * readID12
 *
 * reads data from ID-12 / ID-20 rfid reader
 * @return TRUE if a valid rfid tag number and checksum has been validated
 *
 * @param *code updated pointer to buffer with tag being read
 *
 * Based on code by BARRAGAN, HC Gilje, djmatic, Martijn
 * http://www.arduino.cc/playground/Code/ID12 
 */
boolean readID12(byte *code)
{
  boolean result = false;
  byte val = 0;
  byte bytesIn = 0;
  byte tempbyte = 0;
  byte checksum = 0;

  // read 10 digit code + 2 digit checksum
  while (bytesIn < RFID_TAG_INPUT) 
  {                        
    if( rfidSerial.available()) 
    { 
      val = rfidSerial.read();

      // if CR, LF, ETX or STX before the 10 digit reading -> stop reading
      if((val == 0x0D)||(val == 0x0A)||(val == 0x03)||(val == 0x02)) break;

      // Do Ascii/Hex conversion:
      if ((val >= '0') && (val <= '9')) 
        val = val - '0';
      else if ((val >= 'A') && (val <= 'F'))
        val = 10 + val - 'A';


      // Every two hex-digits, add byte to code:
      if (bytesIn & 1 == 1) 
      {
        // make some space for this hex-digit by
        // shifting the previous hex-digit with 4 bits to the left:
        code[bytesIn >> 1] = (val | (tempbyte << 4));

        // If we're at the checksum byte, Calculate the checksum... (XOR)
        if (bytesIn >> 1 != RFID_TAG_LENGTH) checksum ^= code[bytesIn >> 1]; 
      } 
      else 
      {
        // Store the first hex digit first...
        tempbyte = val;                           
      }

      // ready to read next digit
      bytesIn++;                                
    } 
  }

  // read complete
  if (bytesIn == RFID_TAG_INPUT) 
  { 
    // valid tag
    if(code[5] == checksum) result = true; 
  }

  return result;
}


/*******************************************************************************/
/**
 * clearTag
 *   clear rfid tag buffer
 */
void clearTag(byte *arr, byte len)
{
  byte i;
  for (i=0; i < len ;i++) arr[i] = 0;
}

/**
 * saveTag
 *   save byte read in to rfid tag buffer
 */
void saveTag(byte *tagIn, byte *tagOut)
{
  byte i;
  for (i=0; i < RFID_TAG_LENGTH ;i++) tagOut[i] = tagIn[i];
}


/*******************************************************************************/
/**
 * equal
 *   compares 2 rfid tags
 *   @return true if they are equal
 */
boolean equals(byte *tag1, byte *tag2)
{
  boolean result = false;
  byte j;

  for (j=0; j < RFID_TAG_LENGTH ;j++) 
  {
    if(tag1[j] != tag2[j]) break;
    else if (j == RFID_TAG_LENGTH-1) result = true;
  }    
  return result;
}

/*******************************************************************************/
/** 
 * lcdPrintTime
 *   print date/time to LCD
 */
void lcdPrintTime()
{
  uint8_t u8Status = rtc.get();

  lcd.setCursor(7,0);
  if (!u8Status)
  {
    sprintf(dateString, "%02u:%02u:%02u",
    rtc.time.hour,rtc.time.min, rtc.time.sec);
    lcd.print(dateString);
  }
  else
  {
    lcd.print("RTC ERR: ");
    lcd.print(u8Status, HEX);
  }
}

/*******************************************************************************/
/**
 * writeCsvRecord
 *
 *  Writes a records to the current open file in CSV format
 *    ID, RFIDTAG, TIMESTAMP, VOLTAGE
 *    TimeStamp format yyyy-mm-dd hh:mm:ss
 *
 *  After calling this function the SPI bus should be re-initialised as it is 
 *  initialiased by the SD Card libraries.
 */
int writeCsvRecord()
{
  
 
  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  card.init(SPI_HALF_SPEED);

  // initialize a FAT volume
  volume.init(&card);

  // open the root directory
  root.openRoot(&volume);

  // create a new file
  //char StnID[2];
  //String fileName = 'LOGSTN' + int(system_station_id) + '.CSV';            
  char name[] = "LOGSTN.CSV";
  //fileName.toCharArray(name,sizeof(fileName));
 
  file.open(&root, name, O_CREAT | O_APPEND | O_WRITE);  // create a new file if not exist, else append to existing.

  //field
  //file.print("STATIONID,RFID,TIMESTAMP,VOLTAGE");
  //file.println();                        // CR/LF
  //    file.sync();                           // Flush file record to SDFat
  uint8_t u8Status = rtc.get();        // get the current time and update time structures
  byte i = 0;
  file.print(byteToint(system_station_id));          // Station ID
  file.print(",");             
  for (i=0; i<5; i++)                  // RFID TAG ID
  {
    if (rfidTagCurrent[i] < 16) file.print("0");
    file.print(rfidTagCurrent[i], HEX);
  }
  file.print(",");             
  //filePrintTime();
  file.print("20");                     // TimeStamp as yyyy-mm-dd hh:mm:ss
  if(rtc.time.year <10)
    file.print("0");
  file.print(rtc.time.year,DEC);
  file.print("-");
  if(rtc.time.month <10)
    file.print("0");
  file.print(rtc.time.month,DEC);
  file.print("-");
  if(rtc.time.day <10)
    file.print("0");
  file.print(rtc.time.day,DEC);    
  file.print(" ");
  if(rtc.time.hour < 10)
    file.print("0");
  file.print(rtc.time.hour,DEC);
  file.print(":");
  if(rtc.time.min < 10)
    file.print("0");
  file.print(rtc.time.min,DEC);
  file.print(":");
  if(rtc.time.sec <10)
    file.print("0");
  file.print(rtc.time.sec,DEC);
  file.print(",");
  file.print(battery.getVoltage());      // Battery Voltage as scaled integer e.g. 650 = 6.50 v  i.e. /100 to get voltage
  file.println();                        // CR/LF
  file.sync();                           // Flush file record to SDFat
  file.close();
  return 0;
  
}

/*******************************************************************************/
/**
 *   sdDateTimeCallback
 *
 *    This is called whenever a file is updated on the SD Card by the SDFAT Library
 */
void sdDateTimeCallback(uint16_t* date, uint16_t* time){
  uint16_t year;
  uint8_t month, day, hour, minute, second;

  year = 2000 + rtc.time.year;
  month = rtc.time.month;
  day = rtc.time.day;
  hour = rtc.time.hour;
  minute = rtc.time.min;
  second = rtc.time.sec;

  *date = FAT_DATE(year, month,day);
  *time = FAT_TIME(hour, minute, second); 
}

/*******************************************************************************/
/**
 * wakeUpNow
 *    This routine is attached to the pushbutton interrupt and is executed when interrupt low is detected
 */
void wakeUpNow(){
}

/*******************************************************************************/
/**
 * sleepNow
 *   Puts Ralllog into deep sleep to preserve power
 */
void sleepNow(){

  /* The 5 different modes are:
   *     SLEEP_MODE_IDLE         -the least power savings 
   *     SLEEP_MODE_ADC
   *     SLEEP_MODE_PWR_SAVE
   *     SLEEP_MODE_STANDBY
   *     SLEEP_MODE_PWR_DOWN     -the most power savings
   *
   * For now, we want as much power savings as possible, so we 
   * choose the according 
   * sleep mode: SLEEP_MODE_PWR_DOWN
   */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

  sleep_enable();          // enables the sleep bit in the mcucr register
  // so sleep is possible. just a safety pin 
  cli();

  // Temporary turn off watchdog and kill watchdog if running
  wdt_reset();
  MCUSR = 0;
  WDTCSR |= _BV(WDCE) | _BV(WDE);
  WDTCSR = 0;


  /* Now it is time to enable an interrupt. We do it here so an 
   * accidentally pushed interrupt button doesn't interrupt 
   * our running program. if you want to be able to run 
   * interrupt code besides the sleep function, place it in 
   * setup() for example.
   * 
   * In the function call attachInterrupt(A, B, C)
   * A   can be either 0 or 1 for interrupts on pin 2 or 3.   
   * 
   * B   Name of a function you want to execute at interrupt for A.
   *
   * C   Trigger mode of the interrupt pin. can be:
   *             LOW        a low level triggers
   *             CHANGE     a change in level triggers
   *             RISING     a rising edge of a level triggers
   *             FALLING    a falling edge of a level triggers
   *
   * In all but the IDLE sleep modes only LOW can be used.
   */
  byte spi_save = SPCR;
  SPCR = 0;

  // turn OFF analog parts
  //ACSR = ACSR & 0b11110111 ; // clearing ACIE prevent analog interupts happening during next command
  //ACSR = ACSR | 0b10000000 ; // set ACD bit powers off analog comparator
  //ADCSRA = ADCSRA & 0b01111111 ;  // clearing ADEN turns off analog digital converter
  ADMUX &= B00111111;        // Comparator uses AREF/GND and not internally generated references

  TCCR1B = 0b00000000;       // disable timer1

  power_adc_disable();       // power down onchip subsystems
  power_spi_disable();
  power_twi_disable();
  power_usart0_disable();

  sei();

  attachInterrupt(1,wakeUpNow, LOW); // use interrupt 1 (pin 3) and run function
  // wakeUpNow when pin 3 gets LOW 

#ifdef BODSE
    // Turn off BOD in sleep (picopower devices only)
  //  MCUCR |= _BV(BODSE);
  //  MCUCR |= _BV(BODS);

  // Turn off BOD in sleep (picopower devices only)
  MCUCR = MCUCR | bit(BODSE) | bit(BODS);
  MCUCR = MCUCR &~ bit(BODSE) | bit(BODS);
  SMCR = bit(SM1) | bit(SE);
#endif
  sleep_mode();            // here the device is actually put to sleep!!
  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
  //returns from sleep here
  sleep_disable();         // first thing after waking from sleep:
  // disable sleep...
  detachInterrupt(1);      // disables interrupt 1 on pin 3 so the 
  // wakeUpNow code will not be executed 
  // during normal running time

  power_adc_enable();      // re-enable subsystems
  power_spi_enable();
  power_twi_enable();
  power_usart0_enable();
  SPCR = spi_save;
}

/*******************************************************************************/
// Turns on the RFID Module
void rfidOn(){
  digitalWrite(RFID_EN, HIGH);  // turn on
  digitalWrite(nLED_RED,LOW);
  digitalWrite(nLCD_BL,LOW);    //LCD Backlight On
  //init_LCD();
  lcd.reset();
  lcd.clear();
  lcd.print("Stn:");
  lcd.print(byteToint(system_station_id));
  lcd.setCursor(0,1);
  lcd.print("Swipe Next Card.");
  STATE=STATE_RFIDON;
}

/*******************************************************************************/
// Turns off the RFID Module
void rfidOff(){
  digitalWrite(RFID_EN, LOW);   // turn off
  digitalWrite(nLED_RED,HIGH);
  digitalWrite(nLCD_BL,HIGH);   // LCD Backlight Off
  //init_LCD();
  lcd.reset();
  lcd.clear();
  lcd.print("Stn:");
  lcd.print(byteToint(system_station_id));
  lcd.setCursor(5,1);
  lcd.print("Ready");
  STATE=STATE_IDLE;
}

















