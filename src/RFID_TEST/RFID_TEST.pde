#include <NewSoftSerial.h>
#include <DebounceButton.h>
#include <DogLcd.h>
#include <Wire.h>
#include <i2c_rtc_m41t00s.h>
#include <Fat16.h>
#include <Fat16util.h> // use functions to print strings from flash memory

// settings
#define RESET_ENABLED     0
#define RESET_TIME        1000
#define RFID_REMOVED_TIME 1500
#define RFID_TAG_LENGTH   5 // 5 Bytes
#define RFID_TAG_INPUT    12 // DATA (10 ASCII) + CHECK SUM (2 ASCII)

// digital pins
#define ADC_BAT_MON 6
#define BAT_MON_EN 7
#define SI_PIN 11
#define SCK_PIN 13
#define CSB_PIN 16
#define RS_PIN 14
#define LCD_LINES 2
#define LCD_BL 15
#define RFID_EN 8
#define Button_OK 3
#define LED1 5
#define LED2 6

#define RFID_RX           9
#define RFID_TX           6 // not used


SdCard card;
Fat16 file;
// store error strings in flash to save RAM
#define error(s) error_P(PSTR(s))

// connection to computer established
boolean connected = false;

// software serial connection for rfid reader
NewSoftSerial rfidSerial =  NewSoftSerial(RFID_RX, RFID_TX);
DebounceButton buttonOk = DebounceButton(Button_OK, DBTN_PULLUP_INTERNAL, 50, 1000, 500);
// initialize the library with the numbers of the interface pins
DogLcd lcd(SI_PIN, SCK_PIN, RS_PIN, CSB_PIN );



// instantiate i2c_rtc_m41t00s object
i2c_rtc_m41t00s rtc;
// string to store current time/date
char dateString[50];

int RFID_STATE=0;

char messageString[12];  // incomming message
int inByte = 0;         // incoming serial byte
int announceCount = 5;  // number of times announce message sent

// millis of last Reset 
unsigned int nowReset = 0;
// millis of last seen rfid tag
unsigned int nowLastRfid = 0;
// reader is in reset state
boolean rfidEnabled = false;
// 
boolean rfidTagSeen = false;
// last seen tag
byte rfidTagCurrent[RFID_TAG_LENGTH];
// temp tag
byte rfidTagTemp[6];

void error_P(const char* str) {
  PgmPrint("error: ");
  SerialPrintln_P(str);
  if (card.errorCode) {
    PgmPrint("SD error: ");
    Serial.println(card.errorCode, HEX);
  }
  while(1);
}

void setup() 
{  
  
  
  
  // initialize the SD card
  if (!card.init()) error("card.init");
    
  // initialize a FAT16 volume
  if (!Fat16::init(&card)) error("Fat16::init");
  
  // create the file if it doesn't exist
  char name[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    name[6] = i/10 + '0';
    name[7] = i%10 + '0';
    // O_CREAT - create the file if it does not exist
    // O_EXCL - fail if the file exists
    // O_WRITE - open for write only
    if (file.open(name, O_CREAT | O_EXCL | O_WRITE))break;
  }
  if (!file.isOpen()) error ("create");
    // clear write error
  file.writeError = false;

  
  pinMode(LED1,OUTPUT);
  digitalWrite(LED1,HIGH); //off
  
  pinMode(LED2,OUTPUT);
  digitalWrite(LED2,HIGH); //off
  
  pinMode(BAT_MON_EN, OUTPUT);
  digitalWrite(BAT_MON_EN, LOW); // turn off mo
  buttonOk.onClick = onClickOK;
  
  pinMode(RFID_EN, OUTPUT);
  digitalWrite(RFID_EN, LOW); // turn on

  // initialize i2c interface
  rtc.begin();
  
  // set up the LCD type and the contrast setting for the display 
  lcd.begin(DOG_LCD_M162, 0x28, DOG_LCD_VCC_3V3);
  lcd.clear();
  lcd.noCursor();
  lcd.print("RFID Test");
  lcd.setCursor(0,1);
  //lcd.autoscroll();
  lcd.print("middle btn start");
  //delay(2000);
  //
  Serial.begin(9600);
  rfidSerial.begin(9600);

  // clear current tag
  clearTag(rfidTagCurrent, RFID_TAG_LENGTH);
  establishContact();        //phone home by sending announce message
}



void loop () 
{
  DebounceButton::updateAll();      // update buttons

  byte action = 0;
  unsigned int now = millis();
  
  // receive valid byte via serial 
  
  if(Serial.available() >= 12 && ((millis() - now) < 3000))
  {  
       // get the message
       messageString[0] = Serial.read();   // message start = 'a'
       messageString[1] = Serial.read();   // command
       messageString[2] = Serial.read();   // dst address High
       messageString[3] = Serial.read();   // dst address Low
       messageString[4] = Serial.read();   // Data0
       messageString[5] = Serial.read();   // Data1
       messageString[6] = Serial.read();   // Data2
       messageString[7] = Serial.read();   // Data3
       messageString[8] = Serial.read();   // Data4
       messageString[9] = Serial.read();   // Data5
       messageString[10] = Serial.read();  // Data6
       messageString[11] = Serial.read();  // Data7     
       Serial.print(messageString);
  }
  
  clearTag(rfidTagTemp, 6);
    
  // wait for the next STX byte
  while(rfidSerial.available() && action != 0x02)
       action = rfidSerial.read();
      
     // STX byte found -> RFID tag available
     if(action == 0x02)
     {
       digitalWrite(LED1,LOW);       // turn on LED
       if(readID12(rfidTagTemp))
       {
         nowLastRfid = millis();
         rfidTagSeen = true;
         updateCurrentRfidTag(rfidTagTemp);
       }
       delay(250);
       digitalWrite(LED1,HIGH);       // turn off LED
     }
 
  // delay 100 milliseconds
  //delay(100);
}


/*
 * Establishes contact with host computer
 * Sends an announce message 3 times or until it receives a response
 */
void establishContact()
{
  while(Serial.available() <= 0 && announceCount > 0)
  {
      Serial.print("aA**STARTED-");   // send announce message
      delay(1000);
      announceCount--;
  } 
}


/**
 * print actual tag number to serial
 */
void updateCurrentRfidTag(byte *tagNew)
{  
  // only print changed value     
  if(!equals(tagNew, rfidTagCurrent))
  {
    saveTag(tagNew, rfidTagCurrent);
    
    byte i = 0;
    
    lcd.clear();   
    lcd.print("ID:");
    // STX
    //lcd.print(0x02, BYTE);
    
    for (i=0; i<5; i++) 
    {
      if (rfidTagCurrent[i] < 16) lcd.print("0");
      lcd.print(rfidTagCurrent[i], HEX);
    }
    lcdPrintTime();
    
    filePrintTime();
    for (i=0; i<5; i++) 
    {
      if (rfidTagCurrent[i] < 16) file.print("0");
      file.print(rfidTagCurrent[i], HEX);
    }
    file.print(rfidTagCurrent[i], HEX);
    // ETX
    //lcd.print(0x03, BYTE);
  } else {
    lcd.setCursor(0,1);
    lcd.print("Same Card");
  }
}

/**
 * read data from rfid reader
 * @return rfid tag number
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



/**
 * clear rfid tags
 */
void clearTag(byte *arr, byte len)
{
  byte i;
  for (i=0; i < len ;i++) arr[i] = 0;
}

/**
 * save rfid tag
 */
void saveTag(byte *tagIn, byte *tagOut)
{
  byte i;
  for (i=0; i < RFID_TAG_LENGTH ;i++) tagOut[i] = tagIn[i];
}


/**
 * compare 2 rfid tags
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

void onClickOK(DebounceButton* btn) {
  switch(RFID_STATE){
   case 0:
    digitalWrite(RFID_EN, HIGH); // turn on
    digitalWrite(LED2,LOW);
    RFID_STATE=1;
    lcd.clear();
    lcd.print("Swipe Card..");
    break;
   case 1:
    digitalWrite(RFID_EN, LOW); // turn off
    digitalWrite(LED2,HIGH);
    RFID_STATE=0;
    break;
  }
}

// print date/time to serial interface
void lcdPrintTime()
{
  uint8_t u8Status = rtc.get();
  
  lcd.setCursor(0,1);
  if (!u8Status)
  {
    sprintf(dateString, "%02u:%02u:%02u",
      rtc.time.hour,rtc.time.min, rtc.time.sec);
    lcd.print(dateString);
    //lcd.print(rtc.time.hour);
    //lcd.print(rtc.time.min);
    //lcd.print(rtc.time.sec);
  }
  else
  {
    lcd.print("RTC ERR: ");
    lcd.print(u8Status, HEX);
  }
}

void filePrintTime()
{
  uint8_t u8Status = rtc.get();
  if (!u8Status)
  {
    sprintf(dateString, "%02u:%02u:%02u",
      rtc.time.hour,rtc.time.min, rtc.time.sec);
    file.print(dateString);
  }
  else
  {
    lcd.setCursor(0,1);
    lcd.print("RTC ERR: ");
    lcd.print(u8Status, HEX);
  }
}

