#include <DebounceButton.h>
#include <DogLcd.h>
#include <Battery.h>


#define ADC_BAT_MON 6
#define BAT_MON_EN 7
#define SI_PIN 11
#define SCK_PIN 13
#define CSB_PIN 16
#define RS_PIN 14
#define LCD_LINES 2
#define LCD_BL 15
#define RFID_EN 8
#define Button_OK 2
#define LED1 5


DebounceButton buttonOk = DebounceButton(Button_OK, DBTN_PULLUP_INTERNAL, 50, 1000, 500);
// initialize the library with the numbers of the interface pins
DogLcd lcd(SI_PIN, SCK_PIN, RS_PIN, CSB_PIN );
Battery battery(ADC_BAT_MON, BAT_MON_EN, 0);

int RFID_STATE=0;


void setup(){
  pinMode(LED1,OUTPUT);
  digitalWrite(LED1,HIGH); //off
  
  pinMode(BAT_MON_EN, OUTPUT);
  digitalWrite(BAT_MON_EN, LOW); // turn off mo
  buttonOk.onClick = onClickOK;
  
  pinMode(RFID_EN, OUTPUT);
  digitalWrite(RFID_EN, LOW); // turn on

  // set up the LCD type and the contrast setting for the display 
  lcd.begin(DOG_LCD_M162, 0x28, DOG_LCD_VCC_3V3);
  lcd.print("Battery Test");
  delay(5000);

}

void loop(){
  DebounceButton::updateAll();      // update buttons
  battery.update();                 // update battery status
  
  int reading;
  int intvolt;
  reading = battery.getVoltage();
 
  lcd.clear();
  lcd.print("Bat: ");
  //lcd.print(reading);
  print_int100_dec2(reading);
  lcd.print("v");
}

void onClickOK(DebounceButton* btn) {
  switch(RFID_STATE){
   case 0:
    digitalWrite(RFID_EN, HIGH); // turn on
    digitalWrite(LED1,LOW);
    RFID_STATE=1;
    break;
   case 1:
    digitalWrite(RFID_EN, LOW); // turn off
     digitalWrite(LED1,HIGH);
    RFID_STATE=0;
    break;
  }
}
  
//-----------------------------------------------------------------------------------
// This function prints int that was scaled by 100 with 2 decimal places
//-----------------------------------------------------------------------------------
void print_int100_dec2(int temp) {

  lcd.print(temp/1000,DEC);        // divide by 100 and print interger value
  lcd.print(".");
  if ((temp%1000) < 10) {              // if fractional value has only one digit
    lcd.print("0");               // print a "0" to give it two digits
    lcd.print(temp%1000,DEC);      // get remainder and print fractional value
  }
  else {
    lcd.print(temp%1000,DEC);      // get remainder and print fractional value
  }
}


