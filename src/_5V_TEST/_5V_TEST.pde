#define numButtons 3
#define debounceDepth 10

char buttonRead;
char buttonMemory[debounceDepth] = {
  0,0,0,0,0,0,0,0,0,0};
int buttonHasReleased[numButtons] = {
  true,true,true};
char buttonPressFilter = 0xFF;
int buttonState[numButtons] = {
  0,0,0};

const int EN_5V_PIN = 8;
const int BTN1_PIN = 2;
const int BTN2_PIN = 3;
const int BTN3_PIN = 4;

const int LED_PIN = 6;


void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);    // LED OFF

  pinMode(EN_5V_PIN, OUTPUT);
  digitalWrite(EN_5V_PIN, LOW);   // turn off 5V

  pinMode(BTN1_PIN, INPUT);
  pinMode(BTN2_PIN, INPUT);
  pinMode(BTN3_PIN, INPUT);

  digitalWrite(BTN1_PIN, HIGH);   // pullups on
  digitalWrite(BTN2_PIN, HIGH);   // pullups on
  digitalWrite(BTN3_PIN, HIGH);   // pullups on

}

void loop() {
  int i;

  buttonPressFilter = 0xFF;
  
  for(i=(debounceDepth-1); i>0; i--) {
    buttonMemory[i] = buttonMemory[i-1];
    buttonPressFilter &= buttonMemory[i];
  }

  buttonRead = digitalRead(BTN1_PIN);
  
  if((buttonRead & bitMask[0]) != 1) {  // if current read shows button is pressed
    buttonMemory[0] |= bitMask[0];      // set the memory bit for button
  }
  else {
    buttonMemory[0] &= ~(bitMask[0]);   // clear the memory bit for the button
  }

  buttonPressFilter &= buttonMemory[0]; // AND the final memory bits into the buttonPressFilter

  for(i=0; i<numButtons; i++) {
    if((buttonPressFilter & bitMask[i]) != 1) {
      if(buttonHasReleased[i] == true) {
        if(buttonState[i] == 0) {
          // ENTER CODE HERE FOR WHAT YOU WANT TO HAPPEN WHEN BUTTON IS ON
          buttonState[i] = 1;
        } 
        else {
          // ENTER CODE HERE FOR WHAT YOU WANT TO HAPPEN WHEN BUTTON IS OFF
          buttonState[i] = 0;
        }
      }
      buttonHasReleased[i] = false;
    } 
    else {
      buttonHasReleased[i] = true;
    }
  }


}



