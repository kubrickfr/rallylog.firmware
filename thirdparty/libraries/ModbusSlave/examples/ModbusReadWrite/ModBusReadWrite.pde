
#include <ModbusSlave.h>

/* First step MBS: create an instance */
ModbusSlave mbs;

/* slave registers */
enum {        
        MB_REG0,
        MB_CTRL,        /* Led control on, off or blink */
        MB_TIME,        /* blink time in milliseconds */
        MB_CNT,        /* count the number of blinks */
        MB_REGS	 	/* total number of registers on slave */
};

int asdf=1;
int regs[MB_REGS];
int ledPin = 13;


void setup() 
{
        
/* the Modbus slave configuration parameters */
const unsigned char SLAVE = 1;
const long BAUD = 9600;
const char PARITY = 'n';
const char TXENPIN = 1;

       /* Second step MBS: configure */
      mbs.configure(SLAVE,BAUD,PARITY,TXENPIN);

        pinMode(ledPin, OUTPUT);
        pinMode(2,INPUT);
}

void loop()
{


       asdf=digitalRead(2);
      regs[MB_REG0]=asdf;


  /* Third and las step MBS: update in loop*/
       mbs.update(regs, MB_REGS);

   if(regs[MB_CTRL]==1)
     digitalWrite(ledPin,1);
     else
     digitalWrite(ledPin,0);

}