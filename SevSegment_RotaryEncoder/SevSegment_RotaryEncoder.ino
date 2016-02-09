/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
der GNU General Public License, wie von der Free Software Foundation,
Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
veröffentlichten Version, weiterverbreiten und/oder modifizieren.

Dieses Programm wird in der Hoffnung, dass es nützlich sein wird, aber
OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite
Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
Siehe die GNU General Public License für weitere Details.

Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.
*/

/*
  SevSegment_RotaryEncoder on Arduino Uno / 328P
*/

/*
7Segment Test:

13: PB5
12: PB4
11: PB3
10: PB2
09: PB1
08: PB0
07: PD7

V1.0 - !!OBSOLETE!! Wiring changed

7Segment "0" :     12  11  10  09  08  07
7Segment "1" :             10          07
7Segment "2" : 13      11  10  09  08    
7Segment "3" : 13          10  09  08  07
7Segment "4" : 13  12      10          07
7Segment "5" : 13  12          09  08  07
7Segment "6" : 13  12  11      09  08  07
7Segment "7" :             10      08  07
7Segment "8" : 13  12  11  10  09  08  07
7Segment "9" : 13  12      10  09  08  07
7Segment "A" : 13  12  11  10      08  07

V1.1

*/

#include <Arduino.h>
#include <Wire.h>

/*
  Seven segment constants and variables
*/
uint8_t sev_seg_counter;

#define  sev_seg_1_pin  A2
#define  sev_seg_2_pin  A3
#define  sev_seg_3_pin  4
#define  sev_seg_4_pin  5

uint8_t sev_seg_1 = 11;
uint8_t sev_seg_2 = 11;
uint8_t sev_seg_3 = 11;
uint8_t sev_seg_4 = 11;

#define ch_lvl_pin  6

/*
  Rotary Encoder constants and variables
*/
#define    PinCLK_1   2
#define    PinDT_1    3
volatile  uint8_t       address    = 0;



#define    PinCLK_2   A0     // Used for generating interrupts using CLK signal
#define    PinDT_2    A1     // Used for reading DT signal
volatile uint8_t       dimValue    = 0;


void setup() {
  
  //
  pinMode(sev_seg_1_pin,OUTPUT);
  pinMode(sev_seg_2_pin,OUTPUT);
  pinMode(sev_seg_3_pin,OUTPUT);
  pinMode(sev_seg_4_pin,OUTPUT);

  //Segments
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  
  //
  pinMode(ch_lvl_pin,OUTPUT);
  digitalWrite(ch_lvl_pin,LOW);
  
  sev_seg_counter = 0;
  
  timer1_interrupt_configure();
  timer1_interrupt_1kHz();
  timer1_interrupt_enable();
  
  //set default values to be displayed on 7segment display
  sev_seg_1 = address/10;
  sev_seg_2 = address-sev_seg_1*10;
  sev_seg_3 = dimValue/10;
  sev_seg_4 = dimValue-sev_seg_3*10;
  
  //
  pinMode(PinCLK_1,INPUT);
  pinMode(PinDT_1, INPUT);
  pciSetup(PinCLK_1);
  
  pinMode(PinCLK_2,INPUT);
  pinMode(PinDT_2, INPUT);
  pciSetup(PinCLK_2);
  
  //
  Wire.begin(2);                // join i2c bus with address #2
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
}
  
void loop() {
    /*
    sev_seg_4 = sev_seg_3;
    sev_seg_3 = sev_seg_2;
    sev_seg_2 = sev_seg_1;
    sev_seg_1 = (sev_seg_1+1);
    if(sev_seg_1>9){sev_seg_1 = 0;}
    delay(500);
    */
    
}

void sev_seg_write(uint8_t chiffre){
  PORTB&=~(0b00111111);
  PORTD&=~(0b10000000);
  switch(chiffre)
  {
    case 0:
      PORTB|= (0b00011111);
      PORTD|= (0b10000000);
      break;
    case 1:
      PORTB|= (0b00000100);
      PORTD|= (0b10000000);
      break;
    case 2:
      PORTB|= (0b00101111);
      PORTD|= (0b00000000);
      break;
    case 3:
      PORTB|= (0b00100111);
      PORTD|= (0b10000000);
      break;
    case 4:
      PORTB|= (0b00110100);
      PORTD|= (0b10000000);
      break;
    case 5:
      PORTB|= (0b00110011);
      PORTD|= (0b10000000);
      break;
    case 6:
      PORTB|= (0b00111011);
      PORTD|= (0b10000000);
      break;
    case 7:
      PORTB|= (0b00000101);
      PORTD|= (0b10000000);
      break;
    case 8:
      PORTB|= (0b00111111);
      PORTD|= (0b10000000);
      break;
    case 9:
      PORTB|= (0b00110111);
      PORTD|= (0b10000000);
      break;
    case 10:
      PORTB|= (0b00111101);
      PORTD|= (0b10000000);      
      break;
    default:
      PORTB|= (0b00000000);
      PORTD|= (0b00000000);
      break;    
  }
}

  void timer1_interrupt_configure()
    {
      cli();
      //Timer1 settings
      //set timer1 interrupt at 10Hz
      TCCR1A = 0;// set entire TCCR1A register to 0
      TCCR1B = 0;// same for TCCR1B
      TCNT1  = 0;//initialize counter value to 0
      // set compare match register for 1hz increments
      OCR1A = 1562;// = (8*10^6) / (1*1024) - 1 (must be <65536)
      // turn on CTC mode
      TCCR1B |= (1 << WGM12);
      // Set CS02 bit for 256 prescaler
      TCCR1B |= (1 << CS02);// | (1 << CS10);
      sei();
    }
    
  void timer1_interrupt_enable()
    {
      cli();
      // enable timer compare interrupt
      TIMSK1 |= (1 << OCIE1A);
      sei();
    }
    
  void timer1_interrupt_1kHz()
    {
      cli();
      OCR1A = 31;
      TCNT1 = 0;
      sei();
    }
    
  ISR(TIMER1_COMPA_vect)
    {
      switch(sev_seg_counter)
        {
          case 0:
            digitalWrite(sev_seg_4_pin,LOW);
            sev_seg_write(11);
            sev_seg_write(sev_seg_1);
            digitalWrite(sev_seg_1_pin,HIGH);
            break;
          case 1:
            digitalWrite(sev_seg_1_pin,LOW);
            sev_seg_write(11);
            sev_seg_write(sev_seg_2);
            digitalWrite(sev_seg_2_pin,HIGH);
            break;
          case 2:
            digitalWrite(sev_seg_2_pin,LOW);
            sev_seg_write(11);
            sev_seg_write(sev_seg_3);
            digitalWrite(sev_seg_3_pin,HIGH);
            break;
          case 3:
            digitalWrite(sev_seg_3_pin,LOW);
            sev_seg_write(11);
            sev_seg_write(sev_seg_4);
            digitalWrite(sev_seg_4_pin,HIGH);
            break;
        }
        sev_seg_counter = (sev_seg_counter+1)&(0b11);
    }
    

//Rotary Encoder Section

// -----------------------------------------------------------------------------
// Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK

ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
 {
    if (digitalRead(PinDT_2)==digitalRead(PinCLK_2))
    {
      if(dimValue>0)
      {
        dimValue = dimValue - 1;
        digitalWrite(ch_lvl_pin, HIGH);
      }
    }    
    else
    {
      if(dimValue<100)
      {
        dimValue = dimValue + 1;
        digitalWrite(ch_lvl_pin, HIGH);
      }
    }
    sev_seg_3 = dimValue/10;
    sev_seg_4 = dimValue-sev_seg_3*10;
 }


ISR (PCINT2_vect) // handle pin change interrupt for A0 to A5 here
 {
    if (digitalRead(PinDT_1)==digitalRead(PinCLK_1))
    {
      if(address>0)
      {
        address = address - 1;
        digitalWrite(ch_lvl_pin, HIGH);    
      }
    }    
    else
    {
      if(address<64)
      {
        address = address + 1;
        digitalWrite(ch_lvl_pin, HIGH);    
      }
    }
    sev_seg_1 = address/10;
    sev_seg_2 = address-sev_seg_1*10;
 }

// enable interrupt for pin
void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

// i2c communication interrupt functions

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
  uint8_t array[] = {'a',0,' ','d',0,' '};
  array[1] = address;
  array[4] = dimValue;
  Wire.write(array,6); // respond with message of 6 bytes
  digitalWrite(ch_lvl_pin, LOW);
  // as expected by master
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  uint8_t rec_array[6];
  uint8_t i = 0;
  while ( (0 < Wire.available())&&(i<6) ) { // loop through all but the last
    rec_array[i] = Wire.read();
    Serial.print(i);
    Serial.print(": ");
    Serial.println(rec_array[i]);
    i++;
  }
  while( 0 < Wire.available() ){Wire.read();}    //clear i2c bus
  Serial.println("escaped while loop");
  if(rec_array[0]=='a')
  {
    address  =  rec_array[1];
  }
  if(rec_array[3]=='d')
  {
    dimValue  =  rec_array[4];
  }
}


//Add this to your local boards.txt:
/*
##############################################################

a328pNoOSC.name=328PnoOSC

a328pNoOSC.upload.tool=avrdude
a328pNoOSC.upload.protocol=arduino

a328pNoOSC.bootloader.tool=avrdude
a328pNoOSC.bootloader.unlock_bits=0x3F
a328pNoOSC.bootloader.lock_bits=0x0F

a328pNoOSC.build.f_cpu=8000000L
a328pNoOSC.build.board=AVR_328P
a328pNoOSC.build.core=arduino
a328pNoOSC.build.variant=eightanaloginputs

a328pNoOSC.upload.maximum_size=30720
a328pNoOSC.upload.maximum_data_size=2048
a328pNoOSC.upload.speed=57600

a328pNoOSC.bootloader.low_fuses=0xE2
a328pNoOSC.bootloader.high_fuses=0xDE
a328pNoOSC.bootloader.extended_fuses=0x05
a328pNoOSC.bootloader.file=atmega/ATmegaBOOT_168_atmega328.hex

a328pNoOSC.build.mcu=atmega328p

*/
