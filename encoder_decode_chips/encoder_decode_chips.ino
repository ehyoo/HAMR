  #include <avr/io.h>
#include <stdio.h>

#define LOOPTIME_MICROS 10

long int current_time = 0;
long int next_time = 0;
/* UNO PIN MAPPING
 *  D2 - 4
 *  D3 - 5
 *  D4 - 6
 *  D5 - 11
 *  D6 - 12
 *  D7 - 13
 */
 
// motor 1
#define PIN_M1_D



long int start_time;
void setup() {
  Serial.begin(250000);
  init_pins();

    start_time = micros();
}

long counter = 0;

char m1;
char m2;
char m3;

char cm;

void loop() {
}

void init_pins(){
  DDRD = 0; 
}

void rencoderA_M1()  {
  interrupt_M1_A = (PORTD >> 2) & 1;
  if (interrupt_M1_A != interrupt_M1_B) curr_count_M1--; // encoderA changed before encoderB -> forward
  else                                  curr_count_M1++; // encoderB changed before encoderA -> reverse
}

void rencoderB_M1()  {
  interrupt_M1_B = (PORTD >> 3) & 1;
  if (interrupt_M1_A != interrupt_M1_B) curr_count_M1++; // encoderB changed before encoderA -> reverse
  else                                  curr_count_M1--; // encoderA changed before encoderB -> forward
}

//void rencoderA_M2()  {
//  int encoderA_pin = PIN_M2_ENCODER_OUTA;
//  int encoderB_pin = PIN_M2_ENCODER_OUTB;
//  // Record rising or falling edge from encoder
//  interrupt_M2_A = digitalRead(encoderA_pin);
//
//  //curr_count_M2++;
//  if (interrupt_M2_A != interrupt_M2_B) curr_count_M2--; // encoderA changed before encoderB -> forward
//  else                                  curr_count_M2++; // encoderB changed before encoderA -> reverse
//}
//
//void rencoderB_M2()  {
//  int encoderA_pin = PIN_M2_ENCODER_OUTA;
//  int encoderB_pin = PIN_M2_ENCODER_OUTB;
//  // Record rising or falling edge from encoder
//  interrupt_M2_B = digitalRead(encoderB_pin);
//
//  //curr_count_M2++;
//  if (interrupt_M2_A != interrupt_M2_B) curr_count_M2++; // encoderB changed before encoderA -> reverse
//  else                                  curr_count_M2--; // encoderA changed before encoderB -> forward
//}
