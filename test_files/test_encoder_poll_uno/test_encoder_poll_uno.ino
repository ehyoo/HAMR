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
#define PIN_M1_ENCODER_OUTA 4
#define PIN_M1_ENCODER_OUTB 5
// motor 2 
#define PIN_M2_ENCODER_OUTA 6
#define PIN_M2_ENCODER_OUTB 11
// turret motor
#define PIN_M3_ENCODER_OUTA 12
#define PIN_M3_ENCODER_OUTB 13

/* encoder counters */
long curr_count_M1 = 0; long prev_count_M1 = 0; 
long curr_count_M2 = 0; long prev_count_M2 = 0; 
long curr_count_M3 = 0; long prev_count_M3 = 0; 

/* encoder output state */
char interrupt_M1_A = 0; char interrupt_M1_B = 0;
char interrupt_M2_A = 0; char interrupt_M2_B = 0;
char interrupt_M3_A = 0; char interrupt_M3_B = 0;

char p_interrupt_M1_A = 0; char p_interrupt_M1_B = 0;
char p_interrupt_M2_A = 0; char p_interrupt_M2_B = 0;
char p_interrupt_M3_A = 0; char p_interrupt_M3_B = 0;

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
  if (counter < 1000000){
    interrupt_M1_A = (PORTD >> 2) & 1;
    cm = interrupt_M1_A == p_interrupt_M1_A;
    m1 = interrupt_M1_A = interrupt_M1_B;
    curr_count_M1 += cm & m1;
    curr_count_M1 -= cm & m1;

    interrupt_M2_A = (PORTD >> 3) & 1;
    cm = interrupt_M2_A == p_interrupt_M2_A;
    m1 = interrupt_M1_A = interrupt_M1_B;
    curr_count_M1 += cm & m1;
    curr_count_M1 -= cm & m1;
    
    interrupt_M3_A = (PORTD >> 4) & 1;
    cm = interrupt_M3_A == p_interrupt_M3_A;
    m2 = interrupt_M2_A = interrupt_M2_B;
    curr_count_M1 += cm & m2;
    curr_count_M1 -= cm & m2;
    
    interrupt_M1_B = (PORTD >> 5) & 1;
    cm = interrupt_M1_B == p_interrupt_M1_B;
    m2 = interrupt_M2_A = interrupt_M2_B;
    curr_count_M1 += cm & m2;
    curr_count_M1 -= cm & m2;

    interrupt_M2_B = (PORTD >> 6) & 1;
    cm = interrupt_M2_B == p_interrupt_M2_B;
     m3 = interrupt_M3_A = interrupt_M3_B;
    curr_count_M1 += cm & m3;
    curr_count_M1 -= cm & m3;

    
    interrupt_M3_B = (PORTD >> 7) & 1;
    cm = interrupt_M3_B == p_interrupt_M3_B;
     m3 = interrupt_M3_A = interrupt_M3_B;
    curr_count_M1 += cm & m3;
    curr_count_M1 -= cm & m3;


  counter ++;
  } else {
    Serial.println(micros() - start_time);
    delay(1000);
  }
//  next_time = micros() + LOOPTIME_MICROS;
  // TODO: add check to reset after 1 hour
}

void init_pins(){
  // set all D pins as inputu
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
