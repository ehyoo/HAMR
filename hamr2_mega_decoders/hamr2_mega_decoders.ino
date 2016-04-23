#include <Wire.h>

#define M1_DIR_PIN A5
#define M1_PWM_PIN 6

int decoder0_prev_count = 0;
int decoder1_prev_count = 0;
int decoder2_prev_count = 0;
int decoder0_curr_count = 0;
int decoder1_curr_count = 0;
int decoder2_curr_count = 0;

int decoder0_count0 = 0;
int decoder0_count1 = 0;
int decoder0_count2 = 0;
int decoder0_count3 = 0;
int decoder0_count4 = 0;



unsigned char data_buffer[6] = {0,0,0,0,0,0};
unsigned char test_data_buffer[4] = {1,1,1,1};

void setup() {
 init_decoders();
// init_I2C();

 enable_pwm();

 Serial.begin(250000);
 Serial.println("INITIALIZATION DONE");
}

void enable_pwm(){
//  OCR2A = 20;
//  OCR2B = 10;
//
//  DDRD |= (1 << 6);   //set pin D6 as an output
////  PORTD &= ~(1 << 6);   //set D6 low
//  DDRD |= (1 << 0);   //set pin D6 as an output
//  DDRD |= (1 << 3);   //set pin D6 as an output
//    
//  //TCCR0B &= ~(1 << 0);  // set prescaler to 64 (011), enable clock
//  TCCR2B |= (1 << 0);
//  
//  TCCR2B |= (1 << 3); // WGM02
//  TCCR2A |= (1 << 1); // WGM01
//  TCCR2A |= (1 << 0); // WGM00
//  
//  TCCR2A |= (1 << 7);    //set D6 to toggle
//  TCCR2A |= (1 << 6);

  pinMode(9, OUTPUT);
  
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS20);
  //TCCR2B |= (1 << 0); // prescaler to 1
  
  OCR2A = 20;
  OCR2B = 10;
}

void init_decoders(){
  // SEL, OE, RST, MUX SELECT
  DDRK |= 0b11111;
  DDRF = 0;          //set all decoder pins to input
  
  //Reset decoder
  PORTK &= (1<<2);
  delayMicroseconds(100);
  PORTK |= (1<<2);
}

unsigned int read_decoder(){
  unsigned char lowbyte;
  unsigned int highbyte;
  
  // set OE low and set SEL low to enable inhibit logic
  PORTK &= ~0b11;
  delayMicroseconds(10); //wait for edge
  
  highbyte = read_decoder_byte();
  
  //set select high
  PORTK |= 1;
  delayMicroseconds(10); // wait for edge
  
  lowbyte = read_decoder_byte();

  //set OE high
  PORTK |= (1<<1);

  // invert number (mux outputs inverted signals)
  return ~((highbyte << 8) | lowbyte);
}

unsigned char read_decoder_byte(){  
//  unsigned char count = PINB & 0b00111111; // get lower 6 bytes
//  count |= (PIND & 0b00110000) << 2;
 // count = PINF; 
  return PINF;
}

void toggle_quad() {
  PORTD ^= (1 << 2);
  delayMicroseconds(20);   
  PORTD ^= (1 << 3);   
  delayMicroseconds(20);
}

void select_decoder(int val){
  switch (val) {
    case 0: {
      // M1
      PORTK |= (1<<3);
      PORTK |= (1<<4); 
      break;
    }
    case 1: {
      // M2
      PORTK &= ~(1<<3);
      PORTK |= (1<<4); 
      break;
    }
    case 2: {
      // MT
      PORTK &= ~(1<<4); 
      break;
    }
  }
}

long lastMilli = 0;
int decoder0_average = 0;

void loop() {
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M1_PWM_PIN, OUTPUT);
  
  digitalWrite(M1_DIR_PIN, LOW);
  analogWrite(M1_PWM_PIN, 0);
  while(1){
    if (millis() - lastMilli > 20) {
      lastMilli = millis();
      
      select_decoder(0);
      decoder0_curr_count = read_decoder();
      decoder0_curr_count = ~(decoder0_curr_count);
      
//      decoder0_count1 = decoder0_count0;
//      decoder0_count2 = decoder0_count1;
//      decoder0_count3 = decoder0_count2;
//      decoder0_count4 = decoder0_count3;
//      decoder0_count0 = decoder0_curr_count;
//
//      decoder0_average = (decoder0_count0 + decoder0_count1 + decoder0_count2 +decoder0_count3 + decoder0_count4)/5.0;
//       decoder0_curr_count = decoder0_average;
      Serial.print("count0: ");
      Serial.print(decoder0_curr_count, BIN);
      Serial.print(" (");
      Serial.print(decoder0_curr_count);
      Serial.print(")");
      
      select_decoder(1);
      decoder1_curr_count = read_decoder();
      decoder1_curr_count = ~(decoder1_curr_count);
      Serial.print(" count1: ");
      Serial.print(decoder1_curr_count, BIN);
      Serial.print(" (");
      Serial.print(decoder1_curr_count);
      Serial.print(")");
      
      select_decoder(2);
      decoder2_curr_count = read_decoder();
      Serial.print(" count2: ");
      Serial.print(decoder2_curr_count, BIN);
      Serial.print(" (");
      Serial.print(decoder2_curr_count);
      Serial.print(")");
      Serial.println();

      decoder0_prev_count = decoder0_curr_count;
      decoder1_prev_count = decoder1_curr_count;
      decoder2_prev_count = decoder2_curr_count;

  
  //    data_buffer[0] = (unsigned char) (decoder0_count >> 8);
  //    data_buffer[1] = (unsigned char) (decoder0_count);
  //    data_buffer[2] = (unsigned char) (decoder1_count >> 8);
  //    data_buffer[3] = (unsigned char) (decoder1_count);
  //    data_buffer[4] = (unsigned char) (decoder2_count >> 8);
  //    data_buffer[5] = (unsigned char) (decoder2_count);
    }
  }
}

// I2C stuff
void init_I2C(){
  Wire.begin(99);
  Wire.onRequest(on_I2C_request);
}

void on_I2C_request(){
  Wire.write(test_data_buffer, 4);
}

