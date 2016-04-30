unsigned int decoder0_count = 0;
unsigned int decoder1_count = 0;

#include <Wire.h>

// store data to send over I2C to due
unsigned char data_buff[4] = {0, 0, 0, 0};

unsigned char test_data_buff[4] = {1, 2, 3, 4};


void setup() {
 init_I2C();
 enable_pwm(); // for decoder clock

 while(1);
 init_decoders();
 init_quad(); 
 
 Serial.begin(250000);
 Serial.println("INITIALIZATION DONE");
}


void loop() {

  while(1){
    for(int i = 0; i < 50; i++){
      toggle_quad();
    }
    
    select_decoder(0);
    decoder0_count = read_decoder();
    select_decoder(1);
    decoder1_count = read_decoder();
    
//    if(decoder0_count > 0 || decoder0_count > 0){
      Serial.print("count0: ");
      Serial.print(decoder0_count);
      Serial.print(" count1: ");
      Serial.println(decoder1_count);
//    }
    data_buff[0] = (unsigned char) (decoder0_count >> 8);
    data_buff[1] = (unsigned char) decoder0_count;

    data_buff[2] = (unsigned char) (decoder1_count >> 8);
    data_buff[3] = (unsigned char) decoder1_count;

    delayMicroseconds(10000);
  }
}

/* enable a PWM signal on pin D6 at 101 KHz*/
void enable_pwm(){
//  OCR0A = 20;
//  OCR0B = 10;

  // Part 3 - Enable CTC mode to output wave with frequency based on ADC input
  DDRD |= (1 << 6);   //set pin D6 as an output
  PORTD &= ~(1 << 6);   //set D6 low
  
  
  TCCR0B &= ~(1 << 0);  // set prescaler to 64 (011), enable clock
  TCCR0B |= (1 << 0);
  TCCR0B |= (1 << 0); 

  

  TCCR0B &= ~(1 << 3);    //enable CTC mode
  TCCR0A |= (1 << 1);
  TCCR0A &= ~(1 << 0);

  
  TCCR0A &= ~(1 << 7);    //set D6 to toggle
  TCCR0A |= (1 << 6);

}

/* initialize the quadrature output on pins D2 and D3*/
void init_quad() {
  DDRD |= 1 << 2;
  DDRD |= 1 << 3;
}

/* initialize decoders signals. reset the decoder */
void init_decoders(){
  // enable outputs for SEL, OE, RST, MUX SELECT
  DDRC |= 0b1111;
  DDRB = 0;

  //Reset decoder
  PORTC &= ~(1<<2);
  delayMicroseconds(1000); // did not work well at 100 us
  PORTC |= (1<<2);
}


/* We should be able to lower the delay here by increasing clock speed */
/* read the decoder */
unsigned int read_decoder(){
  unsigned char lowbyte;
  unsigned int highbyte;
  
  // set OE low and set SEL low to enable inhibit logic
  PORTC &= ~0b11;
  delayMicroseconds(10); //wait for edge
  
  highbyte = read_decoder_byte();
  
  //set select high
  PORTC |= 1;
  delayMicroseconds(10); // wait for edge
  
  lowbyte = read_decoder_byte();

  //set OE high
  PORTC |= (1<<1);

  // invert number (mux outputs inverted signals)
  return ~((highbyte << 8) | lowbyte);
}

/* read a bye from the designated decoder pins */
unsigned char read_decoder_byte(){  
  unsigned char count = PINB & 0b00111111; // get lower 6 bits
  count |= (PIND & 0b00110000) << 2; // upper 2 bits
  return count;
}

/*toggle the quadrature output */
void toggle_quad() {
  PORTD ^= (1 << 2);
  delayMicroseconds(50);   
  PORTD ^= (1 << 3);   
  delayMicroseconds(50);
}

/* select a decoder */
void select_decoder(int val){
  if(val){
    PORTC |= (1<<3);  // select decoder 1
  } 
  else {
    PORTC &= ~(1<<3); // select decoder 0
  }
}


/* initialize as I2C slave at address 99 */
void init_I2C(){
 Wire.begin(99);
 Wire.onRequest(on_I2C_request);
}

/* send data to the requester */
void on_I2C_request(){
 Wire.write(test_data_buff, 4);
// Wire.write(0b01100110);
// Wire.write(0);
// Wire.write(7);
// Wire.write(8);
}


