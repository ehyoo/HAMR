unsigned int decoder0_count = 0;
unsigned int decoder1_count = 0;

void setup() {
 init_quad();
 init_decoders();

 enable_pwm();
 
 Serial.begin(250000);
 Serial.println("INITIALIZATION DONE");
}

void enable_pwm(){
  OCR0A = 20;
  OCR0B = 10;

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

void init_quad() {
  DDRD |= 1 << 2;
  DDRD |= 1 << 3;
}

void init_decoders(){
  // SEL, OE, RST, MUX SELECT
  DDRC |= 0b1111;
  DDRB = 0;

  //Reset decoder
  PORTC &= (1<<2);
  delayMicroseconds(100);
  PORTC |= (1<<2);
}

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

unsigned char read_decoder_byte(){  
  unsigned char count = PINB & 0b00111111; // get lower 6 bytes
  count |= (PIND & 0b00110000) << 2;   
  return count;
}

void toggle_quad() {
  PORTD ^= (1 << 2);
  delayMicroseconds(20);   
  PORTD ^= (1 << 3);   
  delayMicroseconds(20);
}

void select_decoder(int val){
  if(val){
    PORTC |= (1<<3);  // select decoder 1
  } 
  else {
    PORTC &= ~(1<<3); // select decoder 0
  }
}

void loop() {
  while(1){
    for(int i = 0; i < 10; i++){
      toggle_quad();
    }
    
    select_decoder(0);
    decoder0_count = read_decoder();
    Serial.print("count0: ");
    Serial.println(decoder0_count,BIN);
    
    select_decoder(1);
    decoder1_count = read_decoder();


    Serial.print("count1: ");
    Serial.println(decoder1_count,BIN);

    delayMicroseconds(100000);
  }
}

// I2C stuff
void init_I2C(){
//  Wire.begin(99);
//  Wire.onRequest(on_I2C_request);
}

void on_I2C_request(){
//  Wire.write(decoder1_count);
//  Wire.write(decoder2_count);
}


