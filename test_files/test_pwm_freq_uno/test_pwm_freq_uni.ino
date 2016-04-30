void setup() {
  // two line method
  DDRB |= 1;    //set pin B0 as an output

  // PWM method:
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


  // for testing
  DDRB |= (1 << 1);    //set pin B1 as an output
  DDRB |= (1 << 2);    //set pin B2 as an output

  PORTB &= ~(1 << 1);  // initialize to low
  PORTB |= (1 << 2);   // initialize to high

// uncomment below to switch to other direction
//  PORTB |= (1 << 1);  // initialize to high
//  PORTB &= ~(1 << 2); // initialize to low  
}

void loop() {
  while(1){
//    PORTB ^= 1; // uncomment this for a 1.6 MHz wave
    test_decoders2(); 
//    delayMicroseconds(1); 
  }
}


void test_decoders2(){
  // modify frequency of wave by changing delay
  PORTB ^= (1 << 1);
  delayMicroseconds(2);
  PORTB ^= (1 << 2);
  delayMicroseconds(2);
}

