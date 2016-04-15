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
}

void loop() {
  while(1){
    PORTB ^= 1; 
//    delayMicroseconds(1); 
  }
}
