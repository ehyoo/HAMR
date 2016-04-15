#include "decoder.h"
#include "Arduino.h"
#include "constants.h"




 int read_decoder(int index) {
   const int* d_pins;
   int sel_pin = DECODER_SEL_PIN;
   int oe_pin = DECODER_OE_PIN;

   switch (index) {
     case 1:
       d_pins = M1_DECODER_D_PINS;
       break;
     case 2:
       d_pins = M2_DECODER_D_PINS;
       break;
     case 3:
       d_pins = MT_DECODER_D_PINS;
       break;
     default:
       return 0;
   }
  
   int states[16];
   digitalWrite(oe_pin, LOW);  // set position inhibit
  
   // read high byte
   digitalWrite(sel_pin, LOW); 
   delayMicroseconds(10);
   for (int i = 0; i < 8; i++) {
     states[i + 8] = digitalRead(d_pins[i]);
   }
  
   // read low byte
   digitalWrite(sel_pin, HIGH);
   delayMicroseconds(10);
   for (int i = 0; i < 8; i++) {
     states[i] = digitalRead(d_pins[i]);
   }

   digitalWrite(oe_pin, HIGH); // reset inhibit logic


   int count = 0;
   for (int i = 15; i >= 0; i--) {
     count = (count << 1) | states[i];
   }

  for (int i = 0; i <= 15; i++){
    Serial.print(d_pins[i%8]);
    Serial.print(" ");
    Serial.print(states[i]);
    Serial.print("\n");
    
  }
  
   return count;
 }
