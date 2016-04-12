// #include "decoder.h"

// int read_decoder(int index) {
//   int* d_pins;
//   int oe_pin = 0;
//   int sel_pin = 0;

//   switch (index) {
//     case 1:
//       sel_pin = M1_DECODER_SEL_PIN;
//       oe_pin = M1_DECODER_OE_PIN;
//       *d_pins = M1_DECODER_D_PINS;
//       break;
//     case 2:
//       sel_pin = M2_DECODER_SEL_PIN;
//       oe_pin = M2_DECODER_OE_PIN;
//       *d_pins = M2_DECODER_D_PINS;
//       break;
//     case 3:
//       sel_pin = MT_DECODER_SEL_PIN;
//       oe_pin = MT_DECODER_OE_PIN;
//       *d_pins = MT_DECODER_D_PINS;
//       break;
//     default:
//       return 0;
//       break;
//   }
  
//   int states[16];
//   digitalWrite(oe_pin, LOW);  // set position inhibit
  
//   // read high byte
//   digitalWrite(sel_pin, LOW); 
//   for (int i = 0; i < 8; i++) {
//     states[i + 8] = digitalRead(d_pins[i]);
//   }
  
//   // read low byte
//   digitalWrite(sel_pin, HIGH);
//   for (int i = 0; i < 8; i++) {
//     states[i] = digitalRead(d_pins[i]);
//   }

//   digitalWrite(oe_pin, HIGH); // reset inhibit logic


//   int count = 0;
//   for (int i = 15; i >= 0; i--) {
//     Serial.println(states[i]);
//     count = (count << 1) | states[i];
//   }
//   return count;
// }
