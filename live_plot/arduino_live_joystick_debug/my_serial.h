#include <Serial.h>

/*
send a byte over usb
*/
void usb_tx_char(char i);

/*
send an int over usb
*/
void usb_tx_int16(int i);

/*
send an unsigned int over usb (same as usb_tx_int16())
*/
void usb_tx_uint16(uint16 i);


/*
send a float
*/
void usb_tx_f32(uint16 i);