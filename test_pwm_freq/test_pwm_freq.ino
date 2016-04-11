// http://www.kerrywong.com/2014/09/21/on-arduino-due-pwm-frequency/

/* modify these values to modify the wave output*/
uint32_t maxCount = 1000;
uint32_t dutyCycleCount = maxCount / 2;
/**/

void setup() {
  // only works on one pin at a time and only on pins 7-10
  // currently set to output square wave 50% duty cycle at 169 kHz
  enablePWM(9, maxCount, dutyCycleCount);
}
 
void loop() 
{

}

void enablePWM(uint32_t pwmPin, uint32_t maxCount, uint32_t dutyCycleCount){
  uint32_t clkAFreq = 42000000ul;
  uint32_t pwmFreq = 42000000ul; 
  
  pmc_enable_periph_clk(PWM_INTERFACE_ID);
  PWMC_ConfigureClocks(clkAFreq, 0, VARIANT_MCK);
 
  PIO_Configure(
    g_APinDescription[pwmPin].pPort,
    g_APinDescription[pwmPin].ulPinType,
    g_APinDescription[pwmPin].ulPin,
    g_APinDescription[pwmPin].ulPinConfiguration);
 
  uint32_t channel = g_APinDescription[pwmPin].ulPWMChannel;
  PWMC_ConfigureChannel(PWM_INTERFACE, channel , pwmFreq, 0, 0);
  PWMC_SetPeriod(PWM_INTERFACE, channel, maxCount);
  PWMC_EnableChannel(PWM_INTERFACE, channel);
  PWMC_SetDutyCycle(PWM_INTERFACE, channel, dutyCycleCount);
 
  pmc_mck_set_prescaler(2);
}
