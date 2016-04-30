// http://www.kerrywong.com/2014/09/21/on-arduino-due-pwm-frequency/

/* modify these values to modify the wave output*/
uint32_t maxCount = 1000;
uint32_t dutyCycleCount = maxCount / 2;
/**/

void setup() {
  // only works on one pin at a time and only on pins 7-10
  // currently set to output square wave 50% duty cycle at 169 kHz
  Serial.begin(250000);
  // pwm_enable();
  // pwm_start(9, maxCount, dutyCycleCount);
  pwm_enable0(9, maxCount, dutyCycleCount);
}
 
void loop() 
{
  Serial.println("running");
}

// void pwm_enable0(uint32_t pwmPin, uint32_t maxCount, uint32_t dutyCycleCount){
//   uint32_t clkAFreq = 42000000ul;
//   uint32_t pwmFreq = 42000000ul; 
  
//   pmc_enable_periph_clk(PWM_INTERFACE_ID);
//   PWMC_ConfigureClocks(clkAFreq, 0, VARIANT_MCK);
 
//   PIO_Configure(
//     g_APinDescription[pwmPin].pPort,
//     g_APinDescription[pwmPin].ulPinType,
//     g_APinDescription[pwmPin].ulPin,
//     g_APinDescription[pwmPin].ulPinConfiguration);
 
//   uint32_t channel = g_APinDescription[pwmPin].ulPWMChannel;
//   PWMC_ConfigureChannel(PWM_INTERFACE, channel , pwmFreq, 0, 0);
//   PWMC_SetPeriod(PWM_INTERFACE, channel, maxCount);
//   PWMC_EnableChannel(PWM_INTERFACE, channel);
//   PWMC_SetDutyCycle(PWM_INTERFACE, channel, dutyCycleCount);
 
//   pmc_mck_set_prescaler(2);
// }

void pwm_enable(){
  Serial.println("test1"); 
  pmc_enable_periph_clk(PWM_INTERFACE_ID);
  Serial.println("test2");  
  PWMC_ConfigureClocks(PWM_FREQUENCY * dutyCycleCount, 0, VARIANT_MCK);
  Serial.println("test3");
}


void pwm_start(uint32_t ulPin, uint32_t maxCount, uint32_t dutyCycleCount){
  // maxCount = mapResolution(maxCount, _writeResolution, PWM_RESOLUTION);
    uint32_t chan = g_APinDescription[ulPin].ulPWMChannel;
    if ((g_pinStatus[ulPin] & 0xF) != PIN_STATUS_PWM) {
      // Setup PWM for this pin
      PIO_Configure(g_APinDescription[ulPin].pPort,
          g_APinDescription[ulPin].ulPinType,
          g_APinDescription[ulPin].ulPin,
          g_APinDescription[ulPin].ulPinConfiguration);
      PWMC_ConfigureChannel(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKA, 0, 0);
      PWMC_SetPeriod(PWM_INTERFACE, chan, dutyCycleCount);
      PWMC_SetDutyCycle(PWM_INTERFACE, chan, maxCount);
      PWMC_EnableChannel(PWM_INTERFACE, chan);
      g_pinStatus[ulPin] = (g_pinStatus[ulPin] & 0xF0) | PIN_STATUS_PWM;
    }

    PWMC_SetDutyCycle(PWM_INTERFACE, chan, maxCount);
}
