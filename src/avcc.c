#ifndef AVCC_C_INCLUDED
#define AVCC_C_INCLUDED


#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>


static uint16_t avcc_get(void)
{
  /* compute avcc using 1.1V reference and ADC mux */
  /* http://code.google.com/p/tinkerit/wiki/SecretVoltmeter */

  /* WARNING: it overwrite adc related registers, which */
  /* WARNING: could be an issue for sel_xxx routines */

  uint8_t i;
  uint16_t x;
  uint16_t sum;

  /* read 1.1V reference against AVcc */
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

  ADCSRA |= _BV(ADEN);

  _delay_ms(5);

  /* convert */
  sum = 0;
  for (i = 0; i != 8; ++i)
  {
    ADCSRA |= _BV(ADSC); 
    while (bit_is_set(ADCSRA, ADSC)) ;
    x = ADCL;
    __asm__ __volatile__ ("nop");
    x |= ((uint16_t)ADCH) << 8;
    x &= 0x3ff;
    sum += x;
  }

  ADCSRA &= ~_BV(ADEN);

  /* back calculate AVcc in mV */
  return (8UL * 1126400UL) / (uint32_t)sum;
}


#endif /* AVCC_C_INCLUDED */
