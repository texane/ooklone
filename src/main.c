#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "./rfm69.c"
#include "./uart.c"


__attribute__((unused)) static uint8_t get_rssi_avg(void)
{
  /* note: actual_rssi = - rfm69_get_rssi / 2 */
  uint16_t i;
  uint32_t sum = 0;
  for (i = 0; i != 1000; ++i) sum += rfm69_get_rssi();
  return (uint8_t)(sum / (uint32_t)i);
}


/* main */

int main(void)
{
  uint8_t pre_state = 0xff;
  uint8_t cur_state;
  uint16_t i = 0;
  uint32_t f;

  uart_setup();
  rfm69_setup();

  sei();

  /* put in rx continuous mode */
  rfm69_set_rx_continuous_mode();

  uart_write((uint8_t*)"rx\r\n", 4);

  while (1)
  {
    cur_state = rfm69_get_data();
    if (cur_state != pre_state)
    {
      if ((i & 0xf) == 0)
      {
	uart_write((uint8_t*)"\r\n", 2);
	uart_write(uint16_to_string(i), 4);
	uart_write((uint8_t*)": ", 2);
      }
      ++i;

      if (cur_state) uart_write((uint8_t*)"1", 1);
      else uart_write((uint8_t*)"0", 1);
      pre_state = cur_state;
    }
  }

  return 0;
}
