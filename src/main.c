#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "./rfm69.c"
#include "./uart.c"


/* main */

int main(void)
{
  uart_setup();
  rfm69_setup();

  /* put in rx continuous mode */
  rfm69_set_rx_continuous_mode();

  /* TODO: wait for rxready interrupt, to be wired */

  while (1)
  {
    uart_write(uint8_to_string(rfm69_read_reg(0x01)), 2);
    uart_write((uint8_t*)"\r\n", 2);

    uart_write(uint8_to_string(rfm69_read_reg(0x0e)), 2);
    uart_write((uint8_t*)"\r\n", 2);

    uart_write(uint8_to_string(rfm69_read_reg(0x10)), 2);
    uart_write((uint8_t*)"\r\n", 2);

    _delay_ms(200);
    _delay_ms(200);
    _delay_ms(200);
    _delay_ms(200);
    _delay_ms(200);
  }

  return 0;
}
