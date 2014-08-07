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


/* sniffer and pulse slicer logic */

#define PULSE_MAX_COUNT 512
static uint8_t pulse_timer[PULSE_MAX_COUNT];
static volatile uint16_t pulse_index;
static volatile uint16_t pulse_count;

#define PULSE_FLAG_DONE (1 << 0)
#define PULSE_FLAG_OVF (1 << 1)
static volatile uint8_t pulse_flags;

ISR(PCINT2_vect)
{
  /* capture counter */
  uint16_t n = TCNT1;

  if (pulse_count == PULSE_MAX_COUNT)
  {
    pulse_flags |= (PULSE_FLAG_OVF | PULSE_FLAG_DONE);
    return ;
  }

  /* restart the timer, ctc mode, 16us resolution. */
  /* top value is 0x100 or 4.08 ms. */
  TCCR1A = 0;
  TCNT1 = 0;
  TCCR1C = 0;
  OCR1A = 0x100;
  TIMSK1 = (1 << 1) | (1 << 0);
  TCCR1B = (1 << 3) | (4 << 0);

  /* store counter */
  if (n > 0xff) n = 0xff;
  pulse_timer[pulse_count++] = (uint8_t)n;
}

static inline void timer1_common_vect(void)
{
  pulse_flags |= PULSE_FLAG_DONE;
}

ISR(TIMER1_OVF_vect)
{
  timer1_common_vect();
}

ISR(TIMER1_COMPA_vect)
{
  timer1_common_vect();
}

ISR(TIMER1_COMPB_vect)
{
  if (pulse_index == pulse_count)
  {
    pulse_flags |= PULSE_FLAG_DONE;
    return ;
  }

  /* play pulse_index and increment */
  if ((pulse_index & 1)) rfm69_set_data_high();
  else rfm69_set_data_low();
  TCNT1 = 0;
  OCR1B = pulse_timer[pulse_index++];
}

static uint16_t pulse_timer_to_us(uint8_t x)
{
  return ((uint16_t)x) * 16;
}

static uint8_t pulse_us_to_timer(uint16_t us)
{
  return (uint8_t)(us / 16);
}

static void uart_write_rn(void)
{
  uart_write((uint8_t*)"\r\n", 2);
}

static void do_listen(void)
{
  /* reset timer for first read in pcint isr */
  TCCR1B = 0;
  TCNT1 = 0;

  /* reset pulse slicer context */
  pulse_count = 0;
  pulse_flags = 0;

  /* put in rx continuous mode */
  rfm69_set_rx_continuous_mode();

  uart_write((uint8_t*)"rx", 2);
  uart_write_rn();

  /* setup dio2 so the first bit start the timer */
  PCICR |= RFM69_IO_DIO2_PCICR_MASK;
  RFM69_IO_DIO2_PCMSK |= RFM69_IO_DIO2_MASK;

  /* wait until done */
  while ((pulse_flags & PULSE_FLAG_DONE) == 0)
  {
    /* TODO: sleep */
  }

  /* stop timer */
  TCCR1B = 0;

  /* disable dio2 pcint interrupt */
  RFM69_IO_DIO2_PCMSK &= ~RFM69_IO_DIO2_MASK;
  PCICR &= ~RFM69_IO_DIO2_PCICR_MASK;

  /* put back in standby mode */
  rfm69_set_standby_mode();
}

static void do_fill(void)
{
  /* fill with a known pulse serie */

  uint16_t i;

#if 0
  for (i = 0; i != 11; ++i)
  {
    pulse_timer[i] = pulse_us_to_timer(4 * i * 100);
  }
#else

  pulse_timer[0] = 0;

  i = 1;

#define castoplug_send_pulse_a()		\
do {						\
  pulse_timer[i++] = pulse_us_to_timer(400);	\
  pulse_timer[i++] = pulse_us_to_timer(940);	\
} while (0)

#define castoplug_send_pulse_b()		\
do {						\
  pulse_timer[i++] = pulse_us_to_timer(1005);	\
  pulse_timer[i++] = pulse_us_to_timer(340);	\
} while (0)

  /* group a */
  castoplug_send_pulse_a();
  castoplug_send_pulse_a();
  castoplug_send_pulse_a();
  castoplug_send_pulse_b();
  castoplug_send_pulse_a();
  castoplug_send_pulse_b();
  castoplug_send_pulse_a();
  castoplug_send_pulse_b();

  /* dev 1 */
  castoplug_send_pulse_a();
  castoplug_send_pulse_a();
  castoplug_send_pulse_a();
  castoplug_send_pulse_b();
  castoplug_send_pulse_a();
  castoplug_send_pulse_b();
  castoplug_send_pulse_a();
  castoplug_send_pulse_b();
  castoplug_send_pulse_a();

  /* cmd off */
  castoplug_send_pulse_b();
  castoplug_send_pulse_a();
  castoplug_send_pulse_b();
  castoplug_send_pulse_a();
  castoplug_send_pulse_b();
  castoplug_send_pulse_a();
  castoplug_send_pulse_a();
  castoplug_send_pulse_a();

#endif

  pulse_count = i;
  pulse_flags = 0;
}

static void do_print(void)
{
  uint16_t i;

  uart_write((uint8_t*)"flags: ", 7);
  uart_write(uint8_to_string(pulse_flags), 2);
  uart_write_rn();

  for (i = 0; i != pulse_count; ++i)
  {
    const uint16_t us = pulse_timer_to_us(pulse_timer[i]);

    if ((i & 0x7) == 0)
    {
      uart_write_rn();
      uart_write(uint16_to_string(i), 4);
    }

    uart_write((uint8_t*)" ", 1);
    uart_write(uint16_to_string(us), 4);
  }

  uart_write_rn();
}

static void do_replay(void)
{
  /* replay the currently stored pulses */

  pulse_flags = 0;
  pulse_index = 1;

  /* put in tx continuous mode */
  rfm69_set_tx_continuous_mode();

  uart_write((uint8_t*)"tx", 2);
  uart_write_rn();

  rfm69_set_data_low();

  /* restart the timer, ctc mode, 16us resolution. */
  /* ocr1b used for top, no max value */
  /* top value is 0x100 or 4.08 ms. */
  TCCR1A = 0;
  TCNT1 = 0;
  TCCR1C = 0;
  OCR1B = 0xff;
  TIMSK1 = 1 << 2;
  TCCR1B = (1 << 3) | (4 << 0);

  while ((pulse_flags & PULSE_FLAG_DONE) == 0)
  {
    /* TODO: sleep */
  }

  rfm69_set_data_low();

  /* disable counter */
  TCCR1B = 0;

  /* put back in standby mode */
  rfm69_set_standby_mode();
}


/* main */

int main(void)
{
  uint8_t x;

  uart_setup();
  rfm69_setup();

  uart_write((uint8_t*)"go", 2);
  uart_write_rn();

  sei();

  while (1)
  {
    uart_read_uint8(&x);

    if (x == 'l')
    {
      do_listen();
      do_print();
    }
    else if (x == 'r')
    {
      do_replay();
    }
    else
    {
      do_fill();
      do_print();
    }
  }

  return 0;
}
