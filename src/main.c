#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "./rfm69.c"

#define CONFIG_UART
#ifdef CONFIG_UART
#include "./uart.c"
#endif /* CONFIG_UART */


__attribute__((unused)) static uint8_t get_rssi_avg(void)
{
  /* note: actual_rssi = - rfm69_get_rssi / 2 */
  uint16_t i;
  uint32_t sum = 0;
  for (i = 0; i != 1000; ++i) sum += rfm69_get_rssi();
  return (uint8_t)(sum / (uint32_t)i);
}


/* sniffer and pulse slicer logic */

#define PULSE_MAX_COUNT 1024
static uint8_t pulse_timer[PULSE_MAX_COUNT];
static volatile uint16_t pulse_index;
static volatile uint16_t pulse_count;

#define PULSE_FLAG_DONE (1 << 0)
#define PULSE_FLAG_OVF (1 << 1)
static volatile uint8_t pulse_flags;

/* pulse timer resolution is 4 or 16 us */
/* note that the counters are stored using 8 bits */
/* values, which equals 1024 or 4096 us and should */
/* be considered when setting pulse_max_timer. */
#define PULSE_TIMER_RES_US 16

#define pulse_us_to_timer(__us) (1 + (__us) / PULSE_TIMER_RES_US)

static inline uint16_t pulse_timer_to_us(uint8_t x)
{
  return ((uint16_t)x) * PULSE_TIMER_RES_US;
}

/* max is 1024 us with 8 bits counter and 4 us resolution */
/* max is 4096 us with 8 bits counter and 16 us resolution */
static const uint16_t pulse_max_timer = pulse_us_to_timer(3000);

static inline void pulse_common_vect(uint8_t flags)
{
  TCCR1B = 0;
  pulse_flags |= flags;
}

/* #define CONFIG_PCINT_ISR */
#ifdef CONFIG_PCINT_ISR
ISR(PCINT2_vect)
#else
static void pcint2_vect(void)
#endif /* CONFIG_PCINT_ISR */
{
  /* capture counter */
  uint16_t n = TCNT1;

  if (pulse_count == PULSE_MAX_COUNT)
  {
    pulse_common_vect(PULSE_FLAG_OVF | PULSE_FLAG_DONE);
    return ;
  }

  /* restart the timer, ctc mode. */
  TCNT1 = 0;
#if (PULSE_TIMER_RES_US == 4)
  TCCR1B = (1 << 3) | (3 << 0);
#elif (PULSE_TIMER_RES_US == 16)
  TCCR1B = (1 << 3) | (4 << 0);
#endif

  /* store counter */
  if (n > pulse_max_timer) n = pulse_max_timer;
  pulse_timer[pulse_count++] = (uint8_t)n;
}

ISR(TIMER1_OVF_vect)
{
  pulse_common_vect(PULSE_FLAG_DONE);
}

ISR(TIMER1_COMPA_vect)
{
  pulse_common_vect(PULSE_FLAG_DONE);
}

ISR(TIMER1_COMPB_vect)
{
  if (pulse_index == pulse_count)
  {
    pulse_common_vect(PULSE_FLAG_DONE);
    return ;
  }

  /* play pulse_index and increment */
  if ((pulse_index & 1)) rfm69_set_data_high();
  else rfm69_set_data_low();
  TCNT1 = 0;
  OCR1B = pulse_timer[pulse_index++];
}

#ifdef CONFIG_UART
static void uart_write_rn(void)
{
  uart_write((uint8_t*)"\r\n", 2);
}
#endif /* CONFIG_UART */

static inline uint8_t filter_data(void)
{
  /* glitch filtering: consider a one only if no */
  /* zero appears within a given pulse count */

  uint8_t i;
  uint8_t x = rfm69_get_data();
  for (i = 0; i != 32; ++i) x &= rfm69_get_data();
  return x;
}

static void do_listen(void)
{
  uint8_t pre_state;
  uint8_t cur_state;

  /* prepare the timer. first read in pcint isr. */
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1A = 0;
  TCCR1C = 0;
  OCR1A = pulse_max_timer;
  TIMSK1 = (1 << 1) | (1 << 0);

  /* reset pulse slicer context */
  pulse_count = 0;
  pulse_flags = 0;

  /* put in rx continuous mode */
  rfm69_set_rx_continuous_mode();

  /* setup dio2 so the first bit start the timer */
#ifdef CONFIG_PCINT_ISR
  PCICR |= RFM69_IO_DIO2_PCICR_MASK;
  RFM69_IO_DIO2_PCMSK |= RFM69_IO_DIO2_MASK;
#endif /* CONFIG_PCINT_ISR */

  pre_state = 0;

  /* wait until done */
  while ((pulse_flags & PULSE_FLAG_DONE) == 0)
  {
#ifdef CONFIG_PCINT_ISR

    /* TODO: sleep */

#else

    cur_state = filter_data();

    if (cur_state == pre_state) continue ;

    pre_state = cur_state;

    pcint2_vect();

#endif /* CONFIG_PCINT_ISR */
  }

  /* stop timer */
  TCCR1B = 0;

#ifdef CONFIG_PCINT_ISR
  /* disable dio2 pcint interrupt */
  RFM69_IO_DIO2_PCMSK &= ~RFM69_IO_DIO2_MASK;
  PCICR &= ~RFM69_IO_DIO2_PCICR_MASK;
#endif /* CONFIG_PCINT_ISR */

  /* put back in standby mode */
  rfm69_set_standby_mode();
}

#ifdef CONFIG_UART
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
#endif /* CONFIG_UART */

static void do_replay(void)
{
  /* replay the currently stored pulses */

  pulse_flags = 0;
  pulse_index = 1;

  /* put in tx continuous mode */
  rfm69_set_tx_continuous_mode();

  rfm69_set_data_low();

  /* restart the timer, ctc mode, 16us resolution. */
  /* ocr1b used for top, no max value */
  /* top value is 0x100 or 4.08 ms. */
  TCCR1A = 0;
  TCNT1 = 0;
  TCCR1C = 0;
  OCR1B = 0xff;
  TIMSK1 = 1 << 2;
#if (PULSE_TIMER_RES_US == 4)
  TCCR1B = 3 << 0;
#elif (PULSE_TIMER_RES_US == 16)
  TCCR1B = 4 << 0;
#endif

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


/* buttons */

#define BUT_COMMON_DDR DDRC
#define BUT_COMMON_PORT PORTC
#define BUT_COMMON_PIN PINC
#define BUT_PLAY_MASK (1 << 0)
#define BUT_RECORD_MASK (1 << 1)
#define BUT_ALL_MASK (BUT_RECORD_MASK | BUT_PLAY_MASK)
#define BUT_COMMON_PCICR_MASK (1 << 1)
#define BUT_COMMON_PCMSK PCMSK1

static volatile uint8_t but_pcint_pin;

ISR(PCINT1_vect)
{
  /* capture pin values */
  but_pcint_pin = BUT_COMMON_PIN;
}

static void but_setup(void)
{
  /* set as input, enable pullups */
  BUT_COMMON_DDR &= ~BUT_ALL_MASK;
  BUT_COMMON_PORT |= BUT_ALL_MASK;

  but_pcint_pin = BUT_ALL_MASK;
}

static uint8_t but_wait(void)
{
  uint8_t x = BUT_ALL_MASK;

  /* sleep mode */
  set_sleep_mode(SLEEP_MODE_IDLE);

  /* enable pin change interrupt */
  PCICR |= BUT_COMMON_PCICR_MASK;
  BUT_COMMON_PCMSK |= BUT_ALL_MASK;

  while (x == BUT_ALL_MASK)
  {
    /* capture but_pcint_pin with interrupts disabled */
    /* if no pin set, then sleep until pcint */
    /* take care of the inverted logic due to pullups */

    cli();

    if ((but_pcint_pin & BUT_ALL_MASK) == BUT_ALL_MASK)
    {
      sleep_enable();
      sei();
      sleep_cpu();
      sleep_bod_disable();
    }

    x = but_pcint_pin;

    sei();

    /* simple debouncing logic. 1 wins over 0. */
    x |= BUT_COMMON_PIN;
    _delay_us(1);
    x |= BUT_COMMON_PIN;
    _delay_us(1);
    x |= BUT_COMMON_PIN;
    _delay_us(1);
    x |= BUT_COMMON_PIN;

    /* conserve only pins of interest */
    x &= BUT_ALL_MASK;
  }

  /* disable pin change interrupt */
  BUT_COMMON_PCMSK &= ~BUT_ALL_MASK;
  PCICR &= ~BUT_COMMON_PCICR_MASK;

  /* inverted logic because of pullups */
  return x ^ BUT_ALL_MASK;
}


/* main */

int main(void)
{
  uint8_t x;
  uint8_t i;

#ifdef CONFIG_UART
  uart_setup();
#endif /* CONFIG_UART */

  rfm69_setup();

  but_setup();

  sei();

  while (1)
  {
    x = but_wait();

    if (x & BUT_RECORD_MASK)
    {
#ifdef CONFIG_UART
      uart_write((uint8_t*)"record", 6);
      uart_write_rn();
#endif /* CONFIG_UART */

      do_listen();

#ifdef CONFIG_UART
      do_print();
#endif /* CONFIG_UART */
    }

    if (x & BUT_PLAY_MASK)
    {
#ifdef CONFIG_UART
      uart_write((uint8_t*)"play", 4);
      uart_write_rn();
#endif /* CONFIG_UART */

      for (i = 0; i != 7; ++i)
      {
	_delay_us(500);
	do_replay();
      }
    }
  }

  return 0;
}
