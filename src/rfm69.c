#ifndef RFM69_C_INCLUDED
#define RFM69_C_INCLUDED


#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "./spi.c"


/* spi.csn is portb2 */
/* spi.sck is portb5 */
/* spi.mosi is portb3 */
/* spi.miso is portb4 */

#define RFM69_IO_CSN_DDR DDRB
#define RFM69_IO_CSN_PORT PORTB
#define RFM69_IO_CSN_MASK (1 << 2)

#define RFM69_IO_DIO0_DDR DDRD
#define RFM69_IO_DIO0_PIN PIND
#define RFM69_IO_DIO0_MASK (1 << 2)

/* data */
#define RFM69_IO_DIO2_DDR DDRD
#define RFM69_IO_DIO2_PIN PIND
#define RFM69_IO_DIO2_PORT PORTD
#define RFM69_IO_DIO2_MASK (1 << 3)
#define RFM69_IO_DIO2_PCICR_MASK (1 << 2)
#define RFM69_IO_DIO2_PCMSK PCMSK2

/* dclk */
#define RFM69_IO_DIO1_DDR DDRD
#define RFM69_IO_DIO1_PIN PIND
#define RFM69_IO_DIO1_PORT PORTD
#define RFM69_IO_DIO1_MASK (1 << 4)

static inline void rfm69_csn_setup(void)
{
  RFM69_IO_CSN_DDR |= RFM69_IO_CSN_MASK;
}

static inline void rfm69_csn_low(void)
{
  RFM69_IO_CSN_PORT &= ~RFM69_IO_CSN_MASK;
}

static inline void rfm69_csn_high(void)
{
  RFM69_IO_CSN_PORT |= RFM69_IO_CSN_MASK;
}

static inline void rfm69_csn_wait(void)
{
  _delay_us(1);
}

static uint8_t rfm69_read_reg(uint8_t i)
{
  /* i the register address */

  uint8_t x;

  rfm69_csn_low();
  rfm69_csn_wait();
  spi_write_uint8(i);
  x = spi_read_uint8();
  rfm69_csn_high();

  return x;
}

static void rfm69_write_reg(uint8_t i, uint8_t x)
{
  /* i the register address */
  /* x the register value */

  rfm69_csn_low();
  rfm69_csn_wait();
  spi_write_uint8((1 << 7) | i);
  spi_write_uint8(x);
  rfm69_csn_high();
}

static void rfm69_read_burst(uint8_t i, uint8_t* x, uint8_t n)
{
  /* i the register address */

  rfm69_csn_low();
  rfm69_csn_wait();
  spi_write_uint8(i);
  for (; n; --n, ++x) *x = spi_read_uint8();
  rfm69_csn_high();
}

static void rfm69_write_burst(uint8_t i, const uint8_t* x, uint8_t n)
{
  /* i the register address */

  rfm69_csn_low();
  rfm69_csn_wait();
  spi_write_uint8((1 << 7) | i);
  for (; n; --n, ++x) spi_write_uint8(*x);
  rfm69_csn_high();
}

static inline void rfm69_write_op_mode(uint8_t x)
{
  rfm69_write_reg(0x01, x);
}

static inline uint8_t rfm69_read_op_mode(void)
{
  return rfm69_read_reg(0x01);
}

static inline void rfm69_write_data_modul(uint8_t x)
{
  rfm69_write_reg(0x02, x);
}

static inline void rfm69_write_bitrate(uint16_t x)
{
  rfm69_write_reg(0x03, (x >> 8) & 0xff);
  rfm69_write_reg(0x04, (x >> 0) & 0xff);
}

static inline void rfm69_write_frf(uint32_t x)
{
  /* carrier frequency */
  /* note: frf lsb must be written last (note p17) */

  rfm69_write_reg(0x07, (x >> 16) & 0xff);
  rfm69_write_reg(0x08, (x >> 8) & 0xff);
  rfm69_write_reg(0x09, (x >> 0) & 0xff);
}

static inline void rfm69_read_frf(uint32_t* x)
{
  /* carrier frequency */

  *x = ((uint32_t)rfm69_read_reg(0x07)) << 16;
  *x |= ((uint32_t)rfm69_read_reg(0x08)) << 8;
  *x |= ((uint32_t)rfm69_read_reg(0x09)) << 0;
}

static inline void rfm69_write_lna(uint8_t x)
{
  rfm69_write_reg(0x18, x);
}

static inline uint8_t rfm69_read_lna(void)
{
  return rfm69_read_reg(0x18);
}

static inline void rfm69_write_rx_bw(uint8_t x)
{
  rfm69_write_reg(0x19, x);
}

static inline void rfm69_write_ook_peak(uint8_t x)
{
  rfm69_write_reg(0x1b, x);
}

static inline void rfm69_write_ook_avg(uint8_t x)
{
  rfm69_write_reg(0x1c, x);
}

static inline void rfm69_write_ook_fix(uint8_t x)
{
  rfm69_write_reg(0x1d, x);
}

static inline uint8_t rfm69_read_rssi_config(void)
{
  return rfm69_read_reg(0x23);
}

static inline void rfm69_write_rssi_config(uint8_t x)
{
  rfm69_write_reg(0x23, x);
}

static inline uint8_t rfm69_read_rssi_value(void)
{
  return rfm69_read_reg(0x24);
}

static inline uint8_t rfm69_get_rssi(void)
{
  rfm69_write_rssi_config(1 << 0);
  while ((rfm69_read_rssi_config() & (1 << 1)) == 0) ;
  return rfm69_read_rssi_value();
}

static inline void rfm69_write_dio_mapping_2(uint8_t x)
{
  rfm69_write_reg(0x26, x);  
}

static inline uint8_t rfm69_read_dio_mapping_2(void)
{
  return rfm69_read_reg(0x26);
}

static inline uint8_t rfm69_read_irq_flags_1(void)
{
  return rfm69_read_reg(0x27);
}

static inline void rfm69_write_rssi_threshold(uint8_t x)
{
  rfm69_write_reg(0x29, x);
}

static void rfm69_setup(void)
{
  uint8_t x;

  spi_setup_master();
  rfm69_csn_setup();
  rfm69_csn_high();

  /* dclk signal */
  RFM69_IO_DIO1_DDR |= RFM69_IO_DIO1_MASK;
  RFM69_IO_DIO1_PORT &= ~RFM69_IO_DIO1_MASK;

  /* data signal */
  RFM69_IO_DIO2_DDR &= ~RFM69_IO_DIO2_MASK;
  RFM69_IO_DIO2_PORT &= ~RFM69_IO_DIO2_MASK;

  /* put in standby mode */
  rfm69_write_op_mode(1 << 2);

  /* bitrate */
  /* maximum ook bitrate is 32 Kbps */
  /* bitrate = fxosc / bitrate */
  /* fxosc = 32MHz */
#define RFM69_BITRATE_KBPS 32.0
  rfm69_write_bitrate((uint16_t)(32000.0 / RFM69_BITRATE_KBPS));

  /* 433.92 MHz carrier frequency */
  /* fcar = fstep * frf */
  /* fstep = fxosc / (2^19) */
  /* fxosc = 32MHz */
#if 0
  /* true frequency */
  rfm69_write_frf((uint32_t)7109345.28);
#else
  /* shifted frequency as seen in rtlsdr */
  rfm69_write_frf((uint32_t)7110515);
#endif

  /* ook related values, cf. 3.4.12 */

#if 1

  /* peak mode: a one is detected when the rssi reaches */
  /* peak_thresh - 6db. the peak_thresh value is updated with */
  /* the maximum rssi value seen so far. when a zero is */
  /* detected, peak_thresh is decremented by peak_thresh_step */
  /* every peak_thresh_dec period until it reaches */
  /* fixed_thresh. */
  /* note that the period depends on the bit rate. */
  /* cf figure 12 for fixed_thresh optimzing algorithm */

  rfm69_write_ook_peak(1 << 6);
  rfm69_write_ook_fix(70);

#else

  /* fixed threshold */

  rfm69_write_ook_peak(0 << 6);
  rfm69_write_ook_fix(70);

#endif

  x = rfm69_read_lna();
  x = (1 << 7) | (1 << 0);
  rfm69_write_lna(x);

  /* bits<7:5>: dcc freq */
  /* bits<4:3>: rx bw mant */
  /* bits<2:0>: rx bw exp */
  rfm69_write_rx_bw((2 << 5) | (1 << 3) | (1 << 0));

  rfm69_write_rssi_threshold(35);
}

static void rfm69_set_rx_continuous_mode(void)
{
  RFM69_IO_DIO2_DDR &= ~RFM69_IO_DIO2_MASK;
  RFM69_IO_DIO2_PORT &= ~RFM69_IO_DIO2_MASK;

  rfm69_write_data_modul((3 << 5) | (1 << 3));
  rfm69_write_op_mode((1 << 7) | (4 << 2));

  while (!(rfm69_read_irq_flags_1() & (1 << 7))) ;
}

static void rfm69_set_tx_continuous_mode(void)
{
  /* ook modulation, continuous tx */

  RFM69_IO_DIO2_DDR |= RFM69_IO_DIO2_MASK;
  RFM69_IO_DIO2_PORT &= ~RFM69_IO_DIO2_MASK;

  rfm69_write_data_modul((3 << 5) | (1 << 3));
  rfm69_write_op_mode((1 << 7) | (3 << 2));

  while (!(rfm69_read_irq_flags_1() & (1 << 7))) ;
}

static void rfm69_set_standby_mode(void)
{
  rfm69_write_op_mode((1 << 7) | (1 << 2));
  while (!(rfm69_read_irq_flags_1() & (1 << 7))) ;
}

static inline uint8_t rfm69_get_data(void)
{
  return RFM69_IO_DIO2_PIN & RFM69_IO_DIO2_MASK;
}

static inline void rfm69_wait_t_data(void)
{
  /* t_data = 250ns = 4 insn at 16MHz */

  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
  __asm__ __volatile__ ("nop");
}

static inline void rfm69_set_data_high(void)
{
  /* data is sampled on the rising edge of dclk/dio1 */

  RFM69_IO_DIO1_PORT &= ~RFM69_IO_DIO1_MASK;
  RFM69_IO_DIO2_PORT |= RFM69_IO_DIO2_MASK;
  rfm69_wait_t_data();
  RFM69_IO_DIO1_PORT |= RFM69_IO_DIO1_MASK;
  /* assume at least t_data from here */
}

static inline void rfm69_set_data_low(void)
{
  RFM69_IO_DIO1_PORT &= ~RFM69_IO_DIO1_MASK;
  RFM69_IO_DIO2_PORT &= ~RFM69_IO_DIO2_MASK;
  rfm69_wait_t_data();
  RFM69_IO_DIO1_PORT |= RFM69_IO_DIO1_MASK;
}


#endif /* RFM69_C_INCLUDED */
