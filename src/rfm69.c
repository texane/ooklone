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
/* dio0 to portd2 */

#define RFM69_IO_CSN_DDR DDRB
#define RFM69_IO_CSN_PORT PORTB
#define RFM69_IO_CSN_MASK (1 << 2)

#define RFM69_IO_DIO0_DDR DDRD
#define RFM69_IO_DIO0_PIN PIND
#define RFM69_IO_DIO0_MASK (1 << 2)

#define RFM69_IO_DIO2_DDR DDRD
#define RFM69_IO_DIO2_PIN PIND
#define RFM69_IO_DIO2_PORT PORTD
#define RFM69_IO_DIO2_MASK (1 << 3)

#define RFM69_IO_DIO4_DDR DDRD
#define RFM69_IO_DIO4_PIN PIND
#define RFM69_IO_DIO4_PORT PORTD
#define RFM69_IO_DIO4_MASK (1 << 4)

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

static void rfm69_setup(void)
{
  uint8_t x;

  spi_setup_master();
  rfm69_csn_setup();
  rfm69_csn_high();

  RFM69_IO_DIO2_DDR &= ~RFM69_IO_DIO2_MASK;
  RFM69_IO_DIO2_PORT &= ~RFM69_IO_DIO2_MASK;

  RFM69_IO_DIO4_DDR &= ~RFM69_IO_DIO4_MASK;
  RFM69_IO_DIO4_PORT &= ~RFM69_IO_DIO4_MASK;

  /* put in standby mode */
  rfm69_write_op_mode(1 << 2);

  /* bitrate */
  /* bitrate = fxosc / bitrate */
  /* fxosc = 32MHz */
#define RFM69_BITRATE_KBPS 10.0
  rfm69_write_bitrate((uint16_t)(32000.0 / RFM69_BITRATE_KBPS));

  /* 433.92 MHz carrier frequency */
  /* fcar = fstep * frf */
  /* fstep = fxosc / (2^19) */
  /* fxosc = 32MHz */
  rfm69_write_frf((uint32_t)7109345.28);

  /* fixed ook threshold */
  /* TODO: should be in a calibration loop */
  rfm69_write_ook_peak(0);
 /* TODO: rfm69_write_ook_avg(); */
  rfm69_write_ook_fix(0xf0 / 4);

  /* set dio4 mapping as for rx ready */
  x = rfm69_read_dio_mapping_2();
  x = (x & ~(3 << 6)) | (1 << 6);
  rfm69_write_dio_mapping_2(x);

  x = rfm69_read_lna();
  x = (x & ~7) | 2;
  rfm69_write_lna(x);
}

static void rfm69_set_rx_continuous_mode(void)
{
  rfm69_write_data_modul((3 << 5) | (1 << 3));
  rfm69_write_op_mode((1 << 7) | (4 << 2));
  while (!(rfm69_read_irq_flags_1() & (1 << 7))) ;
}

static inline uint8_t rfm69_get_data(void)
{
  return RFM69_IO_DIO2_PIN & RFM69_IO_DIO2_MASK;
}
 

#endif /* RFM69_C_INCLUDED */
