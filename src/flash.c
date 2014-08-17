#ifndef FLASH_C_INCLUDED
#define FLASH_C_INCLUDED


#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "./spi.c"


/* model: micron nor flash n25q128a13ese40g */
/* datasheet: n25q_128mb_3v_65nm.pdf */
/* protocol: spi extended */

#define FLASH_PAGE_SIZE 256
#define FLASH_SUBSECTOR_SIZE (4 * 1024)
#define FLASH_SECTOR_SIZE (64 * 1024)

#define FLASH_CSN_DDR DDRB
#define FLASH_CSN_PORT PORTB
#define FLASH_CSN_MASK (1 << 0)

static inline void flash_csn_high(void)
{
  FLASH_CSN_PORT |= FLASH_CSN_MASK;
}

static inline void flash_csn_low(void)
{
  FLASH_CSN_PORT &= ~FLASH_CSN_MASK;
}

static inline void flash_csn_setup(void)
{
  FLASH_CSN_DDR |= FLASH_CSN_MASK;
}

static void flash_setup(void)
{
  flash_csn_setup();
  flash_csn_high();
}

static uint8_t flash_read_reg(uint8_t i)
{
  /* i the command */

  uint8_t x;

  flash_csn_low();
  spi_write_uint8(i);
  x = spi_read_uint8();
  flash_csn_high();

  return x;
}

static inline uint8_t flash_read_flag_status(void)
{
  /* bits */
  /* 7: program or erase done */
  /* 6: erase suspend in effect */
  /* 5: erase error */
  /* 4: program failed */
  /* 3: vpp invalid during program or erase */
  /* 2: program suspend in effect */
  /* 1: attempt to access protected space */
  /* 0: reserved */

  return flash_read_reg(0x70);
}

static uint8_t flash_read_status(void)
{
  /* bits */
  /* 7: write disabled */
  /* 0: write in progress */

  return flash_read_reg(0x70);
}

static void flash_wait_erase(void)
{
  /* wait for write enable bit to be cleared indicating */
  /* the operation has been scheduled. wait for write in */
  /* progress to be cleared, indicating operation is done */

  while (1)
  {
    const uint8_t x = flash_read_status();
    if ((x & ((1 << 7) | (1 << 0))) == (1 << 7)) break ;
  }
}

static inline void flash_wait_program(void)
{
  flash_wait_erase();
}

static void flash_read_id(uint8_t* x)
{
  uint8_t i;

  flash_csn_low();
  spi_write_uint8(0x9e);
  for (i = 0; i != 20; ++i) x[i] = spi_read_uint8();
  flash_csn_high();
}

static void flash_set_wren(void)
{
  /* set the write enable bit */

  flash_csn_low();
  spi_write_uint8(0x06);
  flash_csn_high();  
}

static void flash_erase_sector(uint16_t i)
{
  /* erase one sector */
  /* i the sector offset in FLASH_SECTOR_SIZE units  */

  flash_set_wren();

  flash_csn_low();

  spi_write_uint8(0xd8);

  /* address, lsB first */
  spi_write_uint8(0);
  spi_write_uint8(0);
  spi_write_uint8(i & 0xff);

  flash_csn_high();

  flash_wait_erase();
}

static void flash_erase_subsector(uint16_t i)
{
  /* erase one subsector */
  /* i the subsector offset in FLASH_SUBSECTOR_SIZE units  */

  flash_set_wren();

  flash_csn_low();

  spi_write_uint8(0x20);

  /* address, lsB first */
  spi_write_uint8(0);
  spi_write_uint8(i & 0x0f);
  spi_write_uint8((i >> 4) & 0xff);

  flash_csn_high();

  flash_wait_erase();
}

static void flash_erase_bulk(void)
{
  flash_set_wren();

  flash_csn_low();
  spi_write_uint8(0xc7);
  flash_csn_high();

  flash_wait_erase();
}

static void flash_program_page(uint16_t i, const uint8_t* s)
{
  /* write one page. assume the page previously erased. */
  /* i the page position in FLASH_PAGE_SIZE units */
  /* s the buffer to write of FLASH_PAGE_SIZE bytes */

  uint8_t n;

  flash_set_wren();

  flash_csn_low();

  spi_write_uint8(0x02);

  /* address, lsB first */
  spi_write_uint8(0);
  spi_write_uint8((uint8_t)((i >> 0) & 0xff));
  spi_write_uint8((uint8_t)((i >> 8) & 0xff));

  /* data */
  for (n = 0; n != 0xff; ++n, ++s) spi_write_uint8(*s);
  spi_write_uint8(*s);

  flash_csn_high();

  flash_wait_program();
}

static void flash_read_page(uint16_t i, uint8_t* s)
{
  /* read one page */
  /* i the page position in FLASH_PAGE_SIZE units */
  /* s the buffer to write of FLASH_PAGE_SIZE bytes */

  uint8_t n;

  flash_csn_low();

  spi_write_uint8(0x03);

  /* address, lsB first */
  spi_write_uint8(0);
  spi_write_uint8((uint8_t)((i >> 0) & 0xff));
  spi_write_uint8((uint8_t)((i >> 8) & 0xff));

  for (n = 0; n != 0xff; ++n, ++s) *s = spi_read_uint8();
  *s = spi_read_uint8();

  flash_csn_high();
}


#if 0 /* unit test */

#include "./uart.c"

#define PRINT_LINE()				\
do {						\
  uart_write(uint16_to_string(__LINE__), 4);	\
  uart_write((uint8_t*)"\r\n", 2);		\
} while (0)

static void print_buf(uint8_t* s, uint16_t n)
{
  uint16_t i;

  for (i = 0; i != n; ++i)
  {
    if (i && ((i % 16) == 0)) uart_write((uint8_t*)"\r\n", 2);
    uart_write(uint8_to_string(s[i]), 2);
  }
  uart_write((uint8_t*)"\r\n", 2);
}

static void erase_buf(uint8_t* s, uint16_t n)
{
  uint16_t i;
  for (i = 0; i != n; ++i, ++s) *s = 0x2a;
}

static void fill_buf(uint8_t* s, uint16_t n)
{
  uint16_t i;
  for (i = 0; i != n; ++i, ++s) *s = (uint8_t)i;
}

int main(void)
{
  uint8_t buf[FLASH_PAGE_SIZE];

  /* unselect rfm69 slave */
#define RFM69_IO_CSN_DDR DDRB
#define RFM69_IO_CSN_PORT PORTB
#define RFM69_IO_CSN_MASK (1 << 2)
  RFM69_IO_CSN_DDR |= RFM69_IO_CSN_MASK;
  RFM69_IO_CSN_PORT |= RFM69_IO_CSN_MASK;

  spi_setup_master();
  flash_setup();
  uart_setup();

  while (1)
  {
    erase_buf(buf, 20);
    flash_read_id(buf);
    print_buf(buf, 20);

    PRINT_LINE();

    flash_erase_subsector(0);

    PRINT_LINE();

    fill_buf(buf, FLASH_PAGE_SIZE);
    flash_program_page(0, buf);

    PRINT_LINE();

    erase_buf(buf, FLASH_PAGE_SIZE);
    flash_read_page(0, buf);

    print_buf(buf, FLASH_PAGE_SIZE);

    _delay_ms(250);
    _delay_ms(250);
    _delay_ms(250);
    _delay_ms(250);
  }

  return 0;
}

#endif /* unit test */


#endif /* FLASH_C_INCLUDED */
