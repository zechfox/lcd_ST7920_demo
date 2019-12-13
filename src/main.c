/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/usart.h>

#include <stdio.h>
#include <errno.h>

#define LCD_CMD 1
#define LCD_DATA 0
#define LCD_CS_PIN
#define LCD_RS_PIN
#define LCD_SCLK_PIN
#define LCD_SDA_PIN
#define LCD_RESET_PIN

int _write(int file, char *ptr, int len);
static void gpio_setup(void);
void i2c_setup(void);
void clock_setup(void);
void usart_setup(void);
void dma_setup(void);
void i2c_transfer_by_dma(unsigned char * data, unsigned char size);
void dma_transfer(void);
void lcd_setup(void);
void lcd_tick(void);
void lcd_write_byte(unsigned char data);
void lcd_cs_high(void);
void lcd_cs_low(void);
void lcd_rs_high(void);
void lcd_rs_low(void);
void lcd_reset_high(void);
void lcd_reset_low(void);
void lcd_sda_high(void);
void lcd_sda_low(void);
void lcd_sclk_high(void);
void lcd_sclk_low(void);
void lcd_write_cmd(unsigned char cmd);
void lcd_write_data(unsigned char data);
void lcd_clear_screen(void);
void lcd_draw_buffer(unsigned char *buf);
void lcd_draw_block(unsigned char x, unsigned char y, const unsigned char * blk);
void lcd_display_string(unsigned char x, unsigned char y, char *string);
void lcd_set_address(unsigned char x, unsigned char y);
void delay_us(unsigned int us);
void delay_ms(unsigned char ms);

const unsigned char g_block[8] = {
  0xff,
  0x81,
  0xbd,
  0xa5,
  0xa5,
  0xbd,
  0x81,
  0xff
};
unsigned char ascii_table_5x7[95][5] = {
/*ASCII list:5x7 pixel*/
  {0x00,0x00,0x00,0x00,0x00},//space
  {0x00,0x00,0x4f,0x00,0x00},//!
  {0x00,0x07,0x00,0x07,0x00},//"
  {0x14,0x7f,0x14,0x7f,0x14},//#
  {0x24,0x2a,0x7f,0x2a,0x12},//$
  {0x23,0x13,0x08,0x64,0x62},//%
  {0x36,0x49,0x55,0x22,0x50},//&
  {0x00,0x05,0x07,0x00,0x00},//]
  {0x00,0x1c,0x22,0x41,0x00},//(
  {0x00,0x41,0x22,0x1c,0x00},//)
  {0x14,0x08,0x3e,0x08,0x14},//*
  {0x08,0x08,0x3e,0x08,0x08},//+
  {0x00,0x50,0x30,0x00,0x00},//,
  {0x08,0x08,0x08,0x08,0x08},//-
  {0x00,0x60,0x60,0x00,0x00},//.
  {0x20,0x10,0x08,0x04,0x02},///
  {0x3e,0x51,0x49,0x45,0x3e},//0
  {0x00,0x42,0x7f,0x40,0x00},//1
  {0x42,0x61,0x51,0x49,0x46},//2
  {0x21,0x41,0x45,0x4b,0x31},//3
  {0x18,0x14,0x12,0x7f,0x10},//4
  {0x27,0x45,0x45,0x45,0x39},//5
  {0x3c,0x4a,0x49,0x49,0x30},//6
  {0x01,0x71,0x09,0x05,0x03},//7
  {0x36,0x49,0x49,0x49,0x36},//8
  {0x06,0x49,0x49,0x29,0x1e},//9
  {0x00,0x36,0x36,0x00,0x00},//:
  {0x00,0x56,0x36,0x00,0x00},//;
  {0x08,0x14,0x22,0x41,0x00},//<
  {0x14,0x14,0x14,0x14,0x14},//=
  {0x00,0x41,0x22,0x14,0x08},//>
  {0x02,0x01,0x51,0x09,0x06},//?
  {0x32,0x49,0x79,0x41,0x3e},//@
  {0x7e,0x11,0x11,0x11,0x7e},//A
  {0x7f,0x49,0x49,0x49,0x36},//B
  {0x3e,0x41,0x41,0x41,0x22},//C
  {0x7f,0x41,0x41,0x22,0x1c},//D
  {0x7f,0x49,0x49,0x49,0x41},//E
  {0x7f,0x09,0x09,0x09,0x01},//F
  {0x3e,0x41,0x49,0x49,0x7a},//G
  {0x7f,0x08,0x08,0x08,0x7f},//H
  {0x00,0x41,0x7f,0x41,0x00},//I
  {0x20,0x40,0x41,0x3f,0x01},//J
  {0x7f,0x08,0x14,0x22,0x41},//K
  {0x7f,0x40,0x40,0x40,0x40},//L
  {0x7f,0x02,0x0c,0x02,0x7f},//M
  {0x7f,0x04,0x08,0x10,0x7f},//N
  {0x3e,0x41,0x41,0x41,0x3e},//O
  {0x7f,0x09,0x09,0x09,0x06},//P
  {0x3e,0x41,0x51,0x21,0x5e},//Q
  {0x7f,0x09,0x19,0x29,0x46},//R
  {0x46,0x49,0x49,0x49,0x31},//S
  {0x01,0x01,0x7f,0x01,0x01},//T
  {0x3f,0x40,0x40,0x40,0x3f},//U
  {0x1f,0x20,0x40,0x20,0x1f},//V
  {0x3f,0x40,0x38,0x40,0x3f},//W
  {0x63,0x14,0x08,0x14,0x63},//X
  {0x07,0x08,0x70,0x08,0x07},//Y
  {0x61,0x51,0x49,0x45,0x43},//Z
  {0x00,0x7f,0x41,0x41,0x00},//[
  {0x02,0x04,0x08,0x10,0x20},/*\*/
  {0x00,0x41,0x41,0x7f,0x00},//]
  {0x04,0x02,0x01,0x02,0x04},//^
  {0x40,0x40,0x40,0x40,0x40},//_
  {0x01,0x02,0x04,0x00,0x00},//`
  {0x20,0x54,0x54,0x54,0x78},//a
  {0x7f,0x48,0x48,0x48,0x30},//b
  {0x38,0x44,0x44,0x44,0x44},//c
  {0x30,0x48,0x48,0x48,0x7f},//d
  {0x38,0x54,0x54,0x54,0x58},//e
  {0x00,0x08,0x7e,0x09,0x02},//f
  {0x48,0x54,0x54,0x54,0x3c},//g
  {0x7f,0x08,0x08,0x08,0x70},//h
  {0x00,0x00,0x7a,0x00,0x00},//i
  {0x20,0x40,0x40,0x3d,0x00},//j
  {0x7f,0x20,0x28,0x44,0x00},//k
  {0x00,0x41,0x7f,0x40,0x00},//l
  {0x7c,0x04,0x38,0x04,0x7c},//m
  {0x7c,0x08,0x04,0x04,0x78},//n
  {0x38,0x44,0x44,0x44,0x38},//o
  {0x7c,0x14,0x14,0x14,0x08},//p
  {0x08,0x14,0x14,0x14,0x7c},//q
  {0x7c,0x08,0x04,0x04,0x08},//r
  {0x48,0x54,0x54,0x54,0x24},//s
  {0x04,0x04,0x3f,0x44,0x24},//t
  {0x3c,0x40,0x40,0x40,0x3c},//u
  {0x1c,0x20,0x40,0x20,0x1c},//v
  {0x3c,0x40,0x30,0x40,0x3c},//w
  {0x44,0x28,0x10,0x28,0x44},//x
  {0x04,0x48,0x30,0x08,0x04},//y
  {0x44,0x64,0x54,0x4c,0x44},//z
  {0x08,0x36,0x41,0x41,0x00},//{
  {0x00,0x00,0x77,0x00,0x00},//|
  {0x00,0x41,0x41,0x36,0x08},//}
  {0x04,0x02,0x02,0x02,0x01}//~
};

int _write(int file, char *ptr, int len)
{
  int i;

  if (file == 1) {
    for (i = 0; i < len; i++)
      usart_send_blocking(USART1, ptr[i]);
    return i;
  }

  errno = EIO;
  return -1;
}

static void gpio_setup(void)
{
	// Enable GPIOA clock.
  rcc_periph_clock_enable(RCC_GPIOA);

	// Set GPIO1 (in GPIO port A) to 'output push-pull'.
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);

  // Used GPIO2 for LCD CS signal
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);
  // Used GPIO3 for LCD RESET signal
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO3);
  // Used GPIO4 for LCD RS signal
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);

  rcc_periph_clock_enable(RCC_GPIOB);

  // Used GPIO7 for LCD SDA signal
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO7);
  // Used GPIO6 for LCD SCLK signal
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO6);

}

void lcd_cs_high(void)
{
  gpio_set(GPIOA, GPIO2);
}

void lcd_cs_low(void)
{
  gpio_clear(GPIOA, GPIO2);
}

void lcd_rs_high(void)
{
  gpio_set(GPIOA, GPIO4);
}

void lcd_rs_low(void)
{
  gpio_clear(GPIOA, GPIO4);
}

void lcd_reset_high(void)
{
  gpio_set(GPIOA, GPIO3);
}

void lcd_reset_low(void)
{
  gpio_clear(GPIOA, GPIO3);
}

void lcd_sda_high(void)
{
  gpio_set(GPIOB, GPIO7);
}

void lcd_sda_low(void)
{
  gpio_clear(GPIOB, GPIO7);
}

void lcd_sclk_high(void)
{
  gpio_set(GPIOB, GPIO6);
}

void lcd_sclk_low(void)
{
  gpio_clear(GPIOB, GPIO6);
}

void i2c_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOB);
  // Enable I2C clock
  rcc_periph_clock_enable(RCC_I2C1);
  // does it neccessary?
  rcc_periph_clock_enable(RCC_AFIO);

  // Alternate PB7 as I2C SDA, PB6 as I2C SCL
  // For bi-direction ALT IO, set as output
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_I2C1_SCL | GPIO_I2C1_SDA);

  i2c_peripheral_disable(I2C1);
  i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_36MHZ);
  i2c_set_fast_mode(I2C1);
  i2c_disable_ack(I2C1);
  // fclock for I2C is 36MHz APB2 -> cycle time 28ns 
  // SCL low time@400KHz -> Thigh=800ns
  // CCR = Tlow/Tcycle = 0x1C,9; Using 0x1E
  i2c_set_ccr(I2C1, 0x1e);
  // rise time for 400kHz => 300ns,
  // 100kHz => 1000ns; 300ns/28ns = 10;
  // increment by 1 -> 11;
  i2c_set_trise(I2C1, 0x0b);
  i2c_peripheral_enable(I2C1);
}

void clock_setup(void)
{
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

void delay_us(unsigned int us)
{
  for(unsigned int i = 0; i < us; i++)
  {
    // 72 clock for 1us
    for (unsigned char j = 0; j < 72; j++)	/* Wait a bit. */
      __asm__("nop");
  }
}

void delay_ms(unsigned char ms)
{

  for(unsigned int i = 0; i < ms; i++)
  {
    delay_us(1000);
  }

}

void usart_setup(void)
{

	// Enable clocks for USART1
	rcc_periph_clock_enable(RCC_USART1);

  // Alternate PA9 as USART1 TX, PA10 as USART1 RX
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

  usart_set_baudrate(USART1, 115200);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

  // Finally enable the USART.
  usart_enable(USART1);
}

void dma_setup(void)
{
  // Enable DMA1 clock
	rcc_periph_clock_enable(RCC_DMA1);

  dma_channel_reset(DMA1, DMA_CHANNEL7);

  // configure dma channel for I2C1
  //dma_set_peripheral_address(DMA1, DMA_CHANNEL7, (uint32_t)&I2C1_DR);
  //dma_set_memory_address(DMA1, DMA_CHANNEL7, (uint32_t)data);
  //dma_set_number_of_data(DMA1, DMA_CHANNEL7, size);
  dma_set_read_from_memory(DMA1, DMA_CHANNEL7);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL7);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL7, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(DMA1, DMA_CHANNEL7, DMA_CCR_MSIZE_8BIT);
  dma_set_priority(DMA1, DMA_CHANNEL7, DMA_CCR_PL_VERY_HIGH);

  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL7);

  dma_enable_channel(DMA1, DMA_CHANNEL7);
}

void i2c_transfer_by_dma(unsigned char * data, unsigned char size)
{
  // configure dma channel for I2C1
  dma_set_peripheral_address(DMA1, DMA_CHANNEL7, (uint32_t)&I2C1_DR);
  dma_set_memory_address(DMA1, DMA_CHANNEL7, (uint32_t)data);
  dma_set_number_of_data(DMA1, DMA_CHANNEL7, size);

  i2c_enable_dma(I2C1);
}

void dma_transfer(void)
{

}

void lcd_setup(void)
{

}

void lcd_tick(void)
{
  //delay_us(1);
}

void lcd_write_byte(unsigned char byte)
{
  lcd_cs_low();
  for (unsigned char i = 0; i < 8; i++)
  {
    lcd_sclk_low();
    if (byte & 0x80)
    {
      lcd_sda_high();
    }
    else
    {
      lcd_sda_low();
    }
    lcd_sclk_high();
    lcd_tick();
    byte <<= 1;
  }
  lcd_cs_high();
  lcd_tick();
}

void lcd_write_cmd(unsigned char cmd)
{
  lcd_rs_low();
  lcd_write_byte(cmd);

}

void lcd_write_data(unsigned char data)
{
  lcd_rs_high();
  lcd_write_byte(data);
}

void lcd_clear_screen(void)
{
  for (unsigned char i = 0; i < 8; i++)
  {
    lcd_write_cmd(0xb0 | i);
    lcd_write_cmd(0x10);
    lcd_write_cmd(0x00);
    for (unsigned char j = 0; j < 128; j++)
    {
      lcd_write_data(0x00);
    }
  }
}

void lcd_draw_buffer(unsigned char *buf)
{
  for(unsigned char i = 0; i < 8; i++)
  {
    lcd_write_cmd(0xb0 | i);
    lcd_write_cmd(0x10);
    lcd_write_cmd(0x00);
    for (unsigned char j = 0; j < 128; j++)
    {
      lcd_write_data(*buf++);
    }
  }
}

void lcd_draw_block(unsigned char x, unsigned char y, const unsigned char * blk)
{
  lcd_set_address(x, y);
  for (int i = 0; i < 8; i++) {
    lcd_write_data(*blk++);
  }
}

void lcd_set_address(unsigned char x, unsigned char y)
{
  if (x >= 8 || y >= 21) return;
  y = (y + 1) * 6;
  lcd_write_cmd(0xb0 | (x & 0x0f));
  lcd_write_cmd(0x10 | ((y >> 4) & 0x0f));
  lcd_write_cmd(0x00 | (y & 0x0f));
}

void lcd_display_string(unsigned char x, unsigned char y, char *string)
{
  unsigned char i = 0, j = 0,k = 0;

  while(string[i] != '\0')
  {
    if((string[i] >= 0x20) && (string[i] < 0x7e))
    {
      j = string[i] - 0x20;
      lcd_set_address(x, y);
      for(k = 0;k < 5; k++)
      {
        lcd_write_data(ascii_table_5x7[j][k]);
      }
      y += 1;
    }
    i++; 
  }
}

int main(void)
{
	int cnt;
  clock_setup();

	gpio_setup();
  usart_setup();
  printf("Hello World! \r\n");
  // i2c_setup();
  // dma_setup();
  
  // printf("i2c start send cmd! \r\n");
  lcd_rs_low();
  lcd_cs_high();
  lcd_reset_low();
  delay_us(50);
  lcd_reset_high();
  delay_ms(5);

  //sw reset
  lcd_write_cmd(0xe2);
  // raise voltage 1
  lcd_write_cmd(0x2c);
  // raise voltage 2
  lcd_write_cmd(0x2e);
  // raise voltage 3
  lcd_write_cmd(0x2f);
  // 1/9 bias
  lcd_write_cmd(0xa2);
  // display on
  lcd_write_cmd(0xaf);
  // SEG direction
  lcd_write_cmd(0xa0);
  // COM direction
  lcd_write_cmd(0xc8);
  // regulation
  lcd_write_cmd(0x23);
  // EV.1
  lcd_write_cmd(0x81);
  // EV.2
  lcd_write_cmd(0x2c);
  // power control
  lcd_write_cmd(0x2f);

  // start line
  lcd_write_cmd(0x40);
  lcd_write_cmd(0xb0);
  lcd_write_cmd(0x10);
  lcd_write_cmd(0x00);
  lcd_write_cmd(0xaf);

  lcd_clear_screen();

  lcd_draw_block(0, 0, g_block);
  lcd_display_string(1, 0, "Hello World!");
  lcd_display_string(2, 0, "Guten Tag!");


	/* Blink the LED (PA5) on the board. */
	while (1) {
		/* Manually: */
		// GPIOA_BSRR = GPIO1;		/* LED off */
		// for (i = 0; i < 800000; i++)	/* Wait a bit. */
		// 	__asm__("nop");
		// GPIOA_BRR = GPIO1;		/* LED on */
		// for (i = 0; i < 800000; i++)	/* Wait a bit. */
		// 	__asm__("nop");

		/* Using API functions gpio_set()/gpio_clear(): */
		// gpio_set(GPIOA, GPIO1);	/* LED off */
		// for (i = 0; i < 800000; i++)	/* Wait a bit. */
		// 	__asm__("nop");
		// gpio_clear(GPIOA, GPIO1);	/* LED on */
		// for (i = 0; i < 800000; i++)	/* Wait a bit. */
		// 	__asm__("nop");

		/* Using API function gpio_toggle(): */
		gpio_toggle(GPIOA, GPIO1);	/* LED on/off */
		for (cnt = 0; cnt < 800000; cnt++)	/* Wait a bit. */
			__asm__("nop");
	}

	return 0;
}

