#ifndef SW_SPI_DRIVER
#define SW_SPI_DRIVER

#include "config.h"

// D8  - PB0
// D9  - PB1
// D10 - PB2
// D11 - PB3 (SPI MOSI) SDI
// D12 - PB4 (SPI MISO) SDO
// D13 - PB5 (SPI SCK)

// NRF24L01_CE_PIN   (7)
// NRF24L01_CSN_PIN  (8)


#ifdef SW_SPI_MULTIPROTOCOL4IN1 /* Multiprotocol 4 in 1 */
// SDIO
#define SDI_pin   5                    //D5 = PD5
#define SDI_port PORTD
#define SDI_ipr  PIND
#define SDI_ddr  DDRD

#define SDI_on  SDI_port |= _BV(SDI_pin)
#define SDI_off SDI_port &= ~_BV(SDI_pin)
#define SDI_1 (SDI_ipr & _BV(SDI_pin))
#define SDI_0 (SDI_ipr & _BV(SDI_pin)) == 0x00

#define SDI_input SDI_ddr &= ~_BV(SDI_pin)
#define SDI_output  SDI_ddr |=  _BV(SDI_pin)

//SDO
#define SDO_pin   6                 //D6 = PD6
#define SDO_port  PORTD
#define SDO_ipr   PIND

#define SDO_1 (SDO_ipr & _BV(SDO_pin))
#define SDO_0 ((SDO_ipr & _BV(SDO_pin)) == 0x00)

#define SCLK_port PORTD
#define SCLK_ddr DDRD

#define SCLK_pin  4               //D4 = PD4
#define SCLK_output SCLK_ddr  |=  _BV(SCLK_pin)
#define SCLK_on   SCLK_port |=  _BV(SCLK_pin)
#define SCLK_off  SCLK_port &= ~_BV(SCLK_pin)

#define NRF_CSN_pin   0               //D8 = PB0
#define NRF_CSN_port  PORTB
#define NRF_CSN_ddr   DDRB
#define NRF_CSN_output  NRF_CSN_ddr  |=  _BV(NRF_CSN_pin)
#define NRF_CSN_on    NRF_CSN_port |=  _BV(NRF_CSN_pin)
#define NRF_CSN_off   NRF_CSN_port &= ~_BV(NRF_CSN_pin)
#define NRF_CE_on
#define NRF_CE_off


#elif defined(SW_SPI_NRF24L01_TRANSMITTER) /* nrf24l01 transmitter */

// D8  - PB0
// D9  - PB1
// D10 - PB2
// D11 - PB3 (SPI MOSI) SDI
// D12 - PB4 (SPI MISO) SDO
// D13 - PB5 (SPI SCK)

// NRF24L01_CE_PIN   (7)
// NRF24L01_CSN_PIN  (8)


// SDIO
#define SDI_pin   3                    //D11 = PB3
#define SDI_port PORTB
#define SDI_ipr  PINB
#define SDI_ddr  DDRB

#define SDI_on  SDI_port |= _BV(SDI_pin)
#define SDI_off SDI_port &= ~_BV(SDI_pin)
#define SDI_1 (SDI_ipr & _BV(SDI_pin))
#define SDI_0 (SDI_ipr & _BV(SDI_pin)) == 0x00

#define SDI_input SDI_ddr &= ~_BV(SDI_pin)
#define SDI_output  SDI_ddr |=  _BV(SDI_pin)

//SDO
#define SDO_pin   4                 //D12 = PB4
#define SDO_port  PORTB
#define SDO_ipr   PINB

#define SDO_1 (SDO_ipr & _BV(SDO_pin))
#define SDO_0 ((SDO_ipr & _BV(SDO_pin)) == 0x00)

#if 0
#define SCLK_port PORTB
#define SCLK_ddr DDRB

#define SCLK_pin  5               //D13 = PB5
#define SCLK_output SCLK_ddr  |=  _BV(SCLK_pin)
#define SCLK_on   SCLK_port |=  _BV(SCLK_pin)
#define SCLK_off  SCLK_port &= ~_BV(SCLK_pin)
#else
#define SCLK_port PORTD
#define SCLK_ddr DDRD

#define SCLK_pin  2               //D2 = PD2
#define SCLK_output SCLK_ddr  |=  _BV(SCLK_pin)
#define SCLK_on   SCLK_port |=  _BV(SCLK_pin)
#define SCLK_off  SCLK_port &= ~_BV(SCLK_pin)
#endif

#define NRF_CSN_pin   0               //D8 = PB0
#define NRF_CSN_port  PORTB
#define NRF_CSN_ddr   DDRB
#define NRF_CSN_output  NRF_CSN_ddr  |=  _BV(NRF_CSN_pin)
#define NRF_CSN_on    NRF_CSN_port |=  _BV(NRF_CSN_pin)
#define NRF_CSN_off   NRF_CSN_port &= ~_BV(NRF_CSN_pin)
#define NRF_CE_on
#define NRF_CE_off

#endif


#define NOP() __asm__ __volatile__("nop")
#define XNOP()

void SPI_Write(uint8_t command)
{
  uint8_t n=8; 
  SCLK_off;//SCK start low
  XNOP();
  SDI_off;
  XNOP();
  do
  {
    if(command&0x80)
      SDI_on;
    else
      SDI_off;
    XNOP();
    SCLK_on;
    XNOP();
    XNOP();
    command = command << 1;
    SCLK_off;
    XNOP();
  }
  while(--n) ;
  SDI_on;
}

uint8_t SPI_Read(void)
{
  uint8_t result=0,i;
  for(i=0;i<8;i++)
  {
    result=result<<1;
    if(SDO_1)
      result |= 0x01;
    SCLK_on;
    XNOP();
    XNOP();
#if 1
    NOP();
#else
    __asm__ __volatile__("nop");
#endif
    SCLK_off;
    XNOP();
    XNOP();
  }
  return result;
}

#endif /* SW_SPI_DRIVER */
