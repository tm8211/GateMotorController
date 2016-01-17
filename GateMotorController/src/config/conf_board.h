/**
 * \file
 *
 * \brief User board configuration template
 *
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

#define MC_RS232_USART              (&AVR32_USART0)
#define MC_RS232_USART_RX_PIN       AVR32_USART0_RXD_0_0_PIN
#define MC_RS232_USART_RX_FUNCTION  AVR32_USART0_RXD_0_0_FUNCTION
#define MC_RS232_USART_TX_PIN       AVR32_USART0_TXD_0_0_PIN
#define MC_RS232_USART_TX_FUNCTION  AVR32_USART0_TXD_0_0_FUNCTION

#define MC_USB_USART                (&AVR32_USART1)
#define MC_USB_USART_RX_PIN         AVR32_USART1_RXD_0_0_PIN
#define MC_USB_USART_RX_FUNCTION    AVR32_USART1_RXD_0_0_FUNCTION
#define MC_USB_USART_TX_PIN         AVR32_USART1_TXD_0_0_PIN
#define MC_USB_USART_TX_FUNCTION    AVR32_USART1_TXD_0_0_FUNCTION

// Port A
#define PIN_INP0                    AVR32_PIN_PA03
#define PIN_INP1                    AVR32_PIN_PA04
#define PIN_INP2                    AVR32_PIN_PA08
#define PIN_CH1_CS_N                AVR32_PIN_PA10      // Integer 10 - use AVR32_SPI0_NPCS_0_0_{PIN,FUNCTION}
#define PIN_CH1_MISO                AVR32_PIN_PA11      // Integer 11 - use AVR32_SPI0_MISO_0_0_{PIN,FUNCTION}
#define PIN_CH1_MOSI                AVR32_PIN_PA12      // Integer 12 - use AVR32_SPI0_MOSI_0_0_{PIN,FUNCTION}
#define PIN_CH1_SCK                 AVR32_PIN_PA13      // Integer 13 - use AVR32_SPI0_SCK_0_0_{PIN,FUNCTION}
#define PIN_INP4                    AVR32_PIN_PA14
#define PIN_INP5                    AVR32_PIN_PA15
#define PIN_INP6                    AVR32_PIN_PA16
#define PIN_INP7                    AVR32_PIN_PA17
#define PIN_INP8                    AVR32_PIN_PA18
#define PIN_INP9                    AVR32_PIN_PA19
#define PIN_INP3                    AVR32_PIN_PA20
#define PIN_ADC0                    AVR32_PIN_PA21
#define PIN_ADC1                    AVR32_PIN_PA22
#define PIN_ADC2                    AVR32_PIN_PA23
#define PIN_ADC3                    AVR32_PIN_PA24
#define PIN_ADC4                    AVR32_PIN_PA25
#define PIN_ADC5                    AVR32_PIN_PA26
#define PIN_ADC6                    AVR32_PIN_PA27
#define PIN_ADC7                    AVR32_PIN_PA28

// PORT B
#define PIN_CH1_ENA                 AVR32_PIN_PB02
#define PIN_CH1_DIR                 AVR32_PIN_PB03
#define PIN_CH1_DIS                 AVR32_PIN_PB04
#define PIN_CH1_PWM                 AVR32_PIN_PB05
#define PIN_CH2_DIR                 AVR32_PIN_PB06
#define PIN_CH2_DIS                 AVR32_PIN_PB07
#define PIN_CH2_ENA                 AVR32_PIN_PB08
#define PIN_CH2_PWM                 AVR32_PIN_PB09
#define PIN_LED0                    AVR32_PIN_PB10
#define PIN_LED1                    AVR32_PIN_PB11
#define PIN_LED2                    AVR32_PIN_PB12
#define PIN_LED3                    AVR32_PIN_PB13
#define PIN_LED4                    AVR32_PIN_PB14
#define PIN_LED5                    AVR32_PIN_PB15
#define PIN_LED6                    AVR32_PIN_PB16
#define PIN_LED7                    AVR32_PIN_PB17
#define PIN_LED_BTN                 AVR32_PIN_PB18
#define PIN_OUT0                    AVR32_PIN_PB19
#define PIN_OUT1                    AVR32_PIN_PB20
#define PIN_OUT2                    AVR32_PIN_PB21
#define PIN_OUT3                    AVR32_PIN_PB22
#define PIN_OUT4                    AVR32_PIN_PB23
#define PIN_OUT5                    AVR32_PIN_PB24
#define PIN_OUT6                    AVR32_PIN_PB25
#define PIN_OUT7                    AVR32_PIN_PB26
#define PIN_OUT8                    AVR32_PIN_PB27
#define PIN_OUT9                    AVR32_PIN_PB28

// PORT X
#define PIN_UART0_INV_N             AVR32_PIN_PX17
#define PIN_ZW_AUX_N                AVR32_PIN_PX24
#define PIN_ZW_INT0                 AVR32_PIN_PX25
#define PIN_ZW_TRIAC                AVR32_PIN_PX26
#define PIN_CH2_MISO                AVR32_PIN_PX34      // Integer 70  - AVR32_SPI1_MISO_0_1_{PIN,FUNCTION}
#define PIN_CH2_MOSI                AVR32_PIN_PX35      // Integer 105 - use AVR32_SPI1_MOSI_0_1_{PIN,FUNCTION}
#define PIN_CH2_SCK                 AVR32_PIN_PX36      // Integer 104 - use AVR32_SPI1_SCK_0_1_{PIN,FUNCTION}
#define PIN_CH2_CS_N                AVR32_PIN_PX37      // Integer 103 - use AVR32_SPI1_NPCS_0_1_{PIN,FUNCTION}

#endif // CONF_BOARD_H
