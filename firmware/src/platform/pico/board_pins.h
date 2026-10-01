// =============================================================================
//  Board pin map — the ONLY place GPIO numbers appear. Adjust to your PCB.
//  All values may be overridden with -D on the CMake command line.
//
//  ADS131M02 (SPI0 + PIO0)          Control / reference outputs
//    GP16  MISO  (DOUT)               GP10  SSCB TRIP request   (active high, latched)
//    GP17  CS    (active low)         GP11  HEALTHY heartbeat   (100 Hz square wave)
//    GP18  SCLK                       GP12..15  EVENT code bits 0..3
//    GP19  MOSI  (DIN)                GP9   EVENT strobe
//    GP20  DRDY  (active low)         GP6   DVR reference PWM   (RC-filter -> 0..3.3 V)
//    GP21  CLKIN (5.12 MHz PWM)       GP7   APF reference PWM
//    GP22  SYNC/RESET (active low)    GP4/5 UART1 TX/RX: APF harmonic reference link
//                                     GP26/27/28 optional MCP4822 DAC (SPI1 SCK/TX, CS)
//  Host link                         Annunciation
//    USB CDC (primary)                GP25  status LED (on-board)
//    GP0/1 UART0 TX/RX (mirror)       GP8   alarm LED / buzzer driver
// =============================================================================
#pragma once

#ifndef PQ_PIN_ADC_MISO
#define PQ_PIN_ADC_MISO 16
#endif
#ifndef PQ_PIN_ADC_CS
#define PQ_PIN_ADC_CS 17
#endif
#ifndef PQ_PIN_ADC_SCLK
#define PQ_PIN_ADC_SCLK 18
#endif
#ifndef PQ_PIN_ADC_MOSI
#define PQ_PIN_ADC_MOSI 19
#endif
#ifndef PQ_PIN_ADC_DRDY
#define PQ_PIN_ADC_DRDY 20
#endif
#ifndef PQ_PIN_ADC_CLKIN
#define PQ_PIN_ADC_CLKIN 21
#endif
#ifndef PQ_PIN_ADC_RESET
#define PQ_PIN_ADC_RESET 22
#endif
#define PQ_ADC_SPI spi0

#ifndef PQ_PIN_TRIP
#define PQ_PIN_TRIP 10
#endif
#ifndef PQ_PIN_HEALTHY
#define PQ_PIN_HEALTHY 11
#endif
#ifndef PQ_PIN_EVENT_BIT0
#define PQ_PIN_EVENT_BIT0 12   // bits 0..3 on consecutive pins
#endif
#ifndef PQ_PIN_EVENT_STROBE
#define PQ_PIN_EVENT_STROBE 9
#endif
#ifndef PQ_PIN_PWM_DVR
#define PQ_PIN_PWM_DVR 6
#endif
#ifndef PQ_PIN_PWM_APF
#define PQ_PIN_PWM_APF 7
#endif
#ifndef PQ_PIN_APF_UART_TX
#define PQ_PIN_APF_UART_TX 4
#endif
#ifndef PQ_PIN_APF_UART_RX
#define PQ_PIN_APF_UART_RX 5
#endif
#define PQ_APF_UART uart1
#ifndef PQ_APF_UART_BAUD
#define PQ_APF_UART_BAUD 921600
#endif

#ifndef PQ_PIN_DAC_SCK
#define PQ_PIN_DAC_SCK 26
#endif
#ifndef PQ_PIN_DAC_MOSI
#define PQ_PIN_DAC_MOSI 27
#endif
#ifndef PQ_PIN_DAC_CS
#define PQ_PIN_DAC_CS 28
#endif
#define PQ_DAC_SPI spi1

#ifndef PQ_PIN_HOST_UART_TX
#define PQ_PIN_HOST_UART_TX 0
#endif
#ifndef PQ_PIN_HOST_UART_RX
#define PQ_PIN_HOST_UART_RX 1
#endif
#define PQ_HOST_UART uart0

#ifndef PQ_PIN_STATUS_LED
#ifdef PICO_DEFAULT_LED_PIN
#define PQ_PIN_STATUS_LED PICO_DEFAULT_LED_PIN
#else
#define PQ_PIN_STATUS_LED 25
#endif
#endif
#ifndef PQ_PIN_ALARM
#define PQ_PIN_ALARM 8
#endif
