// =============================================================
// variants/ProMicro_nRF52840/variant.h — Adafruit nRF52 BSP pin
// map for nRF52840 ProMicro (nice!nano-compatible) carrier boards.
//
// Identity pin map in variant.cpp, as for the T114 and XIAO
// variants: Arduino pin N is raw nRF52 GPIO N (P0.X = X, P1.X =
// 32 + X), so boards/<name>.h uses raw GPIO numbers. Pin choices
// follow the ProMicro carrier boards in the MeshCore fork
// (variants/promicro_e28, promicro_e80_lr1121), which share one
// radio wiring.
// =============================================================
#pragma once

#include "WVariant.h"

// 32.768 kHz XTAL on the ProMicro.
#define USE_LFXO
#define VARIANT_MCK             (64000000ul)

// External VCC enable (P0.13). Powers the radio module on the
// ProMicro carrier boards; initVariant() drives it HIGH at boot.
#define PIN_EXT_VCC             (13)

// Battery sense (P0.31 / AIN7) through the carrier's divider.
#define BATTERY_PIN             (31)
#define ADC_RESOLUTION          (12)

// Identity-mapped pin space. P0.00 / P0.01 (XTAL) are unmapped.
#define PINS_COUNT              (48)
#define NUM_DIGITAL_PINS        (48)
#define NUM_ANALOG_INPUTS       (1)
#define NUM_ANALOG_OUTPUTS      (0)

// UART1 (Serial1): ProMicro D1/D0 = P0.06 TX / P0.08 RX.
#define PIN_SERIAL1_TX          (6)
#define PIN_SERIAL1_RX          (8)

// UART2 (Serial2) = openHop's optional protocol UART. Same D1/D0 pins;
// only opened when the board header sets pin_protocol_uart_rx/tx.
#define PIN_SERIAL2_TX          (6)
#define PIN_SERIAL2_RX          (8)

// I2C: P1.04 SDA / P0.11 SCL, as the MeshCore carriers use.
#define WIRE_INTERFACES_COUNT   (1)
#define PIN_WIRE_SDA            (36)
#define PIN_WIRE_SCL            (11)

// Default SPI bus = the radio module: SCK P1.11, MISO P0.02,
// MOSI P1.15, NSS P1.13. openHop's nRF52 radio code calls plain
// SPI.begin(), so the default bus must be the radio's pins.
#define SPI_INTERFACES_COUNT    (1)
#define PIN_SPI_SCK             (43)
#define PIN_SPI_MISO            (2)
#define PIN_SPI_MOSI            (47)
#define PIN_SPI_NSS             (45)

// Blue LED on P0.15 (active high).
#define PIN_LED                 (15)
#define LED_BUILTIN             PIN_LED
#define LED_PIN                 PIN_LED
#define LED_BLUE                PIN_LED
#define LED_RED                 (-1)
#define LED_STATE_ON            HIGH

// User button on P1.00 (active low).
#define PIN_BUTTON1             (32)
#define BUTTON_PIN              PIN_BUTTON1
