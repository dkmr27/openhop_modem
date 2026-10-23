// =============================================================
// boards/promicro_e80_lr1121.h — nRF52840 ProMicro + Ebyte E80
// (LR1121) carrier board.
//
// Hardware: https://github.com/dkmr27/ProMicro-E80. Radio wiring
// matches the MeshCore fork's variants/promicro_e80_lr1121 (raw nRF
// GPIO numbers here; see variants/ProMicro_nRF52840):
//   NSS P1.13, IRQ (LR1121 DIO9) P0.10, RESET P0.09, BUSY P0.29,
//   SCK P1.11, MISO P0.02, MOSI P1.15. The module is powered from
//   the ProMicro's external VCC rail (P0.13), raised in initVariant().
//
// USB-CDC transport only (no Wi-Fi on the nRF52840).
//
// Keep this aggregate positional rather than C++ designated-initialized:
// the nRF52 toolchain uses GCC 7 and does not support non-trivial
// designated initializers in C++.
// =============================================================
#pragma once

#define RADIO_LR1121
#include <RadioLib.h>   // LR11x0::MODE_* and RADIOLIB_LR11X0_DIO* for the RF switch table

inline const BoardConfig BOARD = {
    "ProMicro E80 LR1121",
    "promicro_e80_lr1121",
    "promicro",   // unused — nRF52 has no Wi-Fi/mDNS

    // Radio pins.
    45,  // pin_lora_nss   P1.13
    9,   // pin_lora_rst   P0.09
    29,  // pin_lora_busy  P0.29
    10,  // pin_lora_dio1  P0.10 (LR1121 DIO9 IRQ)
    43,  // pin_lora_sck   P1.11
    2,   // pin_lora_miso  P0.02
    47,  // pin_lora_mosi  P1.15

    {-1, 0, -1, -1, false},  // RF switch is driven by the LR1121's DIOs

    -1,    // pin_lora_tx_led
    true,  // lora_tx_led_active_high

    -1,  // pin_i2c_sda — no display in this firmware profile
    -1,  // pin_i2c_scl
    -1,  // pin_i2c_oled_rst
    -1,  // pin_vext_enable_low

    -1,    // pin_tft_sda
    -1,    // pin_tft_scl
    -1,    // pin_tft_dc
    -1,    // pin_tft_rst
    -1,    // pin_tft_cs
    -1,    // pin_tft_bl
    true,  // tft_bl_active_high
    -1,    // pin_tft_power
    true,  // tft_power_active_high

    32,    // pin_user_button  P1.00
    true,  // user_button_active_low

    {-1, -1, true, 0.0f},  // no battery sense yet

    22,  // max_tx_power_dbm — sub-GHz drive ceiling

    true,  // use_dio3_tcxo — TCXO on the E80
    1.8f,  // tcxo_voltage

    -1,     // sx126x_current_limit_ma — SX126x only
    false,  // sx126x_rx_boosted_gain — SX126x only
    false,  // sx126x_register_patch — SX126x only

    true,   // has_lora_radio
    false,  // has_wifi
    {},     // wifi_antenna_switch
    false,  // has_network

    -1,      // pin_protocol_uart_rx — USB-CDC only
    -1,      // pin_protocol_uart_tx
    921600,  // protocol_uart_baud

    -1,    // pin_gps_uart_rx
    -1,    // pin_gps_uart_tx
    9600,  // gps_uart_baud
    -1,    // pin_gps_enable
    true,  // gps_enable_active_high
    -1,    // pin_gps_reset
    false, // gps_reset_active_high
    true,  // gps_send_casic_config

    {false, BoardConfig::EthernetPhy::NONE, -1, -1, -1, -1, false, false,
     {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}},

    {},  // pmu — no PMU chip on this board

    {{-1, false}, {-1, false}, {-1, false}, {-1, false}},
    0,

    {},  // rf_frontend — no external PA/LNA controls
    0,   // sx126x_agc_reset_interval_ms — SX126x only
    {},  // thermistor
    0,   // pa_ramp_time_us — RadioLib default (48 us)

    13,     // max_tx_power_2g4_dbm — LR1121 HF PA maximum, no 2.4 GHz PA
    1,      // rx_boosted_gain_level — on
    false,  // lr11x0_force_hp_pa — LP PA below 15 dBm, HP above

    // RF switch on LR1121 DIO5/DIO6 (bit0 = DIO5, bit1 = DIO6), as in the
    // MeshCore fork's promicro_e80_lr1121 target.cpp.
    {true,
     {RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6, RF_SWITCH_PIN_NC, RF_SWITCH_PIN_NC},
     {{LR11x0::MODE_STBY,  0b00},
      {LR11x0::MODE_RX,    0b10},
      {LR11x0::MODE_TX,    0b11},
      {LR11x0::MODE_TX_HP, 0b01},
      {LR11x0::MODE_TX_HF, 0b00},
      {LR11x0::MODE_GNSS,  0b00},
      {LR11x0::MODE_WIFI,  0b00}},
     7},

    nullptr,  // boot_config — LR1121 default preset
};
