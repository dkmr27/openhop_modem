// =============================================================
// boards/promicro_e28.h — nRF52840 ProMicro + Ebyte E28 (SX1280)
// carrier board, 2.4 GHz only.
//
// Hardware: https://github.com/dkmr27/ProMicro-E28. Radio wiring
// matches the MeshCore fork's variants/promicro_e28 (raw nRF GPIO
// numbers here; see variants/ProMicro_nRF52840):
//   NSS P1.13, DIO1 P0.10, RESET P0.09, BUSY P0.29,
//   SCK P1.11, MISO P0.02, MOSI P1.15, RXEN P0.17, TXEN P1.06.
//   The module is powered from the ProMicro's external VCC rail
//   (P0.13), raised in initVariant(). No TCXO.
//
// The carrier takes any E28-2G4 module. The drive ceiling is set for
// the 20 dBm PA module (-2 dBm drive); the 27 dBm module needs 0 dBm and
// the 12 dBm (no PA) module up to 13 dBm. Revisit per module (plan §7).
//
// USB-CDC transport only (no Wi-Fi on the nRF52840).
//
// Keep this aggregate positional rather than C++ designated-initialized:
// the nRF52 toolchain uses GCC 7 and does not support non-trivial
// designated initializers in C++.
// =============================================================
#pragma once

#define RADIO_SX1280

inline const BoardConfig BOARD = {
    "ProMicro E28 SX1280",
    "promicro_e28",
    "promicro",   // unused — nRF52 has no Wi-Fi/mDNS

    // Radio pins.
    45,  // pin_lora_nss   P1.13
    9,   // pin_lora_rst   P0.09
    29,  // pin_lora_busy  P0.29
    10,  // pin_lora_dio1  P0.10
    43,  // pin_lora_sck   P1.11
    2,   // pin_lora_miso  P0.02
    47,  // pin_lora_mosi  P1.15

    {-1, 0, 17, 38, false},  // E28 PA/LNA: RXEN P0.17, TXEN P1.06

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

    INT8_MIN,  // max_tx_power_dbm — no sub-GHz path on an SX1280

    false,  // use_dio3_tcxo — crystal
    0.0f,   // tcxo_voltage

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

    {},  // rf_frontend — PA/LNA switched by RadioLib via RXEN/TXEN
    0,   // sx126x_agc_reset_interval_ms — SX126x only
    {},  // thermistor
    0,   // pa_ramp_time_us

    -2,     // max_tx_power_2g4_dbm — drive limit for the E28 20 dBm module
    1,      // rx_boosted_gain_level — SX1280 high-sensitivity mode on
    false,  // lr11x0_force_hp_pa — LR11x0 only
    {},     // rf_switch_table — not used (RXEN/TXEN above)
    nullptr,  // boot_config — SX1280 default preset (2483.0 MHz / 406.25 kHz / SF11)
};
