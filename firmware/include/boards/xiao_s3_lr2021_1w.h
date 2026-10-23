// =============================================================
// boards/xiao_s3_lr2021_1w.h — Seeed XIAO ESP32-S3 + NiceRF
// LoRa2021F33-2G4 (LR2021 with PA on both bands).
//
// Hardware: https://github.com/dkmr27/Xiao-LoRa2021F33-2G4. Wiring
// matches the MeshCore fork's variants/xiao_s3_LoRa2021F33-2G4:
//   D4 GPIO5  → NSS        D1 GPIO2  → IRQ (LR2021 DIO9)
//   D2 GPIO3  → RESET      D3 GPIO4  → BUSY
//   D8 GPIO7  → SCK        D9 GPIO8  → MISO      D10 GPIO9 → MOSI
//   TCXO 3.3 V. RF switch on LR2021 DIO5-DIO8 (table below).
//   GPIO21 is the XIAO's orange user LED; no user button is wired.
//
// POWER LIMITS (chip drive dBm, PA gain not included):
//   2.4 GHz: HARD LIMIT 0 dBm. 0 dBm drive gives ~30 dBm out; more drive
//            can damage the module.
//   Sub-GHz: 22 dBm ceiling; 18 dBm gives almost full output and is the
//            boot level.
//
// USB-CDC native (no USB-UART bridge), so CDC_ON_BOOT=1.
// =============================================================
#pragma once

#define RADIO_LR2021
#include <RadioLib.h>   // LR2021::MODE_* and RADIOLIB_LR2021_DIO* for the RF switch table

// Boot on the default sub-GHz preset at the module's normal 18 dBm drive.
inline const RadioConfig kXiaoS3Lr2021BootConfig = {
    869618000, 62500, 8, 8, 18, 0x12, 16,
};

inline const BoardConfig BOARD = {
    .name        = "XIAO S3 LR2021 1W",
    .fw_suffix   = "xiao_s3_lr2021_1w",
    .mdns_prefix = "xiao-lr2021",

    .pin_lora_nss  = 5,
    .pin_lora_rst  = 3,
    .pin_lora_busy = 4,
    .pin_lora_dio1 = 2,      // LR2021 DIO9 IRQ
    .pin_lora_sck  = 7,
    .pin_lora_miso = 8,
    .pin_lora_mosi = 9,

    .rf_switch = {
        .en_pin            = -1,
        .en_low_hold_ms    = 0,
        .rx_pin            = -1,     // switching is on the LR2021's DIOs
        .tx_pin            = -1,
        .dio2_as_rf_switch = false,
    },

    .pin_i2c_sda      = 43,          // D6
    .pin_i2c_scl      = 44,          // D7
    .pin_i2c_oled_rst = -1,
    .pin_vext_enable_low = -1,

    .pin_user_button        = -1,    // not wired
    .user_button_active_low = true,

    .max_tx_power_dbm = 22,          // sub-GHz drive ceiling (18 = normal drive)

    .use_dio3_tcxo = true,
    .tcxo_voltage  = 3.3f,

    .has_lora_radio = true,
    .has_wifi       = true,
    .has_network    = true,
    .pin_protocol_uart_rx = -1,
    .pin_protocol_uart_tx = -1,
    .protocol_uart_baud   = 921600,
    .ethernet = { .enabled = false },

    .max_tx_power_2g4_dbm = 0,       // HARD LIMIT — more drive damages the module
    .rx_boosted_gain_level = 0,      // off, as in the MeshCore fork
    .lr11x0_force_hp_pa = false,

    // RF switch on LR2021 DIO5-DIO8 (bit0 = DIO5 … bit3 = DIO8), as in the
    // MeshCore fork's xiao_s3_LoRa2021F33-2G4 target.cpp.
    .rf_switch_table = {
        .enabled = true,
        .pins = { RADIOLIB_LR2021_DIO5, RADIOLIB_LR2021_DIO6,
                  RADIOLIB_LR2021_DIO7, RADIOLIB_LR2021_DIO8 },
        .modes = { { LR2021::MODE_STBY,  0b0000 },
                   { LR2021::MODE_RX,    0b0000 },
                   { LR2021::MODE_TX,    0b0010 },
                   { LR2021::MODE_RX_HF, 0b0101 },
                   { LR2021::MODE_TX_HF, 0b1100 } },
        .mode_count = 5,
    },

    .boot_config = &kXiaoS3Lr2021BootConfig,
};
