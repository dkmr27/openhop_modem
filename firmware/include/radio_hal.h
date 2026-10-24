// =============================================================
// radio_hal.h — LoRa radio hardware abstraction
//
// One class, `RadioHal`, with one implementation per radio chip.
// Exactly one implementation is compiled per build, selected by the
// RADIO_<chip> macro the board header declares (board_config.h defaults
// it to RADIO_SX1262).
//
// Method names deliberately mirror RadioLib (and the calls main.cpp
// made before this layer existed): setFrequency, startReceive,
// readData, getRSSI, resetAGC, ... Return codes are RadioLib's
// int16_t status codes (RADIOLIB_ERR_NONE on success), and chips
// without a feature return RADIOLIB_ERR_UNSUPPORTED.
//
// Nothing outside the radio_hal_<chip>.cpp files should include chip
// headers or use chip-specific RadioLib constants (RADIOLIB_SX126X_*).
// =============================================================
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <RadioLib.h>
#include "board_config.h"   // board header first: it picks RADIO_<chip>
#include "radio_band.h"

static_assert(RF_SWITCH_PIN_NC == RADIOLIB_NC,
              "RF_SWITCH_PIN_NC must match RadioLib's not-connected pin value");

class RadioHal {
public:
    // Result of polling a channel scan (CAD) that startChannelScan() began.
    enum ChannelScanState : uint8_t {
        CAD_PENDING,    // scan still running (CAD_DONE not yet latched)
        CAD_CLEAR,      // scan finished, no LoRa activity detected
        CAD_DETECTED,   // scan finished, activity detected
    };

    // ─── Lifecycle ──────────────────────────────────────────
    // Brings up the radio's SPI bus (when the board remaps it) and
    // initialises the chip, including the board's TCXO voltage.
    int16_t begin();
    // TCXO voltage begin() uses: the board's value, or 0 for a crystal.
    static float bootTcxoVoltage();
    // RF switch wiring from BOARD.rf_switch (DIO2 and/or RXEN/TXEN GPIOs).
    void rfSwitchConfigureRadio();
    // Optional board tuning from BOARD (current limit, boosted RX gain,
    // chip register patches). Logs each step on Serial.
    void configureBoardRadioOptions();
    void setDio1Action(void (*func)(void));
    const char* getChipName() const;
    // The chip's boot preset, used when BOARD.boot_config is nullptr.
    static RadioConfig defaultConfig();
    // The settings applied at boot: BOARD.boot_config, else defaultConfig().
    static RadioConfig bootConfig();

    // ─── Capabilities ───────────────────────────────────────
    static uint8_t getChipId();       // RadioChipId
    static uint8_t getFeatures();     // RadioFeature bits
    // The chip's band table (what the chip can do, before board limits).
    static uint8_t getBands(const RadioBandInfo*& bands);
    // The board's drive ceiling for a band; RADIO_POWER_NONE = band unusable.
    static int8_t boardCeiling(RadioBand band);
    // Effective drive ceiling at a frequency: min(chip, board), or
    // RADIO_POWER_NONE if the chip or board can't use that band.
    static int8_t getMaxOutputPower(uint32_t freqHz);
    // True if every field of `cfg` is valid for this chip and board, so
    // applying it can't leave the radio half-configured. Power above the
    // ceiling is allowed (it is clamped); below the chip minimum is not.
    static bool isConfigSupported(const RadioConfig& cfg);

    // ─── Configuration ──────────────────────────────────────
    int16_t standby();
    // When the frequency moves to another band, re-applies the requested
    // power under that band's ceiling.
    int16_t setFrequency(float freqMHz);
    int16_t setBandwidth(float bwKHz);
    int16_t setSpreadingFactor(uint8_t sf);
    int16_t setCodingRate(uint8_t cr);
    // Requests a drive level. It is clamped to the current band's ceiling
    // (getMaxOutputPower) and then the board's PA ramp time is applied.
    int16_t setOutputPower(int8_t power);
    // Drive level actually set by the last successful setOutputPower().
    int8_t  getAppliedOutputPower() const { return appliedPowerDbm_; }
    float   getCurrentLimit();
    int16_t setSyncWord(uint8_t syncWord);
    int16_t setPreambleLength(uint16_t preambleLength);
    int16_t explicitHeader();
    int16_t setCRC(uint8_t len);
    int16_t invertIQ(bool enable);
    int16_t autoLDRO();

    // ─── Receive / transmit ─────────────────────────────────
    // Continuous RX with PREAMBLE_DETECTED latched, which
    // isReceivingPacket() relies on.
    int16_t  startReceive();
    size_t   getPacketLength();
    int16_t  readData(uint8_t* data, size_t len);
    // packet = true: RSSI of the last packet; false: instantaneous RSSI.
    float    getRSSI(bool packet = true);
    float    getSNR();
    uint32_t getTimeOnAir(size_t len);
    int16_t  startTransmit(const uint8_t* data, size_t len);
    int16_t  finishTransmit();

    // True while the chip reports a reception in progress (preamble or
    // valid header latched). Each stage has a staleness bound so a stray
    // flag cannot hold TX off forever.
    bool isReceivingPacket();

    // ─── Channel activity detection ─────────────────────────
    // CAD parameters are in SX126x terms on every chip, as the host sends
    // them in CMD_SET_CAD_PARAMS: symNum is a symbol code (0-4 = 1, 2, 4,
    // 8 or 16 symbols), then detPeak, detMin and exit mode. Each chip maps
    // them to its own CAD command. normalizeCadParams() rewrites them to
    // what the chip actually applies, still in those terms, with 0 for a
    // field the chip replaces with its own per-SF default.
    static void normalizeCadParams(uint8_t& symCode, uint8_t& detPeak,
                                   uint8_t& detMin, uint8_t& exitMode);
    int16_t startChannelScan();
    int16_t startChannelScan(const ChannelScanConfig_t& config);
    // One IRQ status read; the CAD flags stay latched until cleared.
    ChannelScanState pollChannelScan();
    int16_t clearChannelScanFlags();

    // ─── Maintenance ────────────────────────────────────────
    // Warm-sleep AGC recalibration. Call from standby.
    int16_t resetAGC();

private:
    // Chip-specific pieces behind the shared logic in radio_hal_common.cpp.
    static bool acceptsBandwidth(RadioBand band, uint32_t bwHz);
    static bool acceptsPreamble(uint16_t len);
    int16_t setChipFrequency(float freqMHz);
    int16_t setChipOutputPower(int8_t power);
    int16_t applyOutputPower();

    uint32_t rxActivityAt_ = 0;
    bool     rxHeaderSeen_ = false;
    uint32_t freqHz_ = 0;
    int8_t   requestedPowerDbm_ = 0;
    int8_t   appliedPowerDbm_ = 0;
    bool     powerRequested_ = false;
};

extern RadioHal radio;

// SX126x CAD symbol code (0-4) to a symbol count; larger codes clamp to 16.
inline uint8_t cadSymbolsFromCode(uint8_t code) { return (uint8_t)(1u << (code > 4 ? 4 : code)); }

inline RadioConfig RadioHal::bootConfig() {
    return BOARD.boot_config ? *BOARD.boot_config : defaultConfig();
}
