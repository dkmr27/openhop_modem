// =============================================================
// radio_hal_common.cpp — RadioHal logic shared by every chip
//
// Band-aware power limits and config validation. The chip files supply
// the band table (getBands), bandwidth acceptance and the raw frequency
// and power calls.
// =============================================================
#include "radio_hal.h"

int8_t RadioHal::boardCeiling(RadioBand band) {
    return band == RadioBand::Band2G4 ? BOARD.max_tx_power_2g4_dbm
                                      : BOARD.max_tx_power_dbm;
}

int8_t RadioHal::getMaxOutputPower(uint32_t freqHz) {
    const RadioBand band = radioBandFor(freqHz);
    const RadioBandInfo* bands = nullptr;
    const uint8_t count = getBands(bands);
    for (uint8_t i = 0; i < count; ++i) {
        if (bands[i].band == band) {
            return radioPowerCeiling(bands[i].powerMaxDbm, boardCeiling(band));
        }
    }
    return RADIO_POWER_NONE;
}

bool RadioHal::isConfigSupported(const RadioConfig& cfg) {
    const RadioBand band = radioBandFor(cfg.freq_hz);
    const RadioBandInfo* bands = nullptr;
    const uint8_t count = getBands(bands);
    const RadioBandInfo* info = nullptr;
    for (uint8_t i = 0; i < count; ++i) {
        if (bands[i].band == band) info = &bands[i];
    }
    if (info == nullptr) return false;
    if (radioPowerCeiling(info->powerMaxDbm, boardCeiling(band)) == RADIO_POWER_NONE) return false;
    if (cfg.freq_hz < info->freqMinHz || cfg.freq_hz > info->freqMaxHz) return false;
    if (cfg.sf < info->sfMin || cfg.sf > info->sfMax) return false;
    if (cfg.cr < info->crMin || cfg.cr > info->crMax) return false;
    if (!acceptsBandwidth(band, cfg.bandwidth_hz)) return false;
    if (!acceptsPreamble(cfg.preamble_len)) return false;
    if (cfg.power_dbm < info->powerMinDbm) return false;
    return true;
}

int16_t RadioHal::setFrequency(float freqMHz) {
    const int16_t state = setChipFrequency(freqMHz);
    if (state != RADIOLIB_ERR_NONE) return state;
    const uint32_t newHz = (uint32_t)(freqMHz * 1000000.0f + 0.5f);
    const bool bandChanged = radioBandFor(newHz) != radioBandFor(freqHz_);
    freqHz_ = newHz;
    // The drive level is only valid for the band it was clamped for. On a
    // band change, re-clamp before anything can transmit.
    if (bandChanged && powerRequested_) return applyOutputPower();
    return RADIOLIB_ERR_NONE;
}

int16_t RadioHal::setOutputPower(int8_t power) {
    requestedPowerDbm_ = power;
    powerRequested_ = true;
    return applyOutputPower();
}

int16_t RadioHal::applyOutputPower() {
    const int8_t ceiling = getMaxOutputPower(freqHz_);
    if (ceiling == RADIO_POWER_NONE) return RADIOLIB_ERR_INVALID_OUTPUT_POWER;
    const int8_t power = radioClampPower(requestedPowerDbm_, ceiling);
    const int16_t state = setChipOutputPower(power);
    if (state == RADIOLIB_ERR_NONE) appliedPowerDbm_ = power;
    return state;
}
