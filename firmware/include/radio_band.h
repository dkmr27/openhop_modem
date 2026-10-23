// =============================================================
// radio_band.h — Chip-independent band, power and radio-info helpers
//
// Pure functions with no Arduino or RadioLib dependency, so the power
// clamp and the CMD_RADIO_INFO_RESP encoding can be unit-tested on a
// host. RadioHal supplies the per-chip band tables.
// =============================================================
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

enum class RadioBand : uint8_t {
    SubGhz  = 0,
    Band2G4 = 1,
};

// Anything at or above 1 GHz is treated as the 2.4 GHz band.
constexpr uint32_t RADIO_BAND_2G4_MIN_HZ = 1000000000UL;

// Power value meaning "this band is not usable on this board".
constexpr int8_t RADIO_POWER_NONE = INT8_MIN;

inline RadioBand radioBandFor(uint32_t freqHz) {
    return freqHz >= RADIO_BAND_2G4_MIN_HZ ? RadioBand::Band2G4 : RadioBand::SubGhz;
}

// Short identifier used in JSON ("subghz" / "2g4").
inline const char* radioBandName(RadioBand band) {
    return band == RadioBand::Band2G4 ? "2g4" : "subghz";
}

// Highest drive level allowed on a band: the lower of the chip's limit and
// the board's ceiling. A board ceiling of RADIO_POWER_NONE disables the band.
inline int8_t radioPowerCeiling(int8_t chipMaxDbm, int8_t boardCeilingDbm) {
    if (boardCeilingDbm == RADIO_POWER_NONE) return RADIO_POWER_NONE;
    return boardCeilingDbm < chipMaxDbm ? boardCeilingDbm : chipMaxDbm;
}

inline int8_t radioClampPower(int8_t requestedDbm, int8_t ceilingDbm) {
    return requestedDbm > ceilingDbm ? ceilingDbm : requestedDbm;
}

// What a chip can do on one band. Board ceilings are applied separately.
struct RadioBandInfo {
    RadioBand band;
    uint32_t freqMinHz;
    uint32_t freqMaxHz;
    int8_t powerMinDbm;         // chip drive range on this band
    int8_t powerMaxDbm;
    uint8_t sfMin;
    uint8_t sfMax;
    uint8_t crMin;              // coding-rate denominator, e.g. 5 = 4/5
    uint8_t crMax;
    const uint32_t* bandwidthsHz;   // standard bandwidths, for reporting
    uint8_t bandwidthCount;
};

// Chip identifiers reported in CMD_RADIO_INFO_RESP.
enum RadioChipId : uint8_t {
    RADIO_CHIP_SX1262 = 1,
    RADIO_CHIP_LR1121 = 2,
    RADIO_CHIP_SX1280 = 3,
    RADIO_CHIP_LR2021 = 4,
};

// Feature bits reported in CMD_RADIO_INFO_RESP.
enum RadioFeature : uint8_t {
    RADIO_FEATURE_CAD         = 1 << 0,   // CMD_CAD_REQUEST / auto-CAD work
    RADIO_FEATURE_CAD_TUNING  = 1 << 1,   // CMD_SET_CAD_PARAMS peak/min apply
    RADIO_FEATURE_RX_GUARD    = 1 << 2,   // TX defers to a reception in progress
    RADIO_FEATURE_AGC_RESET   = 1 << 3,   // warm-sleep AGC recalibration
};

constexpr uint8_t RADIO_INFO_VERSION = 1;

// Encodes a CMD_RADIO_INFO_RESP payload (layout documented in protocol.h).
// `ceilingsDbm[i]` is the effective power ceiling for bands[i] (see
// radioPowerCeiling); bands whose ceiling is RADIO_POWER_NONE are left out.
// Returns the payload length, or 0 if it doesn't fit in `cap`.
inline size_t encodeRadioInfo(uint8_t* out, size_t cap, uint8_t chipId,
                              uint8_t features, RadioBand currentBand,
                              int8_t appliedPowerDbm,
                              const RadioBandInfo* bands,
                              const int8_t* ceilingsDbm, uint8_t bandCount,
                              const char* chipName) {
    size_t n = 0;
    bool ok = true;
    auto put8 = [&](uint8_t v) {
        if (n + 1 > cap) { ok = false; return; }
        out[n++] = v;
    };
    auto put32 = [&](uint32_t v) {
        for (int i = 0; i < 4; ++i) put8((uint8_t)(v >> (8 * i)));
    };

    uint8_t usable = 0;
    for (uint8_t i = 0; i < bandCount; ++i) {
        if (ceilingsDbm[i] != RADIO_POWER_NONE) ++usable;
    }

    put8(RADIO_INFO_VERSION);
    put8(chipId);
    put8(features);
    put8((uint8_t)currentBand);
    put8((uint8_t)appliedPowerDbm);
    put8(usable);
    for (uint8_t i = 0; i < bandCount; ++i) {
        if (ceilingsDbm[i] == RADIO_POWER_NONE) continue;
        const RadioBandInfo& b = bands[i];
        put8((uint8_t)b.band);
        put32(b.freqMinHz);
        put32(b.freqMaxHz);
        put8((uint8_t)b.powerMinDbm);
        put8((uint8_t)ceilingsDbm[i]);
        put8(b.sfMin);
        put8(b.sfMax);
        put8(b.crMin);
        put8(b.crMax);
        put8(b.bandwidthCount);
        for (uint8_t k = 0; k < b.bandwidthCount; ++k) put32(b.bandwidthsHz[k]);
    }
    size_t nameLen = chipName ? strlen(chipName) : 0;
    if (nameLen > 32) nameLen = 32;
    put8((uint8_t)nameLen);
    for (size_t i = 0; i < nameLen; ++i) put8((uint8_t)chipName[i]);
    return ok ? n : 0;
}
