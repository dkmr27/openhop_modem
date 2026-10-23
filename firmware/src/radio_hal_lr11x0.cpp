// =============================================================
// radio_hal_lr11x0.cpp — RadioHal implementation for LR1121 boards
//
// Dual-band: sub-GHz (LP/HP PA) and 2.4 GHz (HF PA). RadioLib picks
// the PA from the frequency and requested power; RadioHal's shared
// layer clamps power to each band's ceiling and re-applies it when the
// band changes. Chip workarounds follow the MeshCore fork's
// CustomLR1121 (DC-DC regulator, header-error recovery).
//
// CAD is not implemented yet for this chip (plan phase 5):
// startChannelScan() returns RADIOLIB_ERR_UNSUPPORTED and the CAD
// feature bits are clear in getFeatures().
// =============================================================
#include "radio_hal.h"

#if defined(RADIO_LR1121)

#include <Arduino.h>
#include <SPI.h>

namespace {

class OpenHopLR1121 : public LR1121 {
public:
    using LR1121::LR1121;

    int16_t startReceive() override {
        // Include PREAMBLE_DETECTED in the latched IRQs; the passive
        // reception guard (isReceivingPacket) relies on it.
        return LR1121::startReceive(
            RADIOLIB_LR11X0_RX_TIMEOUT_INF,
            RADIOLIB_IRQ_RX_DEFAULT_FLAGS |
                (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED),
            RADIOLIB_IRQ_RX_DEFAULT_MASK,
            0);
    }

    size_t getPacketLength(bool update = true) override {
        size_t len = LR1121::getPacketLength(update);
        if (len == 0 && (getIrqFlags() & RADIOLIB_LR11X0_IRQ_HEADER_ERR)) {
            // A corrupted header can leave the LR1121 shifting later
            // packets; standby returns it to a known-good state and the
            // caller's startReceive() restarts RX (MeshCore CustomLR1121).
            standby();
        }
        return len;
    }
};

OpenHopLR1121 chip = new Module(BOARD.pin_lora_nss, BOARD.pin_lora_dio1,
                                BOARD.pin_lora_rst, BOARD.pin_lora_busy);

// RadioLib keeps pointers to these, so they must outlive setRfSwitchTable().
uint32_t rfSwitchPins[Module::RFSWITCH_MAX_PINS];
Module::RfSwitchMode_t rfSwitchModes[9];

const uint32_t kSubGhzBandwidthsHz[] = { 62500, 125000, 250000, 500000 };
const uint32_t k2G4BandwidthsHz[]    = { 203125, 406250, 812500 };

// Limits match RadioLib's checks in LR1120::setFrequency /
// checkOutputPower and LR11x0::setSpreadingFactor / setCodingRate. The
// sub-GHz minimum depends on the PA: -17 dBm on LP, -9 dBm when the
// board forces HP. Only the 2.4 GHz ISM range of the HF path is listed.
RadioBandInfo kBands[] = {
    { RadioBand::SubGhz, 150000000UL, 960000000UL, -17, 22, 5, 12, 4, 8,
      kSubGhzBandwidthsHz,
      (uint8_t)(sizeof(kSubGhzBandwidthsHz) / sizeof(kSubGhzBandwidthsHz[0])) },
    { RadioBand::Band2G4, 2400000000UL, 2500000000UL, -18, 13, 5, 12, 4, 8,
      k2G4BandwidthsHz,
      (uint8_t)(sizeof(k2G4BandwidthsHz) / sizeof(k2G4BandwidthsHz[0])) },
};

}  // namespace

RadioHal radio;

// ─── Lifecycle ──────────────────────────────────────────────
float RadioHal::bootTcxoVoltage() {
    return BOARD.use_dio3_tcxo ? BOARD.tcxo_voltage : 0.0f;
}

int16_t RadioHal::begin() {
    if (BOARD.pin_lora_sck >= 0 || BOARD.pin_lora_miso >= 0 || BOARD.pin_lora_mosi >= 0) {
#ifdef ARDUINO_ARCH_ESP32
        SPI.begin(BOARD.pin_lora_sck, BOARD.pin_lora_miso,
                  BOARD.pin_lora_mosi, BOARD.pin_lora_nss);
#else
        // nRF52 BSP: the variant's default SPI pins are the radio's.
        SPI.begin();
#endif
    }

    // Start on a sub-GHz channel at -9 dBm, a level every PA accepts;
    // applyConfig() then moves to the boot config with clamped power.
    int16_t state = chip.begin(434.0f, 125.0f, 9, 7,
                               RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE,
                               -9, 8, bootTcxoVoltage());
    if (state != RADIOLIB_ERR_NONE) return state;
    // RadioLib's begin() leaves the LDO regulator selected; the LR1121
    // boards are designed for DC-DC.
    return chip.setRegulatorDCDC();
}

void RadioHal::rfSwitchConfigureRadio() {
    const RfSwitchTable& t = BOARD.rf_switch_table;
    if (t.enabled) {
        for (size_t i = 0; i < Module::RFSWITCH_MAX_PINS; ++i) {
            rfSwitchPins[i] = i < 4 ? t.pins[i] : RADIOLIB_NC;
        }
        uint8_t n = t.mode_count < 8 ? t.mode_count : 8;
        for (uint8_t m = 0; m < n; ++m) {
            rfSwitchModes[m].mode = t.modes[m].mode;
            for (size_t i = 0; i < Module::RFSWITCH_MAX_PINS; ++i) {
                rfSwitchModes[m].values[i] =
                    (i < 4 && (t.modes[m].levels & (1u << i))) ? HIGH : LOW;
            }
        }
        rfSwitchModes[n] = END_OF_MODE_TABLE;
        chip.setRfSwitchTable(rfSwitchPins, rfSwitchModes);
        Serial.printf("[INFO] setRfSwitchTable(%u modes)\n", (unsigned)n);
    }
    // RadioLib's LR11x0 has no setRfSwitchPins(); boards switch through the
    // DIO table above (MCU GPIOs can also be listed in its pins).
}

void RadioHal::configureBoardRadioOptions() {
    if (BOARD.rx_boosted_gain_level > 0) {
        int state = chip.setRxBoostedGainMode(true);
        Serial.printf("[INFO] setRxBoostedGainMode(true) -> %d\n", state);
    }
}

void RadioHal::setDio1Action(void (*func)(void)) {
    // The LR1121 raises every enabled IRQ on its DIO9 line, which the
    // board wires to pin_lora_dio1.
    chip.setPacketReceivedAction(func);
}

const char* RadioHal::getChipName() const { return "LR1121"; }

RadioConfig RadioHal::defaultConfig() {
    // Same sub-GHz preset as the SX1262 boards; the host replaces it with
    // SET_CONFIG before any TX/RX.
    RadioConfig cfg = {};
    cfg.freq_hz      = 869618000;
    cfg.bandwidth_hz = 62500;
    cfg.sf           = 8;
    cfg.cr           = 8;
    cfg.power_dbm    = 22;
    cfg.syncword     = 0x12;
    cfg.preamble_len = 16;
    return cfg;
}

// ─── Capabilities ───────────────────────────────────────────
uint8_t RadioHal::getChipId() { return RADIO_CHIP_LR1121; }

uint8_t RadioHal::getFeatures() { return RADIO_FEATURE_RX_GUARD; }

uint8_t RadioHal::getBands(const RadioBandInfo*& bands) {
    kBands[0].powerMinDbm = BOARD.lr11x0_force_hp_pa ? -9 : -17;
    bands = kBands;
    return (uint8_t)(sizeof(kBands) / sizeof(kBands[0]));
}

bool RadioHal::acceptsBandwidth(RadioBand band, uint32_t bwHz) {
    const float bw = bwHz / 1000.0f;
    if (band == RadioBand::Band2G4) {
        // LR11x0::setBandwidth(bw, high = true) matches exact values.
        return fabsf(bw - 203.125f) <= 0.001f || fabsf(bw - 406.25f) <= 0.001f ||
               fabsf(bw - 812.5f) <= 0.001f;
    }
    // Same bucket test as LR11x0::setBandwidth(bw, high = false).
    if (bw < 0.0f || bw > 510.0f) return false;
    switch ((uint8_t)(bw / 2 + 0.01f)) {
    case 31: case 62: case 125: case 250:
        return true;
    default:
        return false;
    }
}

// ─── Configuration ──────────────────────────────────────────
int16_t RadioHal::standby() { return chip.standby(); }
int16_t RadioHal::setChipFrequency(float freqMHz) {
    if (freqMHz >= 1000.0f) {
        // Image calibration only applies to sub-GHz, and RadioLib's band
        // arithmetic for it overflows a uint8_t above ~1 GHz; skip it.
        return chip.setFrequency(freqMHz, true);
    }
    if (radioBandFor(freqHz_) == RadioBand::Band2G4) {
        // After 2.4 GHz use, every sub-GHz image calibration fails
        // (IMG_CALIB_ERR, reported as SPI_CMD_FAILED) until the chip is reset,
        // even after a full calibrate(). Re-initialise from reset first; the
        // caller re-applies power and applyConfig() the remaining settings.
        int16_t state = begin();
        if (state != RADIOLIB_ERR_NONE) return state;
        rfSwitchConfigureRadio();
        configureBoardRadioOptions();
    }
    return chip.setFrequency(freqMHz);
}

int16_t RadioHal::setBandwidth(float bwKHz) {
    return chip.setBandwidth(bwKHz, radioBandFor(freqHz_) == RadioBand::Band2G4);
}

int16_t RadioHal::setSpreadingFactor(uint8_t sf) { return chip.setSpreadingFactor(sf); }
int16_t RadioHal::setCodingRate(uint8_t cr) { return chip.setCodingRate(cr); }

int16_t RadioHal::setChipOutputPower(int8_t power) {
    const uint32_t rampUs = BOARD.pa_ramp_time_us ? BOARD.pa_ramp_time_us : 48;
    return chip.setOutputPower(power, BOARD.lr11x0_force_hp_pa, rampUs);
}

float   RadioHal::getCurrentLimit() { return -1.0f; }  // no OCP readback on LR11x0
int16_t RadioHal::setSyncWord(uint8_t syncWord) { return chip.setSyncWord(syncWord); }
int16_t RadioHal::setPreambleLength(uint16_t len) { return chip.setPreambleLength(len); }
int16_t RadioHal::explicitHeader() { return chip.explicitHeader(); }
int16_t RadioHal::setCRC(uint8_t len) { return chip.setCRC(len); }
int16_t RadioHal::invertIQ(bool enable) { return chip.invertIQ(enable); }
int16_t RadioHal::autoLDRO() { return chip.autoLDRO(); }

// ─── Receive / transmit ─────────────────────────────────────
int16_t  RadioHal::startReceive() { return chip.startReceive(); }
size_t   RadioHal::getPacketLength() { return chip.getPacketLength(); }
int16_t  RadioHal::readData(uint8_t* data, size_t len) { return chip.readData(data, len); }

float RadioHal::getRSSI(bool packet) {
    // skipReceive = true: an instantaneous reading must not restart RX and
    // drop to standby, which RadioLib does by default.
    return packet ? chip.getRSSI() : chip.getRSSI(false, true);
}

float    RadioHal::getSNR() { return chip.getSNR(); }
uint32_t RadioHal::getTimeOnAir(size_t len) { return chip.getTimeOnAir(len); }
int16_t  RadioHal::startTransmit(const uint8_t* data, size_t len) { return chip.startTransmit(data, len); }
int16_t  RadioHal::finishTransmit() { return chip.finishTransmit(); }

// Same staged guard as the SX1262 build (see radio_hal_sx126x.cpp), on the
// LR11x0 IRQ bits. A header error ends the reception immediately.
bool RadioHal::isReceivingPacket() {
    uint32_t irq = chip.getIrqFlags();
    bool header   = (irq & RADIOLIB_LR11X0_IRQ_SYNC_WORD_HEADER_VALID) != 0;
    bool preamble = (irq & RADIOLIB_LR11X0_IRQ_PREAMBLE_DETECTED) != 0;
    bool hdrError = (irq & RADIOLIB_LR11X0_IRQ_HEADER_ERR) != 0;
    uint32_t now = millis();

    if (hdrError || (!header && rxHeaderSeen_)) {
        rxActivityAt_ = 0;
        rxHeaderSeen_ = false;
        return false;
    }
    if (header) {
        if (!rxHeaderSeen_) { rxHeaderSeen_ = true; rxActivityAt_ = now; }
        uint32_t maxMs = (uint32_t)(chip.getTimeOnAir(MAX_LORA_PAYLOAD) / 1000) * 3 / 2 + 100;
        if (now - rxActivityAt_ > maxMs) {
            chip.clearIrqFlags(RADIOLIB_LR11X0_IRQ_PREAMBLE_DETECTED |
                               RADIOLIB_LR11X0_IRQ_SYNC_WORD_HEADER_VALID);
            rxActivityAt_ = 0;
            rxHeaderSeen_ = false;
            return false;
        }
        return true;
    }
    if (preamble) {
        if (rxActivityAt_ == 0) rxActivityAt_ = now;
        uint32_t preMs = (uint32_t)(chip.getTimeOnAir(1) / 1000) * 3 / 2 + 100;
        if (now - rxActivityAt_ > preMs) {
            chip.clearIrqFlags(RADIOLIB_LR11X0_IRQ_PREAMBLE_DETECTED);
            rxActivityAt_ = 0;
            return false;
        }
        return true;
    }
    rxActivityAt_ = 0;
    rxHeaderSeen_ = false;
    return false;
}

// ─── Channel activity detection (phase 5) ───────────────────
int16_t RadioHal::startChannelScan() { return RADIOLIB_ERR_UNSUPPORTED; }
int16_t RadioHal::startChannelScan(const ChannelScanConfig_t&) { return RADIOLIB_ERR_UNSUPPORTED; }
RadioHal::ChannelScanState RadioHal::pollChannelScan() { return CAD_PENDING; }
int16_t RadioHal::clearChannelScanFlags() { return RADIOLIB_ERR_UNSUPPORTED; }

// ─── Maintenance ────────────────────────────────────────────
int16_t RadioHal::resetAGC() { return RADIOLIB_ERR_UNSUPPORTED; }

#endif  // RADIO_LR1121
