// =============================================================
// radio_hal_lr2021.cpp — RadioHal implementation for LR2021 boards
//
// Dual-band: sub-GHz (LF path) and 2.4 GHz (HF path). RadioLib picks
// the PA from the frequency; RadioHal's shared layer clamps power to
// each band's ceiling and re-applies it when the band changes. Chip
// details follow the MeshCore fork's CustomLR2021 (IRQ on DIO9, TCXO
// fallback, boosted gain only in standby).
//
// CAD is not implemented yet for this chip (plan phase 5):
// startChannelScan() returns RADIOLIB_ERR_UNSUPPORTED and the CAD
// feature bits are clear in getFeatures().
// =============================================================
#include "radio_hal.h"

#if defined(RADIO_LR2021)

#include <Arduino.h>
#include <SPI.h>

namespace {

class OpenHopLR2021 : public LR2021 {
public:
    explicit OpenHopLR2021(Module* mod) : LR2021(mod) {
        // The boards route the LR2021's IRQ from DIO9 to pin_lora_dio1.
        irqDioNum = 9;
    }

    int16_t startReceive() override {
        // Include PREAMBLE_DETECTED in the latched IRQs; the passive
        // reception guard (isReceivingPacket) relies on it.
        return LR2021::startReceive(
            RADIOLIB_LR2021_RX_TIMEOUT_INF,
            RADIOLIB_IRQ_RX_DEFAULT_FLAGS |
                (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED),
            RADIOLIB_IRQ_RX_DEFAULT_MASK,
            0);
    }
};

OpenHopLR2021 chip(new Module(BOARD.pin_lora_nss, BOARD.pin_lora_dio1,
                              BOARD.pin_lora_rst, BOARD.pin_lora_busy));

// RadioLib keeps pointers to these, so they must outlive setRfSwitchTable().
uint32_t rfSwitchPins[Module::RFSWITCH_MAX_PINS];
Module::RfSwitchMode_t rfSwitchModes[9];

const uint32_t kSubGhzBandwidthsHz[] = {
    31250, 41667, 62500, 83333, 125000, 250000, 500000,
};
const uint32_t k2G4BandwidthsHz[] = { 203125, 406250, 812500, 1000000 };

// Limits match RadioLib's checks in LR2021::setFrequency,
// checkOutputPower, setSpreadingFactor and setCodingRate. Only the
// 2.4 GHz ISM range of the HF path is listed.
const RadioBandInfo kBands[] = {
    { RadioBand::SubGhz, 150000000UL, 1090000000UL, -9, 22, 5, 12, 4, 8,
      kSubGhzBandwidthsHz,
      (uint8_t)(sizeof(kSubGhzBandwidthsHz) / sizeof(kSubGhzBandwidthsHz[0])) },
    { RadioBand::Band2G4, 2400000000UL, 2500000000UL, -19, 12, 5, 12, 4, 8,
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
    // Start on a sub-GHz channel at -9 dBm (the LF PA minimum); applyConfig()
    // then moves to the boot config with clamped power.
    int16_t state = chip.begin(434.0f, 125.0f, 9, 7,
                               RADIOLIB_LR2021_LORA_SYNC_WORD_PRIVATE,
                               -9, 8, bootTcxoVoltage());
    if (state == RADIOLIB_ERR_SPI_CMD_FAILED || state == RADIOLIB_ERR_SPI_CMD_INVALID) {
        // Some modules have a crystal rather than a TCXO; retry without one
        // (MeshCore CustomLR2021).
        Serial.printf("[INFO] LR2021 begin with TCXO failed (%d), retrying without\n", state);
        state = chip.begin(434.0f, 125.0f, 9, 7,
                           RADIOLIB_LR2021_LORA_SYNC_WORD_PRIVATE, -9, 8, 0.0f);
    }
    return state;
}

void RadioHal::rfSwitchConfigureRadio() {
    const RfSwitchTable& t = BOARD.rf_switch_table;
    if (!t.enabled) return;
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

void RadioHal::configureBoardRadioOptions() {
    if (BOARD.rx_boosted_gain_level > 0) {
        chip.standby();  // LR2021 only accepts the RX gain mode in standby
        int state = chip.setRxBoostedGainMode(BOARD.rx_boosted_gain_level);
        Serial.printf("[INFO] setRxBoostedGainMode(%u) -> %d\n",
                      (unsigned)BOARD.rx_boosted_gain_level, state);
    }
}

void RadioHal::setDio1Action(void (*func)(void)) { chip.setPacketReceivedAction(func); }

const char* RadioHal::getChipName() const { return "LR2021"; }

RadioConfig RadioHal::defaultConfig() {
    // Same sub-GHz preset as the SX1262 boards; the host replaces it with
    // SET_CONFIG before any TX/RX. Boards with an external PA set a gentler
    // boot_config.
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
uint8_t RadioHal::getChipId() { return RADIO_CHIP_LR2021; }

uint8_t RadioHal::getFeatures() { return RADIO_FEATURE_RX_GUARD; }

uint8_t RadioHal::getBands(const RadioBandInfo*& bands) {
    bands = kBands;
    return (uint8_t)(sizeof(kBands) / sizeof(kBands[0]));
}

bool RadioHal::acceptsBandwidth(RadioBand, uint32_t bwHz) {
    // Same bucket test as LR2021::setBandwidth(), which takes the same
    // set on either path.
    const float bw = bwHz / 1000.0f;
    if (bw < 0.0f || bw > 1010.0f) return false;
    switch ((int)(bw / 2 + 0.01f)) {
    case 15: case 20: case 31: case 41: case 50: case 62:
    case 101: case 125: case 203: case 250: case 406: case 500:
        return true;
    default:
        return false;
    }
}

bool RadioHal::acceptsPreamble(uint16_t) { return true; }  // any length

// ─── Configuration ──────────────────────────────────────────
int16_t RadioHal::standby() { return chip.standby(); }

int16_t RadioHal::setChipFrequency(float freqMHz) {
    const bool bandChange = radioBandFor(freqHz_) != radioBandFor((uint32_t)(freqMHz * 1e6f));
    if (bandChange) {
        // The LR2021 validates SetPaConfig against the current TX power, so
        // switching PAs from e.g. 14+ dBm on the LF PA (HF max is 12) or from
        // below -9 dBm on the HF PA (LF min is -9) is rejected with
        // SPI_CMD_INVALID and RadioLib never applies the new power. Park the
        // power at -9 dBm, valid on both PAs, before changing band; the
        // caller then applies the clamped power for the new band.
        const uint32_t rampUs = BOARD.pa_ramp_time_us ? BOARD.pa_ramp_time_us : 48;
        int16_t parked = chip.setOutputPower(-9, rampUs);
        if (parked != RADIOLIB_ERR_NONE) return parked;
    }
    int16_t state = chip.setFrequency(freqMHz);
    if (state == RADIOLIB_ERR_NONE && bandChange && BOARD.rx_boosted_gain_level > 0) {
        // The gain mode is kept per RX path (LF/HF); set it for the new one.
        state = chip.setRxBoostedGainMode(BOARD.rx_boosted_gain_level);
    }
    return state;
}

int16_t RadioHal::setBandwidth(float bwKHz) { return chip.setBandwidth(bwKHz); }
int16_t RadioHal::setSpreadingFactor(uint8_t sf) { return chip.setSpreadingFactor(sf); }
int16_t RadioHal::setCodingRate(uint8_t cr) { return chip.setCodingRate(cr); }

int16_t RadioHal::setChipOutputPower(int8_t power) {
    const uint32_t rampUs = BOARD.pa_ramp_time_us ? BOARD.pa_ramp_time_us : 48;
    return chip.setOutputPower(power, rampUs);
}

float   RadioHal::getCurrentLimit() { return -1.0f; }  // no OCP readback on LR2021
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
// LR2021 IRQ bits. A header CRC error ends the reception immediately.
bool RadioHal::isReceivingPacket() {
    uint32_t irq = chip.getIrqFlags();
    bool header   = (irq & RADIOLIB_LR2021_IRQ_LORA_HEADER_VALID) != 0;
    bool preamble = (irq & RADIOLIB_LR2021_IRQ_PREAMBLE_DETECTED) != 0;
    bool hdrError = (irq & RADIOLIB_LR2021_IRQ_LORA_HDR_CRC_ERROR) != 0;
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
            chip.clearIrqFlags(RADIOLIB_LR2021_IRQ_PREAMBLE_DETECTED |
                               RADIOLIB_LR2021_IRQ_LORA_HEADER_VALID);
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
            chip.clearIrqFlags(RADIOLIB_LR2021_IRQ_PREAMBLE_DETECTED);
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

#endif  // RADIO_LR2021
