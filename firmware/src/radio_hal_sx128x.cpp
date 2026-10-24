// =============================================================
// radio_hal_sx128x.cpp — RadioHal implementation for SX1280 boards
//
// 2.4 GHz only. External PA modules (e.g. Ebyte E28) switch through
// RXEN/TXEN GPIOs from BOARD.rf_switch. No TCXO.
// =============================================================
#include "radio_hal.h"

#if defined(RADIO_SX1280)

#include <Arduino.h>
#include <SPI.h>

namespace {

class OpenHopSX1280 : public SX1280 {
public:
    using SX1280::SX1280;

    RadioLibTime_t getTimeOnAir(size_t len) override {
        if (getPacketType() != RADIOLIB_SX128X_PACKET_TYPE_LORA) {
            return SX1280::getTimeOnAir(len);
        }
        // RadioLib's SX128x time on air uses a payload numerator 16 bits
        // smaller than the standard Semtech LoRa formula, which under-reports
        // by one coding block (e.g. 495 ms vs ~520 ms measured for 64 B at
        // 2400.5 MHz / 406.25 kHz / SF11 / CR 4/5). Measured TX round trips
        // match the standard formula with LDRO on for SF > 10, the same
        // figure RadioLib's LR11x0 reports at 2.4 GHz, so TX_DONE uses that.
        const uint8_t sf = this->spreadingFactor >> 4;
        uint8_t cr = this->codingRateLoRa;   // 1..4 = 4/5..4/8 (5..7 = long interleave)
        if (cr == 0 || cr > 4) cr = (cr == 7) ? 4 : (cr == 0 ? 1 : cr - 4);
        const bool crc = this->crcLoRa == RADIOLIB_SX128X_LORA_CRC_ON;
        const bool implicitHdr = this->headerType == RADIOLIB_SX128X_LORA_HEADER_IMPLICIT;
        const uint32_t preamble = (this->preambleLengthLoRa & 0x0F) *
                                  (uint32_t(1) << ((this->preambleLengthLoRa & 0xF0) >> 4));
        const int de = sf > 10 ? 1 : 0;
        int32_t num = 8 * (int32_t)len - 4 * sf + 28 + (crc ? 16 : 0) - (implicitHdr ? 20 : 0);
        if (sf < 7) num += 8;
        const int32_t den = 4 * (sf - 2 * de);
        int32_t blocks = num > 0 ? (num + den - 1) / den : 0;
        const float nSymbols = (float)preamble + (sf < 7 ? 6.25f : 4.25f) + 8.0f +
                               (float)(blocks * (cr + 4));
        const float tSymUs = (float)(uint32_t(1) << sf) * 1000.0f / this->bandwidthKhz;
        return (RadioLibTime_t)(nSymbols * tSymUs);
    }
};

OpenHopSX1280 chip = new Module(BOARD.pin_lora_nss, BOARD.pin_lora_dio1,
                                BOARD.pin_lora_rst, BOARD.pin_lora_busy);

// RadioLib keeps pointers to these, so they must outlive setRfSwitchTable().
uint32_t rfSwitchPins[Module::RFSWITCH_MAX_PINS];
Module::RfSwitchMode_t rfSwitchModes[9];

const uint32_t kBandwidthsHz[] = { 203125, 406250, 812500, 1625000 };

// Limits match RadioLib's checks in SX128x::setFrequency, setOutputPower,
// setSpreadingFactor and setCodingRate (LoRa).
const RadioBandInfo kBands[] = {
    { RadioBand::Band2G4, 2400000000UL, 2500000000UL, -18, 13, 5, 12, 4, 8,
      kBandwidthsHz,
      (uint8_t)(sizeof(kBandwidthsHz) / sizeof(kBandwidthsHz[0])) },
};

}  // namespace

RadioHal radio;

// ─── Lifecycle ──────────────────────────────────────────────
float RadioHal::bootTcxoVoltage() { return 0.0f; }  // SX1280 boards use a crystal

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
    // Start at -9 dBm; applyConfig() then applies the boot config with
    // power clamped to the board's 2.4 GHz ceiling.
    return chip.begin(2440.0f, 812.5f, 9, 7, RADIOLIB_SX128X_SYNC_WORD_PRIVATE,
                      -9, 12);
}

void RadioHal::rfSwitchConfigureRadio() {
    if (BOARD.rf_switch.rx_pin >= 0 || BOARD.rf_switch.tx_pin >= 0) {
        uint32_t rx = BOARD.rf_switch.rx_pin >= 0
                          ? (uint32_t)BOARD.rf_switch.rx_pin : RADIOLIB_NC;
        uint32_t tx = BOARD.rf_switch.tx_pin >= 0
                          ? (uint32_t)BOARD.rf_switch.tx_pin : RADIOLIB_NC;
        chip.setRfSwitchPins(rx, tx);
        Serial.printf("[INFO] setRfSwitchPins(rx=%lu tx=%lu)\n",
                      (unsigned long)rx, (unsigned long)tx);
    }
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
}

void RadioHal::configureBoardRadioOptions() {
    if (BOARD.rx_boosted_gain_level > 0) {
        int state = chip.setHighSensitivityMode(true);
        Serial.printf("[INFO] setHighSensitivityMode(true) -> %d\n", state);
    }
}

void RadioHal::setDio1Action(void (*func)(void)) { chip.setDio1Action(func); }

const char* RadioHal::getChipName() const { return "SX1280"; }

RadioConfig RadioHal::defaultConfig() {
    // 2.4 GHz boot preset; the host replaces it with SET_CONFIG before any
    // TX/RX. Power is clamped to the board's 2.4 GHz ceiling regardless.
    RadioConfig cfg = {};
    cfg.freq_hz      = 2483000000UL;
    cfg.bandwidth_hz = 406250;
    cfg.sf           = 11;
    cfg.cr           = 5;
    cfg.power_dbm    = -2;
    cfg.syncword     = 0x12;
    cfg.preamble_len = 16;
    return cfg;
}

// ─── Capabilities ───────────────────────────────────────────
uint8_t RadioHal::getChipId() { return RADIO_CHIP_SX1280; }

uint8_t RadioHal::getFeatures() { return RADIO_FEATURE_CAD | RADIO_FEATURE_RX_GUARD; }

uint8_t RadioHal::getBands(const RadioBandInfo*& bands) {
    bands = kBands;
    return (uint8_t)(sizeof(kBands) / sizeof(kBands[0]));
}

bool RadioHal::acceptsBandwidth(RadioBand band, uint32_t bwHz) {
    if (band != RadioBand::Band2G4) return false;
    // SX128x::setBandwidth() matches exact values.
    const float bw = bwHz / 1000.0f;
    return fabsf(bw - 203.125f) <= 0.001f || fabsf(bw - 406.25f) <= 0.001f ||
           fabsf(bw - 812.5f) <= 0.001f || fabsf(bw - 1625.0f) <= 0.001f;
}

bool RadioHal::acceptsPreamble(uint16_t len) {
    // SX128x::setPreambleLength() rejects odd lengths (LoRa) and rounds even
    // ones up to the next mantissa * 2^exponent value.
    return len >= 2 && (len % 2) == 0;
}

// ─── Configuration ──────────────────────────────────────────
int16_t RadioHal::standby() { return chip.standby(); }
int16_t RadioHal::setChipFrequency(float freqMHz) { return chip.setFrequency(freqMHz); }
int16_t RadioHal::setBandwidth(float bwKHz) { return chip.setBandwidth(bwKHz); }
int16_t RadioHal::setSpreadingFactor(uint8_t sf) { return chip.setSpreadingFactor(sf); }
int16_t RadioHal::setCodingRate(uint8_t cr) { return chip.setCodingRate(cr); }
int16_t RadioHal::setChipOutputPower(int8_t power) { return chip.setOutputPower(power); }
float   RadioHal::getCurrentLimit() { return -1.0f; }  // no OCP on SX128x
int16_t RadioHal::setSyncWord(uint8_t syncWord) { return chip.setSyncWord(syncWord); }
int16_t RadioHal::setPreambleLength(uint16_t len) { return chip.setPreambleLength(len); }
int16_t RadioHal::explicitHeader() { return chip.explicitHeader(); }
// SX128x LoRa CRC takes 0 (off) or 2 (on) bytes only.
int16_t RadioHal::setCRC(uint8_t len) { return chip.setCRC(len ? 2 : 0); }
int16_t RadioHal::invertIQ(bool enable) { return chip.invertIQ(enable); }
// No LDRO setting on SX128x; the chip handles SF11/12 timing itself.
int16_t RadioHal::autoLDRO() { return RADIOLIB_ERR_NONE; }

// ─── Receive / transmit ─────────────────────────────────────
int16_t  RadioHal::startReceive() { return chip.startReceive(); }
size_t   RadioHal::getPacketLength() { return chip.getPacketLength(); }
int16_t  RadioHal::readData(uint8_t* data, size_t len) { return chip.readData(data, len); }
float    RadioHal::getRSSI(bool packet) { return chip.getRSSI(packet); }
float    RadioHal::getSNR() { return chip.getSNR(); }
uint32_t RadioHal::getTimeOnAir(size_t len) { return chip.getTimeOnAir(len); }
int16_t  RadioHal::startTransmit(const uint8_t* data, size_t len) { return chip.startTransmit(data, len); }
int16_t  RadioHal::finishTransmit() { return chip.finishTransmit(); }

// Header-only version of the SX1262 build's guard (see
// radio_hal_sx126x.cpp). At SF11/12 the SX1280 latches PREAMBLE_DETECTED
// on noise (bench, 2026-10-25: 13-59 % of idle-channel probes, unchanged
// with a 30 dB attenuator), and each false flag held TX off for a whole
// short-frame airtime. So only a valid header counts; the preamble window
// is left to CAD, which on this chip showed no false alarms. A header
// error ends the reception immediately.
bool RadioHal::isReceivingPacket() {
    uint32_t irq = chip.getIrqFlags();
    bool header   = (irq & RADIOLIB_SX128X_IRQ_HEADER_VALID) != 0;
    bool hdrError = (irq & RADIOLIB_SX128X_IRQ_HEADER_ERROR) != 0;
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
            chip.clearIrqFlags(RADIOLIB_SX128X_IRQ_HEADER_VALID);
            rxActivityAt_ = 0;
            rxHeaderSeen_ = false;
            return false;
        }
        return true;
    }
    rxActivityAt_ = 0;
    rxHeaderSeen_ = false;
    return false;
}

// ─── Channel activity detection ─────────────────────────────
// The SX128x CAD takes only a symbol count (no detPeak/detMin) and always
// exits to standby; the caller polls the IRQ flags and restarts RX.
void RadioHal::normalizeCadParams(uint8_t& symCode, uint8_t& detPeak,
                                  uint8_t& detMin, uint8_t& exitMode) {
    if (symCode > 4) symCode = 4;
    detPeak = 0;
    detMin = 0;
    exitMode = 0;
}

int16_t RadioHal::startChannelScan() {
    ChannelScanConfig_t cfg = {};
    cfg.cad.symNum = 0x01;  // 2 symbols, the openHop Core default
    return startChannelScan(cfg);
}

int16_t RadioHal::startChannelScan(const ChannelScanConfig_t& config) {
    // RADIOLIB_SX128X_CAD_ON_<n>_SYMB is the symbol code in bits 7:5.
    const uint8_t code = config.cad.symNum > 4 ? 4 : config.cad.symNum;
    ChannelScanConfig_t cfg = {};
    cfg.cad.symNum   = (uint8_t)(code << 5);
    cfg.cad.irqFlags = RADIOLIB_IRQ_CAD_DEFAULT_FLAGS;
    cfg.cad.irqMask  = RADIOLIB_IRQ_CAD_DEFAULT_MASK;
    return chip.startChannelScan(cfg);
}

RadioHal::ChannelScanState RadioHal::pollChannelScan() {
    uint32_t irq = chip.getIrqFlags();
    if (!(irq & RADIOLIB_SX128X_IRQ_CAD_DONE)) return CAD_PENDING;
    return (irq & RADIOLIB_SX128X_IRQ_CAD_DETECTED) ? CAD_DETECTED : CAD_CLEAR;
}

int16_t RadioHal::clearChannelScanFlags() {
    return chip.clearIrqFlags(RADIOLIB_SX128X_IRQ_CAD_DONE |
                              RADIOLIB_SX128X_IRQ_CAD_DETECTED);
}

// ─── Maintenance ────────────────────────────────────────────
int16_t RadioHal::resetAGC() { return RADIOLIB_ERR_UNSUPPORTED; }

#endif  // RADIO_SX1280
