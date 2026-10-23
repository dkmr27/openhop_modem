// =============================================================
// radio_hal_sx126x.cpp — RadioHal implementation for SX1262 boards
// =============================================================
#include "radio_hal.h"

#if defined(RADIO_SX1262)

#include <Arduino.h>
#include <SPI.h>
#include "pa_ramp.h"

namespace {

// BOARD.pa_ramp_time_us → SX126x SetTxParams ramp code. Values the chip
// can't do exactly fall back to 0, which keeps RadioLib's default ramp.
uint8_t paRampSetting(uint16_t us) {
    switch (us) {
    case 10:   return RADIOLIB_SX126X_PA_RAMP_10U;
    case 20:   return RADIOLIB_SX126X_PA_RAMP_20U;
    case 40:   return RADIOLIB_SX126X_PA_RAMP_40U;
    case 80:   return RADIOLIB_SX126X_PA_RAMP_80U;
    case 200:  return RADIOLIB_SX126X_PA_RAMP_200U;
    case 800:  return RADIOLIB_SX126X_PA_RAMP_800U;
    case 1700: return RADIOLIB_SX126X_PA_RAMP_1700U;
    case 3400: return RADIOLIB_SX126X_PA_RAMP_3400U;
    default:   return 0;
    }
}

class OpenHopSX1262 : public SX1262 {
public:
    using SX1262::SX1262;

    int16_t startReceive() override {
        // RadioLib's default RX IRQ set omits PREAMBLE_DETECTED. The passive
        // reception guard needs that flag latched before HEADER_VALID so a
        // TX/CAD request cannot abort a frame during its preamble.
        return SX1262::startReceive(
            RADIOLIB_SX126X_RX_TIMEOUT_INF,
            RADIOLIB_IRQ_RX_DEFAULT_FLAGS |
                (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED),
            RADIOLIB_IRQ_RX_DEFAULT_MASK,
            0);
    }

    int16_t applyRegisterPatch08B5() {
        uint8_t value = 0;
        int16_t state = readRegister(0x08B5, &value, 1);
        if (state != RADIOLIB_ERR_NONE) return state;
        value |= 0x01;
        return writeRegister(0x08B5, &value, 1);
    }
};

// SX1262 / E22P pin map comes from BOARD (see boards/<name>.h). The chip
// object exists from startup, as before this layer, so commands such as
// RADIO_STANDBY stay safe to issue even when begin() failed.
#if defined(BOARD_PHOTON_1W_XIAO_ESP32C6)
SPIClass loraSpi(0);
OpenHopSX1262 chip = new Module(BOARD.pin_lora_nss, BOARD.pin_lora_dio1,
                                BOARD.pin_lora_rst, BOARD.pin_lora_busy,
                                loraSpi);
#elif defined(BOARD_RAK4631_WISMESH_ETH) || defined(BOARD_RAK4631_USB)
// The RAK4631 internal SX1262 uses its own nRF52 SPIM instance on
// P1.11/P1.13/P1.12. On the Ethernet variant this also keeps global SPI free
// for RAK13800/W5100S; the USB-only variant retains the proven radio path.
SPIClass loraSpi(NRF_SPIM2, 45, 43, 44);  // MISO, SCK, MOSI
OpenHopSX1262 chip = new Module(BOARD.pin_lora_nss, BOARD.pin_lora_dio1,
                                BOARD.pin_lora_rst, BOARD.pin_lora_busy,
                                loraSpi);
#else
OpenHopSX1262 chip = new Module(BOARD.pin_lora_nss, BOARD.pin_lora_dio1,
                                BOARD.pin_lora_rst, BOARD.pin_lora_busy);
#endif

}  // namespace

RadioHal radio;

// ─── Lifecycle ──────────────────────────────────────────────
float RadioHal::bootTcxoVoltage() {
    return BOARD.use_dio3_tcxo ? BOARD.tcxo_voltage : 0.0f;
}

int16_t RadioHal::begin() {
    // Bring up the SPI bus for the SX1262 BEFORE chip.begin(). When
    // BOARD.pin_lora_sck/miso/mosi are -1 the board variant's default
    // SPI pins already match the LoRa wiring (Heltec V3, XIAO ESP32-S3
    // for Ikoka) and we leave the bus alone. When they're set the
    // board has remapped SPI (LilyGO T3-S3 etc.) and we must call
    // SPI.begin() with the explicit pins or RadioLib's first SPI
    // transfer fails.
    if (BOARD.pin_lora_sck >= 0 || BOARD.pin_lora_miso >= 0 || BOARD.pin_lora_mosi >= 0) {
#ifdef ARDUINO_ARCH_ESP32
#if defined(BOARD_PHOTON_1W_XIAO_ESP32C6)
        // Seeed XIAO ESP32-C6 Photon variant matches MeshCore's
        // working C6 port, which uses SPIClass(0) for the LoRa bus.
        loraSpi.begin(BOARD.pin_lora_sck, BOARD.pin_lora_miso,
                      BOARD.pin_lora_mosi);
#else
        // ESP32-S3/P4 GPIO matrix: rebind SPI to specific pins
        SPI.begin(BOARD.pin_lora_sck, BOARD.pin_lora_miso,
                  BOARD.pin_lora_mosi, BOARD.pin_lora_nss);
#endif
#else
        // nRF52 BSP: SPI peripheral has fixed pins on its selected
        // instance. RAK4631 WisMesh uses a dedicated LoRa SPI instance;
        // global SPI remains the W5100S Ethernet bus.
#  if defined(BOARD_RAK4631_WISMESH_ETH) || defined(BOARD_RAK4631_USB)
        loraSpi.begin();
#  else
        SPI.begin();
#  endif
#endif
    }

    // RadioLib's no-argument SX1262::begin() assumes a 1.6 V TCXO.
    // Supply the board policy during initialization so targets such as
    // RAK3401 bring up their 1.8 V TCXO before the first radio commands.
    return chip.begin(434.0f, 125.0f, 9, 7,
                      RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                      10, 8, bootTcxoVoltage());
}

void RadioHal::rfSwitchConfigureRadio() {
    // DIO2 + explicit RX/TX pins are independent — the Wio-SX1262 board uses
    // both: DIO2 drives the TX path internally while RXEN (an external GPIO)
    // gates the LNA on the RX path. Older boards (Heltec V3 / T3S3 / RAK3112)
    // only set dio2_as_rf_switch and leave rx_pin/tx_pin == -1, so the second
    // call becomes a no-op for them.
    if (BOARD.rf_switch.dio2_as_rf_switch) {
        int state = chip.setDio2AsRfSwitch(true);
        Serial.printf("[INFO] setDio2AsRfSwitch(true) -> %d\n", state);
    }
    if (BOARD.rf_switch.rx_pin >= 0 || BOARD.rf_switch.tx_pin >= 0) {
        uint32_t rx = BOARD.rf_switch.rx_pin >= 0
                          ? (uint32_t)BOARD.rf_switch.rx_pin
                          : RADIOLIB_NC;
        uint32_t tx = BOARD.rf_switch.tx_pin >= 0
                          ? (uint32_t)BOARD.rf_switch.tx_pin
                          : RADIOLIB_NC;
        chip.setRfSwitchPins(rx, tx);
        Serial.printf("[INFO] setRfSwitchPins(rx=%lu tx=%lu)\n",
                      (unsigned long)rx, (unsigned long)tx);
    }
}

void RadioHal::configureBoardRadioOptions() {
    if (BOARD.sx126x_current_limit_ma > 0) {
        int state = chip.setCurrentLimit(BOARD.sx126x_current_limit_ma);
        Serial.printf("[INFO] setCurrentLimit(%d mA) -> %d\n",
                      (int)BOARD.sx126x_current_limit_ma, state);
    }

    if (BOARD.sx126x_rx_boosted_gain) {
        int state = chip.setRxBoostedGainMode(true);
        Serial.printf("[INFO] setRxBoostedGainMode(true) -> %d\n", state);
    }

    if (BOARD.sx126x_register_patch) {
        int state = chip.applyRegisterPatch08B5();
        Serial.printf("[INFO] SX126x register patch 0x08B5 -> %d\n", state);
    }
}

void RadioHal::setDio1Action(void (*func)(void)) { chip.setDio1Action(func); }

const char* RadioHal::getChipName() const { return "SX1262"; }

RadioConfig RadioHal::defaultConfig() {
    // EU/UK (Narrow), Switzerland preset; the host replaces it with
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

// ─── Configuration ──────────────────────────────────────────
int16_t RadioHal::standby() { return chip.standby(); }
int16_t RadioHal::setFrequency(float freqMHz) { return chip.setFrequency(freqMHz); }
int16_t RadioHal::setBandwidth(float bwKHz) { return chip.setBandwidth(bwKHz); }
int16_t RadioHal::setSpreadingFactor(uint8_t sf) { return chip.setSpreadingFactor(sf); }
int16_t RadioHal::setCodingRate(uint8_t cr) { return chip.setCodingRate(cr); }

int16_t RadioHal::setOutputPower(int8_t power) {
    return applyOutputPowerAndRamp(chip, power, paRampSetting(BOARD.pa_ramp_time_us));
}

float   RadioHal::getCurrentLimit() { return chip.getCurrentLimit(); }
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
float    RadioHal::getRSSI(bool packet) { return chip.getRSSI(packet); }
float    RadioHal::getSNR() { return chip.getSNR(); }
uint32_t RadioHal::getTimeOnAir(size_t len) { return chip.getTimeOnAir(len); }
int16_t  RadioHal::startTransmit(const uint8_t* data, size_t len) { return chip.startTransmit(data, len); }
int16_t  RadioHal::finishTransmit() { return chip.finishTransmit(); }

// PREAMBLE_DETECTED / HEADER_VALID latch in the chip's IRQ status while a
// frame is being received (DIO1 only fires on terminal IRQs, so the flags
// stay readable here; readData() clears them once the frame completes).
// Lets the TX path defer to an ongoing reception instead of aborting it —
// CAD cannot do that: startChannelScan() drops to standby first, and its
// 2-symbol scan is unreliable mid-payload anyway. Parity with MeshCore
// CustomSX1262::isReceiving(). The millis bound keeps stale flags — a
// reception the RX path never got to consume — from wedging TX forever,
// like MeshCore's _maxPayloadMillis.
//
// The live IRQ flags are the source of truth (MeshCore
// CustomSX1262::isReceiving), and each stage carries its own staleness
// bound so a stray flag cannot hold TX off: a lone preamble must turn into
// a valid header within about one preamble + header airtime, a valid
// header into a frame within a worst-case payload airtime.
bool RadioHal::isReceivingPacket() {
    uint32_t irq = chip.getIrqFlags();
    bool header   = (irq & RADIOLIB_SX126X_IRQ_HEADER_VALID) != 0;
    bool preamble = (irq & RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED) != 0;
    uint32_t now = millis();

    if (!header && rxHeaderSeen_) {
        // The header flag went away without us clearing it: the frame ended
        // or was aborted, and the state must follow the chip.
        rxActivityAt_ = 0;
        rxHeaderSeen_ = false;
        return false;
    }
    if (header) {
        if (!rxHeaderSeen_) { rxHeaderSeen_ = true; rxActivityAt_ = now; }
        // Worst-case airtime of a max-size frame at current settings, padded 50%.
        uint32_t maxMs = (uint32_t)(chip.getTimeOnAir(MAX_LORA_PAYLOAD) / 1000) * 3 / 2 + 100;
        if (now - rxActivityAt_ > maxMs) {
            chip.clearIrqFlags(RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED |
                               RADIOLIB_SX126X_IRQ_HEADER_VALID);
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
            chip.clearIrqFlags(RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED);
            rxActivityAt_ = 0;
            return false;
        }
        return true;
    }
    rxActivityAt_ = 0;
    rxHeaderSeen_ = false;
    return false;
}

// ─── Channel activity detection ─────────────────────────────
int16_t RadioHal::startChannelScan() { return chip.startChannelScan(); }

int16_t RadioHal::startChannelScan(const ChannelScanConfig_t& config) {
    return chip.startChannelScan(config);
}

RadioHal::ChannelScanState RadioHal::pollChannelScan() {
    // 16-bit read, matching the uint16_t the CAD loops used before.
    uint16_t irq = chip.getIrqFlags();
    if (!(irq & RADIOLIB_SX126X_IRQ_CAD_DONE)) return CAD_PENDING;
    return (irq & RADIOLIB_SX126X_IRQ_CAD_DETECTED) ? CAD_DETECTED : CAD_CLEAR;
}

int16_t RadioHal::clearChannelScanFlags() {
    return chip.clearIrqFlags(RADIOLIB_SX126X_IRQ_CAD_DONE |
                              RADIOLIB_SX126X_IRQ_CAD_DETECTED);
}

// ─── Maintenance ────────────────────────────────────────────
int16_t RadioHal::resetAGC() { return chip.resetAGC(); }

#endif  // RADIO_SX1262
