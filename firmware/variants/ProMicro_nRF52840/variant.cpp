// =============================================================
// variants/ProMicro_nRF52840/variant.cpp — identity pin map.
//
// Every entry maps Arduino pin N to raw nRF52 GPIO N. Indices 0
// and 1 are the XTAL pins and marked 0xFF so digitalWrite to them
// silently no-ops.
//
// initVariant() runs before setup(); it raises the external VCC
// rail so the radio module is powered before anything touches SPI.
// =============================================================
#include "variant.h"
#include "wiring_constants.h"
#include "wiring_digital.h"

const uint32_t g_ADigitalPinMap[] = {
    0xff, 0xff, 2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13,
    14,   15,   16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27,
    28,   29,   30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41,
    42,   43,   44, 45, 46, 47
};

void initVariant() {
    pinMode(PIN_EXT_VCC, OUTPUT);
    digitalWrite(PIN_EXT_VCC, HIGH);
    pinMode(PIN_BUTTON1, INPUT_PULLUP);
}
