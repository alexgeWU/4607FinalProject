// =============================================================================
// bno08x.h - minimal BNO08x (BNO080/085/086) driver for the Pico SDK.
//
// PURPOSE
//   Connects CEVA's SH-2 library (sh2/, unmodified) to the Pico's I2C and
//   RST pin. Only does what imu.c needs: find the sensor, turn reports on,
//   hand back decoded readings, and say when the sensor has reset.
//   What the readings MEAN is handled in imu.c.
// =============================================================================
#ifndef BNO08X_H
#define BNO08X_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/i2c.h"
#include "sh2.h"
#include "sh2_SensorValue.h"

typedef struct {
    i2c_inst_t *i2c;   // i2c0 or i2c1 (must match the SDA/SCL pins)
    uint sda_pin;
    uint scl_pin;
    uint rst_pin;      // GPIO wired to the BNO08x RST pin (required)
    uint baud;         // I2C clock in Hz, up to 400000
    uint8_t addr;      // 0x4A (SA0/DI pin low) or 0x4B (SA0 high)
} bno08x_config_t;

// Set up I2C + RST, reset the sensor and open the SH-2 link.
// Returns true if the sensor answered at cfg->addr.
bool bno08x_init(const bno08x_config_t *cfg);

// Ask the sensor to send a report (e.g. SH2_GAME_ROTATION_VECTOR) every interval_us microseconds.
bool bno08x_enable_report(sh2_SensorId_t sensor, uint32_t interval_us);

// Call often from the main loop. Returns true and fills *out with ONE reading
// (oldest first); check out->sensorId to see which report it is. Keep calling
// until it returns false to drain everything waiting.
bool bno08x_poll(sh2_SensorValue_t *out);

// True (once) if the sensor reset itself since the last call. A reset turns
// every report off, so the caller must enable them again.
bool bno08x_was_reset(void);

#endif
