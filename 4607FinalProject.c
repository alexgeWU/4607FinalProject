// =============================================================================
// 4607FinalProject.c - main program (ASL glove + air mouse, Pico 2 W).
//
// PURPOSE (current stage: IMU orientation testing)
//   Starts USB serial and the Wi-Fi chip (for the on-board LED), finds the
//   BNO08x IMU on the back of the hand, then loops forever reading it and
//   streaming the values over USB.
//
//   - Which IMU reports are on:  imu_config.h
//   - IMU read/decode code:      imu.c / imu.h
//   - Low-level BNO08x driver:   bno08x.c / bno08x.h
//   - Recording + plotting:      tools/imu_logger.py (needs OUTPUT_CSV 1)
//
//   Later: flex sensors, tap/click detection, mouse/keyboard modes go here
//   (all on the Pico, not on the IMU).
// =============================================================================
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"

#include "imu.h"   // choose which IMU reports are on in imu_config.h

// IMU wiring: I2C0 on GP16 (SDA) / GP17 (SCL) from PICO datasheet, BNO08x RST on GP5, 400 kHz,
// I2C address 0x4A (confirmed with a bus scan).
static const bno08x_config_t k_imu_cfg = {
    .i2c = i2c0,
    .sda_pin = 16,
    .scl_pin = 17,
    .rst_pin = 5,
    .baud = 400000,
    .addr = 0x4A,
};

// Clear the serial monitor screen
void clear_sm(void) {
    printf("\033[2J\033[H");
}

// Serial output:
//   1 = CSV stream for the recorder (python3 tools/imu_logger.py)
//   0 = human-readable screen for the serial monitor
#define OUTPUT_CSV      1
#define CSV_RATE_HZ     100   // samples per second sent to the logger

int main(void)
{
    stdio_init_all();

    // Give the USB serial port a moment to connect so startup messages are visible.
    for (int i = 0; i < 30 && !stdio_usb_connected(); i++) sleep_ms(100);

    // drive the Pico W on-board LED through wifi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    imu_init(&k_imu_cfg);

    imu_data_t imu = {0};   // latest values, e.g. imu.quat, imu.gravity.x, imu.gyro.z
    uint32_t last_print_ms = 0;
    uint32_t last_header_ms = 0;

    while (true) {
        imu_update(&imu);
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // 'z' typed in the serial monitor: the way the fingers point now becomes "away".
        int key = getchar_timeout_us(0);
        if (key == 'z' || key == 'Z') imu_zero_heading(&imu);

#if OUTPUT_CSV
        if (now - last_header_ms >= 1000) {   // resend column names every second
            last_header_ms = now;
            imu_csv_header();
        }
        if (now - last_print_ms >= 1000 / CSV_RATE_HZ) {
            last_print_ms = now;
            imu_csv_row(&imu, now);
        }
#else
        // Print at 10 Hz so the serial port isn't flooded.
        if (now - last_print_ms >= 100) {
            last_print_ms = now;
            clear_sm();
            imu_print(&imu);
        }
#endif
    }
}
