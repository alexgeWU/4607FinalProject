// =============================================================================
// bno08x.c - implementation of the minimal BNO08x driver (see bno08x.h).
//
// The SH-2 library (sh2/) does all the protocol work. This file supplies the
// two things it needs from us:
//   - a HAL: reset the chip and move raw bytes over I2C
//   - callbacks: one for sensor readings (stored in a small queue until
//     bno08x_poll() hands them out) and one for "the sensor reset".
// =============================================================================
#include "bno08x.h"

#include <string.h>
#include "pico/stdlib.h"
#include "sh2_err.h"

#define I2C_TIMEOUT_US 10000

static bno08x_config_t s_cfg;
static bool s_reset_occurred;     // set by on_async_event(), cleared by bno08x_was_reset()

// ---------------------------------------------------------------------------
// Reading queue (ring buffer)
//
// One I2C packet can hold several reports (e.g. rotation vector + gravity +
// gyro). SH-2 calls on_sensor_event() once per report, so each one is queued
// here and bno08x_poll() hands them out oldest first. If the queue is full the
// oldest reading is thrown away.
// ---------------------------------------------------------------------------
#define QUEUE_SIZE 16   // raise this if you enable many fast reports

static sh2_SensorValue_t s_queue[QUEUE_SIZE];
static unsigned s_head;    // next free slot
static unsigned s_tail;    // next reading to hand out
static unsigned s_count;   // readings currently stored

static void queue_push(const sh2_SensorValue_t *v) {
    if (s_count == QUEUE_SIZE) {             // full: drop the oldest
        s_tail = (s_tail + 1) % QUEUE_SIZE;
        s_count--;
    }
    s_queue[s_head] = *v;
    s_head = (s_head + 1) % QUEUE_SIZE;
    s_count++;
}

static bool queue_pop(sh2_SensorValue_t *out) {
    if (s_count == 0) return false;
    *out = s_queue[s_tail];
    s_tail = (s_tail + 1) % QUEUE_SIZE;
    s_count--;
    return true;
}

// ---------------------------------------------------------------------------
// HAL: the 5 functions SH-2 calls to reach the chip.
// ---------------------------------------------------------------------------

static int hal_open(sh2_Hal_t *self) {
    (void)self;
    gpio_put(s_cfg.rst_pin, 0);   // RST is active-low
    sleep_ms(10);
    gpio_put(s_cfg.rst_pin, 1);
    sleep_ms(200);                // let the BNO08x boot before talking to it
    return 0;
}

static void hal_close(sh2_Hal_t *self) { (void)self; }

// Every SHTP packet starts with a 4-byte header whose first 2 bytes are the
// packet length (bit 15 = "continuation" flag). The BNO08x resends the header
// at the start of every I2C read, so: read 4 bytes to learn the length, then
// read the whole packet (header included) in one transaction.
static int hal_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us) {
    (void)self;
    uint8_t header[4];
    if (i2c_read_timeout_us(s_cfg.i2c, s_cfg.addr, header, 4, false, I2C_TIMEOUT_US) != 4)
        return 0;

    uint16_t packet_len = (header[0] | (header[1] << 8)) & 0x7FFF;
    if (packet_len == 0 || packet_len > len)   // nothing waiting, or won't fit
        return 0;

    if (i2c_read_timeout_us(s_cfg.i2c, s_cfg.addr, buf, packet_len, false,
                            I2C_TIMEOUT_US * 4) != packet_len)
        return 0;

    *t_us = time_us_32();
    return packet_len;
}

static int hal_write(sh2_Hal_t *self, uint8_t *buf, unsigned len) {
    (void)self;
    int n = i2c_write_timeout_us(s_cfg.i2c, s_cfg.addr, buf, len, false, I2C_TIMEOUT_US);
    return n < 0 ? 0 : n;
}

static uint32_t hal_get_time_us(sh2_Hal_t *self) {
    (void)self;
    return time_us_32();
}

static sh2_Hal_t s_hal = {
    .open      = hal_open,
    .close     = hal_close,
    .read      = hal_read,
    .write     = hal_write,
    .getTimeUs = hal_get_time_us,
};

// ---------------------------------------------------------------------------
// SH-2 callbacks
// ---------------------------------------------------------------------------

static void on_async_event(void *cookie, sh2_AsyncEvent_t *event) {
    (void)cookie;
    if (event->eventId == SH2_RESET) s_reset_occurred = true;
}

static void on_sensor_event(void *cookie, sh2_SensorEvent_t *event) {
    (void)cookie;
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event) == SH2_OK)   // raw bytes -> struct
        queue_push(&value);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool bno08x_init(const bno08x_config_t *cfg) {
    s_cfg = *cfg;

    i2c_init(s_cfg.i2c, s_cfg.baud);
    gpio_set_function(s_cfg.sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(s_cfg.scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(s_cfg.sda_pin);   // weak internal pull-ups; most breakouts also have their own
    gpio_pull_up(s_cfg.scl_pin);

    gpio_init(s_cfg.rst_pin);
    gpio_set_dir(s_cfg.rst_pin, GPIO_OUT);
    gpio_put(s_cfg.rst_pin, 1);

    if (sh2_open(&s_hal, on_async_event, NULL) != SH2_OK) return false;

    // Asking for the product IDs proves the sensor is really answering.
    sh2_ProductIds_t ids;
    memset(&ids, 0, sizeof ids);
    if (sh2_getProdIds(&ids) != SH2_OK) {
        sh2_close();
        return false;
    }

    sh2_setSensorCallback(on_sensor_event, NULL);
    s_head = s_tail = s_count = 0;
    s_reset_occurred = false;   // the reset we just did on purpose isn't news
    return true;
}

bool bno08x_enable_report(sh2_SensorId_t sensor, uint32_t interval_us) {
    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof config);
    config.reportInterval_us = interval_us;
    return sh2_setSensorConfig(sensor, &config) == SH2_OK;
}

bool bno08x_poll(sh2_SensorValue_t *out) {
    // Only read from the sensor once everything already queued is handed out.
    if (s_count == 0)
        sh2_service();   // reads one packet -> on_sensor_event() per report in it
    return queue_pop(out);
}

bool bno08x_was_reset(void) {
    bool r = s_reset_occurred;
    s_reset_occurred = false;
    return r;
}
