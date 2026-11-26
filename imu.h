// =============================================================================
// imu.h - high-level IMU interface, built on the bno08x driver.
//
// PURPOSE
//   Hides the SH-2 details from main(): find the sensor, turn on the reports
//   chosen in imu_config.h, read them, and keep the latest values in one
//   imu_data_t struct. Also works out the hand pose (palm/finger direction)
//   and whether the hand is holding still, and prints everything (serial
//   screen or CSV for tools/imu_logger.py).
//
// TYPICAL USE
//   imu_init(&cfg);              // once
//   imu_data_t imu = {0};
//   while (1) {
//       imu_update(&imu);        // as often as possible
//       if (imu.still) ... use imu.pose.palm / imu.pose.fingers ...
//   }
// =============================================================================
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>
#include "bno08x.h"
#include "imu_config.h"

typedef struct { float x, y, z; } imu_vec3_t;
typedef struct { float r, i, j, k; } imu_quat_t;   // r = real part

// Hand pose: which way the palm and the fingers point in the room (from the
// quaternion). Up/down are locked to gravity; away/toward/left/right are
// relative to the heading set by imu_zero_heading(). BETWEEN = tilted between
// two directions, or heading not zeroed yet. See the pose section of imu.c.
typedef enum {
    IMU_DIR_BETWEEN = 0,
    IMU_DIR_UP, IMU_DIR_DOWN,
    IMU_DIR_AWAY, IMU_DIR_TOWARD,
    IMU_DIR_LEFT, IMU_DIR_RIGHT,
} imu_dir_t;

typedef struct { imu_dir_t palm, fingers; } imu_pose_t;

// Latest value of each enabled report. A field stays 0 if its report is OFF.
typedef struct {
    // From the Game Rotation Vector
    imu_quat_t quat;             // unit quaternion: the real orientation data
    uint8_t accuracy;           // status bits: 0 = unreliable ... 3 = high
    float heading0;              // room heading counted as "away" (set by imu_zero_heading)
    bool heading_set;

    imu_vec3_t gravity;          // m/s^2, "down" in the sensor frame (Gravity report)
    imu_vec3_t gyro;             // rad/s (Gyroscope calibrated report)

    // Where the fingers / palm point in the room, unit length:
    // x = away, y = left, z = up. The pose is picked from these; use them to
    // see how close a sign sits to a zone edge (+-0.8 = POSE_MIN_COS).
    imu_vec3_t finger_dir, palm_dir;

    // ADDED: worked out by imu_update() (see the ADDED section in imu.c)
    imu_pose_t pose;             // smoothed: only changes after a new pose holds ~100 ms
    bool still;                  // hand has barely rotated for ~250 ms.
                                 // Letter code: only accept a letter while this is true.
} imu_data_t;

// Find the sensor (retrying every second until it answers) and enable every
// report turned on in imu_config.h.
void imu_init(const bno08x_config_t *cfg);

// Call often from the main loop. Reads every waiting report into *d,
// re-enables reports if the sensor reset itself, and updates d->pose / d->still.
void imu_update(imu_data_t *d);

// Make the direction the fingers point right now "away". Done automatically on
// the first reading and after a sensor reset. Returns false (nothing changed)
// if the fingers point nearly straight up or down.
bool imu_zero_heading(imu_data_t *d);

// Print one human-readable line per enabled report (for the serial monitor).
void imu_print(const imu_data_t *d);

// CSV streaming for tools/imu_logger.py: a header naming the enabled columns,
// then one row per sample. The header is resent every second so the logger
// can start listening at any time.
void imu_csv_header(void);
void imu_csv_row(const imu_data_t *d, uint32_t t_ms);

#endif
