// =============================================================================
// imu.c - implementation of the high-level IMU interface (see imu.h).
//
// PURPOSE
//   - imu_init():   find the BNO08x on I2C and enable the reports chosen in
//                   imu_config.h (k_reports[] table below).
//   - imu_update(): drain every waiting report and copy it into imu_data_t
//                   (handle_report() below). Re-enables reports after a sensor
//                   reset, because a reset clears them.
//   - imu_print() / imu_csv_*(): output for the serial monitor or for
//                   tools/imu_logger.py.
//
//   - hand pose (palm/finger direction) from the quaternion, then the ADDED
//     section: pose smoothing + stillness check from the gyro.
//
// CURRENT SETUP
//   Handles Game Rotation Vector, Gravity and Gyroscope (calibrated) only.
// =============================================================================
#include "imu.h"

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"

#define RAD2DEG (180.0f / (float)M_PI)

// Reports to turn on. Sensor IDs are plain numbers, NOT bit flags, so each is
// enabled with its own call. Rows whose rate is OFF are skipped.
static const struct {
    sh2_SensorId_t id;
    uint32_t hz;
    const char *name;
} k_reports[] = {
    { SH2_GAME_ROTATION_VECTOR, IMU_GAME_ROTATION_VECTOR_HZ, "game rotation vector" },
    { SH2_GRAVITY,              IMU_GRAVITY_HZ,              "gravity" },
    { SH2_GYROSCOPE_CALIBRATED, IMU_GYROSCOPE_CALIBRATED_HZ, "gyroscope" },
};

static void enable_reports(void) {
    for (unsigned n = 0; n < sizeof k_reports / sizeof k_reports[0]; n++) {
        if (k_reports[n].hz == 0) continue;
        if (!bno08x_enable_report(k_reports[n].id, 1000000u / k_reports[n].hz))
            printf("Could not enable %s\n", k_reports[n].name);
    }
}

void imu_init(const bno08x_config_t *cfg) {
    while (!bno08x_init(cfg)) {
        printf("BNO08x not answering at 0x%02X. Check wiring.\n", cfg->addr);
        sleep_ms(1000);
    }
    printf("BNO08x ready at 0x%02X\n", cfg->addr);
    enable_reports();
}

static inline void set_vec3(imu_vec3_t *dst, float x, float y, float z) {
    dst->x = x; dst->y = y; dst->z = z;
}

// Copy one decoded report into *d. Only the v->un field matching sensorId is valid.
static void handle_report(imu_data_t *d, const sh2_SensorValue_t *v) {
    switch (v->sensorId) {

    case SH2_GAME_ROTATION_VECTOR: {
        const sh2_RotationVector_t *q = &v->un.gameRotationVector;
        d->quat = (imu_quat_t){ q->real, q->i, q->j, q->k };
        d->accuracy = v->status;   // already just the 2 accuracy bits (0..3)
        if (!d->heading_set) imu_zero_heading(d);   // first reading sets "away"
        break;
    }
    case SH2_GRAVITY:
        set_vec3(&d->gravity, v->un.gravity.x, v->un.gravity.y, v->un.gravity.z);
        break;
    case SH2_GYROSCOPE_CALIBRATED:
        set_vec3(&d->gyro, v->un.gyroscope.x, v->un.gyroscope.y, v->un.gyroscope.z);
        break;

    default:
        break;   // a report we don't handle - ignore it
    }
}

static void update_pose_and_stillness(imu_data_t *d);   // ADDED section, below

void imu_update(imu_data_t *d) {
    // If the sensor reset itself, its report settings are gone - turn them back on.
    if (bno08x_was_reset()) {
        printf("BNO08x reset, re-enabling reports\n");
        enable_reports();
        d->heading_set = false;   // its yaw restarts from 0, so the old "away" is wrong
    }

    // Drain every reading that's waiting.
    sh2_SensorValue_t v;
    while (bno08x_poll(&v))
        handle_report(d, &v);

    update_pose_and_stillness(d);   // ADDED
}

// ---------------------------------------------------------------------------
// Hand pose: which way the palm and the fingers point in the room.
//
// The Game Rotation Vector quaternion turns any direction on the sensor into
// a direction in the room (Z = up). Up/down are locked to gravity and never
// drift. Away/toward/left/right need a reference: "away" is wherever the
// fingers pointed when the heading was zeroed (first reading after start-up,
// or press 'z' in the serial monitor). That reference drifts slowly and does
// not follow you if you turn your body, so re-zero now and then.
// ---------------------------------------------------------------------------

// How the IMU sits on the hand. Check: hand flat, palm down, fingers pointing
// away from you, press 'z' -> should read "palm down, fingers away".
// If palm/fingers come out swapped or flipped, change these two.
static const float k_finger_axis[3] = { 1, 0, 0 };   // +X runs out along the fingers
static const float k_palm_axis[3]   = { 0, 0, -1 };  // -Z comes out of the palm (chip side
                                                     //  faces out of the back of the hand)

#define POSE_MIN_COS 0.8f   // within ~37 deg of a direction, else "between"; raise for stricter

static const char *const k_dir_names[] = {
    "between", "up", "down", "away", "toward", "left", "right",   // imu_dir_t order
};

// Sensor-frame direction -> room-frame direction (Z up), using the quaternion.
static void to_room(const imu_quat_t *q, const float v[3], float out[3]) {
    float r = q->r, i = q->i, j = q->j, k = q->k;
    out[0] = (1 - 2 * (j * j + k * k)) * v[0] + 2 * (i * j - r * k) * v[1] + 2 * (i * k + r * j) * v[2];
    out[1] = 2 * (i * j + r * k) * v[0] + (1 - 2 * (i * i + k * k)) * v[1] + 2 * (j * k - r * i) * v[2];
    out[2] = 2 * (i * k - r * j) * v[0] + 2 * (j * k + r * i) * v[1] + (1 - 2 * (i * i + j * j)) * v[2];
}

bool imu_zero_heading(imu_data_t *d) {
    float f[3];
    to_room(&d->quat, k_finger_axis, f);
    if (f[0] * f[0] + f[1] * f[1] < 0.25f) return false;   // fingers ~vertical: no heading to take
    d->heading0 = atan2f(f[1], f[0]);
    d->heading_set = true;
    return true;
}

// Room direction of one sensor axis, relative to the zeroed heading
// (unit length): x = away, y = left, z = up.
static imu_vec3_t room_vec(const imu_data_t *d, const float axis[3]) {
    float w[3];
    to_room(&d->quat, axis, w);
    float c = cosf(d->heading0), s = sinf(d->heading0);
    return (imu_vec3_t){ c * w[0] + s * w[1], -s * w[0] + c * w[1], w[2] };
}

// Which of the 6 room directions v is closest to, or BETWEEN if none is close.
static imu_dir_t classify(imu_vec3_t v) {
    float a[3] = { v.x, v.y, v.z };
    int n = 0;
    for (int m = 1; m < 3; m++)
        if (fabsf(a[m]) > fabsf(a[n])) n = m;
    if (fabsf(a[n]) < POSE_MIN_COS) return IMU_DIR_BETWEEN;

    static const imu_dir_t pos[3] = { IMU_DIR_AWAY,   IMU_DIR_LEFT,  IMU_DIR_UP };
    static const imu_dir_t neg[3] = { IMU_DIR_TOWARD, IMU_DIR_RIGHT, IMU_DIR_DOWN };
    return a[n] >= 0 ? pos[n] : neg[n];
}

// Pose from this instant's reading (unsmoothed - use d->pose instead).
// Also fills d->finger_dir / d->palm_dir.
static imu_pose_t raw_pose(imu_data_t *d) {
    imu_pose_t p = { IMU_DIR_BETWEEN, IMU_DIR_BETWEEN };
    if (d->heading_set) {
        d->finger_dir = room_vec(d, k_finger_axis);
        d->palm_dir   = room_vec(d, k_palm_axis);
        p.fingers = classify(d->finger_dir);
        p.palm    = classify(d->palm_dir);
    }
    return p;
}

// ===========================================================================
// ADDED: pose smoothing + stillness check. Fills d->pose and d->still.
//
//   Smoothing: d->pose only switches once the raw pose has stayed the same
//   for POSE_HOLD_MS, so it doesn't flicker on the edge between two zones.
//
//   Stillness: d->still is true once the gyro has stayed under STILL_DEG_S
//   for STILL_MS. Letter code should only accept a letter while d->still is
//   true, so letters the hand passes through between signs are ignored.
//   To tune STILL_DEG_S: watch the gyro values while holding a sign.
// ===========================================================================
#define POSE_HOLD_MS  50     // new pose must hold this long before d->pose changes
#define STILL_MS      50     // hand must stay still this long before d->still is true
#define STILL_DEG_S   20.0f   // "still" = rotating slower than this, deg/s

static void update_pose_and_stillness(imu_data_t *d) {
    static imu_pose_t candidate;        // pose waiting to be accepted
    static uint32_t candidate_since;    // when it first appeared (ms)
    static uint32_t last_moving;        // last time the hand was rotating (ms)
    uint32_t now = to_ms_since_boot(get_absolute_time());

    imu_pose_t p = raw_pose(d);
    if (p.palm != candidate.palm || p.fingers != candidate.fingers) {
        candidate = p;                  // pose changed: start timing it
        candidate_since = now;
    } else if (now - candidate_since >= POSE_HOLD_MS) {
        d->pose = candidate;            // held long enough: accept it
    }

    if (IMU_GYROSCOPE_CALIBRATED_HZ) {
        float g = d->gyro.x * d->gyro.x + d->gyro.y * d->gyro.y + d->gyro.z * d->gyro.z;
        if (sqrtf(g) * RAD2DEG > STILL_DEG_S) last_moving = now;
        d->still = (now - last_moving >= STILL_MS);
    } else {
        d->still = true;                // no gyro: can't tell, so never block letters
    }
}
// ========================= end of ADDED section ===========================

// ---------------------------------------------------------------------------
// Output. The IMU_*_HZ values are compile-time constants, so the compiler
// drops the lines for reports that are OFF.
// ---------------------------------------------------------------------------

void imu_print(const imu_data_t *d) {
    if (IMU_GAME_ROTATION_VECTOR_HZ) {
        printf("Quat  r %6.3f  i %6.3f  j %6.3f  k %6.3f  (accuracy %u/3)\n",
               d->quat.r, d->quat.i, d->quat.j, d->quat.k, d->accuracy);
        printf("Fingers away %5.2f  left %5.2f  up %5.2f\n",
               d->finger_dir.x, d->finger_dir.y, d->finger_dir.z);
        printf("Palm    away %5.2f  left %5.2f  up %5.2f\n",
               d->palm_dir.x, d->palm_dir.y, d->palm_dir.z);
        printf("Pose   palm %s, fingers %s  [%s]%s\n",
               k_dir_names[d->pose.palm], k_dir_names[d->pose.fingers],
               d->still ? "STILL" : "moving",
               d->heading_set ? "" : "  (heading not zeroed: point fingers away, press z)");
    }
    if (IMU_GRAVITY_HZ)
        printf("Gravity %6.2f %6.2f %6.2f m/s^2\n", d->gravity.x, d->gravity.y, d->gravity.z);
    if (IMU_GYROSCOPE_CALIBRATED_HZ)
        printf("Gyro   %6.1f %6.1f %6.1f deg/s\n",
               d->gyro.x * RAD2DEG, d->gyro.y * RAD2DEG, d->gyro.z * RAD2DEG);
}

// CSV for tools/imu_logger.py: "H,..." names the columns, "D,..." is one sample.
// Anything else on the serial port is shown by the logger as a log message.
// Columns sharing a prefix and ending in _x/_y/_z, _r/_i/_j/_k or
// _palm/_fingers are drawn on the same plot. finger_x/y/z and palm_x/y/z are
// the room directions (x = away, y = left, z = up).
void imu_csv_header(void) {
    printf("H,t_ms");
    if (IMU_GAME_ROTATION_VECTOR_HZ)
        printf(",quat_r,quat_i,quat_j,quat_k,accuracy"
               ",finger_x,finger_y,finger_z,palm_x,palm_y,palm_z"
               ",pose_palm,pose_fingers,still");
    if (IMU_GRAVITY_HZ)              printf(",grav_x,grav_y,grav_z");
    if (IMU_GYROSCOPE_CALIBRATED_HZ) printf(",gyro_x,gyro_y,gyro_z");
    printf("\n");
}

static void csv_vec3(imu_vec3_t v, float scale) {
    printf(",%.4f,%.4f,%.4f", v.x * scale, v.y * scale, v.z * scale);
}

void imu_csv_row(const imu_data_t *d, uint32_t t_ms) {
    printf("D,%lu", (unsigned long)t_ms);
    if (IMU_GAME_ROTATION_VECTOR_HZ) {
        printf(",%.4f,%.4f,%.4f,%.4f,%u", d->quat.r, d->quat.i, d->quat.j, d->quat.k, d->accuracy);
        csv_vec3(d->finger_dir, 1.0f);
        csv_vec3(d->palm_dir, 1.0f);
        // pose as numbers (imu_dir_t order in imu.h), still as 0/1
        printf(",%d,%d,%d", (int)d->pose.palm, (int)d->pose.fingers, (int)d->still);
    }
    if (IMU_GRAVITY_HZ)              csv_vec3(d->gravity, 1.0f);
    if (IMU_GYROSCOPE_CALIBRATED_HZ) csv_vec3(d->gyro, RAD2DEG);   // deg/s
    printf("\n");
}
