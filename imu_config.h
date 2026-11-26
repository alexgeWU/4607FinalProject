// =============================================================================
// imu_config.h - which BNO08x reports are turned on, and how fast.
//
// PURPOSE
//   This is the only file you edit to change what the IMU sends. Each line is
//   one BNO08x (SH-2) report; the value is its rate in Hz, or OFF.
//   Setting one to OFF stops it on the sensor and drops it from the serial
//   screen and the CSV stream for tools/imu_logger.py.
//
// CURRENT SETUP (IMU on the back of the hand)
//     Game Rotation Vector  -> quaternion -> hand pose (palm/finger direction)
//     Gravity               -> which way "down" is in the sensor frame
//     Gyroscope calibrated  -> stillness check (later: air-mouse cursor speed)
//   Letter logic runs on the Pico, not the IMU.
// =============================================================================
#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

#define OFF 0     // report disabled
#define ON  100   // default rate, Hz (one sample every 10 ms)

// Game Rotation Vector: accel + gyro fused on the chip, NO magnetometer.
// Smooth and unaffected by laptops/wires/magnets near the hand. Yaw starts at 0
// at power-up and drifts slowly; roll/pitch stay locked to gravity and do not
// drift. Fills imu.quat (and from it the hand pose). Max ~400 Hz.
#define IMU_GAME_ROTATION_VECTOR_HZ   ON

// Gravity: the gravity part of acceleration, m/s^2, in the sensor's own frame
// (length ~9.8). Says which way the back of the hand faces relative to down,
// with no angle wrap-around and no gimbal lock near pitch +/-90 deg.
// Fills imu.gravity.
#define IMU_GRAVITY_HZ                ON

// Gyroscope (calibrated): rotation speed around each sensor axis, rad/s
// (shown in deg/s on screen and in the CSV), with the chip's bias removed.
// Stops at ~0 when the hand stops. Fills imu.gyro. Max ~400 Hz.
#define IMU_GYROSCOPE_CALIBRATED_HZ   ON

#endif
