#pragma once

// Accepted on 2026-11-10: C8 control algorithms with A1 LOW/HIGH parameters.
// Units/order: Kp, Ki, Kaw, FF PWM/RPM, FF PWM offset, RPM/s, RPM/s^2.
// These are compile-time boot defaults; serial tuning changes RAM only.
namespace robot_config {
static constexpr float LOW_TUNING[2][7] = {
    {3.24f, 1.20f, .50f, 3.13f,   45.90f,   320, 1500},
    {3.33f,  .40f, .50f, 3.2473f, 58.1475f, 320, 1500}
};
static constexpr float HIGH_TUNING[2][7] = {
    {4.35f, 1.20f, .80f, 3.03f,   46.00f,   320, 1500},
    {4.44f,  .40f, .80f, 3.2473f, 58.1475f, 320, 1500}
};
static constexpr float WHEEL_DIAMETER_M = .100f;
static constexpr float TRACK_M = .400f;
static constexpr float MAX_RPM = 114.592f;
static constexpr float MAX_PWM = 470;
static constexpr float PWM_SLEW = 27;
static constexpr float SYNC_KP = 720;
static constexpr float SYNC_MAX_RPM = 18;
static constexpr float DIVERGENCE_M = .040f;
static constexpr float GAIN_TRANSITION_S = .150f;

// Fixed policy: acceleration limiting + IMU heading correction in HIGH.
// Modes 0 and 1 are not accepted by the exported serial protocol.
static constexpr unsigned char HIGH_MODE = 2;
static constexpr float HIGH_A_MAX = .50f;
static constexpr float HIGH_K_YAW = 1.0f;
static constexpr float HIGH_K_RATE = .15f;
static constexpr float HIGH_MAX_OMEGA = .15f;
static constexpr float HIGH_SLEW_OMEGA = .75f;
}
