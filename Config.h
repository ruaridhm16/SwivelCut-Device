// SwivelCut firmware configuration: pin assignments and tuning constants.
//
// This is the only file that should need editing to rewire the machine,
// retune motion, or change sensor thresholds. Everything here is a
// compile-time constant, so changes cost nothing at runtime.
#pragma once

#include <Arduino.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Stepper drivers (TB6600) and arm geometry
// ---------------------------------------------------------------------------
constexpr int J1_PUL_PIN = 25;
constexpr int J1_DIR_PIN = 26;
constexpr int J2_PUL_PIN = 32;
constexpr int J2_DIR_PIN = 33;
constexpr int ENA_PIN = 27;

constexpr int FULL_STEPS_PER_REV = 200;
constexpr int MICROSTEP = 4;  // TB6600 DIP switches must also be set to 1/4.
constexpr float J1_GEAR_RATIO = 6.0f;
constexpr float J2_GEAR_RATIO = 9.0f;
constexpr float LINK_1_MM = 260.0f;
constexpr float LINK_2_MM = 255.0f;

// Positive when the cutter tip extends farther from J2 than the tracer tip.
// Set this to the measured attachment difference.
constexpr float CUTTER_EXTRA_LENGTH_MM = 0.0f;
constexpr float CUTTER_LINK_2_MM = LINK_2_MM + CUTTER_EXTRA_LENGTH_MM;

constexpr float J1_MIN_DEG = -90.0f;
constexpr float J1_MAX_DEG = 90.0f;
constexpr float J2_MIN_DEG = -180.0f;
constexpr float J2_MAX_DEG = 180.0f;
// When true, encoder-taught paths may be recorded and replayed outside the
// normal joint limits. Direct J1/J2/ANGLES/XY/CUT commands remain limited.
constexpr bool ALLOW_TAUGHT_PATH_OUTSIDE_SOFTWARE_LIMITS = false;
constexpr bool INVERT_J1 = false;
constexpr bool INVERT_J2 = true;

// Change these three levels only if the driver's input wiring is changed.
constexpr uint8_t STEP_ACTIVE = HIGH;
constexpr uint8_t STEP_IDLE = LOW;
constexpr uint8_t OUTPUTS_ENABLED = HIGH;
constexpr uint8_t OUTPUTS_DISABLED = LOW;
constexpr unsigned long DIR_SETUP_US = 100;
constexpr uint32_t STEPPER_TIMER_HZ = 1000000;
constexpr unsigned long STEPPER_MIN_HALF_PERIOD_US = 5;
constexpr unsigned long STEPPER_MIN_STEP_PERIOD_US =
    STEPPER_MIN_HALF_PERIOD_US * 2;
constexpr float DEFAULT_STEP_RATE_HZ = 333.333f;

constexpr float J1_STEPS_PER_DEG =
    FULL_STEPS_PER_REV * MICROSTEP * J1_GEAR_RATIO / 360.0f;
constexpr float J2_STEPS_PER_DEG =
    FULL_STEPS_PER_REV * MICROSTEP * J2_GEAR_RATIO / 360.0f;

// ---------------------------------------------------------------------------
// Blade motor
// ---------------------------------------------------------------------------
constexpr int BLADE_PWM_PIN = 13;
constexpr int BLADE_DIR_PIN = 14;
constexpr int BLADE_PWM_CHANNEL = 0;
constexpr int BLADE_PWM_FREQUENCY_HZ = 1000;
constexpr int BLADE_PWM_RESOLUTION_BITS = 8;
constexpr uint8_t BLADE_PWM_DUTY = 200;
constexpr uint8_t BLADE_DOWN_DIRECTION = HIGH;
constexpr uint8_t BLADE_RETRACT_DIRECTION = LOW;
constexpr float BLADE_DOWN_SECONDS = 0.75f;
constexpr float BLADE_RETRACT_SECONDS = 0.75f;

// ---------------------------------------------------------------------------
// Panel: buttons, WS2812 status LEDs, relay, head-type sensing
// ---------------------------------------------------------------------------
constexpr int START_STOP_BUTTON_PIN = 2;
constexpr int STABILIZATION_BUTTON_PIN = 36;  // VP; external pull-up required.
constexpr int REPEAT_BUTTON_PIN = 39;         // VN; external pull-up required.
constexpr int RELAY_BUTTON_PIN = 0;
constexpr int RELAY_PIN = 15;
constexpr uint8_t RELAY_CONNECTED_LEVEL = HIGH;
constexpr uint8_t RELAY_DISCONNECTED_LEVEL = LOW;

constexpr int BUTTON_LED_DATA_PIN = 4;
constexpr int BUTTON_LED_COUNT = 4;
constexpr uint8_t BUTTON_LED_BRIGHTNESS = 64;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 35;

constexpr int HEAD_ID_PIN = 34;
constexpr unsigned long HEAD_SAMPLE_INTERVAL_MS = 20;
constexpr int HEAD_STABLE_SAMPLE_COUNT = 5;
constexpr int CUTTING_HEAD_ADC_MIN = 400;
constexpr int CUTTING_HEAD_ADC_MAX = 1125;
constexpr int TRACING_HEAD_ADC_MIN = 1500;
constexpr int TRACING_HEAD_ADC_MAX = 2550;
constexpr int HEAD_DISCONNECTED_ADC_MIN = 3500;
// When true, only the tracer ADC range is detected explicitly. Every other
// reading (cutter, disconnected, or unknown) is treated as the cutting head.
constexpr bool ASSUME_CUTTER_UNLESS_TRACER = false;

// ---------------------------------------------------------------------------
// Product workflow (teach / trace / cut)
// ---------------------------------------------------------------------------
constexpr float PRODUCT_TEACH_HZ = 20.0f;
constexpr float PRODUCT_TEACH_MAX_SECONDS = 60.0f;
constexpr float PRODUCT_SMOOTHING_MS = 150.0f;
constexpr float PRODUCT_MAX_DEVIATION_DEG = 1.0f;
// false: settle at every taught point; true: stream the path continuously
// and perform closed-loop settling only at the final point.
constexpr bool CONTINUOUS_TRAJECTORY_REPLAY = true;
// Replay ignores hand-drawing timestamps and uses this repeatable motor pace.
// Lower either value for gentler, more accurate motion.
constexpr float REPLAY_STEP_RATE_HZ = 120.0f;
constexpr float REPLAY_MAX_ACCEL_STEPS_PER_S2 = 60.0f;
constexpr long CONTINUOUS_FEEDBACK_STEP_INTERVAL = 256;
constexpr float TEACH_LIMIT_NOISE_MARGIN_DEG = 1.0f;

// Temporary investigation mode. Enable to compare commanded/measured joints
// and XY, correction iterations, approach directions, and encoder calibration.
// Serial logging can disturb timing, so leave false for normal operation.
constexpr bool MOTION_DIAGNOSTICS = false;
constexpr int MOTION_DIAGNOSTIC_POINT_INTERVAL = 50;

// ---------------------------------------------------------------------------
// AS5600 joint encoders
// ---------------------------------------------------------------------------
// Encoder branches set this to true. The main branch needs no AS5600 modules.
constexpr bool USE_ENCODERS = true;
constexpr uint8_t AS5600_ADDRESS = 0x36;
constexpr uint32_t AS5600_I2C_HZ = 100000;
constexpr int AS5600_READ_ATTEMPTS = 3;
constexpr unsigned long AS5600_RETRY_DELAY_US = 250;
constexpr int J1_SDA_PIN = 18;
constexpr int J1_SCL_PIN = 19;
constexpr int J2_SDA_PIN = 16;
constexpr int J2_SCL_PIN = 17;
constexpr int ENCODER_J1_SIGN = -1;
constexpr int ENCODER_J2_SIGN = 1;

constexpr int MAX_TEACH_POINTS = USE_ENCODERS ? 3000 : 1;

// Derive feedback tolerance from motor full-step accuracy through the current
// microstepping, gearbox ratios, and link lengths. The resulting worst-case
// tip error is converted back to an arm-wide angular tolerance and floored at
// one AS5600 count so the correction loop never targets sub-sensor resolution.
constexpr float MOTOR_FULL_STEP_DEG = 1.8f;
constexpr float MOTOR_STEP_ACCURACY_PCT = 0.05f;
constexpr float MOTOR_STEP_ERROR_DEG =
    MOTOR_FULL_STEP_DEG * MOTOR_STEP_ACCURACY_PCT;
constexpr float MOTOR_MICROSTEP_ERROR_DEG = MOTOR_STEP_ERROR_DEG / MICROSTEP;
constexpr float J1_JOINT_ERROR_DEG = MOTOR_MICROSTEP_ERROR_DEG / J1_GEAR_RATIO;
constexpr float J2_JOINT_ERROR_DEG = MOTOR_MICROSTEP_ERROR_DEG / J2_GEAR_RATIO;
constexpr float TIP_ERROR_MM =
    LINK_1_MM * radians(J1_JOINT_ERROR_DEG) +
    LINK_2_MM * radians(J2_JOINT_ERROR_DEG);
constexpr float ARM_REACH_MM = LINK_1_MM + LINK_2_MM;
constexpr float FEEDBACK_TOLERANCE_DEG_RAW = degrees(TIP_ERROR_MM / ARM_REACH_MM);
constexpr float AS5600_RESOLUTION_DEG = 360.0f / 4096.0f;
constexpr float FEEDBACK_TOLERANCE_FACTOR = 1.0f;
constexpr float FEEDBACK_TOLERANCE_DEG =
    max(FEEDBACK_TOLERANCE_DEG_RAW, AS5600_RESOLUTION_DEG) *
    FEEDBACK_TOLERANCE_FACTOR;
constexpr float FEEDBACK_MAX_ERROR_DEG = 10.0f;
constexpr int FEEDBACK_MAX_CORRECTIONS = 3;
