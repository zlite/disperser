#pragma once

#include <Arduino.h>
#include "secrets.h"

namespace Config {

// Network
constexpr const char* WIFI_SSID = Secrets::WIFI_SSID;
constexpr const char* WIFI_PASSWORD = Secrets::WIFI_PASSWORD;
constexpr char MDNS_HOSTNAME[] = "disperser";

// M5Stack Basic Core -> Module13.2 Stepmotor Driver (M039) bus pins.
constexpr uint8_t MOTOR_A_STEP_PIN = 16;
constexpr uint8_t MOTOR_A_DIR_PIN = 17;
constexpr uint8_t MOTOR_B_STEP_PIN = 12;
constexpr uint8_t MOTOR_B_DIR_PIN = 13;
constexpr uint8_t MOTOR_Z_STEP_PIN = 15;
constexpr uint8_t MOTOR_Z_DIR_PIN = 0;
constexpr uint8_t MODULE_I2C_ADDRESS = 0x27;

// Set these if an axis travels in the opposite direction.
constexpr bool INVERT_MOTOR_A = false;
constexpr bool INVERT_MOTOR_B = false;
// Entosieve's proven setup uses DIR=HIGH toward the upper limit. Logical
// positive Z is downward here, so invert Z to make Home travel upward.
constexpr bool INVERT_MOTOR_Z = true;

// Active-low limit switches on the module inputs.
constexpr uint8_t Z_LIMIT_INPUT = 0;
constexpr uint8_t X_LIMIT_INPUT = 1;
constexpr uint8_t Y_LIMIT_INPUT = 2;
constexpr bool LIMITS_ACTIVE_LOW = true;

// Initial direction assumptions for the first XY homing test. Toggle the
// relevant value if an axis moves away from its switch.
constexpr bool X_HOME_DIRECTION_POSITIVE = false;
constexpr bool Y_HOME_DIRECTION_POSITIVE = false;

// Motion calibration. Defaults assume a 200-step motor, 20-tooth GT2 pulley,
// 2 mm belt pitch, and 1/8 microstepping.
constexpr float XY_STEPS_PER_MM = 80.0f;
// Matches the proven Entosieve setup: 100 steps/cm = 10 steps/mm.
constexpr float Z_STEPS_PER_MM = 10.0f;

constexpr float Z_HOME_TRAVEL_MM = 150.0f;
constexpr float XY_HOME_TRAVEL_MM = 250.0f;
// Provisional post-home park position. These distances use the initial
// 80-steps/mm estimate and should be calibrated from measured travel.
constexpr float XY_PARK_X_MM = 100.0f;
constexpr float XY_PARK_Y_MM = 100.0f;
// Move away from the two switches using the inverse of each confirmed homing
// direction. Physical Y maps to logical X; physical X maps to logical Y.
constexpr float XY_PARK_X_DIRECTION = Y_HOME_DIRECTION_POSITIVE ? -1.0f : 1.0f;
constexpr float XY_PARK_Y_DIRECTION = X_HOME_DIRECTION_POSITIVE ? -1.0f : 1.0f;
// The working Entosieve motion starts at 50 steps/s. Keep Z at that same rate
// because this controller currently has no acceleration ramp.
constexpr float HOME_SPEED_MM_S = 5.0f;
constexpr float XY_HOME_SPEED_MM_S = 2.0f;
constexpr float XY_PARK_SPEED_MM_S = 5.0f;
// Conservative starting speed because motion currently begins without an
// acceleration ramp. Raise this after reliable CoreXY testing.
constexpr float XY_SPEED_MM_S = 5.0f;
constexpr float Z_SPEED_MM_S = 5.0f;

constexpr float DEFAULT_SWISH_SIZE_MM = 2.0f;
constexpr float DEFAULT_SWISH_SPEED_MM_S = 2.0f;
constexpr uint32_t DEFAULT_SWISH_DURATION_SECONDS = 30;
constexpr float MIN_SWISH_SIZE_MM = 0.5f;
constexpr float MAX_SWISH_SIZE_MM = 10.0f;
constexpr float MIN_SWISH_SPEED_MM_S = 0.5f;
constexpr float MAX_SWISH_SPEED_MM_S = 10.0f;
constexpr uint32_t MIN_SWISH_DURATION_SECONDS = 5;
constexpr uint32_t MAX_SWISH_DURATION_SECONDS = 300;
constexpr uint16_t SWISH_CIRCLE_SEGMENTS = 48;
constexpr float Z_LOWER_DISTANCE_MM = 40.0f;

constexpr uint32_t STEP_PULSE_US = 10;
constexpr uint32_t MIN_STEP_INTERVAL_US = 120;
constexpr uint32_t DISPLAY_REFRESH_MS = 200;
constexpr uint32_t WIFI_RETRY_MS = 10000;

}  // namespace Config
