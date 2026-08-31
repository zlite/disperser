#include <Arduino.h>
#include <ESPmDNS.h>
#include <M5Unified.h>
#include <Module_Stepmotor.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>

#include "config.h"
#include "web_page.h"

namespace {

enum class DeviceState : uint8_t {
    Booting,
    NotHomed,
    Homing,
    Ready,
    Positioning,
    Lowering,
    Swishing,
    Paused,
    Stopping,
    Error,
};

enum class MotionCommand : uint32_t { Home = 1, Start = 2 };

enum class MotionType : uint8_t {
    Circular,
    BackAndForth,
    SideToSide,
    Combination,
};

struct CycleSettings {
    MotionType type;
    float speedMmS;
    float sizeMm;
    uint32_t durationSeconds;
};

volatile DeviceState g_state = DeviceState::Booting;
volatile DeviceState g_resumeState = DeviceState::Swishing;
volatile bool g_stopRequested = false;
volatile bool g_pauseRequested = false;
volatile bool g_homed = false;
volatile float g_xMm = 0.0f;
volatile float g_yMm = 0.0f;
volatile float g_zMm = 0.0f;
volatile float g_cycleCenterX = 0.0f;
volatile float g_cycleCenterY = 0.0f;
volatile int8_t g_zLimitState = -1;
volatile int8_t g_xLimitState = -1;
volatile int8_t g_yLimitState = -1;
volatile MotionType g_motionType = MotionType::Circular;
volatile float g_swishSpeedMmS = Config::DEFAULT_SWISH_SPEED_MM_S;
volatile float g_swishSizeMm = Config::DEFAULT_SWISH_SIZE_MM;
volatile uint32_t g_swishDurationSeconds = Config::DEFAULT_SWISH_DURATION_SECONDS;
volatile float g_activeSwishSpeedMmS = Config::DEFAULT_SWISH_SPEED_MM_S;
volatile uint32_t g_activeDurationSeconds = 0;
volatile uint32_t g_cycleStartedMs = 0;
volatile uint32_t g_pauseStartedMs = 0;
volatile uint32_t g_accumulatedPauseMs = 0;
volatile bool g_mdnsReady = false;

TaskHandle_t g_motionTask = nullptr;
String g_error;
WebServer g_server(80);

class StepperModule {
   public:
    bool begin() {
        Wire.begin(21, 22, 400000);
        if (!module_.init(Wire, Config::MODULE_I2C_ADDRESS)) return false;

        // Use the same driver initialization sequence as the working
        // Entosieve firmware. Release all axes; indices 0/1 are CoreXY and 2
        // is the module's Z output.
        module_.resetMotor(0, 0);
        module_.resetMotor(1, 0);
        module_.resetMotor(2, 0);
        module_.enableMotor(0);
        module_.getExtIOStatus();
        return true;
    }

    bool enable(bool enabled) {
        if (enabled) {
            module_.resetMotor(0, 0);
            module_.resetMotor(1, 0);
            module_.resetMotor(2, 0);
        }
        module_.enableMotor(enabled ? 1 : 0);
        return true;
    }

    bool readLimit(uint8_t input, bool& active) {
        uint8_t raw = 0;
        bool readOk = false;

        // The library discards I2C read errors and leaves its cached input
        // byte unchanged. Read register 0 directly so homing can fail safely
        // instead of continuing on a stale "switch open" value.
        for (uint8_t attempt = 0; attempt < 3 && !readOk; ++attempt) {
            Wire.beginTransmission(Config::MODULE_I2C_ADDRESS);
            Wire.write(0x00);
            if (Wire.endTransmission() == 0 &&
                Wire.requestFrom(static_cast<int>(Config::MODULE_I2C_ADDRESS), 1) == 1) {
                raw = Wire.read();
                readOk = true;
            } else {
                delayMicroseconds(100);
            }
        }
        if (!readOk) return false;

        const auto decode = [raw](uint8_t channel) {
            const bool high = (raw & (1U << channel)) != 0;
            return Config::LIMITS_ACTIVE_LOW ? !high : high;
        };
        g_zLimitState = decode(Config::Z_LIMIT_INPUT) ? 1 : 0;
        g_xLimitState = decode(Config::X_LIMIT_INPUT) ? 1 : 0;
        g_yLimitState = decode(Config::Y_LIMIT_INPUT) ? 1 : 0;
        active = decode(input);
        return true;
    }

    bool limitActive(uint8_t input) {
        bool active = false;
        readLimit(input, active);
        return active;
    }

   private:
    Module_Stepmotor module_;
};

StepperModule g_driver;

// Current physical motor positions. Direction tests confirmed this machine's
// CoreXY transform: A = X + Y and B = Y - X.
int32_t g_motorASteps = 0;
int32_t g_motorBSteps = 0;
int32_t g_motorZSteps = 0;

const char* stateName(DeviceState state) {
    switch (state) {
        case DeviceState::Booting: return "BOOTING";
        case DeviceState::NotHomed: return "NOT HOMED";
        case DeviceState::Homing: return "HOMING";
        case DeviceState::Ready: return "READY";
        case DeviceState::Positioning: return "POSITIONING";
        case DeviceState::Lowering: return "LOWERING TRAY";
        case DeviceState::Swishing: return "SWISHING";
        case DeviceState::Paused: return "PAUSED";
        case DeviceState::Stopping: return "SAFE STOP";
        case DeviceState::Error: return "ERROR";
    }
    return "UNKNOWN";
}

const char* motionTypeName(MotionType type) {
    switch (type) {
        case MotionType::Circular: return "circular";
        case MotionType::BackAndForth: return "back-and-forth";
        case MotionType::SideToSide: return "side-to-side";
        case MotionType::Combination: return "combination";
    }
    return "circular";
}

bool parseMotionType(const String& value, MotionType& type) {
    if (value == "circular") type = MotionType::Circular;
    else if (value == "back-and-forth") type = MotionType::BackAndForth;
    else if (value == "side-to-side") type = MotionType::SideToSide;
    else if (value == "combination") type = MotionType::Combination;
    else return false;
    return true;
}

uint32_t activeCycleElapsedMs() {
    const uint32_t started = g_cycleStartedMs;
    if (started == 0) return 0;
    uint32_t paused = g_accumulatedPauseMs;
    if (g_pauseRequested && g_pauseStartedMs != 0) paused += millis() - g_pauseStartedMs;
    return millis() - started - paused;
}

bool swishDurationExpired() {
    return g_activeDurationSeconds > 0 &&
           activeCycleElapsedMs() >= g_activeDurationSeconds * 1000UL;
}

void setDirection(uint8_t pin, bool positive, bool invert) {
    digitalWrite(pin, positive ^ invert ? HIGH : LOW);
}

void pulsePins(bool stepA, bool stepB, bool stepZ) {
    if (stepA) digitalWrite(Config::MOTOR_A_STEP_PIN, HIGH);
    if (stepB) digitalWrite(Config::MOTOR_B_STEP_PIN, HIGH);
    if (stepZ) digitalWrite(Config::MOTOR_Z_STEP_PIN, HIGH);
    delayMicroseconds(Config::STEP_PULSE_US);
    if (stepA) digitalWrite(Config::MOTOR_A_STEP_PIN, LOW);
    if (stepB) digitalWrite(Config::MOTOR_B_STEP_PIN, LOW);
    if (stepZ) digitalWrite(Config::MOTOR_Z_STEP_PIN, LOW);
}

void waitForStepInterval(uint32_t startedUs, uint32_t intervalUs) {
    // delayMicroseconds() busy-waits. At the Entosieve-style 50 step/s Z
    // rate, using it for the entire 20 ms interval starves core 0's idle task
    // and trips the ESP32 task watchdog. Sleep for the coarse portion and
    // reserve busy-waiting for only the final sub-millisecond remainder.
    for (;;) {
        const uint32_t elapsedUs = micros() - startedUs;
        if (elapsedUs >= intervalUs) return;

        const uint32_t remainingUs = intervalUs - elapsedUs;
        if (remainingUs > 2000) {
            const TickType_t sleepTicks = pdMS_TO_TICKS((remainingUs - 1000) / 1000);
            if (sleepTicks > 0) {
                vTaskDelay(sleepTicks);
                continue;
            }
        }
        delayMicroseconds(remainingUs);
        return;
    }
}

bool motionCheckpoint(DeviceState activeState, bool allowPause = true) {
    if (g_stopRequested) return false;
    if (activeState == DeviceState::Swishing && swishDurationExpired()) return false;
    if (!allowPause || !g_pauseRequested) return true;

    g_resumeState = activeState;
    g_state = DeviceState::Paused;
    while (g_pauseRequested && !g_stopRequested) vTaskDelay(pdMS_TO_TICKS(10));
    if (g_stopRequested) return false;
    g_state = activeState;
    return true;
}

bool moveRaw(int32_t targetA, int32_t targetB, int32_t targetZ,
             float maxStepRate, DeviceState activeState,
             bool allowAbort = true) {
    const int32_t deltaA = targetA - g_motorASteps;
    const int32_t deltaB = targetB - g_motorBSteps;
    const int32_t deltaZ = targetZ - g_motorZSteps;
    const uint32_t countA = abs(deltaA);
    const uint32_t countB = abs(deltaB);
    const uint32_t countZ = abs(deltaZ);
    const uint32_t total = max(countA, max(countB, countZ));
    if (total == 0) return true;

    setDirection(Config::MOTOR_A_DIR_PIN, deltaA >= 0, Config::INVERT_MOTOR_A);
    setDirection(Config::MOTOR_B_DIR_PIN, deltaB >= 0, Config::INVERT_MOTOR_B);
    setDirection(Config::MOTOR_Z_DIR_PIN, deltaZ >= 0, Config::INVERT_MOTOR_Z);

    const uint32_t interval = max(
        Config::MIN_STEP_INTERVAL_US,
        static_cast<uint32_t>(1000000.0f / max(1.0f, maxStepRate)));
    uint32_t accumulatorA = 0;
    uint32_t accumulatorB = 0;
    uint32_t accumulatorZ = 0;

    for (uint32_t i = 0; i < total; ++i) {
        if (allowAbort && !motionCheckpoint(activeState)) return false;
        const uint32_t stepStartedUs = micros();

        accumulatorA += countA;
        accumulatorB += countB;
        accumulatorZ += countZ;
        const bool stepA = accumulatorA >= total;
        const bool stepB = accumulatorB >= total;
        const bool stepZ = accumulatorZ >= total;
        if (stepA) accumulatorA -= total;
        if (stepB) accumulatorB -= total;
        if (stepZ) accumulatorZ -= total;

        pulsePins(stepA, stepB, stepZ);
        if (stepA) g_motorASteps += deltaA >= 0 ? 1 : -1;
        if (stepB) g_motorBSteps += deltaB >= 0 ? 1 : -1;
        if (stepZ) g_motorZSteps += deltaZ >= 0 ? 1 : -1;

        waitForStepInterval(stepStartedUs, interval);
    }
    return true;
}

bool moveTo(float xMm, float yMm, float zMm, float speedMmS,
            DeviceState activeState, bool allowAbort = true) {
    const int32_t targetA = lroundf((xMm + yMm) * Config::XY_STEPS_PER_MM);
    const int32_t targetB = lroundf((yMm - xMm) * Config::XY_STEPS_PER_MM);
    const int32_t targetZ = lroundf(zMm * Config::Z_STEPS_PER_MM);

    // Make every segment take distance/speed seconds. This avoids the
    // direction-dependent speed variation that a fixed CoreXY motor rate
    // would produce around a circle.
    const float dx = xMm - g_xMm;
    const float dy = yMm - g_yMm;
    const float dz = zMm - g_zMm;
    const float pathMm = max(sqrtf(dx * dx + dy * dy), fabsf(dz));
    const uint32_t motorSteps = max(
        static_cast<uint32_t>(abs(targetA - g_motorASteps)),
        max(static_cast<uint32_t>(abs(targetB - g_motorBSteps)),
            static_cast<uint32_t>(abs(targetZ - g_motorZSteps))));
    const float durationSeconds = pathMm / max(0.1f, speedMmS);
    const float stepRate = motorSteps / max(0.001f, durationSeconds);
    const bool ok = moveRaw(targetA, targetB, targetZ, stepRate,
                            activeState, allowAbort);

    g_xMm = (g_motorASteps - g_motorBSteps) / (2.0f * Config::XY_STEPS_PER_MM);
    g_yMm = (g_motorASteps + g_motorBSteps) / (2.0f * Config::XY_STEPS_PER_MM);
    g_zMm = g_motorZSteps / Config::Z_STEPS_PER_MM;
    return ok;
}

bool homeOneAxis(uint8_t limitInput, bool stepA, bool dirA,
                 bool stepB, bool dirB, bool stepZ, bool dirZ,
                 uint32_t maximumSteps, float stepRate) {
    setDirection(Config::MOTOR_A_DIR_PIN, dirA, Config::INVERT_MOTOR_A);
    setDirection(Config::MOTOR_B_DIR_PIN, dirB, Config::INVERT_MOTOR_B);
    setDirection(Config::MOTOR_Z_DIR_PIN, dirZ, Config::INVERT_MOTOR_Z);
    const uint32_t interval = max(
        Config::MIN_STEP_INTERVAL_US,
        static_cast<uint32_t>(1000000.0f / max(1.0f, stepRate)));
    for (uint32_t i = 0; i < maximumSteps; ++i) {
        if (!motionCheckpoint(DeviceState::Homing)) return false;
        bool limitActive = false;
        if (!g_driver.readLimit(limitInput, limitActive)) {
            Serial.printf("HOME: limit L%u read failed; motion aborted\n", limitInput);
            g_error = "Limit input read failed; motion aborted";
            return false;
        }
        // Stop on the first asserted sample. Requiring repeated samples can
        // miss a lever switch that releases again as the mechanism flexes.
        if (limitActive) return true;

        const uint32_t stepStartedUs = micros();
        pulsePins(stepA, stepB, stepZ);
        if (stepA) g_motorASteps += dirA ? 1 : -1;
        if (stepB) g_motorBSteps += dirB ? 1 : -1;
        if (stepZ) g_motorZSteps += dirZ ? 1 : -1;
        waitForStepInterval(stepStartedUs, interval);
    }
    return false;
}

bool releaseLimitForFreshEdge(uint8_t limitInput, bool stepA, bool dirA,
                              bool stepB, bool dirB,
                              uint32_t maximumSteps, float stepRate) {
    setDirection(Config::MOTOR_A_DIR_PIN, dirA, Config::INVERT_MOTOR_A);
    setDirection(Config::MOTOR_B_DIR_PIN, dirB, Config::INVERT_MOTOR_B);
    const uint32_t interval = max(
        Config::MIN_STEP_INTERVAL_US,
        static_cast<uint32_t>(1000000.0f / max(1.0f, stepRate)));

    for (uint32_t i = 0; i < maximumSteps; ++i) {
        if (!motionCheckpoint(DeviceState::Homing)) return false;
        bool limitActive = false;
        if (!g_driver.readLimit(limitInput, limitActive)) {
            Serial.printf("HOME: limit L%u read failed while releasing\n",
                          limitInput);
            g_error = "Limit input read failed; motion aborted";
            return false;
        }
        if (!limitActive) return true;

        const uint32_t stepStartedUs = micros();
        pulsePins(stepA, stepB, false);
        if (stepA) g_motorASteps += dirA ? 1 : -1;
        if (stepB) g_motorBSteps += dirB ? 1 : -1;
        waitForStepInterval(stepStartedUs, interval);
    }

    Serial.printf("HOME: limit L%u stayed active during release\n", limitInput);
    g_error = "Limit stayed active while backing off";
    return false;
}

bool performHome() {
    Serial.println("HOME: requested");
    g_error = "";
    g_state = DeviceState::Homing;
    g_homed = false;
    g_stopRequested = false;
    g_pauseRequested = false;
    if (!g_driver.enable(true)) {
        Serial.println("HOME: failed to enable/release motor resets");
        return false;
    }

    // Home Z upward to L0 first.
    if (!homeOneAxis(Config::Z_LIMIT_INPUT, false, false, false, false,
                     true, false,
                     lroundf(Config::Z_HOME_TRAVEL_MM * Config::Z_STEPS_PER_MM),
                     Config::HOME_SPEED_MM_S * Config::Z_STEPS_PER_MM)) {
        Serial.println(g_stopRequested ? "HOME: stopped" : "HOME: L0 not reached");
        return false;
    }
    g_motorZSteps = 0;
    g_zMm = 0;
    Serial.println("HOME: L0 hit; Z set to 0");

    // Confirmed direction mapping: physical X uses A/B in opposite directions,
    // while physical Y uses A/B together.
    const bool xPositive = Config::X_HOME_DIRECTION_POSITIVE;
    Serial.println("HOME: ensuring L1 is open before X seek");
    if (!releaseLimitForFreshEdge(
            Config::X_LIMIT_INPUT,
            true, !xPositive, true, xPositive,
            lroundf(Config::XY_HOME_EDGE_RELEASE_MAX_MM *
                     Config::XY_STEPS_PER_MM),
            Config::XY_HOME_SPEED_MM_S * Config::XY_STEPS_PER_MM)) {
        Serial.println("HOME: could not release L1");
        return false;
    }
    if (!homeOneAxis(Config::X_LIMIT_INPUT,
                     true, xPositive, true, !xPositive, false, false,
                     lroundf(Config::XY_HOME_TRAVEL_MM * Config::XY_STEPS_PER_MM),
                     Config::XY_HOME_SPEED_MM_S * Config::XY_STEPS_PER_MM)) {
        Serial.println(g_stopRequested ? "HOME: stopped" : "HOME: L1 not reached");
        return false;
    }
    Serial.println("HOME: L1 hit; physical X at 0");

    const bool yPositive = Config::Y_HOME_DIRECTION_POSITIVE;
    Serial.println("HOME: ensuring L2 is open before Y seek");
    if (!releaseLimitForFreshEdge(
            Config::Y_LIMIT_INPUT,
            true, !yPositive, true, !yPositive,
            lroundf(Config::XY_HOME_EDGE_RELEASE_MAX_MM *
                     Config::XY_STEPS_PER_MM),
            Config::XY_HOME_SPEED_MM_S * Config::XY_STEPS_PER_MM)) {
        Serial.println("HOME: could not release L2");
        return false;
    }
    if (!homeOneAxis(Config::Y_LIMIT_INPUT,
                     true, yPositive, true, yPositive, false, false,
                     lroundf(Config::XY_HOME_TRAVEL_MM * Config::XY_STEPS_PER_MM),
                     Config::XY_HOME_SPEED_MM_S * Config::XY_STEPS_PER_MM)) {
        Serial.println(g_stopRequested ? "HOME: stopped" : "HOME: L2 not reached");
        return false;
    }
    Serial.println("HOME: L2 hit; physical Y at 0");

    g_motorASteps = 0;
    g_motorBSteps = 0;
    g_xMm = 0;
    g_yMm = 0;
    Serial.println("HOME: all axes set to 0");

    g_state = DeviceState::Positioning;
    const float releaseX = Config::XY_PARK_X_DIRECTION *
                           Config::XY_SWITCH_RELEASE_MM;
    const float releaseY = Config::XY_PARK_Y_DIRECTION *
                           Config::XY_SWITCH_RELEASE_MM;

    // Leave each switch separately at homing speed before starting the longer
    // CoreXY park move. This removes corner preload and avoids asking both
    // belts to accelerate while the carriage is still against its end stops.
    Serial.println("HOME: releasing L1");
    if (!moveTo(releaseX, 0, 0, Config::XY_HOME_SPEED_MM_S,
                DeviceState::Positioning)) {
        Serial.println("HOME: L1 release stopped");
        return false;
    }
    Serial.println("HOME: releasing L2");
    if (!moveTo(releaseX, releaseY, 0, Config::XY_HOME_SPEED_MM_S,
                DeviceState::Positioning)) {
        Serial.println("HOME: L2 release stopped");
        return false;
    }

    Serial.printf("HOME: parking XY at %.1f, %.1f mm\n",
                  Config::XY_PARK_X_DIRECTION * Config::XY_PARK_X_MM,
                  Config::XY_PARK_Y_DIRECTION * Config::XY_PARK_Y_MM);
    if (!moveTo(Config::XY_PARK_X_DIRECTION * Config::XY_PARK_X_MM,
                Config::XY_PARK_Y_DIRECTION * Config::XY_PARK_Y_MM, 0,
                Config::XY_PARK_SPEED_MM_S, DeviceState::Positioning)) {
        Serial.println("HOME: parking stopped");
        return false;
    }

    g_homed = true;
    Serial.println("HOME: parking complete");
    return true;
}

void safeStopAndRaise() {
    g_pauseRequested = false;
    g_stopRequested = false;
    g_state = DeviceState::Stopping;

    // Complete a controlled return to the circle center, then raise Z home.
    moveTo(g_cycleCenterX, g_cycleCenterY, g_zMm,
           g_activeSwishSpeedMmS, DeviceState::Stopping, false);
    moveTo(g_cycleCenterX, g_cycleCenterY, 0,
           Config::Z_SPEED_MM_S, DeviceState::Stopping, false);
    g_cycleStartedMs = 0;
    g_activeDurationSeconds = 0;
    g_state = DeviceState::Ready;
}

bool runLinearPattern(float dx, float dy, const CycleSettings& settings) {
    if (!moveTo(g_cycleCenterX + dx, g_cycleCenterY + dy,
                Config::Z_LOWER_DISTANCE_MM, settings.speedMmS,
                DeviceState::Swishing)) return false;
    if (!moveTo(g_cycleCenterX - dx, g_cycleCenterY - dy,
                Config::Z_LOWER_DISTANCE_MM, settings.speedMmS,
                DeviceState::Swishing)) return false;
    return moveTo(g_cycleCenterX, g_cycleCenterY,
                  Config::Z_LOWER_DISTANCE_MM, settings.speedMmS,
                  DeviceState::Swishing);
}

bool runCircle(bool clockwise, const CycleSettings& settings) {
    if (!moveTo(g_cycleCenterX + settings.sizeMm, g_cycleCenterY,
                Config::Z_LOWER_DISTANCE_MM, settings.speedMmS,
                DeviceState::Swishing)) return false;

    const float direction = clockwise ? -1.0f : 1.0f;
    for (uint16_t segment = 1; segment <= Config::SWISH_CIRCLE_SEGMENTS; ++segment) {
        const float angle = direction * TWO_PI * segment /
                            Config::SWISH_CIRCLE_SEGMENTS;
        if (!moveTo(g_cycleCenterX + settings.sizeMm * cosf(angle),
                    g_cycleCenterY + settings.sizeMm * sinf(angle),
                    Config::Z_LOWER_DISTANCE_MM, settings.speedMmS,
                    DeviceState::Swishing)) return false;
    }
    return moveTo(g_cycleCenterX, g_cycleCenterY,
                  Config::Z_LOWER_DISTANCE_MM, settings.speedMmS,
                  DeviceState::Swishing);
}

bool runPattern(MotionType type, const CycleSettings& settings) {
    switch (type) {
        case MotionType::Circular:
            return runCircle(true, settings) && runCircle(false, settings);
        case MotionType::BackAndForth:
            return runLinearPattern(0, settings.sizeMm, settings);
        case MotionType::SideToSide:
            return runLinearPattern(settings.sizeMm, 0, settings);
        case MotionType::Combination:
            return runCircle(true, settings) && runCircle(false, settings) &&
                   runLinearPattern(settings.sizeMm, 0, settings) &&
                   runLinearPattern(0, settings.sizeMm, settings);
    }
    return false;
}

void performCycle() {
    if (!g_homed) {
        g_state = DeviceState::NotHomed;
        return;
    }
    g_stopRequested = false;
    g_pauseRequested = false;
    g_driver.enable(true);

    const CycleSettings settings{
        g_motionType,
        constrain(static_cast<float>(g_swishSpeedMmS),
                  Config::MIN_SWISH_SPEED_MM_S, Config::MAX_SWISH_SPEED_MM_S),
        constrain(static_cast<float>(g_swishSizeMm),
                  Config::MIN_SWISH_SIZE_MM, Config::MAX_SWISH_SIZE_MM),
        constrain(static_cast<uint32_t>(g_swishDurationSeconds),
                  Config::MIN_SWISH_DURATION_SECONDS,
                  Config::MAX_SWISH_DURATION_SECONDS),
    };
    g_activeSwishSpeedMmS = settings.speedMmS;
    g_activeDurationSeconds = settings.durationSeconds;

    g_cycleCenterX = g_xMm;
    g_cycleCenterY = g_yMm;

    g_state = DeviceState::Lowering;
    if (!moveTo(g_cycleCenterX, g_cycleCenterY,
                Config::Z_LOWER_DISTANCE_MM, Config::Z_SPEED_MM_S,
                DeviceState::Lowering)) {
        safeStopAndRaise();
        return;
    }

    g_state = DeviceState::Swishing;
    g_cycleStartedMs = millis();
    g_pauseStartedMs = 0;
    g_accumulatedPauseMs = 0;
    while (!g_stopRequested && !swishDurationExpired()) {
        if (!runPattern(settings.type, settings)) break;
    }
    safeStopAndRaise();
}

void motionTask(void*) {
    for (;;) {
        uint32_t command = 0;
        xTaskNotifyWait(0, UINT32_MAX, &command, portMAX_DELAY);
        if (command == static_cast<uint32_t>(MotionCommand::Home)) {
            if (performHome()) {
                g_state = DeviceState::Ready;
            } else if (g_stopRequested) {
                g_stopRequested = false;
                g_state = DeviceState::NotHomed;
            } else {
                if (g_error.length() == 0) {
                    g_error = "Home failed: check switches";
                }
                g_state = DeviceState::Error;
            }
        } else if (command == static_cast<uint32_t>(MotionCommand::Start)) {
            performCycle();
        }

        // A command returns only after all requested motion has stopped. Drop
        // the driver's global enable so the motors do not apply holding
        // current while the machine is idle. Keep them energized while paused
        // inside a command so the suspended tray cannot drift and lose its
        // known position.
        g_driver.enable(false);
        Serial.println("MOTORS: idle holding current disabled");
    }
}

bool isMoving(DeviceState state) {
    return state == DeviceState::Homing || state == DeviceState::Positioning ||
           state == DeviceState::Lowering || state == DeviceState::Swishing ||
           state == DeviceState::Paused || state == DeviceState::Stopping;
}

void drawScreen() {
    static uint32_t lastDraw = 0;
    static bool layoutDrawn = false;
    static DeviceState previousState = static_cast<DeviceState>(0xFF);
    static bool homedDrawn = false;
    static bool previousHomed = false;
    static String previousWifi;
    static String previousPosition;
    static String previousError;
    static int8_t previousZLimitState = -2;
    static int8_t previousXLimitState = -2;
    static int8_t previousYLimitState = -2;
    if (millis() - lastDraw < Config::DISPLAY_REFRESH_MS) return;
    lastDraw = millis();

    const DeviceState state = g_state;
    M5.Display.startWrite();
    if (!layoutDrawn) {
        M5.Display.fillScreen(TFT_BLACK);
        M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
        M5.Display.setTextSize(2);
        M5.Display.drawString("BUG DISPERSER", 10, 8);
        M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
        M5.Display.setTextSize(1);
        M5.Display.drawCentreString("HOME", 53, 210);
        layoutDrawn = true;
    }

    if (state != previousState) {
        M5.Display.fillRect(10, 42, 300, 25, TFT_BLACK);
        M5.Display.setTextColor(state == DeviceState::Error ? TFT_RED : TFT_GREEN,
                                TFT_BLACK);
        M5.Display.setTextSize(2);
        M5.Display.drawString(stateName(state), 10, 42);

        M5.Display.fillRect(100, 205, 210, 25, TFT_BLACK);
        M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
        M5.Display.setTextSize(1);
        M5.Display.drawCentreString(isMoving(state) ? "STOP" : "START", 160, 210);
        M5.Display.drawCentreString(state == DeviceState::Paused ? "RESUME" : "PAUSE",
                                    267, 210);
        previousState = state;
    }

    const String wifi = WiFi.status() == WL_CONNECTED
                            ? "disperser.local  " + WiFi.localIP().toString()
                            : "WiFi: connecting...";
    if (wifi != previousWifi) {
        M5.Display.fillRect(10, 75, 300, 18, TFT_BLACK);
        M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        M5.Display.setTextSize(1);
        M5.Display.drawString(wifi, 10, 75);
        previousWifi = wifi;
    }

    char position[80];
    snprintf(position, sizeof(position), "X %6.1f   Y %6.1f   Z %6.1f mm",
             static_cast<double>(g_xMm), static_cast<double>(g_yMm),
             static_cast<double>(g_zMm));
    const String currentPosition(position);
    if (currentPosition != previousPosition) {
        // This line has fixed-width fields, so painting glyph backgrounds is
        // enough to replace it without a visible clear/redraw flash.
        M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        M5.Display.setTextSize(1);
        M5.Display.drawString(position, 10, 100);
        previousPosition = currentPosition;
    }

    if (!homedDrawn || g_homed != previousHomed) {
        M5.Display.fillRect(10, 122, 300, 18, TFT_BLACK);
        M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        M5.Display.setTextSize(1);
        M5.Display.drawString(g_homed ? "Machine homed" : "Home required", 10, 122);
        previousHomed = g_homed;
        homedDrawn = true;
    }

    if (g_zLimitState != previousZLimitState ||
        g_xLimitState != previousXLimitState ||
        g_yLimitState != previousYLimitState) {
        M5.Display.fillRect(10, 145, 300, 18, TFT_BLACK);
        M5.Display.setTextColor((g_zLimitState == 1 || g_xLimitState == 1 ||
                                 g_yLimitState == 1) ? TFT_RED : TFT_WHITE,
                                TFT_BLACK);
        M5.Display.setTextSize(1);
        char limits[64];
        snprintf(limits, sizeof(limits), "L0 Z:%s   L1 X:%s   L2 Y:%s",
                 g_zLimitState < 0 ? "?" : (g_zLimitState ? "HIT" : "open"),
                 g_xLimitState < 0 ? "?" : (g_xLimitState ? "HIT" : "open"),
                 g_yLimitState < 0 ? "?" : (g_yLimitState ? "HIT" : "open"));
        M5.Display.drawString(limits, 10, 145);
        previousZLimitState = g_zLimitState;
        previousXLimitState = g_xLimitState;
        previousYLimitState = g_yLimitState;
    }

    const String error = state == DeviceState::Error ? g_error : String();
    if (error != previousError) {
        M5.Display.fillRect(10, 165, 300, 18, TFT_BLACK);
        if (!error.isEmpty()) {
            M5.Display.setTextColor(TFT_RED, TFT_BLACK);
            M5.Display.setTextSize(1);
            M5.Display.drawString(error, 10, 165);
        }
        previousError = error;
    }
    M5.Display.endWrite();
}

bool requestHome() {
    const DeviceState state = g_state;
    if (isMoving(state) || !g_motionTask) return false;
    Serial.println("COMMAND: Home");
    xTaskNotify(g_motionTask, static_cast<uint32_t>(MotionCommand::Home),
                eSetValueWithOverwrite);
    return true;
}

bool requestStartStop() {
    const DeviceState state = g_state;
    if (isMoving(state)) {
        if (state == DeviceState::Stopping) return false;
        g_stopRequested = true;
        return true;
    }
    if (!g_homed || !g_motionTask) {
        g_state = DeviceState::NotHomed;
        return false;
    }
    xTaskNotify(g_motionTask, static_cast<uint32_t>(MotionCommand::Start),
                eSetValueWithOverwrite);
    return true;
}

bool requestPauseResume() {
    const DeviceState state = g_state;
    if (!isMoving(state) || state == DeviceState::Stopping) return false;
    const bool pause = !g_pauseRequested;
    if (pause) {
        g_pauseStartedMs = millis();
        g_pauseRequested = true;
    } else {
        if (g_pauseStartedMs != 0) g_accumulatedPauseMs += millis() - g_pauseStartedMs;
        g_pauseStartedMs = 0;
        g_pauseRequested = false;
    }
    return true;
}

void handleButtons() {
    if (M5.BtnA.wasPressed()) requestHome();
    if (M5.BtnB.wasPressed()) requestStartStop();
    if (M5.BtnC.wasPressed()) requestPauseResume();
}

String statusJson() {
    String json;
    json.reserve(640);
    const DeviceState state = g_state;
    json = "{";
    json += "\"state\":\"" + String(stateName(state)) + "\",";
    json += "\"moving\":" + String(isMoving(state) ? "true" : "false") + ",";
    json += "\"paused\":" + String(state == DeviceState::Paused ? "true" : "false") + ",";
    json += "\"stopping\":" + String(state == DeviceState::Stopping ? "true" : "false") + ",";
    json += "\"error\":" + String(state == DeviceState::Error ? "true" : "false") + ",";
    json += "\"homed\":" + String(g_homed ? "true" : "false") + ",";
    json += "\"limit\":" + String(g_zLimitState) + ",";
    json += "\"limitZ\":" + String(g_zLimitState) + ",";
    json += "\"limitX\":" + String(g_xLimitState) + ",";
    json += "\"limitY\":" + String(g_yLimitState) + ",";
    json += "\"x\":" + String(g_xMm, 2) + ",";
    json += "\"y\":" + String(g_yMm, 2) + ",";
    json += "\"z\":" + String(g_zMm, 2) + ",";
    json += "\"elapsed\":" + String(activeCycleElapsedMs() / 1000UL) + ",";
    json += "\"activeDuration\":" + String(g_activeDurationSeconds) + ",";
    json += "\"motion\":\"" + String(motionTypeName(g_motionType)) + "\",";
    json += "\"speed\":" + String(g_swishSpeedMmS, 1) + ",";
    json += "\"duration\":" + String(g_swishDurationSeconds) + ",";
    json += "\"size\":" + String(g_swishSizeMm, 1) + ",";
    json += "\"mdns\":" + String(g_mdnsReady ? "true" : "false") + ",";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\"";
    json += "}";
    return json;
}

void sendCommandResult(bool accepted, const char* message) {
    const int code = accepted ? 200 : 409;
    String body = accepted ? "{\"message\":\"" : "{\"error\":\"";
    body += message;
    body += "\"}";
    g_server.send(code, "application/json", body);
}

void setupWebRoutes() {
    g_server.on("/", HTTP_GET, []() {
        g_server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
        g_server.sendHeader("Pragma", "no-cache");
        g_server.send_P(200, "text/html", DISPERSER_WEB_PAGE);
    });
    g_server.on("/api/status", HTTP_GET, []() {
        g_server.sendHeader("Cache-Control", "no-store");
        g_server.send(200, "application/json", statusJson());
    });
    g_server.on("/api/home", HTTP_POST, []() {
        sendCommandResult(requestHome(), "Home accepted");
    });
    g_server.on("/api/start", HTTP_POST, []() {
        const bool wasMoving = isMoving(g_state);
        sendCommandResult(requestStartStop(),
                          wasMoving ? "Safe stop requested" :
                                      (g_homed ? "Cycle started" : "Home Z first"));
    });
    g_server.on("/api/pause", HTTP_POST, []() {
        const bool wasPaused = g_pauseRequested;
        sendCommandResult(requestPauseResume(), wasPaused ? "Resumed" : "Paused");
    });
    g_server.on("/api/settings", HTTP_POST, []() {
        MotionType type = g_motionType;
        if (g_server.hasArg("motion") && !parseMotionType(g_server.arg("motion"), type)) {
            g_server.send(400, "application/json", "{\"error\":\"Unknown motion type\"}");
            return;
        }
        const float speed = constrain(g_server.hasArg("speed") ?
                                          g_server.arg("speed").toFloat() :
                                          static_cast<float>(g_swishSpeedMmS),
                                      Config::MIN_SWISH_SPEED_MM_S,
                                      Config::MAX_SWISH_SPEED_MM_S);
        const float size = constrain(g_server.hasArg("size") ?
                                         g_server.arg("size").toFloat() :
                                         static_cast<float>(g_swishSizeMm),
                                     Config::MIN_SWISH_SIZE_MM,
                                     Config::MAX_SWISH_SIZE_MM);
        const uint32_t duration = constrain(
            g_server.hasArg("duration") ?
                static_cast<uint32_t>(g_server.arg("duration").toInt()) :
                static_cast<uint32_t>(g_swishDurationSeconds),
            Config::MIN_SWISH_DURATION_SECONDS,
            Config::MAX_SWISH_DURATION_SECONDS);
        g_motionType = type;
        g_swishSpeedMmS = speed;
        g_swishSizeMm = size;
        g_swishDurationSeconds = duration;
        g_server.send(200, "application/json", statusJson());
    });
    g_server.onNotFound([]() {
        g_server.send(404, "application/json", "{\"error\":\"Not found\"}");
    });
}

}  // namespace

void setup() {
    auto cfg = M5.config();
    cfg.clear_display = true;
    M5.begin(cfg);
    Serial.begin(115200);

    pinMode(Config::MOTOR_A_STEP_PIN, OUTPUT);
    pinMode(Config::MOTOR_A_DIR_PIN, OUTPUT);
    pinMode(Config::MOTOR_B_STEP_PIN, OUTPUT);
    pinMode(Config::MOTOR_B_DIR_PIN, OUTPUT);
    pinMode(Config::MOTOR_Z_STEP_PIN, OUTPUT);
    pinMode(Config::MOTOR_Z_DIR_PIN, OUTPUT);
    digitalWrite(Config::MOTOR_A_STEP_PIN, LOW);
    digitalWrite(Config::MOTOR_B_STEP_PIN, LOW);
    digitalWrite(Config::MOTOR_Z_STEP_PIN, LOW);

    M5.Display.setBrightness(90);
    g_state = DeviceState::Booting;
    drawScreen();

    if (!g_driver.begin()) {
        g_error = "M039 not found at I2C 0x27";
        g_state = DeviceState::Error;
    } else {
        g_state = DeviceState::NotHomed;
    }

    // Read all limit inputs while stationary so wiring state is visible.
    g_driver.limitActive(Config::Z_LIMIT_INPUT);
    g_driver.limitActive(Config::X_LIMIT_INPUT);
    g_driver.limitActive(Config::Y_LIMIT_INPUT);

    WiFi.mode(WIFI_STA);
    WiFi.setHostname(Config::MDNS_HOSTNAME);
    WiFi.setAutoReconnect(true);
    WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);

    setupWebRoutes();
    g_server.begin();

    xTaskCreatePinnedToCore(motionTask, "motion", 6144, nullptr, 1,
                            &g_motionTask, 0);
}

void loop() {
    static uint32_t lastWifiAttempt = 0;
    static uint32_t lastLimitRead = 0;
    M5.update();
    handleButtons();
    g_server.handleClient();

    if (!isMoving(g_state) && millis() - lastLimitRead >= 250) {
        lastLimitRead = millis();
        g_driver.limitActive(Config::Z_LIMIT_INPUT);
        g_driver.limitActive(Config::X_LIMIT_INPUT);
        g_driver.limitActive(Config::Y_LIMIT_INPUT);
    }
    drawScreen();

    if (WiFi.status() != WL_CONNECTED &&
        millis() - lastWifiAttempt >= Config::WIFI_RETRY_MS) {
        lastWifiAttempt = millis();
        WiFi.reconnect();
    }

    if (WiFi.status() == WL_CONNECTED && !g_mdnsReady) {
        g_mdnsReady = MDNS.begin(Config::MDNS_HOSTNAME);
        if (g_mdnsReady) {
            MDNS.addService("http", "tcp", 80);
            Serial.println("Web UI: http://disperser.local/");
        }
    } else if (WiFi.status() != WL_CONNECTED && g_mdnsReady) {
        MDNS.end();
        g_mdnsReady = false;
    }
    delay(5);
}
