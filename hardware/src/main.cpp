/**
 * @file    main.cpp
 * @brief   Dual-motor closed-loop PID controller for a differential-drive robot.
 *
 * Receives velocity commands over UART from a host (e.g. ROS 2 micro-ROS bridge)
 * in the format:  CMD,<left_rad_s>,<right_rad_s>\n
 * Sends wheel velocity feedback at 50 Hz in the format:  STATE,<left_rad_s>,<right_rad_s>\n
 *
 * Protocol:
 *   Host → MCU:  "START\n"       – initiates communication
 *   MCU  → Host: "READY\n"       – confirms ready
 *   Host → MCU:  "CMD,L,R\n"     – set velocities (rad/s)
 *   MCU  → Host: "STATE,L,R\n"   – measured velocities (rad/s)
 *
 * If no command is received within COMMAND_TIMEOUT ms, motors stop and
 * communication_ready is reset (safety watchdog).
 */

#include <Arduino.h>

// Motor 1 (Left) — encoder A must be on a hardware-interrupt pin (2 or 3 on Nano)
#define ENCODER_A   3   // INT1 – rising-edge interrupt
#define ENCODER_B   5   // Direction sense pin (read inside ISR)

#define MOTOR_PWM   10  // PWM speed output for Motor 1
#define MOTOR_DIR   7   // Direction output for Motor 1

// Motor 2 (Right) — encoder C on INT0
#define ENCODER_C   2   // INT0 – rising-edge interrupt
#define ENCODER_D   4   // Direction sense pin (read inside ISR)

#define MOTOR2_PWM  9   // PWM speed output for Motor 2
#define MOTOR2_DIR  8   // Direction output for Motor 2

// Flip to true if the measured encoder direction is opposite to the motor direction
#define INVERT_ENCODER_1  true   // Invert Motor 1 encoder counting direction
#define INVERT_ENCODER_2  false  // Invert Motor 2 encoder counting direction

static const int MAX_PWM = 255;
static const int MIN_PWM = 0;

// Marked volatile because they are written inside ISRs and read in the main loop
volatile long Motor1count = 0;  // Raw pulse count for Motor 1
volatile long Motor2count = 0;  // Raw pulse count for Motor 2

// Pulses per revolution of the output shaft (encoder resolution × gear ratio)
static const float PPR = 188.3f;

static long previousCount1 = 0;        // Encoder snapshot at last RPM update (Motor 1)
static long previousCount2 = 0;        // Encoder snapshot at last RPM update (Motor 2)
static unsigned long previousTime = 0; // Timestamp (ms) of last RPM update

static const unsigned long RPM_INTERVAL = 100; // RPM update period in ms

static float actualRPM1 = 0.0f;  // Measured RPM for Motor 1
static float actualRPM2 = 0.0f;  // Measured RPM for Motor 2

static float targetRPM1 = 0.0f;  // Desired RPM for Motor 1 (derived from rad/s command)
static float targetRPM2 = 0.0f;  // Desired RPM for Motor 2 (derived from rad/s command)

// PID gains — tune these for your hardware
static float Kp = 0.5f;    // Proportional gain
static float Ki = 0.001f;  // Integral gain
static float Kd = 0.02f;   // Derivative gain

// Prevents integrator from accumulating excessively
static const float INTEGRAL_CLAMP = 500.0f;

// PID state – Motor 1 (Left)
static float error1         = 0.0f;
static float previousError1 = 0.0f;
static float integral1      = 0.0f;
static float derivative1    = 0.0f;
static float pidOutput1     = 0.0f;
static float pwm1           = 0.0f;  // Accumulated PWM (float for smooth integration)

// PID state – Motor 2 (Right)
static float error2         = 0.0f;
static float previousError2 = 0.0f;
static float integral2      = 0.0f;
static float derivative2    = 0.0f;
static float pidOutput2     = 0.0f;
static float pwm2           = 0.0f;

#define BAUD_RATE 115200

static const unsigned long COMMAND_TIMEOUT   = 230; // ms – stop motors if no CMD received
static const unsigned long FEEDBACK_INTERVAL = 20;  // ms – feedback rate (~50 Hz)

// Set to 1 for standalone Serial Monitor debugging.
// Must be 0 when connected to a ROS bridge to avoid corrupting the binary protocol.
#define ENABLE_DEBUG_OUTPUT 1

static bool communication_ready    = false; // True after host sends "START"
static unsigned long last_command_time  = 0; // Timestamp of last valid CMD message
static unsigned long last_feedback_time = 0; // Timestamp of last STATE transmission

// Latest velocity setpoints received from the host (rad/s)
static float left_target_velocity  = 0.0f;
static float right_target_velocity = 0.0f;

/**
 * Convert RPM to angular velocity in rad/s.
 *   rad/s = RPM × π / 30
 */
static inline float rpmToRadS(float rpm) {
    return rpm * (PI / 30.0f);
}

/**
 * Convert angular velocity (rad/s) to RPM.
 *   RPM = rad/s × 30 / π
 */
static inline float radSToRpm(float rad_s) {
    return rad_s * (30.0f / PI);
}

/**
 * Drive Motor 1 at the given PWM level.
 * @param pwm  Desired duty cycle [0, 255]. Clamped internally.
 *
 * NOTE: Direction is currently hardcoded to HIGH (forward).
 *       To support reverse, the sign of the velocity should control MOTOR_DIR
 *       and pwm should be the absolute value.
 */
static void setMotor1(int pwm) {
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM, pwm);
}

/**
 * Drive Motor 2 at the given PWM level.
 * @param pwm  Desired duty cycle [0, 255]. Clamped internally.
 */
static void setMotor2(int pwm) {
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);
    digitalWrite(MOTOR2_DIR, HIGH);
    analogWrite(MOTOR2_PWM, pwm);
}

/**
 * ISR for Motor 1 encoder (triggered on RISING edge of ENCODER_A).
 * Increments or decrements Motor1count based on the state of ENCODER_B
 * for quadrature direction decoding. Keep this as short as possible.
 */
void encoderISR1() {
#if INVERT_ENCODER_1
    if (digitalRead(ENCODER_B)) Motor1count--;
    else                         Motor1count++;
#else
    if (digitalRead(ENCODER_B)) Motor1count++;
    else                         Motor1count--;
#endif
}

/**
 * ISR for Motor 2 encoder (triggered on RISING edge of ENCODER_C).
 */
void encoderISR2() {
#if INVERT_ENCODER_2
    if (digitalRead(ENCODER_D)) Motor2count--;
    else                         Motor2count++;
#else
    if (digitalRead(ENCODER_D)) Motor2count++;
    else                         Motor2count--;
#endif
}

/**
 * Set new velocity targets for both motors.
 * Converts rad/s → RPM for the PID loop.
 * If a target is exactly 0, the motor is stopped immediately and the
 * integrator is cleared to prevent windup on restart.
 *
 * @param left   Left wheel velocity  (rad/s)
 * @param right  Right wheel velocity (rad/s)
 */
static void setMotorTargets(float left, float right) {
    left_target_velocity  = left;
    right_target_velocity = right;

    targetRPM1 = radSToRpm(left);
    targetRPM2 = radSToRpm(right);

    // Immediately kill PWM and reset integrator when target is zero
    if (left == 0.0f) {
        pwm1      = 0.0f;
        integral1 = 0.0f;
        setMotor1(0);
    }
    if (right == 0.0f) {
        pwm2      = 0.0f;
        integral2 = 0.0f;
        setMotor2(0);
    }
}

/**
 * Emergency stop – zero all targets and PID state, then halt both motors.
 * Called on communication timeout or at startup.
 */
static void stopMotors() {
    left_target_velocity  = 0.0f;
    right_target_velocity = 0.0f;

    targetRPM1 = 0.0f;
    targetRPM2 = 0.0f;

    pwm1           = 0.0f;
    pwm2           = 0.0f;
    integral1      = 0.0f;
    integral2      = 0.0f;
    previousError1 = 0.0f;
    previousError2 = 0.0f;

    setMotor1(0);
    setMotor2(0);
}

/** Return the last measured left wheel velocity in rad/s. */
static float getLeftMeasuredVelocity() {
    return rpmToRadS(actualRPM1);
}

/** Return the last measured right wheel velocity in rad/s. */
static float getRightMeasuredVelocity() {
    return rpmToRadS(actualRPM2);
}

/**
 * Update RPM measurements and run PID for both motors.
 * Called every loop() iteration; internally rate-limited to RPM_INTERVAL.
 *
 * Safety notes:
 *  - Encoder counts are read atomically with interrupts disabled.
 *  - dt is guarded against zero/negative values to avoid division by zero
 *    and derivative spikes when millis() wraps (happens every ~49 days).
 *  - Integral windup is clamped to ±INTEGRAL_CLAMP.
 *  - PWM output is clamped to [MIN_PWM, MAX_PWM] before being applied.
 */
static void updateMotorControl() {
    unsigned long currentTime = millis();

    // Rate-limit: only update at RPM_INTERVAL cadence
    if (currentTime - previousTime < RPM_INTERVAL) return;

    // Compute elapsed time (dt) in seconds.
    // If the interval is somehow 0 (e.g. repeated fast calls), skip to avoid
    // division by zero in the derivative term.
    float dt = (float)(currentTime - previousTime) / 1000.0f;
    previousTime = currentTime;

    if (dt <= 0.0f) return; // Guard against zero or millis() wrap-around

    // Atomically snapshot encoder counts (interrupt-safe read)
    noInterrupts();
    long currentCount1 = Motor1count;
    long currentCount2 = Motor2count;
    interrupts();

    // Pulse delta since last update
    long pulses1 = currentCount1 - previousCount1;
    long pulses2 = currentCount2 - previousCount2;

    previousCount1 = currentCount1;
    previousCount2 = currentCount2;

    // RPM = (pulses / PPR) × (60 / dt)
    actualRPM1 = ((float)pulses1 / PPR) * (60.0f / dt);
    actualRPM2 = ((float)pulses2 / PPR) * (60.0f / dt);

    // PID – Motor 1
    error1         = targetRPM1 - actualRPM1;
    integral1     += error1 * dt;
    integral1      = constrain(integral1, -INTEGRAL_CLAMP, INTEGRAL_CLAMP); // Anti-windup
    derivative1    = (error1 - previousError1) / dt;
    previousError1 = error1;

    pidOutput1 = (Kp * error1) + (Ki * integral1) + (Kd * derivative1);

    pwm1 += pidOutput1;
    pwm1  = constrain(pwm1, (float)MIN_PWM, (float)MAX_PWM);

    // PID – Motor 2
    error2         = targetRPM2 - actualRPM2;
    integral2     += error2 * dt;
    integral2      = constrain(integral2, -INTEGRAL_CLAMP, INTEGRAL_CLAMP); // Anti-windup
    derivative2    = (error2 - previousError2) / dt;
    previousError2 = error2;

    pidOutput2 = (Kp * error2) + (Ki * integral2) + (Kd * derivative2);

    pwm2 += pidOutput2;
    pwm2  = constrain(pwm2, (float)MIN_PWM, (float)MAX_PWM);

    setMotor1((int)pwm1);
    setMotor2((int)pwm2);

    // Optional debug output – disabled in ROS protocol mode (ENABLE_DEBUG_OUTPUT 0)
#if ENABLE_DEBUG_OUTPUT
    Serial.print("M1 target="); Serial.print(targetRPM1);
    Serial.print(" actual=");   Serial.print(actualRPM1);
    Serial.print(" err=");      Serial.print(error1);
    Serial.print(" pwm=");      Serial.println((int)pwm1);

    Serial.print("M2 target="); Serial.print(targetRPM2);
    Serial.print(" actual=");   Serial.print(actualRPM2);
    Serial.print(" err=");      Serial.print(error2);
    Serial.print(" pwm=");      Serial.println((int)pwm2);
#endif
}

/**
 * Parse and act on a null-terminated message string.
 *
 * Supported messages:
 *   "START"         – handshake; reply "READY"
 *   "CMD,<l>,<r>"   – set wheel velocities (rad/s)
 *
 * @param message  Null-terminated C-string received from UART.
 *                 strtok modifies the string in-place, which is intentional
 *                 since the buffer is local to readSerial() and reused only
 *                 after the next newline.
 *
 * Memory safety:
 *   - strtok is used on the caller's buffer; no dynamic allocation occurs.
 *   - atof returns 0.0 on parse failure – non-finite values are rejected below.
 */
static void processMessage(char *message) {

    // Handshake
    if (strcmp(message, "START") == 0) {
        communication_ready = true;
        last_command_time   = millis();
        stopMotors();
        Serial.println("READY");
        return;
    }

    // Velocity command: "CMD,<left>,<right>"
    if (strncmp(message, "CMD,", 4) == 0) {

        if (!communication_ready) return; // Ignore commands before handshake

        char *token = strtok(message, ","); // token == "CMD"
        token = strtok(NULL, ",");           // token == left value string

        if (token == NULL) return;           // Malformed – missing left value
        float left_velocity = atof(token);

        token = strtok(NULL, ",");           // token == right value string
        if (token == NULL) return;           // Malformed – missing right value
        float right_velocity = atof(token);

        // Reject NaN, Inf, or any other non-finite result from atof
        if (!isfinite(left_velocity) || !isfinite(right_velocity)) return;

        last_command_time = millis();
        setMotorTargets(left_velocity, right_velocity);
    }
}

/**
 * Non-blocking UART reader.
 * Accumulates incoming characters into a static buffer until '\n' is received,
 * then calls processMessage() with the null-terminated line.
 *
 * Memory safety:
 *   - Buffer is [64] bytes; overflow is handled by discarding the partial
 *     message and resetting the index (prevents buffer overrun).
 *   - '\r' is silently discarded to handle both LF and CRLF line endings.
 */
static void readSerial() {
    static char    buffer[64];
    static uint8_t index = 0;

    while (Serial.available() > 0) {
        char incoming = (char)Serial.read();

        if (incoming == '\n') {
            buffer[index] = '\0';           // Null-terminate
            if (index > 0) {
                processMessage(buffer);
            }
            index = 0;                      // Reset for next message
        }
        else if (incoming == '\r') {
            // Silently discard carriage return (CRLF compatibility)
        }
        else {
            if (index < sizeof(buffer) - 1) {
                buffer[index++] = incoming;
            } else {
                index = 0;                  // Buffer overflow – discard message
            }
        }
    }
}

/**
 * Transmit measured wheel velocities to the host at FEEDBACK_INTERVAL rate.
 * Format: "STATE,<left_rad_s>,<right_rad_s>\n"
 * Transmission is suppressed until the handshake completes.
 */
static void sendFeedback() {
    unsigned long now = millis();

    if (now - last_feedback_time < FEEDBACK_INTERVAL) return;
    last_feedback_time = now;

    if (!communication_ready) return;

    Serial.print("STATE,");
    Serial.print(getLeftMeasuredVelocity(),  4);
    Serial.print(",");
    Serial.println(getRightMeasuredVelocity(), 4);
}

/**
 * Stop the motors and reset communication state if no CMD has been received
 * within COMMAND_TIMEOUT milliseconds.
 * Critical safety feature: if the host crashes or disconnects, the robot
 * halts rather than continuing at the last commanded speed.
 */
static void checkCommandTimeout() {
    if (!communication_ready) return;

    if (millis() - last_command_time > COMMAND_TIMEOUT) {
        stopMotors();
        communication_ready = false;
    }
}

void setup() {
    Serial.begin(BAUD_RATE);

    // Encoder inputs — pull-up to avoid floating pins
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    pinMode(ENCODER_C, INPUT_PULLUP);
    pinMode(ENCODER_D, INPUT_PULLUP);

    // Motor driver outputs
    pinMode(MOTOR_PWM,  OUTPUT);
    pinMode(MOTOR_DIR,  OUTPUT);
    pinMode(MOTOR2_PWM, OUTPUT);
    pinMode(MOTOR2_DIR, OUTPUT);

    // Attach encoder interrupts on RISING edge
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_C), encoderISR2, RISING);

    // Seed the timer so the first RPM calculation has a valid baseline
    previousTime = millis();

    // Ensure motors start at zero speed with clean PID state
    stopMotors();
}

void loop() {
    readSerial();           // 1. Receive and parse incoming UART commands
    checkCommandTimeout();  // 2. Safety watchdog – stop on comms loss
    sendFeedback();         // 3. Transmit measured velocities to host
    updateMotorControl();   // 4. Run PID and update motor PWM
}
