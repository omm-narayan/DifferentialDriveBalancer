/*
 * =====================================================================
 * BalancingWii_STM32 – ULTRA SOFT + HC-05 BLUETOOTH + SMOOTH MOVEMENT
 * =====================================================================
 */

#include <Arduino.h>
#include <Wire.h>

// ======================== HARDWARE PINS ============================
#define X_STEP_PIN   2
#define X_DIR_PIN    5
#define Z_STEP_PIN   4
#define Z_DIR_PIN    7
#define EN_PIN       8
#define LED_PIN      LED_BUILTIN

// ======================== HC-05 BLUETOOTH ==========================
HardwareSerial BT(PC_11, PC_10);

// ======================== ICM-20948 I2C ============================
#define ICM_ADDR     0x69
#define ICM_BANK_SEL 0x7F

// ======================== SMOOTH MOVEMENT GAINS ====================
float SPEED_P = 0.08f;   // reduced for smoother speed response
float SPEED_I = 0.005f;  // minimal integral
float ANGLE_P = 20.0f;   // slightly increased for balance
float ANGLE_D = 16.0f;   // increased damping
float balanceAngleOffset = 2.0f;  // adjust with 'o'

#define MAX_SPEED          1500
#define MAX_TARGET_ANGLE   100    // reduced from 150 (10° instead of 15°)
#define MAX_STEERING       90
#define SPEED_IMAX         20000

#define RISE_SPEED_K       1.0f
#define FALL_ANGLE         70.0f

#define LOOP_PERIOD_US     5000
#define CF_ALPHA           0.98f
#define GYRO_LPF_ALPHA     0.15f

#define MIN_STEP_DELAY     100
#define MAX_STEP_DELAY     3000
#define SPEED_SCALE        100.0f
#define MAX_STEPS_PER_TICK 20
#define ANGLE_DEAD_ZONE    2.0f

// ======================== SAFE MOVEMENT PARAMETERS =================
#define CMD_HOLD_MS    1500
#define MANUAL_SPEED   100
#define RAMP_STEP      10
#define MIN_ANGLE_FOR_CMD 15.0f

enum MoveCmd { MOVE_NONE, MOVE_FWD, MOVE_BWD, MOVE_LEFT, MOVE_RIGHT };
static MoveCmd       moveCmd   = MOVE_NONE;
static unsigned long moveUntil = 0;
static float commandedSpeed = 0.0f;
static float commandedSpeedTarget = 0.0f;

// ======================== GLOBAL VARIABLES =========================
float angle = 0.0f;
float gyroBiasX = 0.0f, gyroBiasY = 0.0f;
int tiltAxis = 1;
float gyroFiltered = 0.0f;

float actualSpeed = 0.0f;
float targetSpeed = 0.0f;
float steering   = 0.0f;
float speedIntegral = 0.0f;
float positionError = 0.0f;
float angleSetpoint = 0.0f;
float outputSpeed = 0.0f;

bool motorDir = true;
int dirHoldCnt = 0;
#define DIR_HOLD_TICKS 4

bool riseModeEnabled = true;
uint8_t risePhase = 2;
float riseSpeed = 0.0f;

static String btBuffer = "";

// ======================== ICM FUNCTIONS ============================
static void icmBank(uint8_t b) {
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(ICM_BANK_SEL);
    Wire.write((b & 0x03) << 4);
    Wire.endTransmission();
}
static void icmWrite(uint8_t bank, uint8_t reg, uint8_t val) {
    icmBank(bank);
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
}
static uint8_t icmRead(uint8_t bank, uint8_t reg) {
    icmBank(bank);
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}
static void icmReadAll(float &ax, float &ay, float &az,
                       float &gx, float &gy, float &gz) {
    icmBank(0);
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(0x2D);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)12);
    uint8_t b[12] = {};
    for (int i = 0; i < 12 && Wire.available(); ++i) b[i] = Wire.read();
    ax = (int16_t)((b[0] << 8) | b[1]) / 16384.0f;
    ay = (int16_t)((b[2] << 8) | b[3]) / 16384.0f;
    az = (int16_t)((b[4] << 8) | b[5]) / 16384.0f;
    gx = (int16_t)((b[6] << 8) | b[7]) / 131.0f;
    gy = (int16_t)((b[8] << 8) | b[9]) / 131.0f;
    gz = (int16_t)((b[10]<< 8) | b[11]) / 131.0f;
}

// ======================== STEPPER ==================================
static inline void stepMotors(bool forward) {
    digitalWrite(X_DIR_PIN, forward ? LOW : HIGH);
    digitalWrite(Z_DIR_PIN, forward ? HIGH : LOW);
    delayMicroseconds(2);
    digitalWrite(X_STEP_PIN, HIGH);
    digitalWrite(Z_STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(X_STEP_PIN, LOW);
    digitalWrite(Z_STEP_PIN, LOW);
}

static void stepTurnLeft() {
    digitalWrite(X_DIR_PIN, HIGH);
    digitalWrite(Z_DIR_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(X_STEP_PIN, HIGH);
    digitalWrite(Z_STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(X_STEP_PIN, LOW);
    digitalWrite(Z_STEP_PIN, LOW);
}

static void stepTurnRight() {
    digitalWrite(X_DIR_PIN, LOW);
    digitalWrite(Z_DIR_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(X_STEP_PIN, HIGH);
    digitalWrite(Z_STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(X_STEP_PIN, LOW);
    digitalWrite(Z_STEP_PIN, LOW);
}

// ======================== BLUETOOTH ================================
static void btPrint(const String &msg) {
    BT.println(msg);
    Serial.println(msg);
}
static void printHelp() {
    btPrint(F("=== COMMANDS ==="));
    btPrint(F("F/B/L/R  move   S  stop"));
    btPrint(F("p/d/s/i:xx.x  tune gains"));
    btPrint(F("o:xx.x  balance offset"));
    btPrint(F("r  reset integrals"));
    btPrint(F("ST  status    ?  help"));
}
static void printStatus() {
    btPrint(F("=== STATUS ==="));
    btPrint("A=" + String(angle, 2) + " deg");
    btPrint("ANGLE_P=" + String(ANGLE_P, 2) +
            " ANGLE_D=" + String(ANGLE_D, 2));
    btPrint("SPEED_P=" + String(SPEED_P, 3) +
            " SPEED_I=" + String(SPEED_I, 3));
    btPrint("offset=" + String(balanceAngleOffset, 2));
}
static void handleCommand(String cmd) {
    cmd.trim();

    // Movement commands (single uppercase letter)
    String upper = cmd;
    upper.toUpperCase();
    if      (upper == "F") { moveCmd = MOVE_FWD;   moveUntil = millis() + CMD_HOLD_MS; commandedSpeedTarget =  MANUAL_SPEED; btPrint(F(">>FWD"));   return; }
    else if (upper == "B") { moveCmd = MOVE_BWD;   moveUntil = millis() + CMD_HOLD_MS; commandedSpeedTarget = -MANUAL_SPEED; btPrint(F(">>BWD"));   return; }
    else if (upper == "L") { moveCmd = MOVE_LEFT;  moveUntil = millis() + CMD_HOLD_MS; btPrint(F(">>LEFT"));  return; }
    else if (upper == "R") { moveCmd = MOVE_RIGHT; moveUntil = millis() + CMD_HOLD_MS; btPrint(F(">>RIGHT")); return; }
    else if (upper == "S")  { moveCmd = MOVE_NONE; commandedSpeedTarget = 0; btPrint(F(">>STOP")); return; }
    else if (upper == "ST") { printStatus(); return; }
    else if (upper == "?")  { printHelp();   return; }

    // Original serial gain commands preserved exactly
    if (cmd.length() < 1) return;
    char c = cmd.charAt(0);
    float val = cmd.substring(1).toFloat();
    switch (c) {
        case 'p': ANGLE_P = val; btPrint("ANGLE_P=" + String(ANGLE_P, 2)); break;
        case 'd': ANGLE_D = val; btPrint("ANGLE_D=" + String(ANGLE_D, 2)); break;
        case 's': SPEED_P = val; btPrint("SPEED_P=" + String(SPEED_P, 3)); break;
        case 'i': SPEED_I = val; btPrint("SPEED_I=" + String(SPEED_I, 3)); break;
        case 'o': balanceAngleOffset = val; btPrint("offset=" + String(balanceAngleOffset, 2)); break;
        case 'f': targetSpeed = constrain(val, -MAX_SPEED, MAX_SPEED);  btPrint("tgtSpd=" + String(targetSpeed, 0)); break;
        case 'b': targetSpeed = constrain(-val, -MAX_SPEED, MAX_SPEED); btPrint("tgtSpd=" + String(targetSpeed, 0)); break;
        case 't': steering = constrain(val, -MAX_STEERING, MAX_STEERING); btPrint("steer=" + String(steering, 0)); break;
        case 'r': speedIntegral = 0; positionError = 0; btPrint(F("integrals reset")); break;
        default:  btPrint("?:[" + cmd + "]"); break;
    }
}
static void readBluetooth() {
    auto processChar = [](char c) {
        if (c == '\n' || c == '\r') {
            if (btBuffer.length() > 0) { handleCommand(btBuffer); btBuffer = ""; }
        } else {
            btBuffer += c;
            if (btBuffer.length() > 32) btBuffer = "";
        }
    };
    while (BT.available())     processChar((char)BT.read());
    while (Serial.available()) processChar((char)Serial.read());
}

// ======================== SETUP ====================================
void setup() {
    Serial.begin(115200);
    BT.begin(115200);
    delay(500);
    Serial.println(F("\n=== BalancingWii SMOOTH MOVEMENT ==="));
    BT.println(F("HC-05 OK — ? for help"));

    const uint8_t pins[] = {X_STEP_PIN, X_DIR_PIN, Z_STEP_PIN, Z_DIR_PIN, EN_PIN, LED_PIN};
    for (uint8_t p : pins) pinMode(p, OUTPUT);
    digitalWrite(X_STEP_PIN, LOW); digitalWrite(X_DIR_PIN, LOW);
    digitalWrite(Z_STEP_PIN, LOW); digitalWrite(Z_DIR_PIN, LOW);
    digitalWrite(EN_PIN, HIGH); digitalWrite(LED_PIN, LOW);

    Wire.begin(); Wire.setClock(400000); delay(200);
    uint8_t who = icmRead(0, 0x00);
    Serial.print(F("WHO_AM_I: 0x")); Serial.println(who, HEX);
    if (who != 0xEA) {
        Serial.println(F("ICM-20948 not found!"));
        while (1) { digitalWrite(LED_PIN, HIGH); delay(150); digitalWrite(LED_PIN, LOW); delay(150); }
    }
    icmWrite(0, 0x06, 0x01); delay(50);
    icmWrite(0, 0x07, 0x00); delay(20);

    btPrint(F("Hold robot STILL and UPRIGHT for 3 seconds..."));
    float sumGX = 0, sumGY = 0, sumAX = 0, sumAY = 0;
    int samples = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 3000) {
        digitalWrite(LED_PIN, (millis() % 200 < 100) ? HIGH : LOW);
        float ax, ay, az, gx, gy, gz;
        icmReadAll(ax, ay, az, gx, gy, gz);
        sumGX += gx; sumGY += gy;
        sumAX += ax; sumAY += ay;
        samples++;
        delay(10);
    }
    digitalWrite(LED_PIN, HIGH);

    gyroBiasX = sumGX / samples;
    gyroBiasY = sumGY / samples;
    tiltAxis = 1;
    Serial.print(F("Tilt axis: Y (accel-Y / gyro-X)\n"));
    Serial.print(F("Gyro bias X: ")); Serial.print(gyroBiasX, 3);
    Serial.print(F("   Y: ")); Serial.println(gyroBiasY, 3);

    float ax, ay, az, gx, gy, gz;
    icmReadAll(ax, ay, az, gx, gy, gz);
    angle = atan2f(ay, az) * 180.0f / PI;
    Serial.print(F("Initial angle: ")); Serial.print(angle, 2); Serial.println(F(" deg"));

    Serial.println(F("\n--- DIRECTION TEST ---"));
    Serial.println(F("Robot will move FORWARD for 1 second, then BACKWARD."));
    digitalWrite(EN_PIN, LOW);
    for (int i = 0; i < 100; i++) { stepMotors(true); delayMicroseconds(2000); }
    delay(1000);
    for (int i = 0; i < 100; i++) { stepMotors(false); delayMicroseconds(2000); }
    delay(500);
    Serial.println(F("Direction test complete. Now balancing...\n"));
    Serial.println(F("  angle | target | speed | stepCnt | dir | delay | offset"));
    Serial.println(F(  "  ------+--------+-------+---------+-----+-------+-------"));
    btPrint(F("READY — ? for help"));
}

// ======================== MAIN LOOP ================================
void loop() {
    static unsigned long loopTimer = 0;
    while (micros() - loopTimer < LOOP_PERIOD_US) {}
    loopTimer = micros();

    // ── BT + command expiry ──────────────────────────────────
    readBluetooth();
    if (moveCmd != MOVE_NONE && millis() > moveUntil) {
        moveCmd = MOVE_NONE;
        commandedSpeedTarget = 0;
    }

    // ── Ramp commanded speed ─────────────────────────────────
    if (commandedSpeed < commandedSpeedTarget) {
        commandedSpeed += RAMP_STEP;
        if (commandedSpeed > commandedSpeedTarget) commandedSpeed = commandedSpeedTarget;
    } else if (commandedSpeed > commandedSpeedTarget) {
        commandedSpeed -= RAMP_STEP;
        if (commandedSpeed < commandedSpeedTarget) commandedSpeed = commandedSpeedTarget;
    }

    // ── Apply commanded speed only if robot is near vertical ──
    if (fabs(angle) < MIN_ANGLE_FOR_CMD) {
        targetSpeed = commandedSpeed;
    } else {
        targetSpeed = 0;
        commandedSpeedTarget = 0;
        commandedSpeed = 0;
        moveCmd = MOVE_NONE;
    }

    // ── Steering for L/R ──────────────────────────────────────
    if (moveCmd == MOVE_LEFT) steering =  30.0f;
    else if (moveCmd == MOVE_RIGHT) steering = -30.0f;
    else if (moveCmd == MOVE_NONE && fabs(steering) < 1.0f) steering = 0.0f;

    // ── IMU ──────────────────────────────────────────────────
    float ax, ay, az, gx, gy, gz;
    icmReadAll(ax, ay, az, gx, gy, gz);

    float accelAngle = atan2f(ay, az) * 180.0f / PI;
    float gyroRaw = gx - gyroBiasX;

    gyroFiltered = GYRO_LPF_ALPHA * gyroRaw + (1.0f - GYRO_LPF_ALPHA) * gyroFiltered;
    angle = CF_ALPHA * (angle + gyroFiltered * (LOOP_PERIOD_US / 1e6f))
            + (1.0f - CF_ALPHA) * accelAngle;

    // ── Fall check ───────────────────────────────────────────
    if (fabs(angle) > FALL_ANGLE) {
        digitalWrite(EN_PIN, HIGH);
        digitalWrite(LED_PIN, LOW);
        speedIntegral = 0;
        outputSpeed = 0;
        risePhase = 2;
        riseSpeed = 0;
        moveCmd = MOVE_NONE;
        commandedSpeedTarget = 0;
        commandedSpeed = 0;
        targetSpeed = 0;
        static unsigned long lastFallMsg = 0;
        if (millis() - lastFallMsg > 1000) {
            btPrint("FALLEN (" + String(angle, 1) + " deg) — pick up");
            lastFallMsg = millis();
        }
        return;
    }
    digitalWrite(EN_PIN, LOW);
    digitalWrite(LED_PIN, HIGH);

    // ════════════════════════════════════════════════════════
    // CORE BALANCING
    // ════════════════════════════════════════════════════════

    if (fabs(targetSpeed) < 15 && fabs(steering) < 15) {
        positionError += actualSpeed * (LOOP_PERIOD_US / 1e6f);
    } else {
        positionError = 0.0f;
    }

    actualSpeed = 0.92f * actualSpeed + 0.08f * outputSpeed;
    float speedError = targetSpeed - actualSpeed - (positionError * 0.01f);
    speedIntegral = constrain(speedIntegral + speedError * (LOOP_PERIOD_US / 1e6f) * SPEED_I,
                              -SPEED_IMAX, SPEED_IMAX);
    angleSetpoint = constrain(SPEED_P * speedError + speedIntegral, -MAX_TARGET_ANGLE, MAX_TARGET_ANGLE);

    if (riseModeEnabled && fabs(angle) > 25.0f && fabs(angle) < FALL_ANGLE) {
        if (risePhase == 0) {
            riseSpeed = constrain(riseSpeed + 0.7f * RISE_SPEED_K, 0, 140);
            outputSpeed = (angle > 0) ? riseSpeed : -riseSpeed;
            steering = 0;
            if (riseSpeed >= 140) { riseSpeed = 0; risePhase = 1; }
        } else if (risePhase == 1) {
            riseSpeed = constrain(riseSpeed + 0.85f * RISE_SPEED_K, 0, 100);
            outputSpeed = (angle > 0) ? -riseSpeed : riseSpeed;
            steering = 0;
            if (riseSpeed >= 100) risePhase = 2;
        } else {
            riseSpeed = 0;
            outputSpeed = 0;
        }
    } else {
        risePhase = 0;
    }

    float correctedAngle = angle - balanceAngleOffset;
    float angleError = angleSetpoint - correctedAngle;
    if (fabs(angleError) < ANGLE_DEAD_ZONE) angleError = 0;

    float acceleration = ANGLE_P * angleError - ANGLE_D * gyroFiltered;
    if (risePhase != 2) {
        outputSpeed += acceleration * (LOOP_PERIOD_US / 1e6f);
        outputSpeed = constrain(outputSpeed, -MAX_SPEED, MAX_SPEED);
    }

    int16_t leftSpeed  = (int16_t)(-outputSpeed + steering);
    int16_t rightSpeed = (int16_t)( outputSpeed + steering);
    leftSpeed  = constrain(leftSpeed,  -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

    float avgSpeed = (fabs(leftSpeed) + fabs(rightSpeed)) / 2.0f;
    bool wantFwd = (outputSpeed > 0);

    if (wantFwd != motorDir) {
        if (++dirHoldCnt >= DIR_HOLD_TICKS) { motorDir = wantFwd; dirHoldCnt = 0; }
    } else {
        dirHoldCnt = 0;
    }

    int stepsThisTick = 0;
    int stepDelay = MAX_STEP_DELAY;
    if (avgSpeed > 1.0f) {
        stepDelay = MAX_STEP_DELAY - (int)(avgSpeed * SPEED_SCALE);
        stepDelay = constrain(stepDelay, MIN_STEP_DELAY, MAX_STEP_DELAY);
        stepsThisTick = constrain((int)(LOOP_PERIOD_US / (stepDelay + 12)), 0, MAX_STEPS_PER_TICK);
        for (int i = 0; i < stepsThisTick; ++i) {
            if (moveCmd == MOVE_LEFT) {
                stepTurnLeft();
            } else if (moveCmd == MOVE_RIGHT) {
                stepTurnRight();
            } else {
                stepMotors(motorDir);
            }
            delayMicroseconds(stepDelay);
        }
    }

    // ── Telemetry ────────────────────────────────────────────
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
        BT.print(F("A:")); BT.print(angle, 1);
        BT.print(F(" T:")); BT.print(angleSetpoint, 1);
        BT.print(F(" S:")); BT.println(avgSpeed, 0);

        Serial.print(angle, 1);         Serial.print(F(" | "));
        Serial.print(angleSetpoint, 1); Serial.print(F(" | "));
        Serial.print(avgSpeed, 1);      Serial.print(F(" | "));
        Serial.print(stepsThisTick);    Serial.print(F(" | "));
        if      (moveCmd == MOVE_LEFT)  Serial.print(F("LEFT "));
        else if (moveCmd == MOVE_RIGHT) Serial.print(F("RIGHT"));
        else                            Serial.print(motorDir ? F("FWD  ") : F("REV  "));
        Serial.print(F(" | ")); Serial.print(stepDelay);
        Serial.print(F(" | ")); Serial.println(balanceAngleOffset, 1);
        lastPrint = millis();
    }
}