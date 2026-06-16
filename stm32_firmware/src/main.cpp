/**
 * STM32F103RCT6 — L1 执行与安全层 (Yahboom ROS V3.0 扩展板)
 *
 * 功能:
 *   1. SBUS 接收 (USART2 PA3, 经三极管反相) — 遥控器最高优先级
 *   2. X5 MotorCmd 解析 (USART1 PA9/PA10, CH340N Micro USB) — 自主跟随
 *   3. 坦克混控 + 双路 ESC PWM (S1=PC3/左, S2=PC2/右)
 *   4. 控制优先级: SBUS(ARMED) > X5 > 60s 超时刹停
 *   5. 蜂鸣器(PC5) + LED(PB5) 状态指示
 *   6. MPU9250 IMU 姿态读取 (SPI1)
 */

#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include "config.h"

// ═══════════════════════════════════════════════════════════════
// 全局状态
// ═══════════════════════════════════════════════════════════════

static Servo     servoLeft, servoRight;
static uint16_t  g_throttle   = PWM_NEUTRAL;
static uint16_t  g_steering   = PWM_NEUTRAL;
static uint32_t  g_lastCmdMs  = 0;
static bool      g_escReady   = false;
static bool      g_motorArmed = false;

// X5 命令缓存
static uint16_t  g_x5Throttle = PWM_NEUTRAL;
static uint16_t  g_x5Steering = PWM_NEUTRAL;
static uint32_t  g_lastX5Ms   = 0;
static bool      g_x5Valid    = false;

// SBUS 状态
static struct {
    bool     valid;
    uint16_t channels[16];
    bool     failsafe;
    bool     lostFrame;
    uint32_t lastFrameMs;
    bool     armedPrev;
} g_sbus;

// MPU9250 数据
static struct {
    bool  present;
    int16_t accel[3];
    int16_t gyro[3];
} g_imu;

// ═══════════════════════════════════════════════════════════════
// CRC8 (poly 0x07, init 0x00 — 与 X5/ESP32/ESP8266 一致)
// ═══════════════════════════════════════════════════════════════

static uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
    }
    return crc;
}

// ═══════════════════════════════════════════════════════════════
// ESC PWM (Servo 库 — PC2/PC3 无硬件 TIM 通道)
// ═══════════════════════════════════════════════════════════════

static void escInit() {
    servoLeft.attach(PIN_ESC_LEFT,  PWM_MIN, PWM_MAX);
    servoRight.attach(PIN_ESC_RIGHT, PWM_MIN, PWM_MAX);
    servoLeft.writeMicroseconds(PWM_NEUTRAL);
    servoRight.writeMicroseconds(PWM_NEUTRAL);
    Serial.print("[ESC] S1=PC3(L) S2=PC2(R) 50Hz 中位=");
    Serial.print(PWM_NEUTRAL);
    Serial.println("us");
}

// 坦克混控: throttle + steering → 左/右独立 PWM
static void escSet(uint16_t throttle, uint16_t steering) {
    int sOff = (int)steering - (int)PWM_NEUTRAL;
    int left  = (int)throttle + sOff;
    int right = (int)throttle - sOff;
    if (left  < PWM_MIN) left  = PWM_MIN;
    if (left  > PWM_MAX) left  = PWM_MAX;
    if (right < PWM_MIN) right = PWM_MIN;
    if (right > PWM_MAX) right = PWM_MAX;
    servoLeft.writeMicroseconds((uint16_t)left);
    servoRight.writeMicroseconds((uint16_t)right);
}

// ═══════════════════════════════════════════════════════════════
// 蜂鸣器 (PC5, NPN S8050, active-HIGH)
// ═══════════════════════════════════════════════════════════════

static void beepInit() {
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
}
static void beep(int ms)  { digitalWrite(PIN_BUZZER, HIGH); delay(ms); digitalWrite(PIN_BUZZER, LOW); }
static void beepArm()     { beep(60); delay(60); beep(60); }
static void beepDisarm()  { beep(250); }

// ═══════════════════════════════════════════════════════════════
// LED — 待确认板载 LED 引脚后启用
// PB5 为外接 RGB 灯带接口，不是普通 GPIO LED
// ═══════════════════════════════════════════════════════════════

// static void ledInit() {
//     pinMode(PIN_LED, OUTPUT);
//     digitalWrite(PIN_LED, LOW);
// }

// ═══════════════════════════════════════════════════════════════
// SBUS 驱动 (USART2 PA3, 100000 baud 8E2, 经三极管反相)
// ═══════════════════════════════════════════════════════════════

static HardwareSerial SerialSbus(PIN_SBUS_RX, PA2);

static void sbusInit() {
    SerialSbus.begin(SBUS_BAUD);
    // SBUS: 8 data + even parity + 2 stop bits
    uint32_t cr1 = USART2->CR1;
    cr1 |= USART_CR1_M;
    cr1 |= USART_CR1_PCE;
    USART2->CR1 = cr1;
    USART2->CR2 |= USART_CR2_STOP_0 | USART_CR2_STOP_1;
    Serial.println("[SBUS] USART2 PA3 @ 100000 8E2");
}

static bool sbusParseFrame(const uint8_t *frame, uint16_t *channels) {
    if (frame[0] != 0x0F || frame[24] != 0x00) return false;
    if (frame[23] & 0x04) return false;

    channels[0]  = ((frame[1]      ) | (frame[2]  << 8)) & 0x07FF;
    channels[1]  = ((frame[2]  >> 3) | (frame[3]  << 5)) & 0x07FF;
    channels[2]  = ((frame[3]  >> 6) | (frame[4]  << 2) | (frame[5] << 10)) & 0x07FF;
    channels[3]  = ((frame[5]  >> 1) | (frame[6]  << 7)) & 0x07FF;
    channels[4]  = ((frame[6]  >> 4) | (frame[7]  << 4)) & 0x07FF;
    channels[5]  = ((frame[7]  >> 7) | (frame[8]  << 1) | (frame[9] << 9)) & 0x07FF;
    channels[6]  = ((frame[9]  >> 2) | (frame[10] << 6)) & 0x07FF;
    channels[7]  = ((frame[10] >> 5) | (frame[11] << 3)) & 0x07FF;
    channels[8]  = ((frame[12]     ) | (frame[13] << 8)) & 0x07FF;
    channels[9]  = ((frame[13] >> 3) | (frame[14] << 5)) & 0x07FF;
    channels[10] = ((frame[14] >> 6) | (frame[15] << 2) | (frame[16] << 10)) & 0x07FF;
    channels[11] = ((frame[16] >> 1) | (frame[17] << 7)) & 0x07FF;
    channels[12] = ((frame[17] >> 4) | (frame[18] << 4)) & 0x07FF;
    channels[13] = ((frame[18] >> 7) | (frame[19] << 1) | (frame[20] << 9)) & 0x07FF;
    channels[14] = ((frame[20] >> 2) | (frame[21] << 6)) & 0x07FF;
    channels[15] = ((frame[21] >> 5) | (frame[22] << 3)) & 0x07FF;

    return true;
}

static void sbusPoll() {
    while (SerialSbus.available() >= SBUS_FRAME_LEN) {
        if (SerialSbus.peek() != 0x0F) {
            SerialSbus.read();
            continue;
        }
        uint8_t frame[SBUS_FRAME_LEN];
        for (int i = 0; i < SBUS_FRAME_LEN; i++) {
            uint32_t t0 = micros();
            while (!SerialSbus.available()) {
                if (micros() - t0 > 2000) return;
            }
            frame[i] = SerialSbus.read();
        }
        if (sbusParseFrame(frame, g_sbus.channels)) {
            g_sbus.failsafe    = frame[23] & 0x10;
            g_sbus.lostFrame   = frame[23] & 0x04;
            g_sbus.valid       = true;
            g_sbus.lastFrameMs = millis();
        }
    }
}

static uint16_t sbusToPwm(uint16_t sbusVal, int sensitivity) {
    int off = (int)sbusVal - 992;
    if (abs(off) <= 20) return PWM_NEUTRAL;
    int val = (int)PWM_NEUTRAL + off * sensitivity / (992 - 172);
    if (val < PWM_MIN) val = PWM_MIN;
    if (val > PWM_MAX) val = PWM_MAX;
    return (uint16_t)val;
}

// ═══════════════════════════════════════════════════════════════
// X5 MotorCmd 解析 (USART1, 与 Serial debug 共享)
// ═══════════════════════════════════════════════════════════════

static void x5ParseMotorCmd() {
    while (Serial.available() >= MOTORCMD_FRAME_LEN) {
        if (Serial.read() != MOTORCMD_HEADER) continue;

        uint8_t buf[5];
        bool ok = true;
        for (int i = 0; i < 5; i++) {
            uint32_t t0 = micros();
            while (!Serial.available()) {
                if (micros() - t0 > 2000) { ok = false; break; }
            }
            if (!ok) break;
            buf[i] = Serial.read();
        }
        if (!ok) break;

        if (buf[4] != crc8(buf, 4)) continue;

        uint16_t thr = buf[0] | ((uint16_t)buf[1] << 8);
        uint16_t str = buf[2] | ((uint16_t)buf[3] << 8);
        if (thr < PWM_MIN || thr > PWM_MAX || str < PWM_MIN || str > PWM_MAX) continue;

        g_x5Throttle = thr;
        g_x5Steering = str;
        g_lastX5Ms   = millis();
        g_x5Valid    = true;
    }
}

// ═══════════════════════════════════════════════════════════════
// MPU9250 IMU (SPI1, 基础读取)
// ═══════════════════════════════════════════════════════════════

constexpr uint8_t MPU9250_WHO_AM_I   = 0x75;
constexpr uint8_t MPU9250_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU9250_ACCEL_XOUT = 0x3B;
constexpr uint8_t MPU9250_GYRO_XOUT  = 0x43;

static uint8_t mpu9250ReadReg(uint8_t reg) {
    digitalWrite(PIN_IMU_NSS, LOW);
    SPI.transfer(reg | 0x80);
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(PIN_IMU_NSS, HIGH);
    return val;
}

static void mpu9250WriteReg(uint8_t reg, uint8_t val) {
    digitalWrite(PIN_IMU_NSS, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(val);
    digitalWrite(PIN_IMU_NSS, HIGH);
}

static void mpu9250ReadBurst(uint8_t reg, uint8_t *buf, uint8_t len) {
    digitalWrite(PIN_IMU_NSS, LOW);
    SPI.transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++)
        buf[i] = SPI.transfer(0x00);
    digitalWrite(PIN_IMU_NSS, HIGH);
}

static bool mpu9250Init() {
    pinMode(PIN_IMU_NSS, OUTPUT);
    digitalWrite(PIN_IMU_NSS, HIGH);

    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV16);

    delay(10);
    uint8_t whoami = mpu9250ReadReg(MPU9250_WHO_AM_I);
    if (whoami != 0x71) {
        Serial.print("[IMU] not found (id=0x");
        Serial.print(whoami, HEX);
        Serial.println(")");
        return false;
    }

    mpu9250WriteReg(MPU9250_PWR_MGMT_1, 0x00);
    delay(100);
    Serial.println("[IMU] MPU9250 SPI1 OK");
    return true;
}

static void mpu9250Read() {
    uint8_t buf[14];
    mpu9250ReadBurst(MPU9250_ACCEL_XOUT, buf, 14);

    g_imu.accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    g_imu.accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    g_imu.accel[2] = (int16_t)((buf[4] << 8) | buf[5]);
    g_imu.gyro[0]  = (int16_t)((buf[8]  << 8) | buf[9]);
    g_imu.gyro[1]  = (int16_t)((buf[10] << 8) | buf[11]);
    g_imu.gyro[2]  = (int16_t)((buf[12] << 8) | buf[13]);
}

// ═══════════════════════════════════════════════════════════════
// setup
// ═══════════════════════════════════════════════════════════════

void setup() {
    beepInit();

    // 显式指定 Serial 引脚 — genericSTM32F103RC 变体不默认映射 USART1
    Serial.setRx(PA10);
    Serial.setTx(PA9);
    Serial.begin(X5_BAUD);
    delay(100);

    Serial.println("\n=== STM32 V3.0 Dual BLDC Controller ===");
    Serial.println("[MCU] STM32F103RCT6 72MHz 256KB Flash");

    escInit();
    sbusInit();
    g_imu.present = mpu9250Init();

    g_motorArmed = false;
    g_escReady   = false;
    g_lastCmdMs  = millis();
    escSet(PWM_NEUTRAL, PWM_NEUTRAL);

    Serial.println("READY. 3s ESC自检后, CH5 ARM 或 X5发指令.\n");
}

// ═══════════════════════════════════════════════════════════════
// loop
// ═══════════════════════════════════════════════════════════════

void loop() {
    uint32_t now = millis();

    // 1. SBUS 帧接收
    sbusPoll();

    // 2. SBUS 有效性检测
    static bool sbusWasValid = false;
    if (g_sbus.valid && now - g_sbus.lastFrameMs > SBUS_TIMEOUT_MS) {
        g_sbus.valid = false;
    }
    if (!g_sbus.valid && sbusWasValid) {
        Serial.println("[SBUS] 信号丢失");
        sbusWasValid = false;
    }
    if (g_sbus.valid && !sbusWasValid) {
        sbusWasValid = true;
        Serial.println("[SBUS] 信号恢复");
    }

    // 3. X5 MotorCmd 接收
    x5ParseMotorCmd();

    // 4. ESC 自检计时
    static bool escDelayDone = false;
    if (!escDelayDone && now >= ESC_INIT_DELAY_MS) {
        escDelayDone = true;
        g_escReady  = true;
        g_lastCmdMs = now;
        Serial.println("[ESC] 自检完成");
    }

    // 5. 控制优先级仲裁
    if (g_escReady) {
        bool sbusActive = g_sbus.valid && !g_sbus.failsafe;
        bool x5Active   = g_x5Valid && (now - g_lastX5Ms < CMD_TIMEOUT_MS);

        uint16_t thr = PWM_NEUTRAL, str = PWM_NEUTRAL;
        const char* source = "TIMEOUT";

        // SBUS CH5 边沿检测
        bool ch5High = g_sbus.valid && (g_sbus.channels[SBUS_CH_ARM] > SBUS_ARM_THRESHOLD);
        static bool lastCh5High = false;

        if (ch5High && !lastCh5High && g_sbus.valid) {
            if (!g_motorArmed) {
                g_motorArmed = true;
                beepArm();
                Serial.println("[SBUS] ARMED (CH5 HIGH)");
            }
        } else if (!ch5High && lastCh5High && g_sbus.valid) {
            if (g_motorArmed) {
                g_motorArmed = false;
                escSet(PWM_NEUTRAL, PWM_NEUTRAL);
                beepDisarm();
                Serial.println("[SBUS] DISARMED (CH5 LOW)");
            }
        }
        lastCh5High = ch5High;

        if (sbusActive && g_motorArmed) {
            thr = sbusToPwm(g_sbus.channels[SBUS_CH_THROTTLE], SBUS_THR_SENSITIVITY);
            str = sbusToPwm(g_sbus.channels[SBUS_CH_STEERING], SBUS_STR_SENSITIVITY);
            source = "SBUS";
            g_lastCmdMs = now;

        } else if (x5Active) {
            thr    = g_x5Throttle;
            str    = g_x5Steering;
            source = "X5";
            g_lastCmdMs = now;

        } else {
            thr    = PWM_NEUTRAL;
            str    = PWM_NEUTRAL;
            source = "TIMEOUT";
        }

        escSet(thr, str);
        g_throttle = thr;
        g_steering = str;

        // 超时锁定
        if (g_motorArmed && now - g_lastCmdMs > CMD_TIMEOUT_MS) {
            g_motorArmed = false;
            escSet(PWM_NEUTRAL, PWM_NEUTRAL);
            beepDisarm();
            Serial.println("[SAFE] 命令超时 60s, 自动锁定!");
        }
    }

    // 6. LED — 待确认板载 LED 引脚后启用

    // 7. 状态输出 (5Hz)
    static uint32_t lastStat;
    if (now - lastStat >= STATUS_INTERVAL_MS) {
        lastStat = now;

        int t = (int)g_throttle, s = (int)g_steering;
        int hi = (int)PWM_NEUTRAL + DIR_THRESHOLD;
        int lo = (int)PWM_NEUTRAL - DIR_THRESHOLD;

        const char* dir;
        if      (t > hi && s > hi) dir = "FWD+R";
        else if (t > hi && s < lo) dir = "FWD+L";
        else if (t < lo && s > hi) dir = "REV+R";
        else if (t < lo && s < lo) dir = "REV+L";
        else if (t > hi) dir = "FWD";
        else if (t < lo) dir = "REV";
        else if (s > hi) dir = "R";
        else if (s < lo) dir = "L";
        else              dir = "STOP";

        int sOff = s - (int)PWM_NEUTRAL;
        int left  = t + sOff; if (left  < PWM_MIN) left = PWM_MIN; if (left > PWM_MAX) left = PWM_MAX;
        int right = t - sOff; if (right < PWM_MIN) right = PWM_MIN; if (right > PWM_MAX) right = PWM_MAX;

        bool sbusOk = g_sbus.valid && !g_sbus.failsafe;

        Serial.print(sbusOk ? (g_motorArmed ? "SBUS ARM" : "SBUS LCK") :
                     g_x5Valid ? "X5      " : "--------");
        Serial.print(" thr="); Serial.print(g_throttle);
        Serial.print(" st="); Serial.print(g_steering);
        Serial.print(" L="); Serial.print(left);
        Serial.print(" R="); Serial.print(right);
        Serial.print(" "); Serial.print(dir);

        if (g_imu.present) {
            Serial.print(" | GyrZ="); Serial.print(g_imu.gyro[2]);
        }
        if (sbusOk) {
            Serial.print(" | CH5=");
            Serial.print(g_sbus.channels[SBUS_CH_ARM] > SBUS_ARM_THRESHOLD ? "HI" : "LO");
        }
        Serial.println();
    }

    // 8. IMU 读取 (10Hz)
    static uint32_t lastImu;
    if (g_imu.present && now - lastImu >= 100) {
        lastImu = now;
        mpu9250Read();
    }
}
