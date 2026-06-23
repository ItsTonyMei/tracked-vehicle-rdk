/**
 * STM32F103RCT6 — L1 执行与安全层 (Yahboom ROS V3.0 扩展板)
 *
 * 功能:
 *   1. SBUS 接收 (USART2 PA3, 100k 8E2, 三极管反相) — WFLY RF209S 适配
 *   2. X5 MotorCmd 解析 (USART1 PA9/PA10, CH340N, 115200) — 6字节 CRC8 帧
 *   3. 坦克混控 + 双路 ESC PWM (S1=PC3/左, S2=PC2/右, Servo 库 50Hz)
 *   4. 控制优先级: 手控 SBUS(CH5 ARM) > 自动 X5 > 60s 超时刹停
 *   5. CH5 ARM/DISARM 防抖 (3帧确认) + 信号丢失需手动重新 ARM
 *   6. CH6 手控/自动模式切换 (LOW=手控, HIGH=自动) + 非阻塞蜂鸣提示
 *   7. SBUS 信号防抖: 5帧确认有效, 2次连续超时判丢失
 *   8. 蜂鸣器(PC5) + LED(PC13, active-LOW) 快/中/慢三速闪烁
 *   9. MPU9250 IMU SPI2 (PB12-15) 姿态读取
 *
 * 踩坑记录:
 *   - genericSTM32F103RC Serial 默认不映射 USART1, 需显式 setRx/setTx
 *   - PCx 在 LQFP64 无硬件 TIM, 必须用 Servo 库; 通用 TIM ISR 被框架占用
 *   - V3.0 CH340N DTR/RTS 未接 NRST/BOOT0, 不支持自动下载
 *   - WFLY RF209S byte24 非标准 0x00, 需放宽帧尾校验; failsafe 用 bit4(0x10)
 *   - delay() 阻塞导致 SBUS 丢帧, 模式切换蜂鸣改用非阻塞状态机
 *   - Servo.attach 会重置 GPIOC, PC13 LED 可能被意外关闭
 *   - ZTW Seal G2 标准舵机 PWM 中位=1500μs (1000-2000μs 标准范围),
 *     原 C06B 项目误用 1275μs 非标值, 非通用规范
 */

#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include "config.h"

// SPI2 for IMU (PB12-PB15 on STM32F103RCT6)
static SPIClass SPI_IMU(PB15, PB14, PB13, PB12);  // MOSI,MISO,SCK,NSS

// ═══════════════════════════════════════════════════════════════
// 全局状态
// ═══════════════════════════════════════════════════════════════

static Servo     servoLeft, servoRight;
static uint16_t  g_throttle   = PWM_NEUTRAL;
static uint16_t  g_steering   = PWM_NEUTRAL;
static uint32_t  g_lastCmdMs  = 0;
static bool      g_escReady   = false;
static bool      g_motorArmed = false;

static uint16_t  g_x5Throttle = PWM_NEUTRAL;
static uint16_t  g_x5Steering = PWM_NEUTRAL;
static uint32_t  g_lastX5Ms   = 0;
static bool      g_x5Valid    = false;

static struct {
    bool     valid;
    uint16_t channels[16];
    bool     failsafe;
    bool     lostFrame;
    uint32_t lastFrameMs;
    bool     armedPrev;
    int      goodFrames;     // 连续好帧计数器 (防抖: 需≥3 帧才判定有效)
} g_sbus;

static struct {
    bool  present;
    int16_t accel[3];
    int16_t gyro[3];
} g_imu;

// ═══════════════════════════════════════════════════════════════
// CRC8 (poly 0x07, init 0x00)
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
// ESC PWM (Servo 库)
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
// 蜂鸣器 (PC5, active-HIGH)
// ═══════════════════════════════════════════════════════════════

static void beepInit() {
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
}
static void beep(int ms)  { digitalWrite(PIN_BUZZER, HIGH); delay(ms); digitalWrite(PIN_BUZZER, LOW); }
static void beepArm()     { beep(60); delay(60); beep(60); }
static void beepDisarm()  { beep(250); }

// 非阻塞蜂鸣 (用于模式切换等不能阻塞主循环的场景)
static uint32_t beepEndMs   = 0;
static uint8_t  beepPattern = 0;  // 0=idle, 1=单声, 2=双声gap, 3=双声
static void beepNonBlocking(int count) {
    beepPattern = count * 2 - 1;  // 1→1(单声), 2→3(双声: on→gap→on)
    beepEndMs   = millis();
    digitalWrite(PIN_BUZZER, HIGH);
}
static void beepPoll() {
    if (beepPattern == 0) return;
    uint32_t now = millis();
    if (beepPattern == 2) {  // gap between double beep
        if (now - beepEndMs >= 60) {
            digitalWrite(PIN_BUZZER, HIGH);
            beepEndMs   = now;
            beepPattern = 1;  // second beep
        }
    } else {  // beep on (pattern 1 or 3)
        if (now - beepEndMs >= 60) {
            digitalWrite(PIN_BUZZER, LOW);
            if (beepPattern == 3) {
                beepEndMs = now;
                beepPattern = 2;  // gap
            } else {
                beepPattern = 0;  // done
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════
// LED (PC13, active-LOW)
// ═══════════════════════════════════════════════════════════════

static void ledInit() {
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LED_ACTIVE_LOW ? HIGH : LOW);
}
static void ledSet(bool on) {
    digitalWrite(PIN_LED, LED_ACTIVE_LOW ? !on : on);
}

// ═══════════════════════════════════════════════════════════════
// SBUS 驱动 (USART2 PA3, 100k 8E2, 三极管反相, 5帧防抖确认)
// WFLY RF209S: byte24 非标, failsafe 用 bit4(0x10)
// ═══════════════════════════════════════════════════════════════

static HardwareSerial SerialSbus(PIN_SBUS_RX, PA2);

static void sbusInit() {
    SerialSbus.begin(SBUS_BAUD);
    uint32_t cr1 = USART2->CR1;
    cr1 |= USART_CR1_M;
    cr1 |= USART_CR1_PCE;
    USART2->CR1 = cr1;
    USART2->CR2 |= USART_CR2_STOP_0 | USART_CR2_STOP_1;
    Serial.println("[SBUS] USART2 PA3 @ 100k 8E2 (WFLY)");
}

static bool sbusParseFrame(const uint8_t *frame, uint16_t *channels) {
    if (frame[0] != 0x0F) return false;
    if (frame[23] & 0x04) return false;   // lost frame
    // WFLY RF209S byte24 varies, skip check

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
    // 逐字节扫描: 读到非 0x0F 就丢弃, 遇到 0x0F 尝试读完整帧
    // 超时从 2000μs 降到 500μs — 100k baud 每字节约 120μs, 500μs 够 4 字节传输
    while (SerialSbus.available() > 0) {
        int b = SerialSbus.read();
        if (b != 0x0F) continue;

        // 疑似帧头, 尝试读取剩余 24 bytes
        uint8_t frame[SBUS_FRAME_LEN];
        frame[0] = 0x0F;
        bool ok = true;
        for (int i = 1; i < SBUS_FRAME_LEN; i++) {
            uint32_t t0 = micros();
            while (!SerialSbus.available()) {
                if (micros() - t0 > 500) { ok = false; break; }
            }
            if (!ok) break;
            frame[i] = SerialSbus.read();
        }
        if (!ok) continue;  // 帧不完整, 丢弃

        if (sbusParseFrame(frame, g_sbus.channels)) {
            g_sbus.failsafe    = frame[23] & 0x10;
            g_sbus.lostFrame   = frame[23] & 0x04;
            g_sbus.lastFrameMs = millis();
            if (++g_sbus.goodFrames >= 5) {
                g_sbus.valid = true;
            }
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
// X5 MotorCmd 解析
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
// MPU9250 IMU (SPI2, PB12-PB15)
// ═══════════════════════════════════════════════════════════════

constexpr uint8_t MPU9250_WHO_AM_I   = 0x75;
constexpr uint8_t MPU9250_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU9250_ACCEL_XOUT = 0x3B;
constexpr uint8_t MPU9250_GYRO_XOUT  = 0x43;

static uint8_t mpu9250ReadReg(uint8_t reg) {
    digitalWrite(PIN_IMU_NSS, LOW);
    SPI_IMU.transfer(reg | 0x80);
    uint8_t val = SPI_IMU.transfer(0x00);
    digitalWrite(PIN_IMU_NSS, HIGH);
    return val;
}

static void mpu9250WriteReg(uint8_t reg, uint8_t val) {
    digitalWrite(PIN_IMU_NSS, LOW);
    SPI_IMU.transfer(reg & 0x7F);
    SPI_IMU.transfer(val);
    digitalWrite(PIN_IMU_NSS, HIGH);
}

static void mpu9250ReadBurst(uint8_t reg, uint8_t *buf, uint8_t len) {
    digitalWrite(PIN_IMU_NSS, LOW);
    SPI_IMU.transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++)
        buf[i] = SPI_IMU.transfer(0x00);
    digitalWrite(PIN_IMU_NSS, HIGH);
}

static bool mpu9250Init() {
    pinMode(PIN_IMU_NSS, OUTPUT);
    digitalWrite(PIN_IMU_NSS, HIGH);

    SPI_IMU.begin();
    SPI_IMU.setBitOrder(MSBFIRST);
    SPI_IMU.setDataMode(SPI_MODE0);
    SPI_IMU.setClockDivider(SPI_CLOCK_DIV16);

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
    Serial.println("[IMU] MPU9250 SPI2 OK");
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
    ledInit();

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
    beepPoll();  // 非阻塞蜂鸣状态机

    // 2. SBUS 有效性检测 (超时需连续触发 2 次才判丢失, 防短暂丢帧误锁)
    static bool     sbusWasValid  = false;
    static uint32_t lastTimeoutMs = 0;
    if (g_sbus.valid && now - g_sbus.lastFrameMs > SBUS_TIMEOUT_MS) {
        if (lastTimeoutMs > 0 && now - lastTimeoutMs < SBUS_TIMEOUT_MS * 2) {
            // 连续第二次超时 → 确认丢失
            g_sbus.valid      = false;
            g_sbus.goodFrames  = 0;
        }
        lastTimeoutMs = now;
    } else {
        lastTimeoutMs = 0;  // 复位超时计数
    }
    if (!g_sbus.valid && sbusWasValid) {
        Serial.println("[SBUS] 信号丢失");
        g_motorArmed = false;
        escSet(PWM_NEUTRAL, PWM_NEUTRAL);
        beepDisarm();
        sbusWasValid = false;
    }
    if (g_sbus.valid && !sbusWasValid) {
        sbusWasValid = true;
        Serial.println("[SBUS] 信号恢复 (需重新 ARM)");
    }

    // 3. X5 MotorCmd
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

        uint16_t thr = PWM_NEUTRAL, str = PWM_NEUTRAL;

        // SBUS CH5 边沿检测 (防抖: 需连续 3 帧稳定在同一状态)
        bool ch5High = g_sbus.valid && (g_sbus.channels[SBUS_CH_ARM] > SBUS_ARM_THRESHOLD);
        static bool lastCh5High = false;
        static int  ch5StableCnt = 0;

        if (ch5High == lastCh5High) {
            if (ch5StableCnt < 3) ch5StableCnt++;
        } else {
            ch5StableCnt = 1;  // 状态变化, 重置计数
        }
        lastCh5High = ch5High;

        // 只在防抖通过后触发边沿
        if (ch5StableCnt == 3) {
            static bool ch5Armed = false;
            if (ch5High && !ch5Armed) {
                ch5Armed = true;
                if (!g_motorArmed) {
                    g_motorArmed = true;
                    beepArm();
                    Serial.println("[SBUS] ARMED (CH5 HIGH)");
                }
            } else if (!ch5High && ch5Armed) {
                ch5Armed = false;
                if (g_motorArmed) {
                    g_motorArmed = false;
                    escSet(PWM_NEUTRAL, PWM_NEUTRAL);
                    beepDisarm();
                    Serial.println("[SBUS] DISARMED (CH5 LOW)");
                }
            }
        }

        // CH6 模式切换: LOW=手控(SBUS), HIGH=自动(X5)
        bool autoMode = sbusActive && (g_sbus.channels[SBUS_CH_MODE] > SBUS_MODE_THRESHOLD);
        static bool lastAutoMode = false;
        if (autoMode != lastAutoMode && sbusActive) {
            beepNonBlocking(autoMode ? 2 : 1);  // 自动两声, 手控一声 (非阻塞)
            Serial.print("[MODE] 切换到 ");
            Serial.println(autoMode ? "自动(X5)" : "手控(RC)");
        }
        lastAutoMode = autoMode;

        if (sbusActive && g_motorArmed && !autoMode) {
            // 手控模式: SBUS 摇杆直接控制
            thr = sbusToPwm(g_sbus.channels[SBUS_CH_THROTTLE], SBUS_THR_SENSITIVITY);
            str = sbusToPwm(g_sbus.channels[SBUS_CH_STEERING], SBUS_STR_SENSITIVITY);
            g_lastCmdMs = now;

        } else if (autoMode && g_motorArmed && g_x5Valid && (now - g_lastX5Ms < CMD_TIMEOUT_MS)) {
            // 自动模式 + 已解锁: X5 指令生效 (需遥控器 CH5=ARM + CH6=AUTO)
            thr = g_x5Throttle;
            str = g_x5Steering;
            g_lastCmdMs = now;
        }

        escSet(thr, str);
        g_throttle = thr;
        g_steering = str;

        if (g_motorArmed && now - g_lastCmdMs > CMD_TIMEOUT_MS) {
            g_motorArmed = false;
            escSet(PWM_NEUTRAL, PWM_NEUTRAL);
            beepDisarm();
            Serial.println("[SAFE] 命令超时 60s, 自动锁定!");
        }
    }

    // 6. LED 闪烁 (PC13, active-LOW)
    static uint32_t lastLedMs;
    static bool     ledOn;
    bool sbusPresent = g_sbus.valid && !g_sbus.failsafe;
    uint32_t iv = g_motorArmed ? (sbusPresent ? 100 : 250) : 500;
    if (now - lastLedMs >= iv) {
        lastLedMs = now;
        ledOn = !ledOn;
        ledSet(ledOn);
    }

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

        bool sbusOk   = g_sbus.valid && !g_sbus.failsafe;
        bool x5Ok     = g_x5Valid && (now - g_lastX5Ms < CMD_TIMEOUT_MS);
        bool autoMode = sbusOk && (g_sbus.channels[SBUS_CH_MODE] > SBUS_MODE_THRESHOLD);

        const char* state;
        if (sbusOk && g_motorArmed && !autoMode)      state = "RC ARM";
        else if (sbusOk && !g_motorArmed && !autoMode) state = "RC LCK";
        else if (sbusOk && autoMode)                   state = "AUTO  ";
        else if (x5Ok)                                 state = "X5    ";
        else                                           state = "--------";

        Serial.print(state);
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
            Serial.print(" CH6=");
            Serial.print(autoMode ? "AUTO" : "MAN");
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
