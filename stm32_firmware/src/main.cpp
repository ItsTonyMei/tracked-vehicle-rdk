/**
 * STM32F103RCT6 — L1 执行与安全层 (Yahboom ROS V3.0 扩展板)
 *
 * 功能:
 *   1. SBUS 接收 (USART2 PA3, 100k 8E2, 三极管反相) — WFLY RF209S 适配
 *   2. X5 MotorCmd 解析 (USART1 PA9/PA10, CH340N, 115200) — 6字节 CRC8 帧
 *   3. 坦克混控 + 双路 ESC PWM (S1=PC3/左, S2=PC2/右, Servo 库 50Hz)
 *   4. 控制优先级: 手控 SBUS(CH5 ARM) > 自动 X5 > 2s 命令超时:
 *      手控模式超时 → 自动锁定; X5 模式超时 → 停车待命不锁定 (仅 CH5 手动锁)
 *   5. IWDG 独立看门狗 (4s 超时, loop 末尾喂狗)
 *   6. CH5 ARM/DISARM 防抖 (连续3帧确认, 帧同步采样) + 信号丢失需手动重新 ARM
 *   7. CH6 手控/自动模式切换: 中值滤波(5帧) + Schmitt滞回 + 非对称稳定确认
 *      (→手控 300ms 紧急接管要快, →X5 1s)
 *   8. SBUS 信号防抖: 5帧确认有效, 2次连续超时判丢失;
 *      帧头 0x0F 需前置 ≥1ms 空闲间隔 (帧内字节~110μs连续, 帧间~11ms),
 *      消除数据字节 0x0F 导致的错位锁定
 *   9. 蜂鸣器(PC5) + LED(PC13, active-LOW) 快/中/慢三速闪烁
 *  10. MPU9250 IMU SPI2 (PB12-15) 姿态读取 (PLL 时钟源)
 *  11. 全量程全速输出: WFLY 中位校准 (raw 1024→PWM 1500), 满杆 ±500μs 全行程;
 *      motor_bridge linear_gain=1000/angular_gain=600 → X5 各方向满 PWM.
 *      (原 PWM 斜率软启动已按用户要求移除 — 若 X5 欠压重启复发需恢复)
 *
 * 踩坑记录: 详见 docs/lessons-learned.md
 *   - PCx 无硬件 TIM → Servo 库; Serial 需显式映射; CH340N 无自动下载
 *   - WFLY RF209S byte24 非标, failsafe bit4; SBUS 阻塞丢帧 → 非阻塞状态机
 */

#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include "config.h"

// SPI2 for IMU (PB12-PB15 on STM32F103RCT6)
static SPIClass SPI_IMU(PIN_IMU_MOSI, PIN_IMU_MISO, PIN_IMU_SCLK, PIN_IMU_NSS);

// ═══════════════════════════════════════════════════════════════
// 全局状态
// ═══════════════════════════════════════════════════════════════

static Servo     servoLeft, servoRight;
static uint16_t  g_throttle   = PWM_NEUTRAL;
static uint16_t  g_steering   = PWM_NEUTRAL;
static uint32_t  g_lastCmdMs  = 0;
static bool      g_escReady   = false;
static bool      g_motorArmed = false;
static bool      g_ch5Armed   = false;  // CH5 边沿检测状态 (防抖通过后置位)
static bool      g_autoMode   = false;  // CH6 模式 (防抖后): false=手控RC, true=X5

static uint16_t  g_x5Throttle = PWM_NEUTRAL;
static uint16_t  g_x5Steering = PWM_NEUTRAL;
static uint32_t  g_lastX5Ms   = 0;
static bool      g_x5Valid    = false;
static bool      g_x5Stale    = false;  // X5 指令超时 (自动模式: 停车待命, 不锁定)

static struct {
    bool     valid;
    uint16_t channels[16];
    bool     failsafe;
    bool     lostFrame;
    uint32_t lastFrameMs;
    int      goodFrames;     // 连续好帧计数器 (防抖: 需≥5 帧才判定有效)
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
    uint8_t crc = CRC8_INIT;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (crc << 1) ^ CRC8_POLY : crc << 1;
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

static void computeMix(uint16_t throttle, uint16_t steering, int &left, int &right) {
    int sOff = (int)steering - (int)PWM_NEUTRAL;
    left  = (int)throttle + sOff;
    right = (int)throttle - sOff;
    if (left  < PWM_MIN) left  = PWM_MIN;
    if (left  > PWM_MAX) left  = PWM_MAX;
    if (right < PWM_MIN) right = PWM_MIN;
    if (right > PWM_MAX) right = PWM_MAX;
}

static void escSet(uint16_t throttle, uint16_t steering) {
    int left, right;
    computeMix(throttle, steering, left, right);
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

// 非阻塞蜂鸣: count=次数, durMs=每次鸣响时长(ms), gap duration = durMs
static uint32_t beepEndMs   = 0;
static uint8_t  beepPattern = 0;  // 0=idle, 1=beeping, 2=gap
static uint8_t  beepCount   = 0;
static uint16_t beepDurMs   = 60;

static void beepNonBlocking(int count, uint16_t durMs = 60) {
    beepCount   = count;
    beepDurMs   = durMs;
    beepEndMs   = millis();
    digitalWrite(PIN_BUZZER, HIGH);
    beepPattern = 1;
}

static void beepPoll() {
    if (beepPattern == 0) return;
    uint32_t now = millis();
    if (beepPattern == 1) {  // beeping
        if (now - beepEndMs >= beepDurMs) {
            digitalWrite(PIN_BUZZER, LOW);
            if (--beepCount > 0) {
                beepEndMs   = now;
                beepPattern = 2;  // gap
            } else {
                beepPattern = 0;  // done
            }
        }
    } else {  // gap (pattern 2)
        if (now - beepEndMs >= beepDurMs) {
            digitalWrite(PIN_BUZZER, HIGH);
            beepEndMs   = now;
            beepPattern = 1;  // beeping
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
    // RM0008: M and PCE bits must be changed only when UE=0
    USART2->CR1 &= ~USART_CR1_UE;
    USART2->CR1 |= USART_CR1_M | USART_CR1_PCE;
    USART2->CR2 |= USART_CR2_STOP_1;  // 2 stop bits (SBUS 8E2)
    USART2->CR1 |= USART_CR1_UE;
    Serial.println("[SBUS] USART2 PA3 @ 100k 8E2 (WFLY)");
}

static bool sbusParseFrame(const uint8_t *frame, uint16_t *channels) {
    if (frame[0] != 0x0F) return false;
    // WFLY RF209S byte24 非标, 不校验固定位 (bit4=failsafe 可变,
    // 其他位的定义与标准 SBUS 不同). 帧完整性由 sbusPoll 的
    // 空闲间隔帧头校验保证 (无帧尾/CRC 可依赖).

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

static bool sbusFrameReady = false;  // 新 SBUS 帧到达标志 (中值滤波同步用)

// SBUS 链路诊断计数器 (5Hz 状态行输出, 现场定位抖动来源):
//   hdrRej — 0x0F 前无空闲间隔被拒: 数据字节误匹配/错位尝试
//   ore    — USART2 溢出丢字节: 打印阻塞/中断延迟导致的物理丢帧
static uint32_t g_sbusHdrRejCnt = 0;
static uint32_t g_sbusOreCnt    = 0;

static void sbusPoll() {
    // 逐字节扫描 + 空闲间隔帧头校验:
    // SBUS 帧周期 14ms, 帧内 25 字节约 110μs/个连续到达, 帧间空闲 ~11ms.
    // 仅当 0x0F 前静默 ≥SBUS_HDR_GAP_US 才认作帧头 — 通道数据中的 0x0F
    // 字节不会触发假帧, 从根上消除"错位锁定" (稳态错位会输出一致错值,
    // 中值滤波对一致错值无效, 必须在帧同步层解决).
    // 注意: 字节经 ISR 环形缓冲, micros() 是读取时刻而非到达时刻.
    // 阻塞打印后排空缓冲时整批帧头会被拒, 最坏多丢 1-2 帧 (~28ms),
    // 由 5 帧有效性防抖吸收 — 宁可拒真帧, 不可收错帧.
    static uint32_t lastByteUs = 0;
    while (SerialSbus.available() > 0) {
        uint32_t rxUs = micros();
        int b = SerialSbus.read();
        if (b != 0x0F) { lastByteUs = rxUs; continue; }

        if (lastByteUs != 0 && rxUs - lastByteUs < SBUS_HDR_GAP_US) {
            lastByteUs = rxUs;
            g_sbusHdrRejCnt++;
            continue;
        }
        lastByteUs = rxUs;

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
            lastByteUs = micros();
        }
        if (!ok) continue;  // 帧不完整, 丢弃

        // ESC PWM 噪声可导致 USART2 奇偶/帧/噪声错误 (FE/NE/PE),
        // 但仅 ORE (数据溢出) 才意味着真正丢字节. FE/NE/PE 的帧
        // 信道数据通常仍然可用 — 下游的 end-byte 校验 + 中值滤波
        // 会处理偶尔的字节错误.
        if (USART2->SR & USART_SR_ORE) {
            USART2->SR = ~USART_SR_ORE;
            g_sbusOreCnt++;
            continue;
        }
        // 清除其他噪声标志 (不影响当前帧判断)
        USART2->SR = ~(USART_SR_FE | USART_SR_NE | USART_SR_PE);

        if (sbusParseFrame(frame, g_sbus.channels)) {
            // Read flags from byte23: bit2=lost_frame, bit4=failsafe (WFLY RF209S non-standard)
            g_sbus.lostFrame   = frame[23] & 0x04;
            g_sbus.failsafe    = frame[23] & 0x10;
            g_sbus.lastFrameMs = millis();
            sbusFrameReady = true;  // 新帧到达, 中值滤波采样同步点

            // lost_frame treated as failsafe — prevents control until good frame returns
            if (g_sbus.lostFrame) {
                g_sbus.failsafe = true;
            }
            if (!g_sbus.failsafe) {
                if (++g_sbus.goodFrames >= 5) {
                    g_sbus.valid = true;
                }
            }
        }
    }
}

// WFLY 校准: 摇杆中位 raw=1024 → PWM 1500 (遥控器 µs 显示 1500 = raw 1024),
// 满杆 raw 352/1695 → PWM 1000/2000 精确全行程 (无钳位损失)
static uint16_t sbusToPwm(uint16_t sbusVal, int sensitivity) {
    int off = (int)sbusVal - SBUS_CENTER;
    if (abs(off) <= 20) return PWM_NEUTRAL;
    int val = (int)PWM_NEUTRAL + off * sensitivity / SBUS_RAW_HALF_SPAN;
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

    mpu9250WriteReg(MPU9250_PWR_MGMT_1, 0x01);  // PLL with X-axis gyro reference
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

    // IWDG: ~4s timeout. LSI oscillator varies 30-50kHz over temp/voltage,
    // so actual timeout ranges ~3.2-5.3s. IWDG started late in setup();
    // code before this point (ESC init, sensor init) is NOT watchdog-protected.
    IWDG->KR = 0x5555;   // unlock
    IWDG->PR = 5;        // prescaler 128 → 40kHz/128 = 312.5 Hz
    IWDG->RLR = 1249;    // timeout = (1249+1)/312.5 = 4.0s
    IWDG->KR = 0xCCCC;   // start

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
        g_x5Stale = false;
        escSet(PWM_NEUTRAL, PWM_NEUTRAL);
        beepNonBlocking(1, 250);  // long beep: disarm
        sbusWasValid = false;
    }
    if (g_sbus.valid && !sbusWasValid) {
        sbusWasValid = true;
        if (millis() > 5000) {  // skip misleading message at boot
            Serial.println("[SBUS] 信号恢复 (需重新 ARM)");
        }
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
    // 刷新 now: sbusPoll/x5ParseMotorCmd 会阻塞数 ms, 且 g_lastX5Ms 用
    // 阻塞后的 millis() 打戳 — 若此处仍用循环顶部捕获的 now, 时间差
    // now - g_lastX5Ms 为负 → uint32 下溢 → 恒判 X5 超时,
    // 每帧触发 超时/恢复 循环, PWM 以 ~5Hz 振荡 (车原地抖动).
    now = millis();
    if (g_escReady) {
        bool sbusActive = g_sbus.valid && !g_sbus.failsafe;

        uint16_t thr = PWM_NEUTRAL, str = PWM_NEUTRAL;

        // ── 帧同步采样: 仅在新 SBUS 帧到达 (~14ms) 时更新 CH5/CH6 ──
        // 信号无效/失控时冻结采样与模式仲裁, 保持上次状态.
        // 修复: 原 CH5 "3帧防抖" 实际按 loop 迭代计数 (~μs级),
        // 3 次迭代 <1ms 即饱和, 等于无防抖 — 一段 14ms 毛刺窗口
        // 即可触发假 ARM/DISARM. 现与 CH6 统一按帧采样.
        static uint16_t ch6buf[5] = {0, 0, 0, 0, 0};
        static uint8_t  ch6idx = 0;
        static uint16_t lastCh6Raw = 0;
        static bool     ch5High  = false;
        static int      ch5StableFrames = 0;
        if (sbusFrameReady) {
            sbusFrameReady = false;
            if (sbusActive) {
                // CH5: 连续 3 帧 (~42ms) 同态才确认
                bool h = g_sbus.channels[SBUS_CH_ARM] > SBUS_ARM_THRESHOLD;
                ch5StableFrames = (h == ch5High) ? ch5StableFrames + 1 : 1;
                if (ch5StableFrames > 3) ch5StableFrames = 3;
                ch5High = h;

                // CH6 跳变诊断 (>500 跳变打印, 经 motor_bridge [DBG] 转发到 ROS)
                uint16_t curCh6Raw = g_sbus.channels[SBUS_CH_MODE];
                if (abs((int)curCh6Raw - (int)lastCh6Raw) > 500 && lastCh6Raw != 0) {
                    Serial.print("[DBG] CH6 ");
                    Serial.print(lastCh6Raw);
                    Serial.print("->");
                    Serial.println(curCh6Raw);
                }
                lastCh6Raw = curCh6Raw;
                ch6buf[ch6idx] = curCh6Raw;
                ch6idx = (ch6idx + 1) % 5;
            }
        }

        // CH5 边沿检测 (防抖通过后才触发, ch5Armed 为文件级全局持久化)
        if (ch5StableFrames == 3) {
            if (ch5High && !g_ch5Armed) {
                g_ch5Armed = true;
                if (!g_motorArmed) {
                    g_motorArmed = true;
                    beepNonBlocking(2);  // double beep: arm
                    Serial.println("[SBUS] ARMED (CH5 HIGH)");
                }
            } else if (!ch5High && g_ch5Armed) {
                g_ch5Armed = false;
                if (g_motorArmed) {
                    g_motorArmed = false;
                    g_x5Stale = false;
                    escSet(PWM_NEUTRAL, PWM_NEUTRAL);
                    beepNonBlocking(1, 250);  // long beep: disarm
                    Serial.println("[SBUS] DISARMED (CH5 LOW)");
                }
            }
        }

        // CH6 中值滤波 (5帧 ≈70ms 取中位) + Schmitt 滞回 + 跳变诊断
        // 关键: ch6buf[] 仅在 sbusFrameReady 时更新 (每 ~14ms 一次),
        // 而非每个 loop 迭代 (~14us). 5 帧覆盖 ~70ms, 单帧尖峰真正被隔离.
        uint16_t s[5] = {ch6buf[0], ch6buf[1], ch6buf[2], ch6buf[3], ch6buf[4]};
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4 - i; j++)
                if (s[j] > s[j+1]) { uint16_t t = s[j]; s[j] = s[j+1]; s[j+1] = t; }
        uint16_t ch6val = s[2];  // 中位数

        static bool autoModeHyst = false;
        if (ch6val > 1500)      autoModeHyst = true;
        else if (ch6val < 600)  autoModeHyst = false;
        // 600-1500: 保持上次 (Schmitt 死区, 覆盖三档开关中位 ~992)

        // 模式切换稳定确认 (非对称): CH6 机械振荡/毛刺已由上游
        // 中值滤波 (~70ms) + Schmitt 死区 (覆盖中位 ~992) 吸收,
        // 此处只需确认"刻意拨动并保持"而非长延时兜底:
        //   → 手控 300ms: 操作者紧急接管通道, 必须快 (安全)
        //   → X5   1s   : 进入自动可从容
        // 仲裁 gate 在 sbusActive 上: 失控/failsafe 期间冻结,
        // 防止失控把模式静默翻回手控.
        static bool     pendingMode     = false;
        static uint32_t pendingSinceMs  = 0;
        if (sbusActive) {
            bool desired = autoModeHyst;
            if (desired != g_autoMode && desired == pendingMode) {
                uint32_t needMs = desired ? MODE_TO_AUTO_MS : MODE_TO_MANUAL_MS;
                if (now - pendingSinceMs > needMs) {
                    g_autoMode    = desired;
                    beepNonBlocking(g_autoMode ? 2 : 1);
                    Serial.print("[MODE] 切换到 ");
                    Serial.println(g_autoMode ? "X5(RDK X5)" : "手控(RC)");
                }
            } else {
                pendingMode    = desired;
                pendingSinceMs = now;
            }
        }

        if (sbusActive && g_motorArmed && !g_autoMode) {
            // 手控模式: SBUS 摇杆直接控制
            thr = sbusToPwm(g_sbus.channels[SBUS_CH_THROTTLE], SBUS_THR_SENSITIVITY);
            str = sbusToPwm(g_sbus.channels[SBUS_CH_STEERING], SBUS_STR_SENSITIVITY);
            g_lastCmdMs = now;

        } else if (g_autoMode && g_motorArmed) {
            // X5模式 + 已解锁: X5 指令生效 (需遥控器 CH5=ARM + CH6=HIGH)
            // 指令超时仅停车待命 (输出中位), 不自动锁定 — 锁定只能由
            // 遥控器 CH5 手动打到锁定位置 (或 SBUS 信号丢失) 触发
            g_lastCmdMs = now;
            if (g_x5Valid && (now - g_lastX5Ms < X5_FRESH_TIMEOUT_MS)) {
                thr = g_x5Throttle;
                str = g_x5Steering;
                if (g_x5Stale) {
                    g_x5Stale = false;
                    Serial.println("[X5] 指令恢复, 继续接管");
                }
            } else if (!g_x5Stale) {
                g_x5Stale = true;
                Serial.println("[SAFE] X5 指令超时, 停车待命 (保持解锁)");
            }
        }

        escSet(thr, str);
        g_throttle = thr;
        g_steering = str;

        if (g_motorArmed && now - g_lastCmdMs > SBUS_LOCK_TIMEOUT_MS) {
            // 仅手控模式可达 (SBUS 失效/failsafe 时): 自动锁定需重新 ARM
            // X5 模式每循环刷新 g_lastCmdMs, 永不触发此锁定
            g_motorArmed = false;
            g_x5Stale = false;
            escSet(PWM_NEUTRAL, PWM_NEUTRAL);
            beepNonBlocking(1, 250);  // long beep: disarm
            Serial.println("[SAFE] 命令超时 2s, 自动锁定!");
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

        int left, right;
        computeMix(g_throttle, g_steering, left, right);

        bool sbusOk   = g_sbus.valid && !g_sbus.failsafe;
        bool x5Ok     = g_x5Valid && (now - g_lastX5Ms < X5_FRESH_TIMEOUT_MS);
        // 显示用防抖后的 g_autoMode, 与实际控制状态一致 (不再用裸通道值)

        const char* state;
        if (sbusOk && g_motorArmed && !g_autoMode)      state = "RC ARM";
        else if (sbusOk && !g_motorArmed && !g_autoMode) state = "RC LCK";
        else if (sbusOk && g_autoMode)                   state = "X5    ";
        else if (x5Ok)                                 state = "X5    ";
        else                                           state = "--------";

        // Status output (~7ms @115200) blocks main loop; SBUS USART2 has
        // only 1-byte HW buffer so some SBUS frames will be lost here.
        // Acceptable: 5-frame debounce + self-sync recovers within 100ms.
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
            Serial.print(g_autoMode ? "AUTO" : "MAN");
            Serial.print(" c1=");
            Serial.print(g_sbus.channels[SBUS_CH_STEERING]);
            Serial.print(" c2=");
            Serial.print(g_sbus.channels[SBUS_CH_THROTTLE]);
            Serial.print(" ore=");
            Serial.print(g_sbusOreCnt);
            Serial.print(" hdr=");
            Serial.print(g_sbusHdrRejCnt);
        }
        Serial.println();
    }

    // 8. IMU 读取 (10Hz)
    static uint32_t lastImu;
    if (g_imu.present && now - lastImu >= 100) {
        lastImu = now;
        mpu9250Read();
    }

    IWDG->KR = 0xAAAA;  // reload watchdog
}
