// ===== Flix STM32 Quadcopter — merged single-file build =====
// All original .ino files merged here to avoid Arduino IDE merge-order issues.

#include <Arduino.h>
#include <MAVLink.h>
#include <SBUS.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ===== All cross-function forward declarations =====
void setupConsole();
void setupParameters();
void setupPower();
void setupLED();
void setupMotors();
void setupIMU();
void setupRC();
void readIMU();
void step();
void computeLoopRate();
bool readRC();
void estimate();
void applyGyro();
void applyAcc();
void applyLevel();
void control();
void interpretControls();
void failsafe();
void rcLossFailsafe();
void descend();
void controlAttitude();
void controlRates();
void controlTorque();
void desaturate();
void sendMotors();
void handleConsole();
void doCommand(String str);
void processMavlink();
void sendMavlink();
void receiveMavlink();
void readVoltage();
void syncParameters();
void print(const char* fmt, ...);
void pause(float duration);
void setLED(bool on);
const char* getModeName();
void configureIMU();
void calibrateGyroOnce();
void calibrateAccelOnce();
bool doParametersCommand(const String& cmd, const String& arg0, const String& arg1);

// MAVLink helpers (called within mavlink section, defined later)
void sendMAVLinkMessage(mavlink_message_t* msg);
void sendMAVLinkPrint(const char* text);
void handleMAVLinkMessage(mavlink_message_t* msg);

// Motor helper
int getDutyCycle(float pwm);

// RC helper
void normalizeRC();

// Safety helpers
void autoFailsafe();
void tiltFailsafe();

// ===== FROM: console.ino =====
// 控制台命令行

#include "util.h"
#include "pid.h"
#include "vector.h"
#include "filter.h"
#include <Arduino.h>
#include <stdarg.h>

extern const int MOT_RL, MOT_RR, MOT_FR, MOT_FL;
extern const int RAW, ACRO, STAB, AUTO;
const char* getModeName();   // control.ino
extern float t, dt, loopRate;
extern int   mode;
extern bool  armed;
extern bool  landed;

static String _consoleBuffer;

void setupConsole() {
    Serial.begin(115200);
    while (!Serial) delay(10);   // 等 USB 串口
    Serial.println(F("\nFlix STM32 Quadcopter Firmware"));
    Serial.println(F("==================================="));
}

// 格式化打印 (同时输出到 Serial 和 MAVLink console)
void print(const char* fmt, ...) {
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
    // MAVLink print 留到 mavlink.ino 处理 (依赖 lib)
}

// 非阻塞暂停 (飞行中禁止用 delay!)
void pause(float duration) {
    float start = t;
    while (t - start < duration) {
        step();
        handleConsole();
        delay(10);
    }
}

// 控制台命令处理
void handleConsole() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (_consoleBuffer.length() > 0) {
                doCommand(_consoleBuffer);
                _consoleBuffer = "";
            }
        } else if (c == 8 || c == 127) {
            // backspace
            if (_consoleBuffer.length() > 0) _consoleBuffer.remove(_consoleBuffer.length() - 1);
        } else {
            _consoleBuffer += c;
        }
    }
}

// 简单字符串分割
static int splitString(const String& s, String& t0, String& t1, String& t2) {
    int idx1 = s.indexOf(' ');
    int idx2 = (idx1 >= 0) ? s.indexOf(' ', idx1 + 1) : -1;
    if (idx1 < 0) { t0 = s; t1 = ""; t2 = ""; return 1; }
    t0 = s.substring(0, idx1);
    if (idx2 < 0) { t1 = s.substring(idx1 + 1); t2 = ""; return 2; }
    t1 = s.substring(idx1 + 1, idx2);
    t2 = s.substring(idx2 + 1);
    return 3;
}

// 命令执行 —— 占位, 具体命令由 parameters/mavlink 等注册
void doCommand(String str) {
    str.trim();
    if (str.length() == 0) return;
    str.toLowerCase();

    String cmd, arg0, arg1;
    splitString(str, cmd, arg0, arg1);

    Serial.print(F("> "));
    Serial.println(str);

    extern bool doParametersCommand(const String& cmd, const String& arg0, const String& arg1);
    extern bool doMavlinkCommand  (const String& cmd, const String& arg0, const String& arg1);
    extern bool doSystemCommand   (const String& cmd, const String& arg0, const String& arg1);

    if      (doSystemCommand   (cmd, arg0, arg1)) return;
    else if (doParametersCommand(cmd, arg0, arg1)) return;
    else if (doMavlinkCommand   (cmd, arg0, arg1)) return;

    Serial.println(F("Unknown command. Type 'help' for list."));
}

bool doSystemCommand(const String& cmd, const String& arg0, const String& arg1) {
    if (cmd == "help" || cmd == "?") {
        Serial.println(F("Commands:"));
        Serial.println(F("  help                - this list"));
        Serial.println(F("  arm / disarm        - arm/disarm motors"));
        Serial.println(F("  stab / acro / raw   - set flight mode"));
        Serial.println(F("  time                - show t, dt, loop rate"));
        Serial.println(F("  status              - show armed, mode, landed"));
        Serial.println(F("  imu                 - show IMU data"));
        Serial.println(F("  mot                 - show motor outputs"));
        Serial.println(F("  reboot              - soft reset"));
        Serial.println(F("  p [<filter>]        - show parameters"));
        Serial.println(F("  p <name> <val>      - set parameter"));
        Serial.println(F("  rc                  - show RC channels"));
        Serial.println(F("  pw                  - show battery voltage"));
        return true;
    }

    if (cmd == "arm") {
        armed = true;
        Serial.println(F("Armed"));
        return true;
    }
    if (cmd == "disarm") {
        armed = false;
        Serial.println(F("Disarmed"));
        return true;
    }

    if (cmd == "stab") { mode = STAB;  Serial.println(F("Mode: STAB"));  return true; }
    if (cmd == "acro") { mode = ACRO;  Serial.println(F("Mode: ACRO"));  return true; }
    if (cmd == "raw")  { mode = RAW;   Serial.println(F("Mode: RAW"));   return true; }

    if (cmd == "time") {
        Serial.print(F("t=")); Serial.print(t, 3);
        Serial.print(F(" dt=")); Serial.print(dt, 6);
        Serial.print(F(" rate=")); Serial.println(loopRate, 0);
        return true;
    }

    if (cmd == "status") {
        extern float thrustTarget;
        Serial.print(F("armed=")); Serial.print(armed);
        Serial.print(F(" mode=")); Serial.print(getModeName());
        Serial.print(F(" landed=")); Serial.println(landed);
        Serial.print(F("thrustTarget=")); Serial.println(thrustTarget, 3);
        return true;
    }

    if (cmd == "imu") {
        extern Vector gyro, acc;
        Serial.print(F("gyro: ")); Serial.print(gyro.x, 5); Serial.print(" ");
                                   Serial.print(gyro.y, 5); Serial.print(" ");
                                   Serial.println(gyro.z, 5);
        Serial.print(F("acc:  ")); Serial.print(acc.x, 3); Serial.print(" ");
                                   Serial.print(acc.y, 3); Serial.print(" ");
                                   Serial.println(acc.z, 3);
        return true;
    }

    if (cmd == "mot") {
        extern float motors[4];
        Serial.print(F("FL=")); Serial.print(motors[MOT_FL], 3);
        Serial.print(F(" FR=")); Serial.print(motors[MOT_FR], 3);
        Serial.print(F(" RL=")); Serial.print(motors[MOT_RL], 3);
        Serial.print(F(" RR=")); Serial.println(motors[MOT_RR], 3);
        return true;
    }

    if (cmd == "rc") {
        extern uint16_t channels[16];
        Serial.print(F("ch:"));
        for (int i = 0; i < 8; i++) { Serial.print(' '); Serial.print(channels[i]); }
        Serial.println();
        return true;
    }

    if (cmd == "pw") {
        extern float voltage;
        Serial.print(F("voltage: ")); Serial.print(voltage, 3); Serial.println(" V");
        return true;
    }

    if (cmd == "reboot") {
        Serial.println(F("Rebooting..."));
        NVIC_SystemReset();
        return true;
    }

    return false;   // 未识别
}


// ===== FROM: control.ino =====
// 飞行控制 —— 三层级联 PID
//
//   姿态环 (roll/pitch P)
//        ↓ 目标角速度
//   角速度环 (roll/pitch/yaw PID)
//        ↓ 目标力矩
//   力矩→电机混控 (X 型四旋翼)

#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "filter.h"
#include "util.h"

// 外部引脚常量 (在 motors.ino 中定义)
extern const int MOT_RL, MOT_RR, MOT_FR, MOT_FL;

// 外部控制输入
extern float controlRoll, controlPitch, controlYaw, controlThrottle, controlMode;

// 外部状态 (estimate.ino / time.ino / motors.ino)
extern Quaternion attitude;
extern Vector rates;
extern float  motors[4];
extern float  t, dt;

// 飞行模式
const int RAW  = 0;
const int ACRO = 1;
const int STAB = 2;
const int AUTO = 3;

int mode = STAB;            // 当前模式
bool armed = false;         // 是否已解锁

// 控制目标
Quaternion attitudeTarget;  // 目标姿态
Vector ratesTarget;         // 目标角速度
Vector ratesExtra;          // 前馈角速度 (yaw 杆在 STAB 下的作用)
Vector torqueTarget;        // 目标力矩
float thrustTarget;         // 总油门 [0,1]

// ===== PID 参数 =====
PID rollRatePID (0.05f, 0.2f, 0.001f, 0.3f, 0.2f);
PID pitchRatePID(0.05f, 0.2f, 0.001f, 0.3f, 0.2f);
PID yawRatePID  (0.3f,  0,    0,      0.3f);
PID rollPID     (6.0f);
PID pitchPID    (6.0f);
PID yawPID      (3.0f);

Vector maxRate(radians(360), radians(360), radians(360));  // 最大角速度
float tiltMax = radians(30);                                // STAB 模式最大倾角
int   flightModes[] = {STAB, STAB, STAB};                   // 模式开关映射

// ===== 主控制函数 =====
void control() {
    interpretControls();
    failsafe();
    controlAttitude();
    controlRates();
    controlTorque();
}

// 把摇杆输入转成控制目标
void interpretControls() {
    // 模式切换
    if      (controlMode < 0.25f) mode = flightModes[0];
    else if (controlMode <= 0.75f) mode = flightModes[1];
    else if (controlMode >  0.75f) mode = flightModes[2];

    if (mode == AUTO) return;

    // 解锁/上锁手势 (油门最低 + 偏航打满)
    if (controlThrottle < 0.05f && controlYaw >  0.95f) armed = true;
    if (controlThrottle < 0.05f && controlYaw < -0.95f) armed = false;

    if (fabsf(controlYaw) < 0.1f) controlYaw = 0;

    thrustTarget = controlThrottle;

    if (mode == STAB) {
        float yawTarget = attitudeTarget.getYaw();
        if (!armed || invalid(yawTarget) || controlYaw != 0)
            yawTarget = attitude.getYaw();   // 解锁或偏航杆推动时重置 yaw 目标

        attitudeTarget = Quaternion::fromEuler(
            Vector(controlRoll * tiltMax, controlPitch * tiltMax, yawTarget));
        ratesExtra = Vector(0, 0, -controlYaw * maxRate.z);
    }

    if (mode == ACRO) {
        attitudeTarget.invalidate();         // 跳过姿态环
        ratesTarget.x =  controlRoll  * maxRate.x;
        ratesTarget.y =  controlPitch * maxRate.y;
        ratesTarget.z = -controlYaw   * maxRate.z;
    }

    if (mode == RAW) {
        attitudeTarget.invalidate();
        ratesTarget.invalidate();
        torqueTarget = Vector(controlRoll, controlPitch, -controlYaw) * 0.1f;
    }
}

// ===== 姿态环 =====
void controlAttitude() {
    if (!armed || attitudeTarget.invalid() || thrustTarget < 0.1f) return;

    const Vector up(0, 0, 1);
    Vector upActual = Quaternion::rotateVector(up, attitude);
    Vector upTarget = Quaternion::rotateVector(up, attitudeTarget);

    Vector error = Vector::rotationVectorBetween(upTarget, upActual);

    ratesTarget.x = rollPID.update(error.x)  + ratesExtra.x;
    ratesTarget.y = pitchPID.update(error.y) + ratesExtra.y;

    float yawError = wrapAngle(attitudeTarget.getYaw() - attitude.getYaw());
    ratesTarget.z = yawPID.update(yawError) + ratesExtra.z;
}

// ===== 角速度环 =====
void controlRates() {
    if (!armed || ratesTarget.invalid() || thrustTarget < 0.1f) return;

    Vector error = ratesTarget - rates;

    torqueTarget.x = rollRatePID.update(error.x);
    torqueTarget.y = pitchRatePID.update(error.y);
    torqueTarget.z = yawRatePID.update(error.z);
}

// ===== 力矩 → 电机混控 (X 型) =====
// 电机排列:  FL(前左, 顺时针)  FR(前右, 逆时针)
//            RL(后左, 逆时针)  RR(后右, 顺时针)
void controlTorque() {
    if (!torqueTarget.valid()) return;

    float& m0 = motors[MOT_FL];   // 前左
    float& m1 = motors[MOT_FR];   // 前右
    float& m2 = motors[MOT_RL];   // 后左
    float& m3 = motors[MOT_RR];   // 后右

    if (!armed) {
        m0 = m1 = m2 = m3 = 0;
        return;
    }

    if (thrustTarget < 0.1f) {
        m0 = m1 = m2 = m3 = 0.1f;   // 怠速
        return;
    }

    // X 型混控公式
    m0 = thrustTarget + torqueTarget.x - torqueTarget.y + torqueTarget.z;
    m1 = thrustTarget - torqueTarget.x - torqueTarget.y - torqueTarget.z;
    m2 = thrustTarget + torqueTarget.x + torqueTarget.y - torqueTarget.z;
    m3 = thrustTarget - torqueTarget.x + torqueTarget.y + torqueTarget.z;

    desaturate();
}

// 去饱和: 整体平移, 优先保证姿态
void desaturate() {
    float max_ = -INFINITY, min_ = INFINITY;
    for (float m : motors) {
        if (m > max_) max_ = m;
        if (m < min_) min_ = m;
    }

    float correction = 0;
    if      (max_ > 1.0f) correction = max_ - 1.0f;
    else if (min_ < 0.0f) correction = min_;

    for (float& m : motors) m -= correction;
}

const char* getModeName() {
    switch (mode) {
        case RAW:  return "RAW";
        case ACRO: return "ACRO";
        case STAB: return "STAB";
        case AUTO: return "AUTO";
        default:   return "UNKNOWN";
    }
}


// ===== FROM: estimate.ino =====
// 姿态估计 (互补滤波)

#include "vector.h"
#include "quaternion.h"
#include "filter.h"
#include "util.h"

// 外部依赖
extern Vector gyro, acc;          // IMU 原始数据
extern float motors[4];           // 电机输出 (判断着陆)
extern float thrustTarget;        // 油门目标 (飞行中辅助回平)
extern float t, dt;               // 时间 (time.ino)

Vector rates;                     // 滤波后的角速度
Quaternion attitude;              // 当前姿态
bool landed;                      // 是否已着陆

float accWeight   = 0.003f;       // 加速度计重力修正权重
float levelWeight = 0.0002f;      // 飞行中回平修正权重

LowPassFilter<Vector> ratesFilter(0.2f);   // 角速度低通, 截止 ~40 Hz

void estimate() {
    applyGyro();
    applyAcc();
    applyLevel();
}

void applyGyro() {
    rates = ratesFilter.update(gyro);
    attitude = Quaternion::rotate(attitude,
        Quaternion::fromRotationVector(rates * dt));
}

void applyAcc() {
    // 判断是否着陆: 电机停转 且 加速度模接近 1g
    bool motorsStop = (motors[0] == 0 && motors[1] == 0 &&
                       motors[2] == 0 && motors[3] == 0);
    landed = motorsStop && fabsf(acc.norm() - ONE_G) < ONE_G * 0.1f;

    if (!landed) return;

    // 用加速度计的重力方向修正姿态
    Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
    Vector correction = Vector::rotationVectorBetween(acc, up) * accWeight;
    attitude = Quaternion::rotate(attitude,
        Quaternion::fromRotationVector(correction));
}

void applyLevel() {
    if (landed) return;
    if (thrustTarget < 0.1f) return;   // 怠速油门跳过

    // 假设飞行员大致保持水平, 施加极弱的回平
    Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
    Vector correction = Vector::rotationVectorBetween(Vector(0, 0, 1), up) * levelWeight;
    attitude = Quaternion::rotate(attitude,
        Quaternion::fromRotationVector(correction));
}


// ===== FROM: flix.ino =====
// ====================================================================
// Flix STM32 Quadcopter Firmware —— 主入口
// ====================================================================
// 基于 STM32F407 + STM32duino Arduino Framework
//
// 架构说明:
//   每个 .ino 文件 "拥有" 自己的全局变量定义, 其他 .ino 用 extern 引用
//   这和原 Flix ESP32 架构保持一致, 避免多重定义 linker error
//
// 子系统文件 (同目录 .ino, PlatformIO / Arduino IDE 一起编译):
//   time.ino       —— 时间基准 (定义 t, dt, loopRate)
//   imu.ino        —— IMU 驱动 (定义 gyro, acc, imuModel, ...)
//   rc.ino         —— SBUS 遥控器 (定义 controlRoll/Pitch/Yaw/Throttle/Mode, channels, ...)
//   estimate.ino   —— 姿态估计 (定义 rates, attitude, landed, ratesFilter, ...)
//   control.ino    —— 级联 PID (定义 PID 对象, mode, armed, torqueTarget, ...)
//   motors.ino     —— 电机 PWM (定义 motors, motorPins, pwmFrequency, MOT_FL/FR/RL/RR)
//   safety.ino     —— Fail-safe (定义 rcLossTimeout, descendTime, disarmTilt)
//   power.ino      —— 电池电压 (定义 voltage, voltageFilter, voltagePin)
//   led.ino        —— LED
//   parameters.ino —— 参数存储 (内部 Flash 读写)
//   console.ino    —— 命令行
//   mavlink.ino    —— MAVLink 遥测
// ====================================================================

#include "vector.h"
#include "quaternion.h"
#include "util.h"
#include <MAVLink.h>
#include <SBUS.h>

// ====== 跨 .ino 函数原型 (放在最前, 保证编译顺序无关) ======
void setupConsole();
void setupParameters();
void setupPower();
void setupLED();
void setLED(bool on);
void setupMotors();
void setupIMU();
void setupRC();
void readIMU();
void step();
bool readRC();
void estimate();
void control();
void sendMotors();
void handleConsole();
void processMavlink();
void readVoltage();
void syncParameters();
void print(const char* fmt, ...);
void pause(float duration);

// ====== 所有全局变量: 由各个子系统 .ino 定义, 这里用 extern 引用 ======
// time.ino
extern float t, dt, loopRate;

// imu.ino
extern Vector gyro, acc;
extern int   imuModel, imuBus;
extern int   imuSckPin, imuMisoPin, imuMosiPin, imuCsPin, imuIntPin;
extern Vector imuRotation;
extern Vector gyroBias, accBias, accScale;

// estimate.ino
extern Vector rates;
extern Quaternion attitude;
extern bool   landed;
extern float  accWeight, levelWeight;

// rc.ino
extern float controlRoll, controlPitch, controlYaw, controlThrottle, controlMode;
extern float controlTime;
extern uint16_t channels[16];
extern int  rcRxPin, rollChannel, pitchChannel, throttleChannel, yawChannel, modeChannel;

// control.ino
extern int  mode;
extern bool armed;
extern Quaternion attitudeTarget;
extern Vector ratesTarget, ratesExtra, torqueTarget;
extern float thrustTarget;

// motors.ino
extern float motors[4];
extern int   motorPins[4];
extern const int MOT_RL, MOT_RR, MOT_FR, MOT_FL;

// power.ino
extern float voltage;
extern int   voltagePin;
extern float voltageScale;

// safety.ino
extern float rcLossTimeout, descendTime, disarmTilt;

// ====== setup: 初始化所有子系统 ======
void setup() {
    setupConsole();
    Serial.println(F("\n=== Flix STM32 Init ==="));

    // 1. 参数 (从 Flash 读 + config.h 默认值)
    setupParameters();

    // 2. 电源电压
    setupPower();

    // 3. LED
    setupLED();
    setLED(true);

    // 4. 电机
    setupMotors();

    // 5. IMU
    setupIMU();

    // 6. RC (SBUS)
    setupRC();

    setLED(false);
    Serial.println(F("=== Init complete ==="));
}

// ====== loop: 主循环 (节拍由 IMU 1kHz 决定) ======
void loop() {
    readIMU();        // ① 读传感器 (阻塞等待 IMU 数据, 决定节拍)
    step();           // ② 更新 t, dt
    readRC();         // ③ 读遥控器
    estimate();       // ④ 姿态估计
    control();        // ⑤ 控制计算
    sendMotors();     // ⑥ PWM 输出
    handleConsole();  // ⑦ 命令行
    processMavlink(); // ⑧ MAVLink 遥测
    readVoltage();    // ⑨ 电池电压
    syncParameters(); // ⑩ 参数落盘
}



// ===== FROM: imu.ino =====
// IMU 驱动 (MPU9250 / ICM20948 / MPU6050 / ICM40609-D)
// 依赖 Arduino 库 FlixPeriph (作者自己的跨平台 IMU 库)
//   PlatformIO:  lib_deps = okalachev/FlixPeriph

#include <SPI.h>
#include <Wire.h>
#include <FlixPeriph.h>
#include "vector.h"
#include "filter.h"
#include "util.h"

IMU *imu = nullptr;
int imuModel = 1;   // 1=MPU9250, 2=ICM20948, 3=MPU6050, 4=ICM40609-D
int imuBus   = 0;   // 0=SPI, 1=I2C

int imuSckPin  = -1;
int imuMisoPin = -1;
int imuMosiPin = -1;
int imuCsPin   = -1;
int imuIntPin  = -1;
int imuSdaPin  = SDA;
int imuSclPin  = SCL;

Vector imuRotation(0, 0, PI / 2);   // IMU 板在 PCB 上的欧拉角安装方向

Vector gyro;          // 陀螺原始输出, rad/s
Vector gyroBias;      // 零偏

Vector acc;           // 加速度原始输出, m/s²
Vector accBias;
Vector accScale(1, 1, 1);

LowPassFilter<Vector> gyroBiasFilter(0.001f);

extern float t;

void setupIMU() {
    Serial.println(F("Setup IMU"));

    delete imu;
    imu = nullptr;

    if (imuModel == 3) imuBus = 1;   // MPU6050 只有 I2C 版

    if (imuBus == 0) {
        // STM32duino SPI: 引脚由 board variant 固定, 没有 setSCLK/setMISO/setMOSI
        // 如需换引脚, 请在 variant 或 CubeMX 中重新映射
        SPI.begin();
        imu = IMU::create(imuModel, SPI, imuCsPin, imuIntPin);
    } else {
        Wire.setSDA(imuSdaPin);
        Wire.setSCL(imuSclPin);
        Wire.begin();
        imu = IMU::create(imuModel, Wire, imuIntPin);
    }

    if (!imu) {
        Serial.println(F("ERROR: IMU create failed"));
        return;
    }

    imu->begin();
    configureIMU();
}

void configureIMU() {
    imu->setAccelRange(IMU::ACCEL_RANGE_4G);
    imu->setGyroRange (IMU::GYRO_RANGE_2000DPS);
    imu->setDLPF      (IMU::DLPF_MAX);
    imu->setRate      (IMU::RATE_1KHZ_APPROX);
    imu->setupInterrupt();
}

void readIMU() {
    if (!imu) return;

    imu->waitForData();
    imu->getGyro(gyro.x, gyro.y, gyro.z);
    imu->getAccel(acc.x, acc.y, acc.z);
    calibrateGyroOnce();

    // 应用零偏和比例
    acc  = (acc - accBias) / accScale;
    gyro = gyro - gyroBias;

    // 旋转到机体系
    Quaternion rot = Quaternion::fromEuler(imuRotation);
    acc  = Quaternion::rotateVector(acc,  rot.inversed());
    gyro = Quaternion::rotateVector(gyro, rot.inversed());
}

void calibrateGyroOnce() {
    static Delay landedDelay(2.0f);
    if (!landedDelay.update(landed)) return;
    gyroBias = gyroBiasFilter.update(gyro);
}

void calibrateAccel() {
    Serial.println(F("Calibrating accelerometer"));
    imu->setAccelRange(IMU::ACCEL_RANGE_2G);

    Serial.println(F("1/6 Place level [8s]"));  pause(8);  calibrateAccelOnce();
    Serial.println(F("2/6 Place nose up [8s]")); pause(8);  calibrateAccelOnce();
    Serial.println(F("3/6 Place nose dn [8s]")); pause(8);  calibrateAccelOnce();
    Serial.println(F("4/6 Place right [8s]"));  pause(8);  calibrateAccelOnce();
    Serial.println(F("5/6 Place left [8s]"));   pause(8);  calibrateAccelOnce();
    Serial.println(F("6/6 Place upside [8s]")); pause(8);  calibrateAccelOnce();

    Serial.println(F("Done!"));
    configureIMU();
}

void calibrateAccelOnce() {
    const int samples = 1000;
    static Vector accMax(-INFINITY, -INFINITY, -INFINITY);
    static Vector accMin( INFINITY,  INFINITY,  INFINITY);

    Vector sum(0, 0, 0);
    for (int i = 0; i < samples; i++) {
        imu->waitForData();
        Vector s;
        imu->getAccel(s.x, s.y, s.z);
        sum = sum + s;
    }
    Vector avg = sum / (float)samples;

    if (avg.x > accMax.x) accMax.x = avg.x;
    if (avg.y > accMax.y) accMax.y = avg.y;
    if (avg.z > accMax.z) accMax.z = avg.z;
    if (avg.x < accMin.x) accMin.x = avg.x;
    if (avg.y < accMin.y) accMin.y = avg.y;
    if (avg.z < accMin.z) accMin.z = avg.z;

    accScale = (accMax - accMin) / 2.0f / ONE_G;
    accBias  = (accMax + accMin) / 2.0f;
}

// external forward: estimate.ino 里 landed 变量
extern bool landed;


// ===== FROM: led.ino =====
// LED (PC13 低电平亮)

#define LED_PERIOD 500000   // 闪烁周期 (micros)

#ifndef LED_PIN
#define LED_PIN PC13
#endif

static bool _ledState = false;

void setupLED() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);   // 默认关
}

void setLED(bool on) {
    if (on == _ledState) return;
    _ledState = on;
    digitalWrite(LED_PIN, on ? LOW : HIGH);
}

void blinkLED() {
    setLED((micros() / LED_PERIOD) % 2);
}


// ===== FROM: mavlink.ino =====
// MAVLink 通信 (通过 USB 虚拟串口 Serial)
// STM32F407 无内置 WiFi, 先通过 USB 与 QGroundControl 通信

#include <MAVLink.h>
#include "util.h"
#include "vector.h"
#include "quaternion.h"
#include "filter.h"
#include <Arduino.h>

extern Vector gyro, acc, rates;
extern Quaternion attitude;
extern float  t, loopRate, voltage;
extern float  motors[4];
extern uint16_t channels[16];
extern float  controlTime;
extern bool   armed, landed;
extern int    mode;
extern float  thrustTarget;

int mavlinkSysId = 1;

Rate telemetrySlow     (2);
Rate telemetryAttitude (20);
Rate telemetryRC       (10);
Rate telemetryMotors   (10);
Rate telemetryIMU      (15);

float mavlinkTime = NAN;
static String mavlinkPrintBuffer;

void processMavlink() {
    sendMavlink();
    receiveMavlink();
}

void sendMavlink() {
    mavlink_message_t msg;
    uint32_t timeMs = (uint32_t)(t * 1000);

    // ===== 心跳 =====
    if (telemetrySlow) {
        uint8_t baseMode =
            (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0) |
            ((mode == STAB) ? MAV_MODE_FLAG_STABILIZE_ENABLED : 0) |
            ((mode == AUTO) ? MAV_MODE_FLAG_AUTO_ENABLED : MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
        uint8_t state = landed ? MAV_STATE_STANDBY : MAV_STATE_ACTIVE;
        mavlink_msg_heartbeat_pack(
            mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
            MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
            baseMode, mode, state);
        sendMAVLinkMessage(&msg);
    }

    if (!valid(mavlinkTime)) return;   // 收不到地面站消息前只发心跳

    // 发送 print buffer
    if (mavlinkPrintBuffer.length() > 0) {
        sendMAVLinkPrint(mavlinkPrintBuffer.c_str());
        mavlinkPrintBuffer = "";
    }

    if (telemetrySlow) {
        mavlink_msg_extended_sys_state_pack(
            mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
            MAV_VTOL_STATE_UNDEFINED,
            landed ? MAV_LANDED_STATE_ON_GROUND : MAV_LANDED_STATE_IN_AIR);
        sendMAVLinkMessage(&msg);
    }

    if (telemetryAttitude) {
        mavlink_msg_attitude_quaternion_pack(
            mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
            timeMs,
            attitude.w,  attitude.x,  -attitude.y, -attitude.z,
            rates.x,    -rates.y,    -rates.z,
            NULL);
        sendMAVLinkMessage(&msg);
    }

    if (telemetryMotors) {
        float controls[8] = {motors[0], motors[1], motors[2], motors[3], 0, 0, 0, 0};
        mavlink_msg_actuator_control_target_pack(
            mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
            timeMs, 0, controls);
        sendMAVLinkMessage(&msg);
    }

    if (telemetryRC && channels[0]) {
        mavlink_msg_rc_channels_raw_pack(
            mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
            (uint32_t)(controlTime * 1000), 0,
            channels[0], channels[1], channels[2], channels[3],
            channels[4], channels[5], channels[6], channels[7],
            UINT8_MAX);
        sendMAVLinkMessage(&msg);
    }

    if (telemetrySlow && valid(voltage)) {
        uint16_t v = (uint16_t)(voltage * 1000);
        uint16_t voltages[10] = {v, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                                  UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX};
        uint16_t voltagesExt[4] = {0, 0, 0, 0};
        uint8_t  remaining = (uint8_t)(constrain(mapf(voltage, 3.4f, 4.2f, 0, 1), 0, 1) * 100);
        mavlink_msg_battery_status_pack(
            mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
            0, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO, INT16_MAX,
            voltages, -1, -1, -1, remaining, 0, MAV_BATTERY_CHARGE_STATE_OK,
            voltagesExt, 0, 0);
        sendMAVLinkMessage(&msg);
    }
}

void sendMAVLinkMessage(mavlink_message_t* msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    Serial.write(buf, len);
}

void sendMAVLinkPrint(const char* text) {
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
        MAV_SEVERITY_INFO, text, 0, 0);  // 新版 API 需要 id + chunk_seq
    sendMAVLinkMessage(&msg);
}

void mavlinkPrint(const char* text) {
    mavlinkPrintBuffer += text;
}

void receiveMavlink() {
    static mavlink_channel_t chan = MAVLINK_COMM_0;
    uint16_t len = Serial.available();   // 关键: 每次读新鲜的 available, 不要 static
    if (len == 0) return;
    if (len > 64) len = 64;
    uint8_t rx[64];
    Serial.readBytes(rx, len);

    mavlink_message_t msg;
    mavlink_status_t status;
    for (uint16_t i = 0; i < len; i++) {
        if (mavlink_parse_char(chan, rx[i], &msg, &status)) {
            mavlinkTime = t;
            handleMAVLinkMessage(&msg);
        }
    }
}

void handleMAVLinkMessage(mavlink_message_t* msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t h;
            mavlink_msg_heartbeat_decode(msg, &h);
            if (h.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) armed = true;
            break;
        }
        case MAVLINK_MSG_ID_SET_MODE: {
            mavlink_set_mode_t m;
            mavlink_msg_set_mode_decode(msg, &m);
            if (m.custom_mode >= 0 && m.custom_mode <= 3) {
                mode = m.custom_mode;
                armed = !!(m.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
            }
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t c;
            mavlink_msg_command_long_decode(msg, &c);
            if (c.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                armed = (c.param1 > 0.5f);
            }
            break;
        }
    }
}

bool doMavlinkCommand(const String& cmd, const String& arg0, const String& arg1) {
    if (cmd == "mav") {
        Serial.print(F("MAVLink connected: "));
        Serial.println(valid(mavlinkTime));
        return true;
    }
    return false;
}


// ===== FROM: motors.ino =====
// 电机 PWM 驱动 —— STM32F407 (TIM4 CH1-4)
// 用 STM32duino analogWrite 系列实现

#include "util.h"
#include <Arduino.h>

float motors[4];          // 归一化油门 [0,1]

int motorPins[4] = {-1, -1, -1, -1};   // 引脚号, 由 config.h setDefaults() 填写
int pwmFrequency  = 40000;              // 40 kHz
int pwmResolution = 10;                 // 10 位 = 1024 级 (analogWrite 上限 1023)
int pwmStop = 0;
int pwmMin  = 0;
int pwmMax  = -1;                       // -1 = 占空比模式

// 电机索引常量
const int MOT_RL = 0;
const int MOT_RR = 1;
const int MOT_FR = 2;
const int MOT_FL = 3;

static int _pwmMaxValue = 1023;   // 10 位上限

void setupMotors() {
    // 设置 PWM 频率 (STM32duino 专用)
    analogWriteResolution(pwmResolution);
    analogWriteFrequency(pwmFrequency);   // STM32duino API: analogWriteFrequency (不是 ESP32 的 analogWriteFreq)

    for (int i = 0; i < 4; i++) {
        if (motorPins[i] < 0) continue;
        pinMode(motorPins[i], OUTPUT);
    }
    sendMotors();   // 先输出一次, 确保停转
}

void sendMotors() {
    for (int i = 0; i < 4; i++) {
        if (motorPins[i] < 0) continue;
        analogWrite(motorPins[i], getDutyCycle(motors[i]));
    }
}

int getDutyCycle(float value) {
    if (value < 0) value = 0;
    if (value > 1) value = 1;

    if (pwmMax >= 0) {
        // PWM 值模式 (ESC 协议等)
        float pwm = mapf(value, 0, 1, pwmMin, pwmMax);
        if (value == 0) pwm = pwmStop;
        return (int)pwm;
    } else {
        // 占空比模式
        return (int)(value * _pwmMaxValue);
    }
}

bool motorsActive() {
    return motors[0] != 0 || motors[1] != 0 || motors[2] != 0 || motors[3] != 0;
}

void testMotor(int n, float thrust) {
    if (n < 0 || n > 3) return;
    motors[n] = thrust;
    sendMotors();
    delay(50);
    motors[n] = 0;
    sendMotors();
}


// ===== FROM: parameters.ino =====
// 参数存储 —— 基于 STM32duino EEPROM (内部 Flash emulation)
// 每个参数以 [magic(2B) | 每个参数 4B value] 的格式线性存储
// EEPROM size: STM32F407 默认 2KB (可在 board variant 里改)

#include <EEPROM.h>
#include "config.h"
#include "util.h"
#include "filter.h"
#include "pid.h"
#include "vector.h"

extern int channelZero[16], channelMax[16];
extern int rollChannel, pitchChannel, throttleChannel, yawChannel, modeChannel;
extern int rcRxPin, voltagePin;
extern float voltageScale;
extern float rcLossTimeout, descendTime, disarmTilt;
extern LowPassFilter<float>  voltageFilter;
extern LowPassFilter<Vector> gyroBiasFilter;
extern LowPassFilter<Vector> ratesFilter;

extern PID rollRatePID, pitchRatePID, yawRatePID;
extern PID rollPID, pitchPID, yawPID;
extern Vector maxRate;
extern float tiltMax;
extern int   flightModes[3];

extern int pwmFrequency, pwmResolution, pwmStop, pwmMin, pwmMax;
extern int motorPins[4];
extern int imuModel, imuBus;
extern int imuSckPin, imuMisoPin, imuMosiPin, imuCsPin, imuSdaPin, imuSclPin, imuIntPin;
extern Vector imuRotation;
extern Vector accBias, accScale;
extern Vector gyroBias;
extern float accWeight, levelWeight;

extern const int MOT_FL, MOT_FR, MOT_RL, MOT_RR;
extern void setupMotors();
extern void setupIMU();
extern bool motorsActive();

// ===== 轻量 EEPROM 存储层 =====
static const uint16_t EEPROM_MAGIC = 0xBABE;
static const uint16_t EEPROM_SIZE  = 2048;    // STM32F407 默认 EEPROM emulation size

static const uint16_t PARAM_NAME_LEN = 16;     // 名字最多 15 字符 + \0
static uint16_t paramOffset = 0;               // 下一个参数在 EEPROM 里的偏移

// 每个参数存储格式: [16B name][4B value as float] — 共 20 字节
static const uint16_t PARAM_SLOT_SIZE = PARAM_NAME_LEN + 4;

// ===== 参数结构 =====
struct Parameter {
    const char* name;
    bool isInteger;
    union { float* f; int* i; };
    float initial;
    float cache;
    void (*callback)();

    Parameter() : name(nullptr), isInteger(false), f(nullptr),
                  initial(0), cache(0), callback(nullptr) {}
    Parameter(const char* n, float* var, void (*cb)() = nullptr)
        : name(n), isInteger(false), f(var), initial(0), cache(0), callback(cb) {}
    Parameter(const char* n, int* var, void (*cb)() = nullptr)
        : name(n), isInteger(true), i(var), initial(0), cache(0), callback(cb) {}
};

// ===== 参数注册表 =====
Parameter parameters[] = {
    {"CTL_RATE_R_P",  &rollRatePID.p},
    {"CTL_RATE_R_I",  &rollRatePID.i},
    {"CTL_RATE_R_D",  &rollRatePID.d},
    {"CTL_RATE_R_WU", &rollRatePID.windup},
    {"CTL_RATE_R_D_A",&rollRatePID.lpf.alpha},
    {"CTL_RATE_P_P",  &pitchRatePID.p},
    {"CTL_RATE_P_I",  &pitchRatePID.i},
    {"CTL_RATE_P_D",  &pitchRatePID.d},
    {"CTL_RATE_P_WU", &pitchRatePID.windup},
    {"CTL_RATE_P_D_A",&pitchRatePID.lpf.alpha},
    {"CTL_RATE_Y_P",  &yawRatePID.p},
    {"CTL_RATE_Y_I",  &yawRatePID.i},
    {"CTL_RATE_Y_D",  &yawRatePID.d},
    {"CTL_RATE_Y_WU", &yawRatePID.windup},
    {"CTL_RATE_Y_D_A",&yawRatePID.lpf.alpha},
    {"CTL_RATE_R_MAX",&maxRate.x},
    {"CTL_RATE_P_MAX",&maxRate.y},
    {"CTL_RATE_Y_MAX",&maxRate.z},
    {"CTL_ATT_R_P",   &rollPID.p},
    {"CTL_ATT_P_P",   &pitchPID.p},
    {"CTL_ATT_Y_P",   &yawPID.p},
    {"CTL_ATT_MAX",   &tiltMax},
    {"CTL_FLT_M_0",   &flightModes[0]},
    {"CTL_FLT_M_1",   &flightModes[1]},
    {"CTL_FLT_M_2",   &flightModes[2]},
    {"IMU_MODEL",     &imuModel},
    {"IMU_BUS",       &imuBus},
    {"IMU_PIN_SCK",   &imuSckPin},
    {"IMU_PIN_MISO",  &imuMisoPin},
    {"IMU_PIN_MOSI",  &imuMosiPin},
    {"IMU_PIN_CS",    &imuCsPin},
    {"IMU_PIN_SDA",   &imuSdaPin},
    {"IMU_PIN_SCL",   &imuSclPin},
    {"IMU_PIN_INT",   &imuIntPin},
    {"IMU_ROT_R",     &imuRotation.x},
    {"IMU_ROT_P",     &imuRotation.y},
    {"IMU_ROT_Y",     &imuRotation.z},
    {"IMU_ACC_B_X",   &accBias.x},
    {"IMU_ACC_B_Y",   &accBias.y},
    {"IMU_ACC_B_Z",   &accBias.z},
    {"IMU_ACC_S_X",   &accScale.x},
    {"IMU_ACC_S_Y",   &accScale.y},
    {"IMU_ACC_S_Z",   &accScale.z},
    {"IMU_GYRO_B_A",  &gyroBiasFilter.alpha},
    {"EST_ACC_W",     &accWeight},
    {"EST_LVL_W",     &levelWeight},
    {"EST_RATES_A",   &ratesFilter.alpha},
    {"MOT_PIN_FL",    &motorPins[MOT_FL], setupMotors},
    {"MOT_PIN_FR",    &motorPins[MOT_FR], setupMotors},
    {"MOT_PIN_RL",    &motorPins[MOT_RL], setupMotors},
    {"MOT_PIN_RR",    &motorPins[MOT_RR], setupMotors},
    {"MOT_PWM_FREQ",  &pwmFrequency, setupMotors},
    {"MOT_PWM_RES",   &pwmResolution, setupMotors},
    {"MOT_PWM_STOP",  &pwmStop},
    {"MOT_PWM_MIN",   &pwmMin},
    {"MOT_PWM_MAX",   &pwmMax},
    {"RC_RX_PIN",     &rcRxPin},
    {"RC_ZERO_0",     &channelZero[0]},
    {"RC_ZERO_1",     &channelZero[1]},
    {"RC_ZERO_2",     &channelZero[2]},
    {"RC_ZERO_3",     &channelZero[3]},
    {"RC_ZERO_4",     &channelZero[4]},
    {"RC_ZERO_5",     &channelZero[5]},
    {"RC_ZERO_6",     &channelZero[6]},
    {"RC_ZERO_7",     &channelZero[7]},
    {"RC_MAX_0",      &channelMax[0]},
    {"RC_MAX_1",      &channelMax[1]},
    {"RC_MAX_2",      &channelMax[2]},
    {"RC_MAX_3",      &channelMax[3]},
    {"RC_MAX_4",      &channelMax[4]},
    {"RC_MAX_5",      &channelMax[5]},
    {"RC_MAX_6",      &channelMax[6]},
    {"RC_MAX_7",      &channelMax[7]},
    {"RC_ROLL",       &rollChannel},
    {"RC_PITCH",      &pitchChannel},
    {"RC_THROTTLE",   &throttleChannel},
    {"RC_YAW",        &yawChannel},
    {"RC_MODE",       &modeChannel},
    {"PWR_PIN",       &voltagePin},
    {"PWR_SCALE",     &voltageScale},
    {"PWR_LPF_A",     &voltageFilter.alpha},
    {"SF_LOSS_T",     &rcLossTimeout},
    {"SF_DESCEND_T",  &descendTime},
    {"SF_DISARM_T",   &disarmTilt},
};

// ===== 内部辅助 =====
static uint16_t getParamCount() {
    return sizeof(parameters) / sizeof(parameters[0]);
}

// 参数名 → EEPROM 偏移 (线性扫描)
static int findParamOffset(const char* name) {
    uint16_t magic;
    EEPROM.get(0, magic);
    if (magic != EEPROM_MAGIC) return -1;

    uint16_t count;
    EEPROM.get(2, count);

    for (uint16_t i = 0; i < count; i++) {
        uint16_t off = 4 + i * PARAM_SLOT_SIZE;
        char slotName[PARAM_NAME_LEN] = {0};
        for (uint8_t j = 0; j < PARAM_NAME_LEN; j++) {
            slotName[j] = EEPROM.read(off + j);
        }
        if (strncmp(name, slotName, PARAM_NAME_LEN - 1) == 0) {
            return off;
        }
    }
    return -1;
}

// 单个参数写 EEPROM
static void writeParamToEEPROM(const Parameter& p, uint16_t index) {
    uint16_t off = 4 + index * PARAM_SLOT_SIZE;

    // 写名字 (STM32duino EEPROM 没有公开 data 指针, 只用 write)
    char name[PARAM_NAME_LEN] = {0};
    strncpy(name, p.name, PARAM_NAME_LEN - 1);
    for (uint8_t j = 0; j < PARAM_NAME_LEN; j++)
        EEPROM.write(off + j, (uint8_t)name[j]);

    // 写值
    float v = p.isInteger ? (float)(*p.i) : *p.f;
    uint8_t buf[4];
    memcpy(buf, &v, 4);
    for (uint8_t j = 0; j < 4; j++)
        EEPROM.write(off + PARAM_NAME_LEN + j, buf[j]);
}

// 从 EEPROM 读参数值
static float readParamFromEEPROM(uint16_t off) {
    float v = 0;
    uint8_t buf[4];
    for (int i = 0; i < 4; i++) buf[i] = EEPROM.read(off + PARAM_NAME_LEN + i);
    memcpy(&v, buf, 4);
    return v;
}

// ===== 对外 API =====
void setupParameters() {
    Serial.println(F("Setup parameters"));
    setDefaults();

    // STM32duino EEPROM 库无 begin() / commit(), 直接 put()/get()

    uint16_t magic, count;
    EEPROM.get(0, magic);
    EEPROM.get(2, count);

    uint16_t myCount = getParamCount();

    if (magic != EEPROM_MAGIC || count != myCount) {
        // 版本不匹配 → 重建
        Serial.println(F("EEPROM init (magic/count mismatch, rebuild)"));
        EEPROM.put(0, EEPROM_MAGIC);
        EEPROM.put(2, myCount);
        for (uint16_t i = 0; i < myCount; i++) {
            writeParamToEEPROM(parameters[i], i);
            parameters[i].initial = parameters[i].isInteger ? (float)*parameters[i].i : *parameters[i].f;
            parameters[i].cache = parameters[i].initial;
        }
        return;
    }

    // 正常加载
    for (uint16_t i = 0; i < myCount; i++) {
        Parameter& p = parameters[i];
        p.initial = p.isInteger ? (float)*p.i : *p.f;

        uint16_t off = 4 + i * PARAM_SLOT_SIZE;
        char slotName[PARAM_NAME_LEN] = {0};
        for (uint8_t j = 0; j < PARAM_NAME_LEN; j++)
            slotName[j] = EEPROM.read(off + j);

        if (slotName[0] != '\0' && strcmp(p.name, slotName) == 0) {
            float v = readParamFromEEPROM(off);
            if (p.isInteger) *p.i = (int)v;
            else             *p.f = v;
        }

        p.cache = p.isInteger ? (float)*p.i : *p.f;
    }
}

int parametersCount() { return (int)getParamCount(); }

float getParameter(int index) {
    if (index < 0 || index >= (int)getParamCount()) return NAN;
    return parameters[index].isInteger ? (float)*parameters[index].i : *parameters[index].f;
}

float getParameter(const char* name) {
    for (uint16_t i = 0; i < getParamCount(); i++) {
        if (strcasecmp(parameters[i].name, name) == 0)
            return parameters[i].isInteger ? (float)*parameters[i].i : *parameters[i].f;
    }
    return NAN;
}

bool setParameter(const char* name, float value) {
    for (uint16_t i = 0; i < getParamCount(); i++) {
        if (strcasecmp(parameters[i].name, name) == 0) {
            Parameter& p = parameters[i];
            if (p.isInteger && !isfinite(value)) return false;
            if (p.isInteger) *p.i = (int)value;
            else             *p.f = value;
            if (p.callback) p.callback();
            return true;
        }
    }
    return false;
}

void syncParameters() {
    static Rate rate(1.0f);
    if (!rate) return;
    if (motorsActive()) return;

    bool changed = false;
    for (uint16_t i = 0; i < getParamCount(); i++) {
        Parameter& p = parameters[i];
        float cur = p.isInteger ? (float)*p.i : *p.f;
        if (floatEquals(cur, p.cache)) continue;

        uint16_t off = 4 + i * PARAM_SLOT_SIZE;
        writeParamToEEPROM(p, i);
        p.cache = cur;
        changed = true;
    }
    // STM32duino EEPROM 无 commit(), put() 即写入
}

// ===== Console 命令 =====
bool doParametersCommand(const String& cmd, const String& arg0, const String& arg1) {
    if (cmd == "p") {
        if (arg1.length() > 0) {
            if (setParameter(arg0.c_str(), arg1.toFloat())) {
                Serial.print(arg0); Serial.print('='); Serial.println(getParameter(arg0.c_str()), 6);
            } else {
                Serial.print(F("Not found: ")); Serial.println(arg0);
            }
        } else {
            for (uint16_t i = 0; i < getParamCount(); i++) {
                String name = parameters[i].name;
                if (arg0.length() == 0 || name.startsWith(arg0)) {
                    float v = getParameter((int)i);
                    float init = parameters[i].initial;
                    Serial.print(name);
                    Serial.print('\t');
                    Serial.print(v, 4);
                    if (!floatEquals(v, init)) {
                        Serial.print("  [def ");
                        Serial.print(init, 4);
                        Serial.print("]");
                    }
                    Serial.println();
                }
            }
        }
        return true;
    }

    if (cmd == "preset") {
        // 把 magic 清掉, 下次启动会重建
        EEPROM.put(0, (uint16_t)0);
        // STM32duino EEPROM.put() 即写入, 无 commit()
        Serial.println(F("EEPROM cleared, reboot to apply defaults"));
        return true;
    }

    return false;
}


// ===== FROM: power.ino =====
// 电源管理 —— 电池电压监测

#include "filter.h"
#include "util.h"
#include <Arduino.h>

float voltage = NAN;
LowPassFilter<float> voltageFilter(1.0f);   // 一阶低通
int   voltagePin   = -1;
float voltageScale = 2.0f;                  // 分压电阻比

void setupPower() {
    if (voltagePin < 0) return;
    pinMode(voltagePin, INPUT);
    analogReadResolution(12);   // STM32 12 位 ADC
}

void readVoltage() {
    if (voltagePin < 0) return;

    static Rate rate(10);   // 10 Hz
    if (!rate) return;

    int raw = analogRead(voltagePin);
    float v = (raw / 4095.0f) * 3.3f * voltageScale;   // 参考电压 3.3V
    voltage = voltageFilter.update(v);
}


// ===== FROM: rc.ino =====
// SBUS 遥控器接收 (用 STM32duino Uart 自己构造 USART1, PA10=RX PA9=TX)
#include <SBUS.h>
#include "util.h"
#include <Arduino.h>

Uart rcSerial(PA10, PA9);   // USART1
SBUS rc(rcSerial);

int rcRxPin = -1;   // -1 = 禁用 (或使用默认 Serial1 RX)

uint16_t channels[16];     // 原始 SBUS 通道
int channelZero[16];
int channelMax[16];

float controlRoll = 0, controlPitch = 0, controlYaw = 0, controlThrottle = 0;
float controlMode = NAN;
float controlTime = NAN;

int rollChannel = -1, pitchChannel = -1, throttleChannel = -1,
    yawChannel = -1, modeChannel = -1;

extern float t;

void setupRC() {
    Serial.println(F("Setup RC (SBUS @100000)"));
    rcSerial.begin(100000);
    rc.begin();

    for (int i = 0; i < 16; i++) {
        channels[i]   = 1024;
        channelZero[i] = 1024;
        channelMax[i]  = 1500;
    }
}

bool readRC() {
    if (!rc.read()) return false;

    rc.getChannels(channels);
    normalizeRC();
    controlTime = t;
    return true;
}

void normalizeRC() {
    float controls[16];
    for (int i = 0; i < 16; i++) {
        float in_min = channelZero[i];
        float in_max = channelMax[i];
        if (in_max == in_min) in_max = in_min + 1;
        controls[i] = mapf(channels[i], in_min, in_max, 0.0f, 1.0f);
        if (controls[i] < 0)   controls[i] = 0;
        if (controls[i] > 1)   controls[i] = 1;
    }

    controlRoll    = rollChannel    < 0 ? 0 : controls[rollChannel];
    controlPitch   = pitchChannel   < 0 ? 0 : controls[pitchChannel];
    controlYaw     = yawChannel     < 0 ? 0 : controls[yawChannel];
    controlThrottle= throttleChannel< 0 ? 0 : controls[throttleChannel];
    controlMode    = modeChannel    < 0 ? NAN : controls[modeChannel];
}


// ===== FROM: safety.ino =====
// Fail-safe 保护

#include "vector.h"
#include "quaternion.h"
#include "util.h"

extern float t, dt;
extern float controlRoll, controlPitch, controlYaw, controlThrottle, controlTime;
extern Quaternion attitude;

float rcLossTimeout = 1.0f;    // RC 失联超时, s
float descendTime   = 10.0f;   // 失控下降时间, s
float disarmTilt    = radians(120);  // 翻机上锁阈值

extern int   mode;
extern bool  armed;
extern float thrustTarget;
extern Quaternion attitudeTarget;

void failsafe() {
    rcLossFailsafe();
    autoFailsafe();
    tiltFailsafe();
}

// RC 失联保护
void rcLossFailsafe() {
    if (!armed) return;
    if (t - controlTime > rcLossTimeout) {
        descend();
    }
}

// 自动下降
void descend() {
    mode = AUTO;
    attitudeTarget = Quaternion();
    thrustTarget -= dt / descendTime;
    if (thrustTarget < 0) {
        thrustTarget = 0;
        armed = false;
    }
}

// AUTO 模式下被飞行员重新接管
void autoFailsafe() {
    static float roll0, pitch0, yaw0, throttle0;
    bool changed = fabsf(roll0    - controlRoll)    > 0.05f ||
                   fabsf(pitch0   - controlPitch)   > 0.05f ||
                   fabsf(yaw0     - controlYaw)     > 0.05f ||
                   fabsf(throttle0- controlThrottle) > 0.05f;

    if (changed && mode == AUTO) {
        mode = STAB;
    }

    roll0     = controlRoll;
    pitch0    = controlPitch;
    yaw0      = controlYaw;
    throttle0 = controlThrottle;
}

// 翻机自动上锁
void tiltFailsafe() {
    if (!armed) return;
    if (mode != STAB) return;

    Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
    float tilt = acosf(up.z);
    if (disarmTilt && tilt > disarmTilt) {
        armed = false;
    }
}


// ===== FROM: time.ino =====
// 时间相关

float t = NAN;    // 当前时间, s
float dt;         // 上一步时间差, s
float loopRate;   // 主循环频率, Hz

// STM32duino 已提供 micros() 函数 (DWT 或 HAL_GetTick 计数)
void step() {
    float now = micros() / 1000000.0f;
    dt = now - t;
    t = now;

    if (!(dt > 0)) {
        dt = 0;   // 第一步或复位时
    }

    computeLoopRate();
}

void computeLoopRate() {
    static float windowStart = 0;
    static uint32_t rate = 0;
    rate++;
    if (t - windowStart >= 1.0f) {   // 1 秒窗口
        loopRate = rate;
        windowStart = t;
        rate = 0;
    }
}


