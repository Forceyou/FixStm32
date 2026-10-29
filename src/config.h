// STM32F407 引脚映射与默认参数
// 基于 STM32F407VET6 开发板 (黑块)

#pragma once

// ===== 电机 PWM 引脚 (TIM4) =====
// TIM4_CH1 ~ TIM4_CH4: PD12 ~ PD15
#define MOT_RL_PIN  PD12   // 后左  (TIM4_CH1)
#define MOT_RR_PIN  PD13   // 后右  (TIM4_CH2)
#define MOT_FR_PIN  PD14   // 前右  (TIM4_CH3)
#define MOT_FL_PIN  PD15   // 前左  (TIM4_CH4)

// ===== IMU SPI 引脚 (SPI1) =====
// GY-91 (MPU9250) / ICM20948
#define IMU_SCK_PIN   PA5    // SPI1_SCK
#define IMU_MISO_PIN  PA6    // SPI1_MISO
#define IMU_MOSI_PIN  PA7    // SPI1_MOSI
#define IMU_CS_PIN    PE4    // 片选
#define IMU_INT_PIN   -1     // 中断脚 (可选, -1 表示使用定时器等待)

// ===== RC SBUS 引脚 (USART1) =====
#define RC_RX_PIN     PA10   // USART1_RX
#define RC_TX_PIN     PA9    // USART1_TX (可选)

// ===== 电池电压 =====
#define VOLTAGE_PIN   PA0    // ADC1_IN0
#define VOLTAGE_SCALE 4.03f  // 分压电阻比 (R上+R下)/R下 = (10k+3.3k)/3.3k

// ===== LED =====
#define LED_PIN       PC13   // 板载蓝色 LED (低电平亮)

// ===== USB 虚拟串口 =====
// STM32F407 的 USB OTG FS 自动映射为 Serial

// ===== 默认参数 =====
void setDefaults() {
    // 电机引脚
    extern int motorPins[4];
    motorPins[MOT_RL] = MOT_RL_PIN;
    motorPins[MOT_RR] = MOT_RR_PIN;
    motorPins[MOT_FR] = MOT_FR_PIN;
    motorPins[MOT_FL] = MOT_FL_PIN;

    // PWM 参数
    extern int pwmFrequency;
    pwmFrequency = 40000;   // 40 kHz (有刷电机推荐)

    // IMU 默认 SPI + MPU9250
    extern int imuModel, imuBus;
    imuModel = 1;   // 1 = MPU9250, 2 = ICM20948, 3 = MPU6050
    imuBus = 0;     // 0 = SPI, 1 = I2C

    extern int imuSckPin, imuMisoPin, imuMosiPin, imuCsPin, imuIntPin;
    imuSckPin  = IMU_SCK_PIN;
    imuMisoPin = IMU_MISO_PIN;
    imuMosiPin = IMU_MOSI_PIN;
    imuCsPin   = IMU_CS_PIN;
    imuIntPin  = IMU_INT_PIN;

    // RC
    extern int rcRxPin;
    rcRxPin = RC_RX_PIN;

    // 电压
    extern int voltagePin;
    extern float voltageScale;
    voltagePin   = VOLTAGE_PIN;
    voltageScale = VOLTAGE_SCALE;
}
