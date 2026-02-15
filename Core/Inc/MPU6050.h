/**
  ******************************************************************************
  * 作者：哔哩哔哩Up主"阿枫的手工铺"
  * 用途：开源免费，用户可以任意查看、使用和修改，并应用到自己的项目之中
  * 许可证：MIT许可证
  * 许可证详情：请查看项目根目录下的LICENSE文件
  * 声明：若二次转载需要注明出处及作者信息，
  *       程序版权归"阿枫的手工铺"所有，任何人或组织不得将其据为己有
  * 
  * 程序名称：			  	  MPU6050 6轴运动跟踪设备驱动程序
  * 程序创建时间：			  2026.02.15
  * 当前程序版本：			  V1.0
  * 当前版本发布时间：		2026.02.15
  * 
  * 如果您发现程序中的漏洞，或者有更好的建议和意见，欢迎发送邮件到：afeng_studio@yeah.net
  ******************************************************************************

  ******************************************************************************
  * @brief MPU6050 6轴运动跟踪设备驱动函数使用说明
  *
  * 1. 硬件接口
  *    - 使用硬件I2C接口（I2C2），已在main.c中初始化
  *    - I2C地址：0xD0（AD0引脚接地）
  *    - 如果AD0引脚接VCC，则I2C地址为0xD2
  *
  * 2. 软件接口
  *    - 驱动函数基于STM32 HAL库的I2C阻塞式传输函数HAL_I2C_Master_Transmit/Receive实现，Timeout=100ms
  *
  * 3. 主要使用函数
  *    - 初始化函数：
  *        void MPU6050_Init(I2C_HandleTypeDef *I2Cx, MPU6050_InitStruct_t Init);
  *        void MPU6050_GetDefaultInit(MPU6050_InitStruct_t *Init);
  *    - 数据读取函数：
  *        uint8_t MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_DateStruct_t *DataStruct);
  *        uint8_t MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_DateStruct_t *DataStruct);
  *        uint8_t MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_DateStruct_t *DataStruct);
  *        uint8_t MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_DateStruct_t *DataStruct);
  *    - 辅助函数：
  *        uint8_t MPU6050_Read_Reg(I2C_HandleTypeDef *I2Cx, uint8_t reg);
  *        void MPU6050_Write_Reg(I2C_HandleTypeDef *I2Cx, uint8_t reg, uint8_t data);
  *
  * 4. MPU6050初始化结构体参数说明
  *    - MPU6050_GetDefaultInit()函数默认初始化值：
  *        - SMPLRT_DIV: 7（输出数据率1kHz）
  *        - GyroFilterSwitch: MPU6050_GYRO_FILTER_DISABLE（禁用陀螺仪数字低通滤波器）
  *        - DLPF_CFG: MPU6050_DLPF_CFG_0（加速度计260Hz，陀螺仪256Hz，延迟0ms）
  *        - GyroFS: MPU6050_FS_SEL_250DPS（±250°/s）
  *        - AccelFS: MPU6050_AFS_SEL_4G（±4g）
  *        - sleepMode: MPU6050_SLEEP_DISABLE（睡眠模式禁用）
  *        - cycleMode: MPU6050_CYCLE_DISABLE（循环模式禁用）
  *        - tempSwitch: MPU6050_TEMP_SENSOR_ENABLE（温度传感器启用）
  *        - clkSource: MPU6050_CLKSEL_INTERNAL_8MHZ（内部8MHz振荡器）
  *        - standbyAxes: MPU6050_ALL_AXES_ACTIVE（所有轴激活）
  *
  * 5. 滤波器配置说明
  *    - DLPF_CFG：共享数字低通滤波器配置
  *        * 陀螺仪（当GyroFilterSwitch = MPU6050_GYRO_FILTER_ENABLE时使用此配置）
  *        * 加速度计（始终使用此配置）
  *        * 温度传感器（使用与加速度计相同的采样率）
  *    - GyroFilterSwitch：陀螺仪数字低通滤波器启用/禁用开关
  *        * 禁用时（MPU6050_GYRO_FILTER_DISABLE），陀螺仪DLPF被旁路（bypassed）
  *           - 陀螺仪信号不经过数字低通滤波器处理
  *           - 使用最高采样率（8kHz）
  *           - 输出原始高频信号，无滤波延迟
  *           - 适用于需要最高带宽和最低延迟的应用
  *        * 启用时（MPU6050_GYRO_FILTER_ENABLE），陀螺仪使用DLPF_CFG配置
  *           - 陀螺仪信号经过数字低通滤波器处理
  *           - 滤波器带宽由DLPF_CFG配置决定
  *           - 可以降低噪声，但会引入滤波延迟
  *           - 适用于需要降低噪声的应用
  *
  * 6. 使用示例（读取所有传感器数据）
  *    @code
  *    // 1. 定义初始化结构体
  *    MPU6050_InitStruct_t mpu6050_init;
  *    
  *    // 2. 获取默认配置
  *    MPU6050_GetDefaultInit(&mpu6050_init);
  *    
  *    // 3. 修改配置（可选）
  *    mpu6050_init.GyroFS = MPU6050_FS_SEL_1000DPS;  // ±1000°/s
  *    mpu6050_init.AccelFS = MPU6050_AFS_SEL_8G;     // ±8g
  *    mpu6050_init.DLPF_CFG = MPU6050_DLPF_CFG_2;    // 加速度计94Hz，陀螺仪98Hz
  *    
  *    // 4. 初始化MPU6050
  *    MPU6050_Init(&hi2c2, mpu6050_init);
  *    
  *    // 5. 定义数据存储结构体
  *    MPU6050_DateStruct_t mpu6050_data;
  *    
  *    // 6. 循环读取所有传感器数据
  *    while (1)
  *    {
  *        MPU6050_Read_All(&hi2c2, &mpu6050_data);
  *        
  *        // 处理数据
  *        // mpu6050_data.Ax, mpu6050_data.Ay, mpu6050_data.Az (加速度，单位：g)
  *        // mpu6050_data.Gx, mpu6050_data.Gy, mpu6050_data.Gz (角速度，单位：°/s)
  *        // mpu6050_data.Temperature (温度，单位：°C)
  *        
  *        HAL_Delay(100); // 100ms延迟
  *    }
  *    @endcode
  *
  * 7. 注意事项
  *    - MPU6050使用I2C通信，确保I2C总线已正确初始化
  *    - 读取数据前必须初始化MPU6050
  *    - MPU6050_Read_All()函数一次性读取所有传感器数据，效率最高
  *    - 原始数据（RAW值）是16位有符号整数
  *    - 转换后的物理值（Ax, Ay, Az, Gx, Gy, Gz）是浮点数
  *    - 温度传感器精度为±1°C
  *
  * 8. 传感器规格
  *    - 陀螺仪量程：±250, ±500, ±1000, ±2000 °/s
  *    - 加速度计量程：±2, ±4, ±8, ±16 g
  *    - 温度传感器范围：-40°C 到 +85°C
  *    - 数字低通滤波器：可编程，带宽5Hz到260Hz
  *    - 输出数据率：4Hz到8kHz（可编程）
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/* Digital Low Pass Filter Configuration */
typedef enum
{
    MPU6050_DLPF_CFG_0 = 0x00,  /*!< Accelerometer: 260Hz, Gyroscope: 256Hz, Delay: 0ms */
    MPU6050_DLPF_CFG_1 = 0x01,  /*!< Accelerometer: 184Hz, Gyroscope: 188Hz, Delay: 2.0ms */
    MPU6050_DLPF_CFG_2 = 0x02,  /*!< Accelerometer: 94Hz,  Gyroscope: 98Hz,  Delay: 3.0ms */
    MPU6050_DLPF_CFG_3 = 0x03,  /*!< Accelerometer: 44Hz,  Gyroscope: 42Hz,  Delay: 4.9ms */
    MPU6050_DLPF_CFG_4 = 0x04,  /*!< Accelerometer: 21Hz,  Gyroscope: 20Hz,  Delay: 8.5ms */
    MPU6050_DLPF_CFG_5 = 0x05,  /*!< Accelerometer: 10Hz,  Gyroscope: 10Hz,  Delay: 13.8ms */
    MPU6050_DLPF_CFG_6 = 0x06,  /*!< Accelerometer: 5Hz,   Gyroscope: 5Hz,   Delay: 19.0ms */
} MPU6050_DLPF_t;

/* Gyroscope Full Scale Range Configuration (bits 4:3 of GYRO_CONFIG register) */
typedef enum
{
    MPU6050_FS_SEL_250DPS  = 0x00,  /*!< ±250 °/s */
    MPU6050_FS_SEL_500DPS  = 0x08,  /*!< ±500 °/s (bit 3) */
    MPU6050_FS_SEL_1000DPS = 0x10,  /*!< ±1000 °/s (bit 4) */
    MPU6050_FS_SEL_2000DPS = 0x18   /*!< ±2000 °/s (bits 4:3 = 3) */
} MPU6050_GYRO_Full_Scale_Range_t;

/* Gyroscope Filter Configuration  */
typedef enum
{
    MPU6050_GYRO_FILTER_ENABLE  = 0x00,  /*!< Enable digital low-pass filter */
    MPU6050_GYRO_FILTER_DISABLE = 0x01   /*!< Disable digital low-pass filter */
} MPU6050_GYRO_FILTER_Switch_t;

/* Accelerometer Full Scale Range Selection (AFS_SEL) - bits 4:3 of ACCEL_CONFIG register) */
typedef enum
{
    MPU6050_AFS_SEL_2G  = 0x00,  /*!< ±2g */
    MPU6050_AFS_SEL_4G  = 0x08,  /*!< ±4g (bit 3) */
    MPU6050_AFS_SEL_8G  = 0x10,  /*!< ±8g (bit 4) */
    MPU6050_AFS_SEL_16G = 0x18   /*!< ±16g (bits 4:3 = 3) */
} MPU6050_ACCEL_Full_Scale_Range_t;

/* Sleep Mode Control (SLEEP) - bit 6 of PWR_MGMT_1 register */
typedef enum
{
    MPU6050_SLEEP_DISABLE = 0x00,  /*!< Sleep mode disabled (normal operation) */
    MPU6050_SLEEP_ENABLE  = 0x40   /*!< Sleep mode enabled (bit 6) */
} MPU6050_SLEEP_Mode_t;

/* Cycle Mode Control (CYCLE) - bit 5 of PWR_MGMT_1 register */
typedef enum
{
    MPU6050_CYCLE_DISABLE = 0x00,  /*!< Cycle mode disabled (normal operation) */
    MPU6050_CYCLE_ENABLE  = 0x20   /*!< Cycle mode enabled (bit 5) */
} MPU6050_CYCLE_mode_t;

/* Temperature Sensor Control (TEMP_DIS) - bit 4 of PWR_MGMT_1 register */
typedef enum
{
    MPU6050_TEMP_SENSOR_ENABLE   = 0x00,  /*!< Temperature sensor enabled */
    MPU6050_TEMP_SENSOR_DISABLE  = 0x10   /*!< Temperature sensor disabled (bit 4) */
} MPU6050_TEMP_Switch_t;

/* Clock Source Selection (CLKSEL) - bits 2:0 */
typedef enum
{
    MPU6050_CLKSEL_INTERNAL_8MHZ    = 0x00,  /*!< Internal 8MHz oscillator */
    MPU6050_CLKSEL_PLL_X_GYRO       = 0x01,  /*!< PLL with X axis gyroscope reference */
    MPU6050_CLKSEL_PLL_Y_GYRO       = 0x02,  /*!< PLL with Y axis gyroscope reference */
    MPU6050_CLKSEL_PLL_Z_GYRO       = 0x03,  /*!< PLL with Z axis gyroscope reference */
    MPU6050_CLKSEL_PLL_EXT_32KHZ    = 0x04,  /*!< PLL with external 32.768kHz reference */
    MPU6050_CLKSEL_PLL_EXT_19MHZ    = 0x05,  /*!< PLL with external 19.2MHz reference */
    MPU6050_CLKSEL_RESERVED         = 0x06,  /*!< Reserved */
    MPU6050_CLKSEL_STOP_CLOCK       = 0x07   /*!< Stops the clock and keeps timing generator in reset */
} MPU6050_CLK_SOURSE_t;

/* Power Management 2 Register (PWR_MGMT_2) - bits 5:0 */
typedef enum
{
    MPU6050_ALL_AXES_ACTIVE = 0x00,  /*!< All axes active (all bits cleared) */
    
    MPU6050_STBY_ZA         = 0x20,  /*!< Accelerometer Z axis standby (bit 5) */
    MPU6050_STBY_YA         = 0x10,  /*!< Accelerometer Y axis standby (bit 4) */
    MPU6050_STBY_XA         = 0x08,  /*!< Accelerometer X axis standby (bit 3) */
    MPU6050_STBY_ZG         = 0x04,  /*!< Gyroscope Z axis standby (bit 2) */
    MPU6050_STBY_YG         = 0x02,  /*!< Gyroscope Y axis standby (bit 1) */
    MPU6050_STBY_XG         = 0x01   /*!< Gyroscope X axis standby (bit 0) */
} MPU6050_STANDBY_AXES;

/**
  * @brief  MPU6050 Initialization Configuration Structure
  */
typedef struct
{
  uint8_t SMPLRT_DIV;                         /*!< Sample Rate Divider register value
                                                   Output Data Rate: 
                                                   8kHz / (1 + SMPLRT_DIV) if GyroFilterSwitch = MPU6050_GYRO_FILTER_DISABLE 
                                                   8kHz / (1 + SMPLRT_DIV) if GyroDLPF = MPU6050_DLPF_CFG_0 or MPU6050_DLPF_CFG_1
                                                   1kHz / (1 + SMPLRT_DIV) for other DLPF_CFG values
                                                   This parameter must be set to a value between 0 and 255 */

  MPU6050_GYRO_FILTER_Switch_t GyroFilterSwitch;    /*!< Gyroscope digital low-pass filter enable/disable switch
                                                   This parameter can be a value of @ref MPU6050_GYRO_FILTER_Switch_t */         

  MPU6050_DLPF_t DLPF_CFG;                    /*!< Digital low-pass filter configuration
                                                   This parameter can be a value of @ref MPU6050_DLPF_t */

  MPU6050_GYRO_Full_Scale_Range_t GyroFS;     /*!< Gyroscope full scale range selection
                                                   This parameter can be a value of @ref MPU6050_GYRO_Full_Scale_Range_t */

  MPU6050_ACCEL_Full_Scale_Range_t AccelFS;   /*!< Accelerometer full scale range selection
                                                   This parameter can be a value of @ref MPU6050_ACCEL_Full_Scale_Range_t */

  MPU6050_SLEEP_Mode_t sleepMode;             /*!< Sleep mode control
                                                   This parameter can be a value of @ref MPU6050_SLEEP_Mode_t */

  MPU6050_CYCLE_mode_t cycleMode;             /*!< Cycle mode control
                                                   This parameter can be a value of @ref MPU6050_CYCLE_mode_t */

  MPU6050_TEMP_Switch_t tempSwitch;           /*!< Temperature sensor control
                                                   This parameter can be a value of @ref MPU6050_TEMP_Switch_t */

  MPU6050_CLK_SOURSE_t clkSource;             /*!< Clock source selection
                                                   This parameter can be a value of @ref MPU6050_CLK_SOURSE_t */

  MPU6050_STANDBY_AXES standbyAxes;           /*!< Standby axes configuration
                                                   Use bitwise OR (|) to combine multiple standby axes
                                                   Example: MPU6050_STBY_XA | MPU6050_STBY_YA puts X and Y accelerometer axes in standby
                                                   This parameter is a bit mask of @ref MPU6050_STANDBY_AXES */
} MPU6050_InitStruct_t;

/**
  * @brief  MPU6050 Data Structure
  * @note   Contains raw and converted sensor data from MPU6050
  */
typedef struct
{
  /* Raw sensor data (16-bit signed integers) */
  int16_t Accel_X_RAW;  /*!< Raw accelerometer X-axis data */
  int16_t Accel_Y_RAW;  /*!< Raw accelerometer Y-axis data */
  int16_t Accel_Z_RAW;  /*!< Raw accelerometer Z-axis data */
  int16_t Gyro_X_RAW;   /*!< Raw gyroscope X-axis data */
  int16_t Gyro_Y_RAW;   /*!< Raw gyroscope Y-axis data */
  int16_t Gyro_Z_RAW;   /*!< Raw gyroscope Z-axis data */

  /* Converted sensor data (floating point physical values) */
  float Ax;             /*!< Accelerometer X-axis value in g */
  float Ay;             /*!< Accelerometer Y-axis value in g */
  float Az;             /*!< Accelerometer Z-axis value in g */
  float Gx;             /*!< Gyroscope X-axis value in °/s */
  float Gy;             /*!< Gyroscope Y-axis value in °/s */
  float Gz;             /*!< Gyroscope Z-axis value in °/s */
  float Temperature;    /*!< Temperature value in °C */
} MPU6050_DateStruct_t;

/* Exported constants --------------------------------------------------------*/

/* MPU6050 I2C Address */
#define MPU6050_ADDR 0xD0  /*!< MPU6050 I2C address (AD0 pin grounded) */

/* MPU6050 Register Addresses */
#define MPU6050_SMPLRT_DIV     0x19  /*!< Sample Rate Divider */
#define MPU6050_CONFIG         0x1A  /*!< Configuration */
#define MPU6050_GYRO_CONFIG    0x1B  /*!< Gyroscope Configuration */
#define MPU6050_ACCEL_CONFIG   0x1C  /*!< Accelerometer Configuration */
#define MPU6050_ACCEL_XOUT_H   0x3B  /*!< Accelerometer X-axis High Byte */
#define MPU6050_ACCEL_XOUT_L   0x3C  /*!< Accelerometer X-axis Low Byte */
#define MPU6050_ACCEL_YOUT_H   0x3D  /*!< Accelerometer Y-axis High Byte */
#define MPU6050_ACCEL_YOUT_L   0x3E  /*!< Accelerometer Y-axis Low Byte */
#define MPU6050_ACCEL_ZOUT_H   0x3F  /*!< Accelerometer Z-axis High Byte */
#define MPU6050_ACCEL_ZOUT_L   0x40  /*!< Accelerometer Z-axis Low Byte */
#define MPU6050_TEMP_OUT_H     0x41  /*!< Temperature High Byte */
#define MPU6050_TEMP_OUT_L     0x42  /*!< Temperature Low Byte */
#define MPU6050_GYRO_XOUT_H    0x43  /*!< Gyroscope X-axis High Byte */
#define MPU6050_GYRO_XOUT_L    0x44  /*!< Gyroscope X-axis Low Byte */
#define MPU6050_GYRO_YOUT_H    0x45  /*!< Gyroscope Y-axis High Byte */
#define MPU6050_GYRO_YOUT_L    0x46  /*!< Gyroscope Y-axis Low Byte */
#define MPU6050_GYRO_ZOUT_H    0x47  /*!< Gyroscope Z-axis High Byte */
#define MPU6050_GYRO_ZOUT_L    0x48  /*!< Gyroscope Z-axis Low Byte */
#define MPU6050_PWR_MGMT_1     0x6B  /*!< Power Management 1 */
#define MPU6050_PWR_MGMT_2     0x6C  /*!< Power Management 2 */
#define MPU6050_WHO_AM_I       0x75  /*!< Who Am I (Device ID) */

/* MPU6050 Device ID */
#define MPU6050_WHO_AM_I_VALUE 0x68  /*!< MPU6050 device ID */

/* Exported functions --------------------------------------------------------*/

/* Initialization Functions */
void MPU6050_Init(I2C_HandleTypeDef *I2Cx, MPU6050_InitStruct_t Init);
void MPU6050_GetDefaultInit(MPU6050_InitStruct_t *Init);

/* Register Access Functions */
uint8_t MPU6050_Read_Reg(I2C_HandleTypeDef *I2Cx, uint8_t reg);
void MPU6050_Write_Reg(I2C_HandleTypeDef *I2Cx, uint8_t reg, uint8_t data);

/* Data Reading Functions */
uint8_t MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_DateStruct_t *DataStruct);
uint8_t MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_DateStruct_t *DataStruct);
uint8_t MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_DateStruct_t *DataStruct);
uint8_t MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_DateStruct_t *DataStruct);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */
