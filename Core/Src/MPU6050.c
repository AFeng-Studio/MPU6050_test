/**
  ******************************************************************************
  * @file    MPU6050.c
  * @brief   MPU6050 6-axis motion tracking device driver
  ******************************************************************************
  * @attention
  *
  * MPU6050 driver implementation
  * Communication via I2C2 (already initialized in main.c)
  * Configuration:
  *   - Internal clock source
  *   - Gyroscope range: ±1000 °/s
  *   - Accelerometer range: ±4g
  *   - Digital Low Pass Filter: 8kHz output rate, low latency mode
  *   - Sample rate: 200Hz
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "MPU6050.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/
static float Accel_Sensitivity = 0.0f;
static float Gyro_Sensitivity = 0.0f;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize MPU6050 with specified configuration
  * @param  I2Cx: I2C handle pointer (should be &hi2c2 for I2C2)
  * @param  Init: Initialization configuration structure
  * @retval None
  */
void MPU6050_Init(I2C_HandleTypeDef *I2Cx, MPU6050_InitStruct_t Init)
{
    uint8_t check;
    uint8_t config_value;
    
    /* Check device ID */
    check = MPU6050_Read_Reg(I2Cx, MPU6050_WHO_AM_I);
    if (check != MPU6050_WHO_AM_I_VALUE)
    {
        return;  /* Device not found or communication error */
    }
    
    /* 1. Power Management Configuration (PWR_MGMT_1 register - 0x6B) */
    MPU6050_Write_Reg(I2Cx, MPU6050_PWR_MGMT_1, 0x80);  // Device reset
    HAL_Delay(100);
    
    /* Set power mode, cycle mode, temperature sensor and clock source*/
    config_value=0;
    config_value = Init.sleepMode | Init.cycleMode | Init.tempSwitch | Init.clkSource;
    MPU6050_Write_Reg(I2Cx, MPU6050_PWR_MGMT_1, config_value);
    
    /* 2. Sample Rate Configuration (SMPLRT_DIV register - 0x19) */
    MPU6050_Write_Reg(I2Cx, MPU6050_SMPLRT_DIV, Init.SMPLRT_DIV);
    
    /* 3. Digital Filter Configuration (CONFIG register - 0x1A) */
    config_value = 0; 
    config_value = Init.DLPF_CFG & 0x07;
    MPU6050_Write_Reg(I2Cx, MPU6050_CONFIG, config_value);
    
    /* 4. Gyroscope Configuration (GYRO_CONFIG register - 0x1B) */
    config_value = 0; 
    config_value = Init.GyroFS | Init.GyroFilterSwitch;
    MPU6050_Write_Reg(I2Cx, MPU6050_GYRO_CONFIG, config_value);
    
    /* 5. Accelerometer Configuration (ACCEL_CONFIG register - 0x1C) */
    MPU6050_Write_Reg(I2Cx, MPU6050_ACCEL_CONFIG, Init.AccelFS);
    
    /* 6. Power Management 2 Configuration (PWR_MGMT_2 register - 0x6C) */
    /* Set standby axes configuration */
    MPU6050_Write_Reg(I2Cx, MPU6050_PWR_MGMT_2, Init.standbyAxes);
    
    /* 7. Calculate sensitivity values for conversion */
    switch (Init.GyroFS)
    {
        case MPU6050_FS_SEL_250DPS:
            Gyro_Sensitivity = 131.0f;  /* 32768/250 */
            break;
        case MPU6050_FS_SEL_500DPS:
            Gyro_Sensitivity = 65.5f;   /* 32768/500 */
            break;
        case MPU6050_FS_SEL_1000DPS:
            Gyro_Sensitivity = 32.8f;   /* 32768/1000 */
            break;
        case MPU6050_FS_SEL_2000DPS:
            Gyro_Sensitivity = 16.4f;   /* 32768/2000 */
            break;
        default:
            Gyro_Sensitivity = 131.0f;
            break;
    }
    
    /* Accelerometer sensitivity calculation */
    switch (Init.AccelFS)
    {
        case MPU6050_AFS_SEL_2G:
            Accel_Sensitivity = 16384.0f;  /* 32768/2 */
            break;
        case MPU6050_AFS_SEL_4G:
            Accel_Sensitivity = 8192.0f;   /* 32768/4 */
            break;
        case MPU6050_AFS_SEL_8G:
            Accel_Sensitivity = 4096.0f;   /* 32768/8 */
            break;
        case MPU6050_AFS_SEL_16G:
            Accel_Sensitivity = 2048.0f;   /* 32768/16 */
            break;
        default:
            Accel_Sensitivity = 8192.0f;
            break;
    }
}

/**
  * @brief  Write a byte to MPU6050 register
  * @param  I2Cx: I2C handle pointer
  * @param  reg: Register address
  * @param  data: Data to write
  * @retval HAL status
  */
void MPU6050_Write_Reg(I2C_HandleTypeDef *I2Cx, uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg, data};
    
    if (HAL_I2C_Master_Transmit(I2Cx, MPU6050_ADDR, buffer, 2, 100) != HAL_OK)
    {
        return;
    }
}

/**
  * @brief  Read a byte from MPU6050 register
  * @param  I2Cx: I2C handle pointer
  * @param  reg: Register address
  * @retval Register value or HAL_ERROR on error
  */
uint8_t MPU6050_Read_Reg(I2C_HandleTypeDef *I2Cx, uint8_t reg)
{
    uint8_t data = 0;
    
    /* Send register address */
    if (HAL_I2C_Master_Transmit(I2Cx, MPU6050_ADDR, &reg, 1, 100) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    /* Read register value */
    if (HAL_I2C_Master_Receive(I2Cx, MPU6050_ADDR, &data, 1, 100) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    return data;
}


/**
  * @brief  Read accelerometer data from MPU6050
  * @param  I2Cx: I2C handle pointer
  * @param  DataStruct: Pointer to MPU6050_t structure to store data
  * @retval HAL status
  */
uint8_t MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_DateStruct_t *DataStruct)
{
    uint8_t buffer[6];
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    
    /* Send starting register address */
    if (HAL_I2C_Master_Transmit(I2Cx, MPU6050_ADDR, &reg, 1, 100) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    /* Read 6 bytes (X, Y, Z accelerometer data) */
    if (HAL_I2C_Master_Receive(I2Cx, MPU6050_ADDR, buffer, 6, 100) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    /* Combine high and low bytes */
    DataStruct->Accel_X_RAW = (int16_t)((buffer[0] << 8) | buffer[1]);
    DataStruct->Accel_Y_RAW = (int16_t)((buffer[2] << 8) | buffer[3]);
    DataStruct->Accel_Z_RAW = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    /* Convert raw data to g values */
    DataStruct->Ax = DataStruct->Accel_X_RAW / Accel_Sensitivity;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / Accel_Sensitivity;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Sensitivity;
    
    return HAL_OK;
}

/**
  * @brief  Read gyroscope data from MPU6050
  * @param  I2Cx: I2C handle pointer
  * @param  DataStruct: Pointer to MPU6050_t structure to store data
  * @retval HAL status
  */
uint8_t MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_DateStruct_t *DataStruct)
{
    uint8_t buffer[6];
    uint8_t reg = MPU6050_GYRO_XOUT_H;
    
    /* Send starting register address */
    if (HAL_I2C_Master_Transmit(I2Cx, MPU6050_ADDR, &reg, 1, 100) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    /* Read 6 bytes (X, Y, Z gyroscope data) */
    if (HAL_I2C_Master_Receive(I2Cx, MPU6050_ADDR, buffer, 6, 100) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    /* Combine high and low bytes */
    DataStruct->Gyro_X_RAW = (int16_t)((buffer[0] << 8) | buffer[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)((buffer[2] << 8) | buffer[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    /* Convert raw data to °/s values */
    DataStruct->Gx = DataStruct->Gyro_X_RAW / Gyro_Sensitivity;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / Gyro_Sensitivity;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / Gyro_Sensitivity;
    
    return HAL_OK;
}

/**
  * @brief  Get default initialization parameters for MPU6050
  * @param  Init: Pointer to MPU6050_InitStruct_t structure to fill with default values
  * @note   Default configuration:
  *         - Output data rate: 1kHz
  *         - Gyroscope range: ±250 °/s
  *         - Accelerometer range: ±4g
  *         - Sleep mode: disabled
  *         - Cycle mode: disabled
  *         - Temperature sensor: enabled
  *         - Digital low-pass filter: disabled 
  *         - Other parameters: common values for typical applications
  */
void MPU6050_GetDefaultInit(MPU6050_InitStruct_t *Init)
{
    /* Sample Rate Configuration */
    /* For 1kHz output data rate with DLPF disabled (8kHz base rate):
       Output Rate = 8kHz / (1 + SMPLRT_DIV) = 1kHz
       => SMPLRT_DIV = 7 */
    Init->SMPLRT_DIV = 7;
    
    /* Gyroscope Configuration */
    Init->GyroFilterSwitch = MPU6050_GYRO_FILTER_DISABLE;  /* Disable DLPF */
    Init->DLPF_CFG = MPU6050_DLPF_CFG_0;                   /* DLPF configuration (shared between gyro and accel) */
    Init->GyroFS = MPU6050_FS_SEL_250DPS;                  /* ±250 °/s */
    
    /* Accelerometer Configuration */
    Init->AccelFS = MPU6050_AFS_SEL_4G;                    /* ±4g  */
    
    /* Power Management Configuration */
    Init->sleepMode = MPU6050_SLEEP_DISABLE;               /* Sleep mode disabled  */
    Init->cycleMode = MPU6050_CYCLE_DISABLE;               /* Cycle mode disabled  */
    Init->tempSwitch = MPU6050_TEMP_SENSOR_ENABLE;         /* Temperature sensor enabled  */
    Init->clkSource = MPU6050_CLKSEL_INTERNAL_8MHZ;        /* Use internal 8MHz oscillator  */
    Init->standbyAxes = MPU6050_ALL_AXES_ACTIVE;           /* All axes active (no standby) */
}

/**
  * @brief  Read temperature data from MPU6050
  * @param  I2Cx: I2C handle pointer
  * @param  DataStruct: Pointer to MPU6050_t structure to store data
  * @retval HAL status
  */
uint8_t MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_DateStruct_t *DataStruct)
{
    uint8_t buffer[2];
    uint8_t reg = MPU6050_TEMP_OUT_H;
    
    /* Send starting register address */
    if (HAL_I2C_Master_Transmit(I2Cx, MPU6050_ADDR, &reg, 1, 100) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    /* Read 2 bytes (temperature data) */
    if (HAL_I2C_Master_Receive(I2Cx, MPU6050_ADDR, buffer, 2, 100) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    /* Combine high and low bytes */
    int16_t temp_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    
    /* Convert raw temperature to Celsius
       According to MPU6050 datasheet:
       Temperature in °C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53 */
    DataStruct->Temperature = (float)temp_raw / 340.0f + 36.53f;
    
    return HAL_OK;
}

/**
  * @brief  Read all sensor data (accelerometer, gyroscope, temperature) from MPU6050
  * @param  I2Cx: I2C handle pointer
  * @param  DataStruct: Pointer to MPU6050_t structure to store data
  * @retval HAL status
  */
uint8_t MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_DateStruct_t *DataStruct)
{
    uint8_t buffer[14];  // 6 accel + 2 temp + 6 gyro = 14 bytes
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    
    /* Send starting register address */
    if (HAL_I2C_Master_Transmit(I2Cx, MPU6050_ADDR, &reg, 1, 100) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    /* Read 14 bytes (all sensor data) */
    if (HAL_I2C_Master_Receive(I2Cx, MPU6050_ADDR, buffer, 14, 100) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    /* Parse accelerometer data (bytes 0-5) */
    DataStruct->Accel_X_RAW = (int16_t)((buffer[0] << 8) | buffer[1]);
    DataStruct->Accel_Y_RAW = (int16_t)((buffer[2] << 8) | buffer[3]);
    DataStruct->Accel_Z_RAW = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    /* Parse temperature data (bytes 6-7) */
    int16_t temp_raw = (int16_t)((buffer[6] << 8) | buffer[7]);
    DataStruct->Temperature = (float)temp_raw / 340.0f + 36.53f;
    
    /* Parse gyroscope data (bytes 8-13) */
    DataStruct->Gyro_X_RAW = (int16_t)((buffer[8] << 8) | buffer[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)((buffer[10] << 8) | buffer[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    /* Convert raw data to physical values */
    DataStruct->Ax = DataStruct->Accel_X_RAW / Accel_Sensitivity;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / Accel_Sensitivity;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Sensitivity;
    
    DataStruct->Gx = DataStruct->Gyro_X_RAW / Gyro_Sensitivity;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / Gyro_Sensitivity;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / Gyro_Sensitivity;
    
    return HAL_OK;
}
