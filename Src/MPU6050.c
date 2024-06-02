
#include "MPU6050.h"

#include <math.h>

MPU6050_StatusTypeDef MPU6050_Init_Int(MPU6050_HandleTypeDef *hmpu)
{
  uint8_t data;

  HAL_I2C_Mem_Read(hmpu->hi2c, hmpu->adress, WHO_AM_I_REG, 1, &data, 1, 100);

  /* Check if device response is correct, WHO_AM_I_REG always response with 0x68 */
  if (data != 0x68)
  {
    return MPU_ERROR;
  }

  /* Waking the sensor up */
  data = 0x00;
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, PWR_MGMT_1_REG, 1, &data, 1, 100);

  /* Setting device sample rate */
  if (hmpu->sample_rate == 0)
  {
    data = 0x00;
  }
  else if (hmpu->dlpf == DLPF_0)
  {
    data = (8000 / hmpu->sample_rate) - 1;
  }
  else
  {
    data = (1000 / hmpu->sample_rate) - 1;
  }
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, SMPLRT_DIV_REG, 1, &data, 1, 100);

  /* Disable FSYNC and setting DLPF */
  data = (uint8_t)(hmpu->dlpf);
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, CONFIG_REG, 1, &data, 1, 100);

  /* Enabling data ready interrupt */
  data = 0x01;
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, INT_ENABLE_REG, 1, &data, 1, 100);

  /* Clearing interrupt status register when reading FIFO buffer */
  data = 0x10;
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, INT_PIN_CFG_REG, 1, &data, 1, 100);

  /* Set accel config (0x00 = 2g, 0x08 = 4g, 0x10 = 8g, 0x18 = 16g */
  data = (uint8_t)(hmpu->afs << 3);
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, ACCEL_CONFIG_REG, 1, &data, 1, 100);

  /* Set gyro config (0x00 = 250º/s, 0x08 = 500º/s, 0x10 = 1000º/s, 0x18 = 2000º/s */
  data = (uint8_t)(hmpu->gfs << 3);
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, GYRO_CONFIG_REG, 1, &data, 1, 100);

  return MPU_OK;
}

MPU6050_StatusTypeDef MPU6050_Init(MPU6050_HandleTypeDef *hmpu)
{
  uint8_t data;

  HAL_I2C_Mem_Read(hmpu->hi2c, hmpu->adress, WHO_AM_I_REG, 1, &data, 1, 100);

  /* Check if device response is correct, WHO_AM_I_REG always response with 0x68 */
  if (data != 0x68)
  {
    return MPU_ERROR;
  }

  /* Waking the sensor up */
  data = 0x00;
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, PWR_MGMT_1_REG, 1, &data, 1, 100);

  /* Setting device sample rate */
  if (hmpu->sample_rate == 0)
  {
    data = 0x00;
  }
  else if (hmpu->dlpf == DLPF_0)
  {
    data = (8000 / hmpu->sample_rate) - 1;
  }
  else
  {
    data = (1000 / hmpu->sample_rate) - 1;
  }
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, SMPLRT_DIV_REG, 1, &data, 1, 100);

  /* Disable FSYNC and setting DLPF */
  data = (uint8_t)(hmpu->dlpf);
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, CONFIG_REG, 1, &data, 1, 100);

  /* Set accel config (0x00 = 2g, 0x08 = 4g, 0x10 = 8g, 0x18 = 16g */
  data = (uint8_t)(hmpu->afs << 3);
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, ACCEL_CONFIG_REG, 1, &data, 1, 100);

  /* Set gyro config (0x00 = 250º/s, 0x08 = 500º/s, 0x10 = 1000º/s, 0x18 = 2000º/s */
  data = (uint8_t)(hmpu->gfs << 3);
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, GYRO_CONFIG_REG, 1, &data, 1, 100);

  return MPU_OK;
}

MPU6050_StatusTypeDef MPU6050_Read_Temp(MPU6050_HandleTypeDef *hmpu, MPU6050_t *mpu)
{
  uint8_t data[2];
  int16_t temp;

  HAL_I2C_Mem_Read(hmpu->hi2c, hmpu->adress, TEMP_OUT_H_REG, 1, data, 2, 100);
  temp = (int16_t)(data[0] << 8 | data[1]);

  mpu->temp = (float)temp / (float)340 + (float)36.53;

  return MPU_OK;
}

MPU6050_StatusTypeDef MPU6050_Read_Accel(MPU6050_HandleTypeDef *hmpu, MPU6050_t *mpu)
{
  uint8_t data[6];
  int16_t x_accel, y_accel, z_accel;

  HAL_I2C_Mem_Read(hmpu->hi2c, hmpu->adress, ACCEL_XOUT_H_REG, 1, data, 6, 100);

  x_accel = (int16_t)(data[0] << 8 | data[1]);
  y_accel = (int16_t)(data[2] << 8 | data[3]);
  z_accel = (int16_t)(data[4] << 8 | data[5]);

  /* Calculate accelerometer values base on the accel full scale range */
  float conv_factor = pow(2, (14 - hmpu->afs));

  mpu->accel[0] = (float)x_accel / conv_factor;
  mpu->accel[1] = (float)y_accel / conv_factor;
  mpu->accel[2] = (float)z_accel / conv_factor;

  mpu->accel[0] -= hmpu->accel_bias[0];
  mpu->accel[1] -= hmpu->accel_bias[1];
  mpu->accel[2] -= hmpu->accel_bias[2];

  return MPU_OK;
}

MPU6050_StatusTypeDef MPU6050_Read_Gyro(MPU6050_HandleTypeDef *hmpu, MPU6050_t *mpu)
{
  uint8_t data[6];
  int16_t x_gyro, y_gyro, z_gyro;

  HAL_I2C_Mem_Read(hmpu->hi2c, hmpu->adress, GYRO_XOUT_H_REG, 1, data, 6, 100);

  x_gyro = (int16_t)(data[0] << 8 | data[1]);
  y_gyro = (int16_t)(data[2] << 8 | data[3]);
  z_gyro = (int16_t)(data[4] << 8 | data[5]);

  /* Calculate gyro values base on the gyro full scale range */
  float conv_factor = 131.0f / pow(2, hmpu->gfs);

  mpu->gyro[0] = (float)x_gyro / conv_factor;
  mpu->gyro[1] = (float)y_gyro / conv_factor;
  mpu->gyro[2] = (float)z_gyro / conv_factor;

  mpu->gyro[0] -= hmpu->gyro_bias[0];
  mpu->gyro[1] -= hmpu->gyro_bias[1];
  mpu->gyro[2] -= hmpu->gyro_bias[2];

  return MPU_OK;
}

MPU6050_StatusTypeDef MPU6050_Read_All(MPU6050_HandleTypeDef *hmpu, MPU6050_t *mpu)
{

  uint8_t data[14];
  int16_t x_accel, y_accel, z_accel, temp, x_gyro, y_gyro, z_gyro;

  HAL_I2C_Mem_Read(hmpu->hi2c, hmpu->adress, ACCEL_XOUT_H_REG, 1, data, 14, 100);

  x_accel = (int16_t)(data[0] << 8 | data[1]);
  y_accel = (int16_t)(data[2] << 8 | data[3]);
  z_accel = (int16_t)(data[4] << 8 | data[5]);
  temp    = (int16_t)(data[6] << 8 | data[7]);
  x_gyro  = (int16_t)(data[8] << 8 | data[9]);
  y_gyro  = (int16_t)(data[10] << 8 | data[11]);
  z_gyro  = (int16_t)(data[12] << 8 | data[13]);

  /* Calculate accelerometer and gyro values base on the full scale range */
  float accel_conv_factor = pow(2, (14 - hmpu->afs));
  float gyro_conv_factor = 131.0f / pow(2, hmpu->gfs);

  mpu->accel[0] = (float)x_accel / accel_conv_factor;
  mpu->accel[1] = (float)y_accel / accel_conv_factor;
  mpu->accel[2] = (float)z_accel / accel_conv_factor;
  mpu->temp     = (float)temp / 340.0f + 36.53f;
  mpu->gyro[0]  = (float)x_gyro / gyro_conv_factor;
  mpu->gyro[1]  = (float)y_gyro / gyro_conv_factor;
  mpu->gyro[2]  = (float)z_gyro / gyro_conv_factor;

  /* Apply bias correction */
  mpu->accel[0] -= hmpu->accel_bias[0];
  mpu->accel[1] -= hmpu->accel_bias[1];
  mpu->accel[2] -= hmpu->accel_bias[2];
  mpu->gyro[0]  -= hmpu->gyro_bias[0];
  mpu->gyro[1]  -= hmpu->gyro_bias[1];
  mpu->gyro[2]  -= hmpu->gyro_bias[2];

  return MPU_OK;
}

MPU6050_StatusTypeDef MPU6050_Read_DMA(MPU6050_HandleTypeDef *hmpu, MPU6050_t *mpu)
{
  HAL_I2C_Mem_Read_DMA(hmpu->hi2c, hmpu->adress, ACCEL_XOUT_H_REG, 1, mpu->buffer, 14);

  return MPU_OK;
}

MPU6050_StatusTypeDef MPU6050_Process_DMA(MPU6050_HandleTypeDef *hmpu, MPU6050_t *mpu)
{
  int16_t x_accel, y_accel, z_accel, temp, x_gyro, y_gyro, z_gyro;

  x_accel = (int16_t)(mpu->buffer[0] << 8 | mpu->buffer[1]);
  y_accel = (int16_t)(mpu->buffer[2] << 8 | mpu->buffer[3]);
  z_accel = (int16_t)(mpu->buffer[4] << 8 | mpu->buffer[5]);
  temp    = (int16_t)(mpu->buffer[6] << 8 | mpu->buffer[7]);
  x_gyro  = (int16_t)(mpu->buffer[8] << 8 | mpu->buffer[9]);
  y_gyro  = (int16_t)(mpu->buffer[10] << 8 | mpu->buffer[11]);
  z_gyro  = (int16_t)(mpu->buffer[12] << 8 | mpu->buffer[13]);

  /* Calculate accelerometer and gyro values base on the full scale range */
  float accel_conv_factor = pow(2, (14 - hmpu->afs));
  float gyro_conv_factor = 131.0f / pow(2, hmpu->gfs);

  mpu->accel[0] = (float)x_accel / accel_conv_factor;
  mpu->accel[1] = (float)y_accel / accel_conv_factor;
  mpu->accel[2] = (float)z_accel / accel_conv_factor;
  mpu->temp     = (float)temp / 340.0f + 36.53f;
  mpu->gyro[0]  = (float)x_gyro / gyro_conv_factor;
  mpu->gyro[1]  = (float)y_gyro / gyro_conv_factor;
  mpu->gyro[2]  = (float)z_gyro / gyro_conv_factor;

  /* Apply bias correction */
  mpu->accel[0] -= hmpu->accel_bias[0];
  mpu->accel[1] -= hmpu->accel_bias[1];
  mpu->accel[2] -= hmpu->accel_bias[2];
  mpu->gyro[0]  -= hmpu->gyro_bias[0];
  mpu->gyro[1]  -= hmpu->gyro_bias[1];
  mpu->gyro[2]  -= hmpu->gyro_bias[2];

  return MPU_OK;
}
