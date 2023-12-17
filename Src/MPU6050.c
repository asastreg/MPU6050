
#include "MPU6050.h"


MPU6050_StatusTypeDef MPU6050_Init(MPU6050_HandleTypeDef *hmpu)
{
  uint8_t data;

  HAL_I2C_Mem_Read(hmpu->hi2c , hmpu->adress, WHO_AM_I_REG, 1, &data, 1, hmpu->timeout);

  /* Check if device response is correct, WHO_AM_I_REG always response with 0x68 */
  if(data != 0x68)
  {
	  return MPU_ERROR;
  }

  /* waking the sensor up */
  data = 0x00;
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, PWR_MGMT_1_REG, 1, &data, 1, hmpu->timeout);

  /* Set data rate of 1KHz */
  data = 0x07;
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, SMPLRT_DIV_REG, 1, &data, 1, hmpu->timeout);

  /* Disable FSYNC and setting DLPF */
  data = 0x00;
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, CONFIG_REG, 1, &data, 1, hmpu->timeout);

  /* Set accel config (0x00 = 2g, 0x08 = 4g, 0x10 = 8g, 0x18 = 16g */
  data = (uint8_t)(hmpu->afs << 3);
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, ACCEL_CONFIG_REG, 1, &data, 1, hmpu->timeout);

  /* Set gyro config (0x00 = 250º/s, 0x08 = 500º/s, 0x10 = 1000º/s, 0x18 = 2000º/s */
  data = (uint8_t)(hmpu->gfs << 3);
  HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->adress, GYRO_CONFIG_REG, 1, &data, 1, hmpu->timeout);

  return MPU_OK;
}


MPU6050_StatusTypeDef MPU6050_Read_Temp(MPU6050_HandleTypeDef *hmpu, MPU6050_t *mpu)
{
  uint8_t data[2];
  int16_t temp;

  HAL_I2C_Mem_Read(hmpu->hi2c, hmpu->adress, TEMP_OUT_H_REG, 1, data, 2, hmpu->timeout);
  temp = (int16_t)(data[0] << 8 | data[1]);

  mpu->temp_raw = temp;
  mpu->temp = (float)temp / (float)340 + (float)36.53;

  return MPU_OK;
}


MPU6050_StatusTypeDef MPU6050_Read_Accel(MPU6050_HandleTypeDef *hmpu, MPU6050_t *mpu)
{
  uint8_t data[6];
  int16_t x_accel, y_accel, z_accel;

  HAL_I2C_Mem_Read(hmpu->hi2c, hmpu->adress, ACCEL_XOUT_H_REG, 1, data, 6, hmpu->timeout);

  x_accel = (int16_t)(data[0] << 8 | data[1]);
  y_accel = (int16_t)(data[2] << 8 | data[3]);
  z_accel = (int16_t)(data[4] << 8 | data[5]);

  mpu->x_accel_raw = x_accel;
  mpu->y_accel_raw = y_accel;
  mpu->z_accel_raw = z_accel;

  /* Calculate accelerometer values base on the accel full scale range */
  switch (hmpu->afs) {
	case AFS_0:
		mpu->x_accel = (float)(x_accel / 16384);
		mpu->y_accel = (float)(y_accel / 16384);
		mpu->z_accel = (float)(z_accel / 16384);
		break;
	case AFS_1:
		mpu->x_accel = (float)(x_accel / 8192);
		mpu->y_accel = (float)(y_accel / 8192);
		mpu->z_accel = (float)(z_accel / 8192);
		break;
	case AFS_2:
		mpu->x_accel = (float)(x_accel / 4096);
		mpu->y_accel = (float)(y_accel / 4096);
		mpu->z_accel = (float)(z_accel / 4096);
		break;
	case AFS_3:
		mpu->x_accel = (float)(x_accel / 2048);
		mpu->y_accel = (float)(y_accel / 2048);
		mpu->z_accel = (float)(z_accel / 2048);
		break;
	default:
		mpu->x_accel = (float)(x_accel / 16384);
		mpu->y_accel = (float)(y_accel / 16384);
		mpu->z_accel = (float)(z_accel / 16384);
		break;
  }

  return MPU_OK;
}


MPU6050_StatusTypeDef MPU6050_Read_Gyro(MPU6050_HandleTypeDef *hmpu, MPU6050_t *mpu)
{
  uint8_t data[6];
  int16_t x_gyro, y_gyro, z_gyro;

  HAL_I2C_Mem_Read(hmpu->hi2c, hmpu->adress, GYRO_XOUT_H_REG, 1, data, 6, hmpu->timeout);

  x_gyro = (int16_t)(data[0] << 8 | data[1]);
  y_gyro = (int16_t)(data[2] << 8 | data[3]);
  z_gyro = (int16_t)(data[4] << 8 | data[5]);

  mpu->x_gyro_raw = x_gyro;
  mpu->y_gyro_raw = y_gyro;
  mpu->z_gyro_raw = z_gyro;

  /* Calculate gyro values base on the gyro full scale range */
    switch (hmpu->afs) {
  	case AFS_0:
  		mpu->x_gyro = (float)(x_gyro / 16384);
  		mpu->y_gyro = (float)(y_gyro / 16384);
  		mpu->z_gyro = (float)(z_gyro / 16384);
  		break;
  	case AFS_1:
  		mpu->x_gyro = (float)(x_gyro / 8192);
  		mpu->y_gyro = (float)(y_gyro / 8192);
  		mpu->z_gyro = (float)(z_gyro / 8192);
  		break;
  	case AFS_2:
  		mpu->x_gyro = (float)(x_gyro / 4096);
  		mpu->y_gyro = (float)(y_gyro / 4096);
  		mpu->z_gyro = (float)(z_gyro / 4096);
  		break;
  	case AFS_3:
  		mpu->x_gyro = (float)(x_gyro / 2048);
  		mpu->y_gyro = (float)(y_gyro / 2048);
  		mpu->z_gyro = (float)(z_gyro / 2048);
  		break;
  	default:
  		mpu->x_gyro = (float)(x_gyro / 16384);
  		mpu->y_gyro = (float)(y_gyro / 16384);
  		mpu->z_gyro = (float)(z_gyro / 16384);
  		break;
    }

  return MPU_OK;
}


MPU6050_StatusTypeDef MPU6050_Read_All(MPU6050_HandleTypeDef *hmpu, MPU6050_t *mpu)
{
	MPU6050_Read_Accel(hmpu, mpu);
	MPU6050_Read_Gyro(hmpu, mpu);
	MPU6050_Read_Temp(hmpu, mpu);

  return MPU_OK;
}

