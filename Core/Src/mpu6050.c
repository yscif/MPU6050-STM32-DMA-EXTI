/*
 * mpu6050.c
 *
 *  Created on: 08.10.2018
 *  	License: MIT
 *      Author: Mateusz Salamon
 *      Based on:
 *      	 - MPU-6000 and MPU-6050 Product Specification Revision 3.4
 *      	 - MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2
 *      	 - i2cdevlib by Jeff Rowberg on MIT license
 *      	 - SparkFun MPU-9250 Digital Motion Processor (DMP) Arduino Library on MIT License
 *
 *		www.msalamon.pl
 *		mateusz@msalamon.pl
 *
 *	Website: https://msalamon.pl/6-stopni-swobody-z-mpu6050-na-stm32/
 *	GitHub: https://github.com/lamik/MPU6050_STM32_HAL
 */


#include "mpu6050.h"
#include "math.h"

#define I2C_TIMEOUT 1000

I2C_HandleTypeDef *i2c;

const float mpuScale[] = {16384.0f, 131.072f}; // acc, gyro
float mpu[2][3];
uint8_t mpu_buffer[14];

//
// CONFIG
//
void MPU6050_SetDlpf(uint8_t Value)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xF8;
	tmp |= (Value & 0x7);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
}

//
// PWR_MGMT_1
//
void MPU6050_DeviceReset(uint8_t Reset)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1<<MPU6050_PWR1_DEVICE_RESET_BIT);
	tmp |= ((Reset & 0x1) << MPU6050_PWR1_DEVICE_RESET_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetSleepEnabled(uint8_t Enable)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1<<MPU6050_PWR1_SLEEP_BIT);
	tmp |= ((Enable & 0x1) << MPU6050_PWR1_SLEEP_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}


//
//	PWR_MGMT_2
//
void MPU6050_SetLowPowerWakeUpFrequency(uint8_t Frequency)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0x3F;
	tmp |= (Frequency & 0x3) << MPU6050_PWR2_LP_WAKE_CTRL_BIT;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_AccelerometerAxisStandby(uint8_t XA_Stby, uint8_t YA_Stby, uint8_t ZA_Stby)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xC7;
	tmp |= ((XA_Stby&0x1) << MPU6050_PWR2_STBY_XA_BIT)|((YA_Stby&0x1) << MPU6050_PWR2_STBY_YA_BIT)|((ZA_Stby&0x1) << MPU6050_PWR2_STBY_ZA_BIT) ;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_GyroscopeAxisStandby(uint8_t XG_Stby, uint8_t YG_Stby, uint8_t ZG_Stby)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xF8;
	tmp |= ((XG_Stby&0x1) << MPU6050_PWR2_STBY_XG_BIT)|((YG_Stby&0x1) << MPU6050_PWR2_STBY_YG_BIT)|((ZG_Stby&0x1) << MPU6050_PWR2_STBY_ZG_BIT) ;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
}

//
// Read Data
//
void MPU6050_GetAccelerometerRAW(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t tmp[6];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, tmp, 6, I2C_TIMEOUT);

	*x = (((int16_t)tmp[0]) << 8) | tmp[1];
	*y = (((int16_t)tmp[2]) << 8) | tmp[3];
	*z = (((int16_t)tmp[4]) << 8) | tmp[5];
}

void MPU6050_GetAllScalled(float (*mpuRaw)[3])
{
	uint8_t tmp[14];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, tmp, 14, I2C_TIMEOUT);

	for(uint8_t coordinat = 0; coordinat < 2; coordinat++)
	{
		for(uint8_t axis = 0; axis < 3; axis++)
		{
		mpu[coordinat][axis] = ((float)((int16_t)((((int16_t)tmp[(coordinat*8)+(axis*2)]) << 8) | tmp[(coordinat*8)+(axis*2)+1])) / mpuScale[coordinat]);
		}
	}
}


void MPU6050_GetAllScaled()
{
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, mpu_buffer, 14, I2C_TIMEOUT);

	for(uint8_t coordinat = 0; coordinat < 2; coordinat++)
	{
		for(uint8_t axis = 0; axis < 3; axis++)
		{
		mpu[coordinat][axis] = ((float)((int16_t)((((int16_t)mpu_buffer[(coordinat*8)+(axis*2)]) << 8) | mpu_buffer[(coordinat*8)+(axis*2)+1])) / mpuScale[coordinat]);// - mpuErr[coordinat][axis];
		}
	}
}


void MPU6050_AllScale_DMA()
{
	for(uint8_t coordinat = 0; coordinat < 2; coordinat++)
	{
		for(uint8_t axis = 0; axis < 3; axis++)
		{
		mpu[coordinat][axis] = ((float)((int16_t)((((int16_t)mpu_buffer[(coordinat*8)+(axis*2)]) << 8) | mpu_buffer[(coordinat*8)+(axis*2)+1])) / mpuScale[coordinat]);// - mpuErr[coordinat][axis];
		}
	}	
}


void MPU6050_GetGyroscopeRAW(int16_t *gyroRaw)//int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t tmp[6];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, 1, tmp, 6, I2C_TIMEOUT);

	gyroRaw[X] = (((int16_t)tmp[0]) << 8) | tmp[1];
	gyroRaw[Y] = (((int16_t)tmp[2]) << 8) | tmp[3];
	gyroRaw[Z] = (((int16_t)tmp[4]) << 8) | tmp[5];
}
/*
#define M_PI		3.14159265358979323846
*/
/*
static float wrap(float angle,float limit){
  while (angle >  limit) angle -= 2*limit;
  while (angle < -limit) angle += 2*limit;
  return angle;
}
*/
void MPU6050_GetRollPitch(float* Roll, float* Pitch, float* Yaw, const float dt)
{
	*Roll = atan2(mpu[acc][Y], mpu[acc][Z]) * 180.0 / M_PI;
	*Pitch = -(atan2(mpu[acc][X], invSqrt(mpu[acc][Y]*mpu[acc][Y] + mpu[acc][Z]*mpu[acc][Z]))*180.0)/M_PI; 
	*Yaw += mpu[gyro][Z] * dt; 

	//https://github.com/rfetick/MPU6050_light/blob/master/src/MPU6050_light.cpp
	/*
	int8_t sgZ = mpu[acc][Z]<0 ? -1 : 1;
	float angelAccX = atan2(mpu[acc][Y], sgZ*invSqrt(mpu[acc][Z]*mpu[acc][Z] + mpu[acc][X]*mpu[acc][X])) * 180.f/M_PI;
	float angelAccY = - atan2(mpu[acc][X],  invSqrt(mpu[acc][Z]*mpu[acc][Z] + mpu[acc][Y]*mpu[acc][Y])) * 180.f/M_PI;

	*Roll = wrap(filterGyroCoef*(angleAccX + wrap(angleX +     gyroX*dt - angleAccX,180)) + (1.0-filterGyroCoef)*angleAccX,180);
	*Pitch = wrap(filterGyroCoef*(angleAccY + wrap(angleY + sgZ*gyroY*dt - angleAccY, 90)) + (1.0-filterGyroCoef)*angleAccY, 90);
  	*Yaw += gyroZ*dt;
	*/

	//https://github.com/jarzebski/Arduino-MPU6050/blob/master/MPU6050_gyro_pitch_roll_yaw/MPU6050_gyro_pitch_roll_yaw.ino
	/*
	pitch = pitch + mpu[gyro][Y] * dt;
  	roll = roll + mpu[gyro][X] * dt;
  	yaw = yaw + mpu[gyro][Z] * dt;
	*/

	//https://forum.arduino.cc/t/converting-rotation-angles-from-mpu6050-to-roll-pitch-yaw/392641/2
	/*
	pitch = 180 * atan (accelerationX/sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ))/M_PI;
	roll = 180 * atan (accelerationY/sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/M_PI;
	yaw = 180 * atan (accelerationZ/sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/M_PI
	*/

	//https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
	/*
	accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) 
	accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
	gyroAngleX = gyroAngleX + GyroX * elapsedTime; // elapsedeTime = dt/1000 Saniye cinsinden gecen zaman
  	gyroAngleY = gyroAngleY + GyroY * elapsedTime; // deg/s * s = deg
	yaw =  yaw + GyroZ * elapsedTime;
	roll = 0.96 * gyroAngleX + 0.04 * accAngleX; // Complementary filter - combine acceleromter and gyro angle values
	pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
	*/
}


uint8_t MPU6050_GetDeviceID(void)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, 1, &tmp, 1, I2C_TIMEOUT);
	return tmp<<1;
}

//
//	Initialization
//
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
	i2c = hi2c;

	if(MPU6050_GetDeviceID() != MPU6050_ADDRESS) return 0;

	uint8_t tmp;

	tmp = 0x00;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	HAL_Delay(100);
	tmp = 0x03;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	
	tmp = 0x03;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp = 0x04;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 1, &tmp, 1, I2C_TIMEOUT);
	tmp = 0x00;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 1, &tmp, 1, I2C_TIMEOUT); // +-250degree/s
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, &tmp, 1, I2C_TIMEOUT); // +_2g

	tmp = 0x30;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);

	return 1;
}
void MPU6050_Read_DMA(void)
{

	HAL_I2C_Mem_Read_DMA(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, mpu_buffer, 14);
}

void MPU6050_Start_IRQ(void) //Enable Int
{
	uint8_t tmp = 0x01;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);

	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void MPU6050_Stop_IRQ(void) //Disable_Int
{
	uint8_t tmp = 0x00;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);

}

void MPU6050_Set_Calibrate_Gyro(uint8_t *data)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_XG_OFFS_USRH , 1, data, 6, I2C_TIMEOUT);
}

#define MPU_ERR_SAMPLING_COUNTER	10000// max: 65536

uint8_t* MPU6050_Calibrate_Gyro()
{
	int32_t gyroBias[3] = {0, 0, 0};
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int16_t gyroRaw[3];

	MPU6050_Calibrate_Gyro(data);

	for(uint16_t c = 0; c < MPU_ERR_SAMPLING_COUNTER; c++)
	{
		MPU6050_GetGyroscopeRAW(gyroRaw);
		gyroBias[X] += gyroRaw[X];
		gyroBias[Y] += gyroRaw[Y];
		gyroBias[Z] += gyroRaw[Z];
		HAL_Delay(5);
	}
	
	gyroBias[X] /= MPU_ERR_SAMPLING_COUNTER;
	gyroBias[Y] /= MPU_ERR_SAMPLING_COUNTER;
	gyroBias[Z] /= MPU_ERR_SAMPLING_COUNTER;

	data[0] = (-gyroBias[X]/4  >> 8) & 0xFF;
	data[1] = (-gyroBias[X]/4)       & 0xFF;
	data[2] = (-gyroBias[Y]/4  >> 8) & 0xFF;
	data[3] = (-gyroBias[Y]/4)       & 0xFF;
	data[4] = (-gyroBias[Z]/4  >> 8) & 0xFF;
	data[5] = (-gyroBias[Z]/4)       & 0xFF;

	MPU6050_Set_Calibrate_Gyro(data);

	return data;
}



float q0 = 1.0f; //initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float B_madgwick = 0.04;  //Madgwick filter parameter

void MPU6050_Madgwick(float *roll, float *pitch, float *yaw, const float invSampleFreq)//float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq, float* roll_IMU, float* pitch_IMU, float* yaw_IMU) {
{
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  mpu[gyro][X] *= 0.0174533f;
  mpu[gyro][Y] *= 0.0174533f;
  mpu[gyro][Z] *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * mpu[gyro][X] - q2 * mpu[gyro][Y] - q3 * mpu[gyro][Z]);
  qDot2 = 0.5f * (q0 * mpu[gyro][X] + q2 * mpu[gyro][Z] - q3 * mpu[gyro][Y]);
  qDot3 = 0.5f * (q0 * mpu[gyro][Y] - q1 * mpu[gyro][Z] + q3 * mpu[gyro][X]);
  qDot4 = 0.5f * (q0 * mpu[gyro][Z] + q1 * mpu[gyro][Y] - q2 * mpu[gyro][X]);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((mpu[acc][X] == 0.0f) && (mpu[acc][Y] == 0.0f) && (mpu[acc][Z] == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(mpu[acc][X] * mpu[acc][X] + mpu[acc][Y] * mpu[acc][Y] + mpu[acc][Z] * mpu[acc][Z]);
    mpu[acc][X] *= recipNorm;
    mpu[acc][Y] *= recipNorm;
    mpu[acc][Z] *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * mpu[acc][X] + _4q0 * q1q1 - _2q1 * mpu[acc][Y];
    s1 = _4q1 * q3q3 - _2q3 * mpu[acc][X] + 4.0f * q0q0 * q1 - _2q0 * mpu[acc][Y] - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * mpu[acc][Z];
    s2 = 4.0f * q0q0 * q2 + _2q0 * mpu[acc][X] + _4q2 * q3q3 - _2q3 * mpu[acc][Y] - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * mpu[acc][Z];
    s3 = 4.0f * q1q1 * q3 - _2q1 * mpu[acc][X] + 4.0f * q2q2 * q3 - _2q2 * mpu[acc][Y];
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //compute angles
  *roll = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  *pitch = -asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
  *yaw = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

//Fast inverse sqrt for madgwick filter
float invSqrt(float x) {
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}

