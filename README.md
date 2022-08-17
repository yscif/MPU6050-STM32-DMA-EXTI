# MPU6050-STM32-DMA-EXTI
MPU6050 DMA &amp; EXTI HAL library  real time orientation with Madgwick Orientation &amp; Complementary Filter

This repository aims to keep the efficiency at the highest level by giving the workload to all possible hardware in order to achieve maximum performance with minimum code and processing power in STM32 microcontrollers of the MPU6050 sensor.

General operation:\n
The sensor takes samples every 5ms (200Hz) and generates new Accel and Gyro data using the internal DLPF
Sends interrupt signal from external interrupt pin as soon as data is generated
STM32 detects external interrupt and sets up dma to receive data from i2c non-blocking
When the DMA reading is complete, the received data is scaled, then the orientation is calculated by applying the Madgwick Orientation or Complementary Filter.
The process is automatically repeated every time the sensor generates new data (every 5ms).

Madgwick Orientation Filter & Fast inverse sqrt for madgwick filter is taken from https://github.com/nickrehm/dRehmFlight.

![image](https://user-images.githubusercontent.com/46872345/185007435-a8fb399f-57a0-4f41-9ded-d58873d16ab7.png)

![image](https://user-images.githubusercontent.com/46872345/185007529-cc45e033-06ba-4f4b-8734-67b6365c1f24.png)

![image](https://user-images.githubusercontent.com/46872345/185007616-bda04e12-21f0-45b5-969c-24e972678c14.png)

![image](https://user-images.githubusercontent.com/46872345/185007690-9a96c3e7-6601-4af9-a40a-ca5d41a3ced0.png)
