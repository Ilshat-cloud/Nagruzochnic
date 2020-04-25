#include "stm32f3xx_hal.h"
#ifndef lsm303dlhs
#define lsm303dlhs


extern I2C_HandleTypeDef hi2c1;
#define ACCEL_NORMAL_MODE                       ((uint8_t)0x00)
#define ACCEL_LOWPOWER_MODE                       ((uint8_t)0x08)
#define ACCEL_ODR_1HZ                   ((uint8_t)0x10)
#define ACCEL_ODR_10HZ               ((uint8_t)0x20)
#define ACCEL_ODR_25HZ             ((uint8_t)0x30)
#define ACCEL_ODR_50HZ               ((uint8_t)0x40)
#define ACCEL_ODR_100HZ               ((uint8_t)0x50)
#define ACCEL_ODR_200HZ              ((uint8_t)0x60)
#define ACCEL_ODR_400HZ                ((uint8_t)0x70)
#define ACCEL_ODR_1620_HZ_LP                ((uint8_t)0x80)
#define ACCEL_ODR_1344HZ                ((uint8_t)0x90)
#define ACCEL_X_ENABLE               ((uint8_t)0x01)
#define ACCEL_Y_ENABLE                 ((uint8_t)0x02)
#define ACCEL_Z_ENABLE                 ((uint8_t)0x04)
#define ACCEL_AXES_ENABLE                 ((uint8_t)0x07)
#define ACCEL_AXES_DISABLE                ((uint8_t)0x00)
#define ACCEL_HR_ENABLE                 ((uint8_t)0x08)
#define ACCEL_HR_DISABLE                  ((uint8_t)0x00)
#define ACCEL_FULLSCALE_2G               ((uint8_t)0x00)
#define ACCEL_FULLSCALE_4G               ((uint8_t)0x10)
#define ACCEL_FULLSCALE_8G               ((uint8_t)0x20)
#define ACCEL_FULLSCALE_16G               ((uint8_t)0x30)
#define ACCEL_UPDATE_CONTINOUS               ((uint8_t)0x00)
#define ACCEL_UPDATE_SINGLE               ((uint8_t)0x80)
#define ACCEL_BLE_LSB               ((uint8_t)0x00)
#define ACCEL_BLE_MSB               ((uint8_t)0x40)
#define ACCEL_HPM_NORMAL_MODE_RES               ((uint8_t)0x00)
#define ACCEL_HPM_REF_SIGNAL               ((uint8_t)0x40)
#define ACCEL_HPM_NORMAL_MODE               ((uint8_t)0x80)
#define ACCEL_HPM_AUTORESET_INT               ((uint8_t)0xC0)
#define ACCEL_HPFCF_8              ((uint8_t)0x00)
#define ACCEL_HPFCF_16               ((uint8_t)0x10)
#define ACCEL_HPFCF_32               ((uint8_t)0x20)
#define ACCEL_HPFCF_64               ((uint8_t)0x30)
#define ACCEL_HPF_AOI1_DISABLE              ((uint8_t)0x00)
#define ACCEL_HPF_AOI1_ENABLE              ((uint8_t)0x01)
#define ACCEL_HPF_AOI2_DISABLE              ((uint8_t)0x00)
#define ACCEL_HPF_AOI2_ENABLE              ((uint8_t)0x02)
#define ACCEL_CTRL_REG1_A              0x20
#define ACCEL_CTRL_REG2_A               0x21
#define ACCEL_CTRL_REG3_A               0x22
#define ACCEL_CTRL_REG4_A              0x23
#define ACCEL_CTRL_REG5_A               0x24
#define ACCEL_ACC_SENSITIVITY_2G              ((uint8_t)1)
#define ACCEL_ACC_SENSITIVITY_4G              ((uint8_t)2)
#define ACCEL_ACC_SENSITIVITY_8G              ((uint8_t)4)
#define ACCEL_ACC_SENSITIVITY_16G              ((uint8_t)12)
#define ACCEL_OUT_X_L_A              0x28
#define ACCEL_OUT_X_H_A               0x29
#define ACCEL_OUT_Y_L_A              0x2A
#define ACCEL_OUT_Y_H_A               0x2B
#define ACCEL_OUT_Z_L_A               0x2C
#define ACCEL_OUT_Z_H_A               0x2D


void Accel_init(uint16_t Init);
uint8_t Accel_IO_Read(uint16_t DevAddr, uint8_t RegAddr);
void Accel_IO_Write(uint16_t DevAddr, uint8_t RegAddr, uint8_t Value);
void Accel_getXYZ(int16_t* pData);
static uint8_t I2Cx_ReadData(uint16_t Addr,uint8_t Reg);
static void I2Cx_WriteData(uint16_t Addr,uint8_t Reg, uint8_t Value);


#endif