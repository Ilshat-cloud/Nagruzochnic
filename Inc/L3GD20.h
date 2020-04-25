#include "stm32f3xx_hal.h"
#ifndef L3GD20
#define L3GD20


extern SPI_HandleTypeDef hspi1;
#define L3GD20_WHO_AM_I     ((uint8_t)0x0F)
#define L3GD20_CTRL_REG1     ((uint8_t)0x20)
#define L3GD20_CTRL_REG2     ((uint8_t)0x21)
#define L3GD20_CTRL_REG3     ((uint8_t)0x22)
#define L3GD20_CTRL_REG4     ((uint8_t)0x23)
#define L3GD20_CTRL_REG5     ((uint8_t)0x24)
#define L3GD20_REFERENCE     ((uint8_t)0x25)
#define L3GD20_OUT_TEMP     ((uint8_t)0x26)
#define L3GD20_STATUS_REG    ((uint8_t)0x27)
#define L3GD20_OUT_X_L     ((uint8_t)0x28)
#define L3GD20_OUT_X_H     ((uint8_t)0x29)
#define L3GD20_OUT_Y_L     ((uint8_t)0x2A)
#define L3GD20_OUT_Y_H     ((uint8_t)0x2B)
#define L3GD20_OUT_Z_L     ((uint8_t)0x2C)
#define L3GD20_OUT_Z_H     ((uint8_t)0x2D)
#define L3GD20_FIFO_CTRL_REG     ((uint8_t)0x2E)
#define L3GD20_FIFO_SRC_REG     ((uint8_t)0x2F)
#define L3GD20_INT1_CFG     ((uint8_t)0x30)
#define L3GD20_INT1_SRC     ((uint8_t)0x31)
#define L3GD20_INT1_TSH_XH     ((uint8_t)0x32)
#define L3GD20_INT1_TSH_XL     ((uint8_t)0x33)
#define L3GD20_INT1_TSH_YH     ((uint8_t)0x34)
#define L3GD20_INT1_TSH_YL     ((uint8_t)0x35)
#define L3GD20_INT1_TSH_ZH     ((uint8_t)0x36)
#define L3GD20_INT1_TSH_ZL     ((uint8_t)0x37)
#define L3GD20_DURIATION    ((uint8_t)0x38)




void Giro_init(void);


#endif