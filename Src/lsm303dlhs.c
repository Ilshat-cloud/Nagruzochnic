#include "lsm303dlhs.h"


void Accel_init(uint16_t Init)
{ if (Accel_IO_Read(0x32,0x0F)==0x33)
  {
      HAL_GPIO_WritePin(LD4_GPIO_Port,LD4_Pin,SET);
  }

  uint8_t ctrl =0x00;
  ctrl=(uint8_t) Init;
  Accel_IO_Write(0x32, ACCEL_CTRL_REG1_A , ctrl);
  ctrl=(uint8_t)Init<<8;
  Accel_IO_Write(0x32, ACCEL_CTRL_REG4_A , ctrl);
}

void Accel_Filter_Conf(uint8_t FilterStruct)
{
  uint8_t tmp = Accel_IO_Read(0x32,ACCEL_CTRL_REG2_A);
  tmp&=0x0C;
  tmp|=FilterStruct;
  Accel_IO_Write(0x32, ACCEL_CTRL_REG2_A , tmp);
}

void Accel_getXYZ(int16_t* pData)
{
  int16_t pRawData[3];
  uint8_t ctrlx[2]={0,0};
  int8_t buffer[6];
  uint8_t i=0;
  uint8_t sensitivity=ACCEL_ACC_SENSITIVITY_2G;
  ctrlx[0]=Accel_IO_Read(0x32,ACCEL_CTRL_REG4_A);
  ctrlx[1]=Accel_IO_Read(0x32,ACCEL_CTRL_REG5_A);
  
  buffer[0]=Accel_IO_Read(0x32,ACCEL_OUT_X_L_A);
  buffer[1]=Accel_IO_Read(0x32,ACCEL_OUT_X_H_A);
  buffer[2]=Accel_IO_Read(0x32,ACCEL_OUT_Y_L_A);
  buffer[3]=Accel_IO_Read(0x32,ACCEL_OUT_Y_H_A);
  buffer[4]=Accel_IO_Read(0x32,ACCEL_OUT_Z_L_A);
  buffer[5]=Accel_IO_Read(0x32,ACCEL_OUT_Z_H_A);
  
  if(!(ctrlx[0]&ACCEL_BLE_MSB))
  {
    for(i=0;i<3;i++)
    {
      pRawData[i]=((int16_t)((uint16_t)buffer[2*i+1]<<8)+buffer[2*i]);         
    }    
  } else
  {
    for(i=0;i<3;i++)
    {
      pRawData[i]=((int16_t)((uint16_t)buffer[2*i]<<8)+buffer[2*i+1]);      
    }
    
  }
  switch(ctrlx[0]&ACCEL_ACC_SENSITIVITY_16G)
  {
  case ACCEL_ACC_SENSITIVITY_2G:
    sensitivity=ACCEL_ACC_SENSITIVITY_2G;
    break;
  case ACCEL_ACC_SENSITIVITY_4G:
    sensitivity=ACCEL_ACC_SENSITIVITY_4G;
    break;  
  case ACCEL_ACC_SENSITIVITY_8G:
    sensitivity=ACCEL_ACC_SENSITIVITY_8G;
    break;  
  case ACCEL_ACC_SENSITIVITY_16G:
    sensitivity=ACCEL_ACC_SENSITIVITY_16G;
    break;  
  }
  
  for(i=0;i<3;i++)
  {
    pData[i]=(pRawData[i]*sensitivity);
  }
}












uint8_t Accel_IO_Read(uint16_t DevAddr, uint8_t RegAddr)
{

  return I2Cx_ReadData(DevAddr,RegAddr);
}

void Accel_IO_Write(uint16_t DevAddr, uint8_t RegAddr, uint8_t Value)
{
  I2Cx_WriteData(DevAddr,RegAddr,Value);
}

static uint8_t I2Cx_ReadData(uint16_t Addr,uint8_t Reg)
{
  HAL_StatusTypeDef status=HAL_OK;
  uint8_t value =0;
  status= HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value,1,0x1000);
  if (status != HAL_OK)
    {  
      return HAL_ERROR;
    }
  return value;
}

static void I2Cx_WriteData(uint16_t Addr,uint8_t Reg, uint8_t Value)
{
 
 HAL_I2C_Mem_Write(&hi2c1, Addr,(uint16_t) Reg, I2C_MEMADD_SIZE_8BIT, &Value,1,0x1000);
}