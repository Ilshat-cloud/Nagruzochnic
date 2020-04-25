#include "L3GD20.h"
uint8_t Data;
uint8_t Addr;
void Giro_init(void)
{ 
  Addr=0x8F;
  HAL_SPI_TransmitReceive(&hspi1, &Addr, &Data, 1,0x1000);
    if (Data==0xD4)
  {
      HAL_GPIO_WritePin(LD5_GPIO_Port,LD5_Pin,SET);
  }
  Addr=0x00;
  HAL_SPI_TransmitReceive(&hspi1, &Addr, &Data, 1,0x1000);
  if (Data==0xD4)
  {
      HAL_GPIO_WritePin(LD5_GPIO_Port,LD5_Pin,SET);
  }
  
  
}
/*
  uint8_t ctrl =0x00;
  ctrl=(uint8_t) Init;
  Giro_IO_Write(0x32, ACCEL_CTRL_REG1_A , ctrl);
  ctrl=(uint8_t)Init<<8;
  Giro_IO_Write(0x32, ACCEL_CTRL_REG4_A , ctrl);*/



/*
void Giro_getXYZ(int16_t* pData)
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



*/








