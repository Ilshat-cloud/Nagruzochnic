
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "usb_device.h"
#include "ssd1306.h"
#include "lsm303dlhs.h"
#include "L3GD20.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

DMA_HandleTypeDef dma_adc1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_DMA_Ini(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void screen0(void);
void screen1(void);
void screen2(void);
void screen3(void);
void screen4(void);
void screen5(void);
void screen6(void);
void screen7(void);
void screen8(void);
void screen10(void);    //just for luls
void startScreen(void);



char R[10]="Ch323";
uint16_t i;   //set value current
uint16_t i2;  //cur value current
uint16_t P;   //set value power
uint16_t P2;  //cur value
volatile uint16_t dma[2];
uint16_t ADC1value;
uint16_t ADC2value;
uint16_t filtr;
uint8_t nitro; // need for speed iteration
uint16_t PWM1; //PWM Compare
int16_t buffer[3];  // accel buf
uint16_t time_s;  //time
uint16_t capacity_result; 

//-------giro------

uint16_t OUT_X;
uint16_t OUT_Y;
uint16_t OUT_Z;
float foutx;
float fouty;
float foutz;
 
float translate(uint16_t result){
    return result*1.0;
}
float ms(float data){
    return data*8.75;
}





  /* USER CODE BEGIN 1 */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port,CS_I2C_SPI_Pin,SET);
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();  
  MX_DMA_Ini();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_USB_DEVICE_Init();
  ssd1306_Init();
  
  
  //----------DMA---------

  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&dma,2);

  //---------accel----------------
  uint16_t ctrl=(ACCEL_NORMAL_MODE|ACCEL_ODR_50HZ|ACCEL_AXES_ENABLE);
  ctrl|=((ACCEL_UPDATE_CONTINOUS|ACCEL_BLE_LSB|ACCEL_HR_ENABLE)<<8);
  Accel_init(ctrl);
  ctrl=(uint8_t)(ACCEL_HPM_NORMAL_MODE|ACCEL_HPFCF_16|ACCEL_HPF_AOI1_DISABLE|ACCEL_HPF_AOI2_DISABLE);
  Accel_Filter_Conf(ctrl);
  Accel_getXYZ(buffer);
  uint32_t oldtime=HAL_GetTick();

  //---------GIRO--------------
 // void Giro_init(void);
  uint8_t Data;
  uint8_t Data2[2]={0x8F,0x00};
  uint8_t Addr;

  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port,CS_I2C_SPI_Pin,RESET);  
  Addr=0x20;  
  HAL_SPI_TransmitReceive(&hspi1, &Addr, &Data, 1,0x1000);
  Addr=0x0F; 
  HAL_SPI_TransmitReceive(&hspi1, &Addr, &Data, 1,0x1000);
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port,CS_I2C_SPI_Pin,SET);

  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port,CS_I2C_SPI_Pin,RESET);
  Addr=0x8F;  
  HAL_SPI_TransmitReceive(&hspi1, &Addr, &Data, 1,0x1000);
  Addr=0x00;  
  HAL_SPI_TransmitReceive(&hspi1, &Addr, &Data, 1,0x1000);
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port,CS_I2C_SPI_Pin,SET);  
  if (Data==0xD4)
  {
      HAL_GPIO_WritePin(LD5_GPIO_Port,LD5_Pin,SET);
  }

  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port,CS_I2C_SPI_Pin,RESET);  
  Addr=0x23;  
  HAL_SPI_TransmitReceive(&hspi1, &Addr, &Data, 1,0x1000);
  Addr=0x00; 
  HAL_SPI_TransmitReceive(&hspi1, &Addr, &Data, 1,0x1000);
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port,CS_I2C_SPI_Pin,SET);

  //--------DAC---------
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1); //dac enable


  //read from flash
   i=*(__IO uint32_t*)User_Page_Adress;
   P=*(__IO uint32_t*)User_Page_Adress2;
   uint8_t flag =6;  //set to 6
   
  
   //----------Horse---------------
   startScreen();
   ssd1306_Fill(Black);
   ssd1306_SetCursor(32,0); 
   startScreen();
   ssd1306_Fill(Black);
   ssd1306_SetCursor(64,0); 
   startScreen();
   ssd1306_Fill(Black);
   ssd1306_SetCursor(96,0); 
   startScreen();
  
  /* Infinite loop */
  while (1)
  {
    if (filtr>5000){
    switch(flag){
    case 0:
      screen0();
      //Choise power or current 
      nitro=125;
      if (HAL_GPIO_ReadPin(Plus_GPIO_Port,Plus_Pin)==GPIO_PIN_SET) 
      {
         flag=1;
      }
      if (HAL_GPIO_ReadPin(Minus_GPIO_Port,Minus_Pin)==GPIO_PIN_SET) 
      {
         flag=4;
      }
     
      break;
    case 1:
      screen1();
      //increase or decrease P
      if (HAL_GPIO_ReadPin(Plus_GPIO_Port,Plus_Pin)==GPIO_PIN_SET) 
      {
       
         P++;
         if ((nitro>150)&&(P>51)){ P=P+50;}
         nitro++; 
         if (nitro>175) {nitro=175;}
      }
      if (HAL_GPIO_ReadPin(Minus_GPIO_Port,Minus_Pin)==GPIO_PIN_SET) 
      {
         
         P--;
         if ((nitro<75)&&(P>51)){ P=P-50;}
         nitro--;
         if (nitro<50) {nitro=50;}
      }
       if (HAL_GPIO_ReadPin(Start_GPIO_Port,Start_Pin)==GPIO_PIN_SET) 
      {
          //----flash---
          HAL_FLASH_Unlock();
          FLASH_PageErase(User_Page_Adress);
          CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD ,User_Page_Adress2,P);
          HAL_FLASH_Lock();
          flag=2;
      }

      break;
    case 2:
      screen2();
      // Start or back P
      if (HAL_GPIO_ReadPin(Plus_GPIO_Port,Plus_Pin)==GPIO_PIN_SET) 
      {
         HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
         flag=3;
      }
      if (HAL_GPIO_ReadPin(Minus_GPIO_Port,Minus_Pin)==GPIO_PIN_SET) 
      {
         flag=6;
         HAL_Delay(500);
      }

      break;
    case 3:
      //stop P
      screen3();
      if (HAL_GPIO_ReadPin(Start_GPIO_Port,Start_Pin)==GPIO_PIN_SET) 
      {
        HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
         flag=6;
      }

      break;
    case 4: 
      screen4();
       //increase or decrease I
        if (HAL_GPIO_ReadPin(Plus_GPIO_Port,Plus_Pin)==GPIO_PIN_SET) 
      {
         i++;
         if ((nitro>150)&&(i>51)){ i=i+50;}
         nitro++; 
         if (nitro>175) {nitro=175;}
      }
      if (HAL_GPIO_ReadPin(Minus_GPIO_Port,Minus_Pin)==GPIO_PIN_SET) 
      {
          i--;
          if ((nitro<75)&&(i>51)){ i=i-50;}
          nitro--;
          if (nitro<50) {nitro=50;}
      }
       if (HAL_GPIO_ReadPin(Start_GPIO_Port,Start_Pin)==GPIO_PIN_SET) 
      {
          //----flash---
          HAL_FLASH_Unlock();
          FLASH_PageErase(User_Page_Adress);
          CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD ,User_Page_Adress,i);
          HAL_FLASH_Lock();
          flag=5;
      }

      break;
    case 5:
      screen5();
      
       if (HAL_GPIO_ReadPin(Plus_GPIO_Port,Plus_Pin)==GPIO_PIN_SET) //litium
      {
        HAL_Delay(1000);
        HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
        time_s=0;
        flag=7;
      }
      if (HAL_GPIO_ReadPin(Minus_GPIO_Port,Minus_Pin)==GPIO_PIN_SET) 
      {
         flag=6;
         HAL_Delay(500);
         
      }
  
      break;
    case 6:
      HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
      flag=9;
     
      break;

    case 7:
      screen7();
    // litium
         if (P2<280) 
      {
         HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
         capacity_result=i*time_s/3600;
         flag=8;
      }
      if ((HAL_GPIO_ReadPin(Start_GPIO_Port,Start_Pin)==GPIO_PIN_SET)&&(HAL_GPIO_ReadPin(Plus_GPIO_Port,Plus_Pin)==GPIO_PIN_RESET)) 
      {
         flag=6;
         HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);  
      }
      break;
    case 8:
      screen8();
    // litium
      if (HAL_GPIO_ReadPin(Start_GPIO_Port,Start_Pin)==GPIO_PIN_SET) 
      {
         flag=6;
        
         
      }
      break;
      case 9:
      screen6();
      // Stop I
        if (HAL_GPIO_ReadPin(Start_GPIO_Port,Start_Pin)==GPIO_PIN_SET) 
      {
         HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
         flag=0;
      }
        if (HAL_GPIO_ReadPin(Plus_GPIO_Port,Plus_Pin)==GPIO_PIN_SET) 
      {
         i++;
         if ((nitro>150)&&(i>51)){ i=i+50;}
         nitro++; 
         if (nitro>175) {nitro=175;}
      }
      if (HAL_GPIO_ReadPin(Minus_GPIO_Port,Minus_Pin)==GPIO_PIN_SET) 
      {
          i--;
          if ((nitro<75)&&(i>51)){ i=i-50;}
          nitro--;
          if (nitro<50) {nitro=50;}
      }
     
      break;
      
      
    }
    filtr=0;
    }filtr++;
    
    
    //increase power
    if (flag==3){
      if ((P2>P)&&(PWM1>1))
      {
        PWM1--;
      } else if ((P2<P)&&(PWM1<1000)){
        PWM1++;
      }
    }
    
    //increase current
     if ((flag==9)||(flag==7)){
      if ((i2>i)&&(PWM1>1))
      {
        PWM1--;
      } else if ((i2<i)&&(PWM1<1000)){
        PWM1++;
      }
    }
   
    
    //--------PWM---------
     __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,PWM1);
    //--------ADC---------
    
   


    //--------DAC---------
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, i2);
    //--------time_sec-------
    if (HAL_GetTick()>(oldtime+500))
    {
    P2=dma[0];
    i2=dma[1];
    oldtime=HAL_GetTick();
    time_s++;
  //  screen10();
    }
 
  }


}






//-------------------------screen0------------------------
void screen0() {
  ssd1306_Fill(Black);
  ssd1306_SetCursor(10, 0);
  ssd1306_WriteString("+ Power Mode", Font_7x10, White);
  ssd1306_SetCursor(10, 14);
  ssd1306_WriteString("- Capacity Mode", Font_7x10, White);
  ssd1306_UpdateScreen();
}
//--------------------------------------------------------


//-------------------------screen1------------------------
void screen1() {
  ssd1306_Fill(Black);
  ssd1306_SetCursor(10, 0);
  ssd1306_WriteString("Set Power ", Font_7x10, White);
  ssd1306_SetCursor(10, 14);
  sprintf(R,"%d",P);
  ssd1306_WriteString(R, Font_7x10, White);
  ssd1306_WriteString(" mW", Font_7x10, White);
  ssd1306_UpdateScreen();
}
//--------------------------------------------------------

//-------------------------screen2------------------------
void screen2() {
  ssd1306_Fill(Black);
  ssd1306_SetCursor(1, 0);
  ssd1306_WriteString("Power ", Font_7x10, White);
  sprintf(R,"%d",P);
  ssd1306_WriteString(R, Font_7x10, White);
  ssd1306_WriteString(" mW", Font_7x10, White);
  ssd1306_SetCursor(10, 14);
  ssd1306_WriteString("+ Start - Back", Font_7x10, White);
  ssd1306_UpdateScreen();
}

//--------------------------------------------------------

//-------------------------screen3------------------------
  void screen3() {
  ssd1306_Fill(Black);
  ssd1306_SetCursor(10, 0);
  ssd1306_WriteString("Set: ", Font_7x10, White);
  sprintf(R,"%d",P); //temporary
  ssd1306_WriteString(R, Font_7x10, White);
  ssd1306_WriteString(" mW", Font_7x10, White);
  ssd1306_SetCursor(10, 14);
  ssd1306_WriteString("Cur: ", Font_7x10, White);
  sprintf(R,"%d",P2);   
  ssd1306_WriteString(R, Font_7x10, White);
  ssd1306_WriteString(" mW", Font_7x10, White);
  ssd1306_UpdateScreen();
}
//--------------------------------------------------------

//-------------------------screen4------------------------
void screen4() {
  ssd1306_Fill(Black);
  ssd1306_SetCursor(10, 0);
  ssd1306_WriteString("Set Current ", Font_7x10, White);
  ssd1306_SetCursor(10, 14);
  sprintf(R,"%d",i);
  ssd1306_WriteString(R, Font_7x10, White);
  ssd1306_WriteString(" mA", Font_7x10, White);
  ssd1306_UpdateScreen();
}
//--------------------------------------------------------

//-------------------------screen5------------------------
void screen5() {
  ssd1306_Fill(Black);
  ssd1306_SetCursor(10, 0);
  ssd1306_WriteString("I ", Font_7x10, White);
  sprintf(R,"%d",i);
  ssd1306_WriteString(R, Font_7x10, White);
  ssd1306_WriteString(" mA", Font_7x10, White);
  ssd1306_SetCursor(10, 14);
  ssd1306_WriteString("+Start -Back", Font_7x10, White);
  ssd1306_UpdateScreen();
}
//--------------------------------------------------------

//-------------------------screen6------------------------
void screen6() {
  ssd1306_Fill(Black);
  ssd1306_SetCursor(10, 0);
  ssd1306_WriteString("Set: ", Font_7x10, White);
  sprintf(R,"%d",i);
  ssd1306_WriteString(R, Font_7x10, White);
  ssd1306_WriteString(" mA", Font_7x10, White);
  ssd1306_SetCursor(10, 14);
  ssd1306_WriteString("Cur: ", Font_7x10, White);
  sprintf(R,"%d",i2);
  ssd1306_WriteString(R, Font_7x10, White);
  ssd1306_WriteString(" mA", Font_7x10, White);
  ssd1306_UpdateScreen();
}
//--------------------------------------------------------


//-------------------------screen7------------------------
void screen7() {
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Set:", Font_7x10, White);
  sprintf(R,"%d",i);
  ssd1306_WriteString(R, Font_7x10, White);
  ssd1306_WriteString(" mA Accum", Font_7x10, White);
  ssd1306_SetCursor(0, 14);
  ssd1306_WriteString("Cur:", Font_7x10, White);
  sprintf(R,"%d",i2);
  ssd1306_WriteString(R, Font_7x10, White);
  ssd1306_WriteString("mA ", Font_7x10, White);
  sprintf(R,"%d",time_s);
  ssd1306_WriteString(R, Font_7x10, White);
  ssd1306_WriteString("s", Font_7x10, White);
  ssd1306_UpdateScreen();
}
//--------------------------------------------------------

//-------------------------screen8------------------------
void screen8() {
  ssd1306_Fill(Black);
  ssd1306_SetCursor(30, 0);
  ssd1306_WriteString("Capacity:", Font_7x10, White);
  ssd1306_SetCursor(10, 14);
  sprintf(R,"%d",capacity_result);
  ssd1306_WriteString(R, Font_7x10, White);
  ssd1306_WriteString("mA/h", Font_7x10, White);
  ssd1306_UpdateScreen();
}
//--------------------------------------------------------


//-------------------------screen10------------------------
void screen10() {
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  sprintf(R,"%d",P2);
  ssd1306_WriteString("X->", Font_7x10, White);
  ssd1306_WriteString(R, Font_7x10, White);
  ssd1306_SetCursor(0, 20);
  sprintf(R,"%d",i2);
  ssd1306_WriteString("Z->", Font_7x10, White);
  ssd1306_WriteString(R, Font_7x10, White);
  ssd1306_UpdateScreen();
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


//----------------------dma inint------------------------------
static void MX_DMA_Ini(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn,0,0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
    
   /* 
dma_adc1.Instance
dma_adc1.Init.Direction=DMA_PERIPH_TO_MEMORY;
dma_adc1.Init.MemDataAlignment=DMA_MDATAALIGN_HALFWORD;
dma_adc1.Init.MemInc=DMA_MINC_DISABLE;
dma_adc1.Init.Mode=DMA_NORMAL;
dma_adc1.Init.PeriphDataAlignment
dma_adc1.Init.PeriphInc=DMA_MINC_DISABLE;
dma_adc1.Init.Priority=DMA_PRIORITY_LOW;
HAL_DMA_Init(dma_adc1);

dma_adc1.ChannelIndex
dma_adc1.DmaBaseAddress
dma_adc1.ErrorCode

dma_adc1.Lock
dma_adc1.Parent
dma_adc1.State
dma_adc1.XferAbortCallback
dma_adc1.XferCpltCallback
dma_adc1.XferErrorCallback
dma_adc1.XferHalfCpltCallback

  */

}
 
    



/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode =  ADC_SCAN_ENABLE ;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV ;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x0000020B;
  hi2c2.Init.OwnAddress1 = 22;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin 
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin 
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin 
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin 
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin 
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin 
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin 
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin 
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Start_Pin Plus_Pin Minus_Pin */
  GPIO_InitStruct.Pin = Start_Pin|Plus_Pin|Minus_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
