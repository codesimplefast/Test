/**
  ******************************************************************************
  * @file    UART/UART_TwoBoards_ComIT/Src/main.c 
  * @author  MCD Application Team
  * @brief   This sample code shows how to use UART HAL API to transmit
  *          and receive a data buffer with a communication process based on
  *          IT transfer. 
  *          The communication is done using 2 Boards.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F0xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_TwoBoards_ComIT
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TRANSMITTER_BOARD
/* Uncomment the line below according to the Test you will perform, but only one per Test as every Test will be done in an endless loop */
// #define TEST_CLOCK
// #define TEST_OW_UART
// #define TEST_DEBUG_UART
// #define TEST_LED
// #define TEST_NTC
// #define TEST_BUTTON
 #define TEST_I2C


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef OWUartHandle;
UART_HandleTypeDef DEBUGUartHandle;
/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle;
/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;

/* Counter Prescaler value */
uint32_t uhPrescalerValue = 0;

__IO ITStatus UartReady = RESET;
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */

/* Buffer used for transmission */
uint8_t Debug_aTxBuffer[] = "Test";
//uint8_t OW_aTxBuffer[] = "OWire";
uint8_t OW_aTxBuffer[] = "e";
/* Buffer used for reception */
uint8_t Debug_aRxBuffer[DEBUG_USART_RXBUFFERSIZE];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void SystemClock_ConfigHSI(void);
static void Error_Handler(void);
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F0xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 48 MHz */
  //SystemClock_ConfigHSI();
  SystemClock_Config();////Hat am 04.08.2017 nicht gefunzt, wahrscheinlich hat µC Versorgungproblem

/* Test the Clock ------------------------------------------------------------*/
#if defined (TEST_CLOCK)
  /* Output Clock on PA8 */
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
  #define PERIOD_VALUE  (uint32_t)(250 - 1)  /* Period Value  */
  #define PULSE_VALUE   (uint32_t)(PERIOD_VALUE/2)
  TIMx_CLK_ENABLE();
  
  /* Initialize TIMx peripheral as follows:
     + Prescaler = (SystemCoreClock / 16000000) - 1
     + Period = (666 - 1)
     + ClockDivision = 0
     + Counter direction = Up
  */
  TimHandle.Instance = TIM3;

  TimHandle.Init.Prescaler         = 40000;//teilt den Eingangstakt herunter 48Mhz/40000=1200Hz -> 0,833ms
  TimHandle.Init.Period            = PERIOD_VALUE;//max Zählwert, danach von vorne
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;//Teiler für Eingangstakt
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  //HAL_NVIC_SetPriority(TIM3_IRQn,4,1);
  //HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_TIM_Base_Init(&TimHandle);
  HAL_TIM_Base_Start(&TimHandle);
  //HAL_TIM_Base_Start_IT(&TimHandle);
  BSP_LED_Init(LED1);
  while(1)
  {
  int timerValue = __HAL_TIM_GET_COUNTER(&TimHandle);
    if (timerValue < PULSE_VALUE)//250*0,833ms= 208,25ms Wenn clock stimmt sollte die LED 208ms an sein und 208ms aus sein
    {
      GPIOB->BSRR = (uint32_t)LED1_PIN;
    }
    else
    {
      GPIOB->BRR = (uint32_t)LED1_PIN;
    }
  }
#endif  

/* Test the BUTTON -----------------------------------------------------------*/ 
#if defined (TEST_BUTTON)
  /* Configure User push-button in Interrupt mode */
  /* Wenn Button gedrückt wird, wird die Fkt HAL_GPIO_EXTI_Callback() aufgerufen, diese ist in main.c */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  /* Configure LEDs */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  HAL_Delay(2000);
  /*Show Start Sequence */
  BSP_LED_On(LED1);
  HAL_Delay(200);
  BSP_LED_On(LED2);
  HAL_Delay(200);
  BSP_LED_On(LED3);
  HAL_Delay(200);
  BSP_LED_On(LED4);
  HAL_Delay(200);
  BSP_LED_Off(LED1);
  BSP_LED_Off(LED2);
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
  HAL_Delay(200);
  /*Show Start Sequence end */
  while(1)
  {
    BSP_LED_On(LED1);
    HAL_Delay(200);
    BSP_LED_Off(LED1);
    HAL_Delay(200);
  }
    
#endif
  
/* Test the NTC --------------------------------------------------------------*/ 
#if defined (TEST_NTC)  
  /* Configure NTC */
  BSP_NTC_Init();
  BSP_NTC_Disable();
  BSP_NTC_Enable();
  BSP_NTC_Disable();
#endif
  
/* Test the LEDs -------------------------------------------------------------*/  
#if defined (TEST_LED)  
  /* Configure LEDs */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  /* Test the LEDs */
  while(1)
  {
    BSP_LED_On(LED1);
    HAL_Delay(350);
    BSP_LED_Off(LED1);
    HAL_Delay(350);
    BSP_LED_On(LED2);
    HAL_Delay(350);
    BSP_LED_Off(LED2);
    HAL_Delay(350);
    BSP_LED_On(LED3);
    HAL_Delay(350);           
    BSP_LED_Off(LED3);
    HAL_Delay(350);
    BSP_LED_On(LED4);
    HAL_Delay(350);
    BSP_LED_Off(LED4);//alle 4 LEDs haben am 04.08.2017 gefunzt im Einzelschritt
    HAL_Delay(350);
  }
#endif
  
/* Test the I2C_EEPROM -------------------------------------------------------*/  
#if defined (TEST_I2C)
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  //BSP_LED_On(LED1);
  //04.09.2017
  /* Useful variables during communication */
  uint16_t Memory_Address;
  int Remaining_Bytes;
  /* Buffer used for transmission */
  uint8_t aTxBuffer[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34, 0x34};
  /* Buffer used for reception */
  uint8_t aRxBuffer[16];
  /* used a ON Semi CAT24C16 16kB(2048x8) EEPROM */
  /* C16 has a fixed Addr 0xA0(shift 1 pos. left) as the remaining 3Bits of the HW Addr will be used to set the Memory-Addr */
  /* so we have 11Bits for the Memory-Addr form 0x000 to 0x0x7FF -> 0x800=2048 */
  #define I2C_Speed               200000
  //#define I2C1_SLAVE_ADDRESS7     0xA0//ToDo diese Adresse ist auch in main.h aber 0x30F warum 12Bits 
  #define EEPROM_PAGESIZE         16
  #define EEPROM_HW_ADDRESS       0xA0   /* For Part C16 the Pins A0, A1 and A2 are not connected, so A0=x  A1=x  A2=x (shift 1 pos. left)*/
  /* EEPROM TIMING is calculated in case of the I2C Clock source is the SYSCLK = 48 MHz */
  /* Set TIMING to 0x00E0D3FF to reach 100 KHz speed (Rise time = 50ns, Fall time = 10ns) */
  #define EEPROM_TIMING           0x00E0D3FF
  #define I2C_FREQ   (100000) // We don't need fast I2C. 100KHz is fine here.
  #define TIMEOUT     (1000) /* Can't be sure when I2C routines return. Interrupts while polling hardware may result in unknown delays. */
  #define EEPROM_MAX_TRIALS       300
  #define I2C_XFER_TIMEOUT_MAX    300
  //uint8_t aTxBuffer[] = 0x31;
  uint8_t slave_addr = 0x00;
  uint8_t reg = 0x00;
  /*##-1- Configure the I2C peripheral ######################################*/
  I2cHandle.Instance             = I2C1;
  I2cHandle.Init.Timing          = EEPROM_TIMING;//I2C_TIMING;
  //I2cHandle.Init.ClockSpeed      = I2C_FREQ;
  I2cHandle.Init.OwnAddress1     = 0x00;//I2C_ADDRESS;
  I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.OwnAddress2     = 0x00;//0xFF;
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  
  if(HAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Enable the Analog I2C Filter */
  HAL_I2CEx_ConfigAnalogFilter(&I2cHandle,I2C_ANALOGFILTER_ENABLE);
  for (int i=0; i<127; i++)
  {
    if(HAL_I2C_Master_Transmit(&I2cHandle, (i<<1), &reg, 1, TIMEOUT)== HAL_OK)
    {
      slave_addr = i;
      //BSP_LED_On(LED2);
      break;
    }
  }
  //if( HAL_I2C_Mem_Write_IT(&I2cHandle, (uint16_t)EEPROM_HW_ADDRESS, 0, I2C_MEMADD_SIZE_8BIT, (uint8_t *)aTxBuffer, 1) != HAL_OK )
  //{
    //Error_Handler();
  //}
     /* Wait for the end of the transfer */
    /*  Before starting a new communication transfer, you need to check the current   
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the 
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */  
    //while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
    //{
    //}

    /* Check if the EEPROM is ready for a new operation */
    //while (HAL_I2C_IsDeviceReady(&I2cHandle, EEPROM_HW_ADDRESS, EEPROM_MAX_TRIALS, I2C_XFER_TIMEOUT_MAX) == HAL_TIMEOUT);

    /* Wait for the end of the transfer */
    //while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
    //{
    //}
  //if(HAL_I2C_Mem_Read_IT(&I2cHandle, (uint16_t) EEPROM_HW_ADDRESS, 0, I2C_MEMADD_SIZE_8BIT, (uint8_t *)aRxBuffer, 1) != HAL_OK )
  //{
    //Error_Handler();
  //}
  /* Write EEPROM_PAGESIZE */
  /* Initialize Memory address to 0 since EEPROM write will start from address 0 */
  Memory_Address = 0; 
  //if(HAL_I2C_Mem_Write_DMA(&I2cHandle , (uint16_t)EEPROM_HW_ADDRESS, Memory_Address, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(aTxBuffer + Memory_Address), EEPROM_PAGESIZE)!= HAL_OK)
  //{
    /* Writing process Error */
    //Error_Handler();
  //}
   // while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
  //{
  //}

  //if (HAL_I2C_Mem_Read_DMA(&I2cHandle, (uint16_t)EEPROM_HW_ADDRESS, 0, I2C_MEMADD_SIZE_8BIT, (uint8_t *)aRxBuffer, 1) != HAL_OK)
  //{
    /* Reading process Error */
    //Error_Handler();
  //}
  //while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
  //{
  //}
  //if (HAL_I2C_Mem_Read_DMA(&I2cHandle, (uint16_t)EEPROM_HW_ADDRESS, 1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)aRxBuffer, 5) != HAL_OK)
  //{
    /* Reading process Error */
    //Error_Handler();
  //}
  /* Memory Test for Addr 0 to 255 */
  for(Memory_Address=0;Memory_Address<256; Memory_Address++)
  {
      if(HAL_I2C_Mem_Write_DMA(&I2cHandle , (uint16_t)EEPROM_HW_ADDRESS, Memory_Address, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(aTxBuffer), 1)!= HAL_OK)
      {
        /* Writing process Error */
        Error_Handler();
        BSP_LED_On(LED2);
      }
      while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
      {
      }
      /* Check if the EEPROM is ready for a new operation */
      while (HAL_I2C_IsDeviceReady(&I2cHandle, EEPROM_HW_ADDRESS, EEPROM_MAX_TRIALS, I2C_XFER_TIMEOUT_MAX) == HAL_TIMEOUT);

      /* Wait for the end of the transfer */
      while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
      {
      }
      aRxBuffer[0] = 0x00;
      if (HAL_I2C_Mem_Read_DMA(&I2cHandle, (uint16_t)EEPROM_HW_ADDRESS, Memory_Address, I2C_MEMADD_SIZE_8BIT, (uint8_t *)aRxBuffer, 1) != HAL_OK)
      {
        /* Reading process Error */
        Error_Handler();
        BSP_LED_On(LED3);
      }
      /* Wait for the end of the transfer */
      while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
      {
      }
      if(aRxBuffer[0] != aTxBuffer[0])
      {
        Error_Handler();
        BSP_LED_On(LED1);
      }
      
  }
      

    
  while(1)
  {
  }
  //Lesetest
  uint8_t data=0; 
  if((HAL_I2C_Master_Transmit(&I2cHandle, (slave_addr<<1), &reg, 1, TIMEOUT) != HAL_OK)|| (HAL_I2C_Master_Receive(&I2cHandle, slave_addr, &data, 1, TIMEOUT) != HAL_OK))
  {
    data=0xFF; 
    BSP_LED_On(LED3);
  } 
  //Schreibtest
  uint8_t ret=0;
  reg = 0x00;
  data = 0x31;
  uint8_t buf[] = {reg, data}; 
  if(HAL_I2C_Master_Transmit(&I2cHandle, (slave_addr<<1), buf, 2, TIMEOUT) != HAL_OK)
  {
    ret = 0xFF;
  }
  //Rücklesen
  if((HAL_I2C_Master_Transmit(&I2cHandle, (slave_addr<<1), &reg, 1, TIMEOUT) != HAL_OK)|| (HAL_I2C_Master_Receive(&I2cHandle, slave_addr, &data, 1, TIMEOUT) != HAL_OK))
  {
    data=0xFF; 
    BSP_LED_Off(LED3);
  } 
   // while(HAL_I2C_Master_Transmit(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t*)aTxBuffer, TXBUFFERSIZE, 10000)!= HAL_OK)
  //{
    /* Error_Handler() function is called when Timeout error occurs.
       When Acknowledge failure occurs (Slave don't acknowledge its address)
       Master restarts communication */
    //if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
    //{
      //Error_Handler();
    //}
  //}
  while(1)
  {
    // 
      
  }
}/*end main*/
  /**
  * @brief  Tx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of DMA Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Turn LED1 on: Transfer in transmission process is correct */
  //BSP_LED_On(LED1);
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Turn LED2 on: Transfer in reception process is correct */
  //BSP_LED_On(LED2);
}

/**
  * @brief  I2C error callbacks.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Turn LED3 on: Transfer error in reception/transmission process */
  BSP_LED_On(LED4); 
}


#endif
    
/* Test the Debug UART -------------------------------------------------------*/  
#if defined (TEST_DEBUG_UART)   
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  //DEBUGUartHandle.Instance        = USART2;

  DEBUGUartHandle.Init.BaudRate   = 9600;
  DEBUGUartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  DEBUGUartHandle.Init.StopBits   = UART_STOPBITS_1;
  DEBUGUartHandle.Init.Parity     = UART_PARITY_NONE;
  DEBUGUartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  DEBUGUartHandle.Init.Mode       = UART_MODE_TX_RX;
  DEBUGUartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; 
  
  if( BSP_COM_DeInit(DEBUG_USART, &DEBUGUartHandle) != HAL_OK)//ToDo Check ob es funzt
  {
    Error_Handler();
  }
  /* es können folgende Callbacks aufgerufen werden: HAL_UART_ErrorCallback ToDo */
  if (BSP_COM_Init(DEBUG_USART, &DEBUGUartHandle) != HAL_OK)
  {
    Error_Handler();
  }
  //HAL_UART_Transmit(&DEBUGUartHandle, (uint8_t*)Debug_aTxBuffer,DEBUG_USART_TXBUFFERSIZE,2000);//Hat am 04.08.2017 gefunzt nur im Einzelschritt, wahrscheinlich hat µC Versorgungproblem
  if(HAL_UART_Transmit_IT(&DEBUGUartHandle, (uint8_t*)Debug_aTxBuffer, DEBUG_USART_TXBUFFERSIZE)!= HAL_OK)
  {
    Error_Handler();
  }
  while(1)
  {
  }
#endif  

/* Test the OWire UART -------------------------------------------------------*/  
#if defined (TEST_DEBUG_UART)  
  /* Test the OWire  */
  OWUartHandle.Init.BaudRate   = 9600;
  OWUartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  OWUartHandle.Init.StopBits   = UART_STOPBITS_1;
  OWUartHandle.Init.Parity     = UART_PARITY_NONE;
  OWUartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  OWUartHandle.Init.Mode       = UART_MODE_TX_RX;
  OWUartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; 
  
  if( BSP_COM_DeInit(OW_USART, &OWUartHandle) != HAL_OK)//ToDo Check ob es funzt
  {
    Error_Handler();
  }
  /* es können folgende Callbacks aufgerufen werden: HAL_UART_ErrorCallback ToDo */
  if (BSP_COM_Init(OW_USART, &OWUartHandle) != HAL_OK)
  {
    Error_Handler();
  }
  //HAL_UART_Transmit(&DEBUGUartHandle, (uint8_t*)Debug_aTxBuffer,DEBUG_USART_TXBUFFERSIZE,2000);//Hat am 04.08.2017 gefunzt nur im Einzelschritt, wahrscheinlich hat µC Versorgungproblem
  if(HAL_UART_Transmit_IT(&OWUartHandle, (uint8_t*)OW_aTxBuffer, OW_USART_TXBUFFERSIZE)!= HAL_OK)
  {
    Error_Handler();
  }
  while(1)
  {
  }
#endif
  /* Put UART peripheral in reception process ###########################*/  
  //if(HAL_UART_Receive_IT(&DEBUGUartHandle, (uint8_t *)Debug_aRxBuffer, 1) != HAL_OK)
  //{
    //Error_Handler();
  //}
 /* if(HAL_UART_DeInit(&DEBUGUartHandle) != HAL_OK)
  {
    Error_Handler();
  } */ 
 /*if(HAL_UART_Init(&DEBUGUartHandle) != HAL_OK)
  {
    Error_Handler();
  }*/
  //HAL_UART_Transmit(&DEBUGUartHandle, (uint8_t*)Debug_aTxBuffer,DEBUG_USART_TXBUFFERSIZE,2000); 
  
  
  
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PREDIV                         = 1
  *            PLLMUL                         = 6
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Select HSE Oscillator as PLL source */
  /* PLL configuration: PLLCLK = PREDIV1CLK * PLLMUL = 8 * 6 = 48 MHz */ 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK and PCLK1 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI/2)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 8000000
  *            PREDIV                         = 1
  *            PLLMUL                         = 12
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
void SystemClock_ConfigHSI(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* No HSE Oscillator on Nucleo, Activate PLL with HSI/2 as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}
/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;

  
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;
  
  
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == USER_BUTTON_PIN)
  {  
    //UserButtonStatus = 1;
    BSP_LED_Toggle(LED2);
    //HAL_Delay(500);
    //BSP_LED_Off(LED2);
    
  }
}
/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while(1)
  {
    /* Error if LED2 is slowly blinking (1 sec. period) */
    BSP_LED_Toggle(LED2); 
    HAL_Delay(250); 
  }  
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
