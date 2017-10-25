/**
  ******************************************************************************
  * @file    stm32f0xx_Des_Eval.h
  * @author  M. Feile
  * @brief   This file contains definitions for:
  *          - LEDs  
  *          - Push-button
  *          - EEPROM 
  *          - UART1
  *          - UART2 
  *            available on Des_Eval
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F0XX_DES_EVAL_H
#define __STM32F0XX_DES_EVAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup BSP
  * @{
  */

/** @defgroup STM32F0XX_NUCLEO STM32F0XX-NUCLEO
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
   
/** 
* @brief check define for STM32F0XX_DES_EVAL board  
  */ 
#if defined (USE_STM32F030C8_DES_EVAL)
  /** @defgroup STM32F0XX Exported Types
    * @{
    */
      
  /**
 * @brief LED Types Definition
 */    
  typedef enum 
  {
    LED1 = 0,
    LED2 = 1,
    LED3 = 2,
    LED4 = 3,
  } Led_TypeDef;

  /**
 * @brief BUTTON Types Definition
 */
  typedef enum 
  {  
    BUTTON_USER = 0,
    /* Alias */
    BUTTON_KEY  = BUTTON_USER
  } Button_TypeDef;

  typedef enum 
  {  
    BUTTON_MODE_GPIO = 0,
    BUTTON_MODE_EXTI = 1
  } ButtonMode_TypeDef; 
  
  /**
   * @brief COM Types Definition
   */
  typedef enum 
  {
    OW_USART = 0,
    DEBUG_USART = 1
  } COM_TypeDef;

  /**
    * @}
    */ 

  /** @defgroup STM32F0XX Exported Constants
    * @{
    */ 

  /** @defgroup STM32F0XX_ Analog Switch Constants
    * @{
    */
  #define ANALOG_SW_PIN                    GPIO_PIN_11
  #define ANALOG_SW_PORT                   GPIOB
  #define ANALOG_SW_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()  
  #define ANALOG_SW_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOB_CLK_DISABLE()

  /** @defgroup STM32F0XX_ LED Constants
    * @{
    */
  #define LEDn                               4

  
  #define LED1_PIN                         GPIO_PIN_15
  #define LED1_GPIO_PORT                   GPIOB
  #define LED1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()  
  #define LED1_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()

  #define LED2_PIN                         GPIO_PIN_14
  #define LED2_GPIO_PORT                   GPIOB
  #define LED2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()  
  #define LED2_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()

  #define LED3_PIN                         GPIO_PIN_13
  #define LED3_GPIO_PORT                   GPIOB
  #define LED3_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()  
  #define LED3GPIO_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()

  #define LED4_PIN                         GPIO_PIN_12
  #define LED4_GPIO_PORT                   GPIOB
  #define LED4_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()  
  #define LED4_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()
    
  #define LEDx_GPIO_CLK_ENABLE(__INDEX__)   do { if((__INDEX__) == 0) LED1_GPIO_CLK_ENABLE(); if((__INDEX__) == 1) LED2_GPIO_CLK_ENABLE(); if((__INDEX__) == 2) LED3_GPIO_CLK_ENABLE(); if((__INDEX__) == 3) LED4_GPIO_CLK_ENABLE();} while(0)
  #define LEDx_GPIO_CLK_DISABLE(__INDEX__)  do { if((__INDEX__) == 0) LED1_GPIO_CLK_DISABLE(); if((__INDEX__) == 1) LED2_GPIO_CLK_DISABLE(); if((__INDEX__) == 2) LED3_GPIO_CLK_DISABLE(); if((__INDEX__) == 3) LED4_GPIO_CLK_DISABLE();} while(0)

  /**
    * @}
    */
      
  /** @defgroup STM32F0XX TIM Constants
    * @{
    */
  /* Definition for TIMx clock resources */
  #define TIMx_CLK_ENABLE()              __HAL_RCC_TIM3_CLK_ENABLE() 
  
  /* Definition for TIMx Channel Pins */
  #define TIMx_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOB_CLK_ENABLE()
  #define TIMx_GPIO_PORT_CHANNEL1        GPIOB
  #define TIMx_GPIO_PIN_CHANNEL1         GPIO_PIN_1
  #define TIMx_GPIO_AF_CHANNEL1          GPIO_AF1_TIM3
  /**
    * @}
    */  
  /** @defgroup STM32F0XX UART Constants
    * @{
    */
  #define COMn                             2
  
  /**
  * @brief Definition for COM port1, connected to USART2
  */
  /* Definition for USART1 clock resources */
  #define OW_USART_IF                      USART1
  #define OW_USART_CLK_ENABLE()            __HAL_RCC_USART1_CLK_ENABLE()
  #define OW_USART_RX_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()
  #define OW_USART_TX_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()

  #define OW_USART_FORCE_RESET()           __HAL_RCC_USART1_FORCE_RESET()
  #define OW_USART_RELEASE_RESET()         __HAL_RCC_USART1_RELEASE_RESET()

  /* Definition for USARTx Pins */
  #define OW_USART_TX_PIN                  GPIO_PIN_9
  #define OW_USART_TX_GPIO_PORT            GPIOA
  #define OW_USART_TX_AF                   GPIO_AF1_USART1
  #define OW_USART_RX_PIN                  GPIO_PIN_10
  #define OW_USART_RX_GPIO_PORT            GPIOA
  #define OW_USART_RX_AF                   GPIO_AF1_USART1
  #define OW_USART_AF                      GPIO_AF1_USART1         

  /* Definition for USARTx's NVIC */
  #define OW_USART_IRQn                    USART1_IRQn
  #define OW_USART_IRQHandler              USART1_IRQHandler

  /* Size of Trasmission buffer */
  #define OW_USART_TXBUFFERSIZE            (COUNTOF(OW_aTxBuffer) - 1)
  /* Size of Reception buffer */
  #define OW_USART_RXBUFFERSIZE            OW_USART_TXBUFFERSIZE
  
  /* Definition for USART2 clock resources */
  #define DEBUG_USART_IF                   USART2
  #define DEBUG_USART_CLK_ENABLE()         __HAL_RCC_USART2_CLK_ENABLE()
  #define DEBUG_USART_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
  #define DEBUG_USART_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

  #define DEBUG_USART_FORCE_RESET()        __HAL_RCC_USART2_FORCE_RESET()
  #define DEBUG_USART_RELEASE_RESET()      __HAL_RCC_USART2_RELEASE_RESET()

  /* Definition for USARTx Pins */
  #define DEBUG_USART_TX_PIN               GPIO_PIN_2
  #define DEBUG_USART_TX_GPIO_PORT         GPIOA
  #define DEBUG_USART_TX_AF                GPIO_AF1_USART2
  #define DEBUG_USART_RX_PIN               GPIO_PIN_3
  #define DEBUG_USART_RX_GPIO_PORT         GPIOA
  #define DEBUG_USART_RX_AF                GPIO_AF1_USART2
  #define DEBUG_USART_AF                   GPIO_AF1_USART2 
  /* Definition for USARTx's NVIC */
  #define DEBUG_USART_IRQn                 USART2_IRQn
  #define DEBUG_USART_IRQHandler           USART2_IRQHandler
  
  /* enable/disable Interrupts */
  //__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);

  /* Size of Trasmission buffer */
  #define DEBUG_USART_TXBUFFERSIZE         (COUNTOF(Debug_aTxBuffer) - 1)
  /* Size of Reception buffer */
  #define DEBUG_USART_RXBUFFERSIZE         DEBUG_USART_TXBUFFERSIZE

  /* Definition of USART Clock enable */
  #define COMx_CLK_ENABLE(__INDEX__)              do { if((__INDEX__) == OW_USART) OW_USART_CLK_ENABLE(); if((__INDEX__) == DEBUG_USART) DEBUG_USART_CLK_ENABLE();} while(0)
  #define COMx_CLK_DISABLE(__INDEX__)             do { if((__INDEX__) == OW_USART) OW_USART_CLK_DISABLE(); if((__INDEX__) == DEBUG_USART) DEBUG_USART_CLK_DISABLE();} while (0)
  
  #define AFIOCOMx_CLK_ENABLE(__INDEX__)          do { if((__INDEX__) == OW_USART) AFIOCOM1_CLK_ENABLE(); if((__INDEX__) == DEBUG_USART) AFIOCOM1_CLK_ENABLE();} while(0)
  #define AFIOCOMx_CLK_DISABLE(__INDEX__)         (((__INDEX__) == OW_USART) ? AFIOCOM1_CLK_DISABLE() : 0)

  #define AFIOCOMx_REMAP(__INDEX__)               (((__INDEX__) == OW_USART) ? (AFIO->MAPR |= (AFIO_MAPR_USART2_REMAP)) : 0)

  #define COMx_TX_GPIO_CLK_ENABLE(__INDEX__)      do { if((__INDEX__) == OW_USART) OW_USART_TX_GPIO_CLK_ENABLE(); if((__INDEX__) == DEBUG_USART) DEBUG_USART_TX_GPIO_CLK_ENABLE();} while(0)
  #define COMx_TX_GPIO_CLK_DISABLE(__INDEX__)     do { if((__INDEX__) == OW_USART) OW_USART_TX_GPIO_CLK_DISABLE(); if((__INDEX__) == DEBUG_USART) DEBUG_USART_TX_GPIO_CLK_DISABLE();} while(0)

  #define COMx_RX_GPIO_CLK_ENABLE(__INDEX__)      do { if((__INDEX__) == OW_USART) OW_USART_RX_GPIO_CLK_ENABLE(); if((__INDEX__) == DEBUG_USART) DEBUG_USART_RX_GPIO_CLK_ENABLE();} while(0)
  #define COMx_RX_GPIO_CLK_DISABLE(__INDEX__)     do { if((__INDEX__) == OW_USART) OW_USART_RX_GPIO_CLK_DISABLE(), if((__INDEX__) == DEBUG_USART) DEBUG_USART_RX_GPIO_CLK_DISABLE();} while(0)
    
  #define COMx_FORCE_RESET(__INDEX__)             do { if((__INDEX__) == OW_USART) OW_USART_FORCE_RESET(); if((__INDEX__) == DEBUG_USART) DEBUG_USART_FORCE_RESET();} while(0)  
  #define COMx_RELEASE_RESET(__INDEX__)           do { if((__INDEX__) == OW_USART) OW_USART_RELEASE_RESET(); if((__INDEX__) == DEBUG_USART) DEBUG_USART_RELEASE_RESET();} while(0)  

  /* Exported macro ------------------------------------------------------------*/
  #define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

  /**
    * @}
    */   
  
  /** @defgroup STM32F0XX BUTTON Constants
    * @{
    */  
  #define BUTTONn                            1

  /**
    * @brief User push-button
    */
  #define USER_BUTTON_PIN                         GPIO_PIN_13
  #define USER_BUTTON_GPIO_PORT                   GPIOC
  #define USER_BUTTON_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()   
  #define USER_BUTTON_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOC_CLK_DISABLE()  
  #define USER_BUTTON_EXTI_LINE                   GPIO_PIN_13
  #define USER_BUTTON_EXTI_IRQn                   EXTI4_15_IRQn
  /* Aliases */
  //MF #define KEY_BUTTON_PIN                        USER_BUTTON_PIN
  //MF #define KEY_BUTTON_GPIO_PORT                  USER_BUTTON_GPIO_PORT
  //MF #define KEY_BUTTON_GPIO_CLK_ENABLE()          USER_BUTTON_GPIO_CLK_ENABLE()
  //MF #define KEY_BUTTON_GPIO_CLK_DISABLE()         USER_BUTTON_GPIO_CLK_DISABLE()
  //MF #define KEY_BUTTON_EXTI_LINE                  USER_BUTTON_EXTI_LINE
  //MF #define KEY_BUTTON_EXTI_IRQn                  USER_BUTTON_EXTI_IRQn

  #define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == BUTTON_USER) USER_BUTTON_GPIO_CLK_ENABLE();} while(0)
  #define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == BUTTON_USER) ? USER_BUTTON_GPIO_CLK_DISABLE() : 0)
  /**
    * @}
    */ 

  /** @defgroup STM32F0XX_NUCLEO_BUS BUS Constants
    * @{
    */ 
  /*###################### SPI1 ###################################*/
  #define NUCLEO_SPIx                                 SPI1
  #define NUCLEO_SPIx_CLK_ENABLE()                  __HAL_RCC_SPI1_CLK_ENABLE()

  #define NUCLEO_SPIx_SCK_AF                          GPIO_AF0_SPI1
  #define NUCLEO_SPIx_SCK_GPIO_PORT                   GPIOA
  #define NUCLEO_SPIx_SCK_PIN                         GPIO_PIN_5
  #define NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
  #define NUCLEO_SPIx_SCK_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()

  #define NUCLEO_SPIx_MISO_MOSI_AF                    GPIO_AF0_SPI1
  #define NUCLEO_SPIx_MISO_MOSI_GPIO_PORT             GPIOA
  #define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
  #define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOA_CLK_DISABLE()
  #define NUCLEO_SPIx_MISO_PIN                        GPIO_PIN_6
  #define NUCLEO_SPIx_MOSI_PIN                        GPIO_PIN_7
  /* Maximum Timeout values for flags waiting loops. These timeouts are not based
     on accurate values, they just guarantee that the application will not remain
     stuck if the SPI communication is corrupted.
     You may modify these timeout values depending on CPU frequency and application
     conditions (interrupts routines ...). */   
  #define NUCLEO_SPIx_TIMEOUT_MAX                   1000


  /**
    * @brief  SD Control Lines management
    */
  #define SD_CS_LOW()       HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET)
  #define SD_CS_HIGH()      HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET)

  /**
    * @brief  LCD Control Lines management
    */
  #define LCD_CS_LOW()      HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET)
  #define LCD_CS_HIGH()     HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET)
  #define LCD_DC_LOW()      HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
  #define LCD_DC_HIGH()     HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET)
       
  /**
    * @brief  SD Control Interface pins (shield D4)
    */
  #define SD_CS_PIN                                 GPIO_PIN_5
  #define SD_CS_GPIO_PORT                           GPIOB
  #define SD_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOB_CLK_ENABLE()
  #define SD_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOB_CLK_DISABLE()

  /**
    * @brief  LCD Control Interface pins (shield D10)
    */
  #define LCD_CS_PIN                                 GPIO_PIN_6
  #define LCD_CS_GPIO_PORT                           GPIOB
  #define LCD_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOB_CLK_ENABLE()
  #define LCD_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOB_CLK_DISABLE()
      
  /**
    * @brief  LCD Data/Command Interface pins
    */
  #define LCD_DC_PIN                                 GPIO_PIN_9
  #define LCD_DC_GPIO_PORT                           GPIOA
  #define LCD_DC_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOA_CLK_ENABLE()
  #define LCD_DC_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOA_CLK_DISABLE()

  /*##################### ADC1 ###################################*/
  /**
    * @brief  ADC Interface pins
    *         used to detect motion of Joystick available on Adafruit 1.8" TFT shield
    */
  #define NUCLEO_ADCx                                 ADC1
  #define NUCLEO_ADCx_CLK_ENABLE()                  __HAL_RCC_ADC1_CLK_ENABLE()
  #define NUCLEO_ADCx_CLK_DISABLE()                 __HAL_RCC_ADC1_CLK_DISABLE()

  #define NUCLEO_ADCx_GPIO_PORT                       GPIOB
  #define NUCLEO_ADCx_GPIO_PIN                        GPIO_PIN_0
  #define NUCLEO_ADCx_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOB_CLK_ENABLE()
  #define NUCLEO_ADCx_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOB_CLK_DISABLE()

  /**
    * @}
    */

  /**
    * @}
    */
#else
 #error Defintion missing
#endif

/** @defgroup STM32F0XX_NUCLEO_Exported_Functions Exported Functions
  * @{
  */
uint32_t  BSP_GetVersion(void);
HAL_StatusTypeDef BSP_PWM_Init(TIM_HandleTypeDef* htim);
HAL_StatusTypeDef BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef* huart); 
HAL_StatusTypeDef BSP_COM_DeInit(COM_TypeDef COM, UART_HandleTypeDef* huart);

/** @defgroup STM32F0XX_NTC_Functions
  * @{
  */ 
void      BSP_NTC_Init(void);
void      BSP_NTC_DeInit(void);
void      BSP_NTC_Enable(void);
void      BSP_NTC_Disable(void);
//void      BSP_NTC_Toggle(void);
/**
  * @}
  */ 

/** @defgroup STM32F0XX_ANALOG_SWITCH_Functions
  * @{
  */ 
void      BSP_ANALOG_SW_Init(void);
void      BSP_ANALOG_SW_DeInit(void);
void      BSP_ANALOG_SW_B1_A(void);
void      BSP_ANALOG_SW_B0_A(void);
void      BSP_ANALOG_SW_Toggle(void);
/**
  * @}
  */ 

/** @defgroup STM32F0XX_NUCLEO_LED_Functions  LED Functions
  * @{
  */ 
void      BSP_LED_Init(Led_TypeDef Led);
void      BSP_LED_DeInit(Led_TypeDef Led);
void      BSP_LED_On(Led_TypeDef Led);
void      BSP_LED_Off(Led_TypeDef Led);
void      BSP_LED_Toggle(Led_TypeDef Led);
/**
  * @}
  */ 

/** @addtogroup STM32F0XX_NUCLEO_BUTTON_Functions
  * @{
  */                
void      BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void      BSP_PB_DeInit(Button_TypeDef Button);
uint32_t  BSP_PB_GetState(Button_TypeDef Button);
#if defined(HAL_ADC_MODULE_ENABLED)
uint8_t          BSP_JOY_Init(void);
JOYState_TypeDef BSP_JOY_GetState(void);
void             BSP_JOY_DeInit(void);
#endif /* HAL_ADC_MODULE_ENABLED */


/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F0XX_DES_EVAL_H */

    
/************************ (C) COPYRIGHT Varta *****END OF FILE****/

