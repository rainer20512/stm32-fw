#ifndef __QENCODE_CONFIG_H
#define __QENCODE_CONFIG_H

/* No further includes neccessary, will be included by qencoder.c */

#if defined(USE_QENCODER)

  #if defined(USE_QENC1_TIM1)
    #define QTIM1               TIM1
    /* PA8,PA9 */
    #define QENC1TIM_CH1                        { GPIO_PIN_8, GPIOA, GPIO_AF1_TIM1, GPIO_PULLUP, "QEncTim1_Ch1" } 
    #define QENC1TIM_CH2                        { GPIO_PIN_9, GPIOA, GPIO_AF1_TIM1, GPIO_PULLUP, "QEncTim1_Ch2" } 
    #define QENC1TIM_IRQn                       TIM1_CC_IRQn
    #define QENC1TIM_IRQHandler                 TIM1_CC_IRQHandler
  #elif defined(USE_QENC1_TIM2)
    #define QTIM1               TIM2
    /* PA0, PA1 */
    #define QENC1TIM_CH1                        { GPIO_PIN_0, GPIOA, GPIO_AF1_TIM2, GPIO_PULLUP, "QEnc1Tim2_Ch1" } 
    #define QENC1TIM_CH2                        { GPIO_PIN_1, GPIOA, GPIO_AF1_TIM2, GPIO_PULLUP, "QEnc1Tim2_Ch2" } 
    #define QENC1TIM_IRQn                       TIM2_IRQn
    #define QENC1TIM_IRQHandler                 TIM2_IRQHandler
  #elif defined(USE_QENC1_TIM3)
    #define QTIM1               TIM3
    /* PA6, PA7 */
    #define QENC1TIM_CH1                        { GPIO_PIN_6, GPIOA, GPIO_AF2_TIM3, GPIO_PULLUP, "QEnc1Tim3_Ch1" } 
    #define QENC1TIM_CH2                        { GPIO_PIN_7, GPIOA, GPIO_AF2_TIM3, GPIO_PULLUP, "QEnc1Tim3_Ch2" } 
    #define QENC1TIM_IRQn                       TIM3_IRQn
    #define QENC1TIM_IRQHandler                 TIM3_IRQHandler
  #elif defined(USE_QENC1_TIM4)
    #define QTIM1               TIM4
    /* PB6,PB7 */
    #define QENC1TIM_CH1                        { GPIO_PIN_6, GPIOB, GPIO_AF2_TIM4, GPIO_PULLUP, "QEnc1Tim4_Ch1" } 
    #define QENC1TIM_CH2                        { GPIO_PIN_7, GPIOB, GPIO_AF2_TIM4, GPIO_PULLUP, "QEnc1Tim4_Ch2" } 
    #define QENC1TIM_IRQn                       TIM4_IRQn
    #define QENC1TIM_IRQHandler                 TIM4_IRQHandler
  #elif defined(USE_QENC1_TIM5)
    #define QTIM1               TIM5
    /* PA0, PA1 */
    #define QENC1TIM_CH1                        { GPIO_PIN_0, GPIOA, GPIO_AF2_TIM5, GPIO_PULLUP, "QEnc1Tim5_Ch1" } 
    #define QENC1TIM_CH2                        { GPIO_PIN_1, GPIOA, GPIO_AF2_TIM5, GPIO_PULLUP, "QEnc1Tim5_Ch2" } 
    #define QENC1TIM_IRQn                       TIM5_IRQn
    #define QENC1TIM_IRQHandler                 TIM5_IRQHandler
  #elif defined(USE_QENC1_TIM6)
    #define QTIM1               TIM8
    /* PC6, PC7 */
    #define QENC1TIM_CH1                        { GPIO_PIN_6, GPIOC, GPIO_AF3_TIM8, GPIO_PULLUP, "QEnc1Tim8_Ch1" } 
    #define QENC1TIM_CH2                        { GPIO_PIN_7, GPIOC, GPIO_AF3_TIM8, GPIO_PULLUP, "QEnc1Tim8_Ch2" } 
    #define QENC1TIM_IRQn                       TIM8_CC_IRQn
    #define QENC1TIM_IRQHandler                 TIM8_CC_IRQHandler
  #elif defined(USE_QENC1_LPTIM1)
    #error "LPTIM1 reserved for internal timer functions"
    #if 0
        #define QTIM1               LPTIM1
        /* PB5, PC2 */
        #define QENC1TIM_CH1                        { GPIO_PIN_5, GPIOB, GPIO_AF1_LPTIM1, "QEnc1Lptim1_Ch1" }
        #define QENC1TIM_CH2                        { GPIO_PIN_2, GPIOC, GPIO_AF1_LPTIM1, "QEnc1Lptim1_Ch2" }
        #define QENC1TIM_IRQn                       LPTIM1_IRQn
        #define QENC1TIM_IRQHandler                 LPTIM1_IRQHandler
    #endif
  #else
    #error "NO timer definition for Quadrature encoder 1"
  #endif  

  /* Use the Button IRQ Prio (relatively low prio) also for Timer Encoder Interrupt  */
  #define QENC1_IRQ                         { QENC1TIM_IRQn, BUTTON_IRQ_PRIO, 0 }

  /* Define the Push Button GPIO line, this will be configured for EXTI interrupt */
  #define PBTN_IRQ                          { BUTTON_IRQ_PRIO, 0 }
  #define QENC1_PBTN1                       { GPIO_PIN_8, GPIOA, GPIO_MODE_IT_RISING_FALLING, GPIO_SPEED_FREQ_LOW, GPIO_PULLUP, HW_IO_NORMAL, HW_INPUT, PBTN_IRQ, "QEnc1_Btn" }
  #define QENC1_DIVIDER                     2   /* Counts per notch */
#endif // if defined(USE_QENCODER)

#endif // __QENCODE_CONFIG_H