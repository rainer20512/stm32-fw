/*
  u8x8cb.c
  
  STM32L031
  
  
*/
#include "config/config.h"

#if USE_U8G2 > 0

#include "u8x8.h"
#include "dev/io_dev.h"
#include "dev/timer_dev.h"

/*
  I2C:
    PA9: Clock
    PA10: Data
*/

#ifdef KEKS 
uint8_t u8x8_gpio_and_delay_stm32l4_sw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
      /* only support for software I2C*/
    
      RCC->IOPENR |= RCC_IOPENR_IOPAEN;		/* Enable clock for GPIO Port A */
      __NOP();
      __NOP();
      
      GPIOA->MODER &= ~GPIO_MODER_MODE10;	/* clear mode for PA10 */
      //GPIOA->MODER |= GPIO_MODER_MODE10_0;	/* Output mode for PA10 */
      GPIOA->OTYPER &= ~GPIO_OTYPER_OT_10;	/* no open drain for PA10 */
      GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEED10;	/* low speed for PA10 */
      GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD10;	/* no pullup/pulldown for PA10 */
      //GPIOA->BSRR = GPIO_BSRR_BS_10;		/* atomic set PA10 */
    
      GPIOA->MODER &= ~GPIO_MODER_MODE9;	/* clear mode for PA9 */
      //GPIOA->MODER |= GPIO_MODER_MODE9_0;	/* Output mode for PA9 */
      GPIOA->OTYPER &= ~GPIO_OTYPER_OT_9;	/* no open drain for PA9 */
      GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEED9;	/* low speed for PA9 */
      GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD9;	/* no pullup/pulldown for PA9 */
      //GPIOA->BSRR = GPIO_BSRR_BS_9;		/* atomic set PA9 */
        
      break;
    case U8X8_MSG_DELAY_NANO:
      /* not required for SW I2C */
      break;
    
    case U8X8_MSG_DELAY_10MICRO:
      /* not used at the moment */
      break;
    
    case U8X8_MSG_DELAY_100NANO:
      /* not used at the moment */
      break;
   
    case U8X8_MSG_DELAY_MILLI:
      delay_micro_seconds(arg_int*1000UL);
      break;
    case U8X8_MSG_DELAY_I2C:
      /* arg_int is 1 or 4: 100KHz (5us) or 400KHz (1.25us) */
      delay_micro_seconds(arg_int<=2?5:1);
      break;
    
    case U8X8_MSG_GPIO_I2C_CLOCK:
      
      if ( arg_int == 0 )
      {
	GPIOA->MODER &= ~GPIO_MODER_MODE9;	/* clear mode for PA10 */
	GPIOA->MODER |= GPIO_MODER_MODE9_0;	/* Output mode for PA10 */
	GPIOA->BSRR = GPIO_BSRR_BR_9;		/* atomic clr PA9 */
      }
      else
      {
	//GPIOA->BSRR = GPIO_BSRR_BS_9;		/* atomic set PA9 */
	GPIOA->MODER &= ~GPIO_MODER_MODE9;	/* clear mode for PA9: input mode */
      }
      break;
    case U8X8_MSG_GPIO_I2C_DATA:
      
      if ( arg_int == 0 )
      {
	GPIOA->MODER &= ~GPIO_MODER_MODE10;	/* clear mode for PA10 */
	GPIOA->MODER |= GPIO_MODER_MODE10_0;	/* Output mode for PA10 */
	GPIOA->BSRR = GPIO_BSRR_BR_10;		/* atomic clr PA10 */
      }
      else
      {
	//GPIOA->BSRR = GPIO_BSRR_BS_10;		/* atomic set PA10 */
	// input mode
	GPIOA->MODER &= ~GPIO_MODER_MODE10;	/* clear mode for PA10: input mode */
      }
      break;
/*
    case U8X8_MSG_GPIO_MENU_SELECT:
      u8x8_SetGPIOResult(u8x8, Chip_GPIO_GetPinState(LPC_GPIO, KEY_SELECT_PORT, KEY_SELECT_PIN));
      break;
    case U8X8_MSG_GPIO_MENU_NEXT:
      u8x8_SetGPIOResult(u8x8, Chip_GPIO_GetPinState(LPC_GPIO, KEY_NEXT_PORT, KEY_NEXT_PIN));
      break;
    case U8X8_MSG_GPIO_MENU_PREV:
      u8x8_SetGPIOResult(u8x8, Chip_GPIO_GetPinState(LPC_GPIO, KEY_PREV_PORT, KEY_PREV_PIN));
      break;
    
    case U8X8_MSG_GPIO_MENU_HOME:
      u8x8_SetGPIOResult(u8x8, Chip_GPIO_GetPinState(LPC_GPIO, KEY_HOME_PORT, KEY_HOME_PIN));
      break;
*/
    default:
      u8x8_SetGPIOResult(u8x8, 1);
      break;
  }
  return 1;
}

#endif
/*
  SPI:
    PB02: CD     IO03
    PA15: nSEL   IO06
    PB05: MOSI   IO05
    N/C : Reset   
    PB03: SCK    IO04
*/

#define G8X2_DNC  3
#define G8X2_SCK  4
#define G8X2_MOSI 5
#define G8X2_NSEL 6


uint8_t u8x8_gpio_and_delay_stm32l4_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
      // IO pins are initialized by IO device, nothing to do here 
      break;
    case U8X8_MSG_DELAY_NANO:
      /* required for SPI, but seems to work without any delay (at least with 2MHz system clock) */
      //delay_micro_seconds(1);
      break;
    
    case U8X8_MSG_DELAY_10MICRO:
      BASTMR_DelayUs(arg_int*10UL);
      break;
    
    case U8X8_MSG_DELAY_100NANO:
      /* not used at the moment */
      BASTMR_DelayUs((arg_int+9)/10);
      break;
   
    case U8X8_MSG_DELAY_MILLI:
      BASTMR_DelayUs(arg_int*1000UL);
      break;
    
    case U8X8_MSG_DELAY_I2C:
      /* arg_int is 1 or 4: 100KHz (5us) or 400KHz (1.25us) */
      BASTMR_DelayUs(arg_int<=2?5:1);
      break;
    
    case U8X8_MSG_GPIO_SPI_CLOCK:      
      if ( arg_int == 0 )
	IO_OutputLow(G8X2_SCK);
      else
	IO_OutputHigh(G8X2_SCK);
      break;
      
    case U8X8_MSG_GPIO_SPI_DATA:
      if ( arg_int == 0 )
	IO_OutputLow(G8X2_MOSI);
      else
	IO_OutputHigh(G8X2_MOSI);
      break;
      
   case U8X8_MSG_GPIO_CS:				// CS (chip select) pin: Output level in arg_int
      if ( arg_int == 0 )
	IO_OutputLow(G8X2_NSEL);
      else
	IO_OutputHigh(G8X2_NSEL);
      break;
      
    case U8X8_MSG_GPIO_DC:				// DC (data/cmd, A0, register select) pin: Output level in arg_int
      if ( arg_int == 0 )
	IO_OutputLow(G8X2_DNC);
      else
	IO_OutputHigh(G8X2_DNC);
      break;
      
    case U8X8_MSG_GPIO_RESET:			// Reset pin: Output level in arg_int/*
        // Reset not connected
      break;
      
    default:
      u8x8_SetGPIOResult(u8x8, 1);
      break;
  }
  return 1;
}

#ifdef KEKS

uint8_t u8x8_byte_stm32l0_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) 
{
  uint8_t *data;
  switch(msg) {
    case U8X8_MSG_BYTE_SEND:
      data = (uint8_t *)arg_ptr;
      while( arg_int > 0 ) 
     {
       while( ( SPI1->SR & SPI_SR_TXE ) == 0 )
          ;
        *(uint8_t *)&(SPI1->DR) = *data;
        data++;
        arg_int--;
      }  
      break;
    case U8X8_MSG_BYTE_INIT:
      /* enable clock for SPI */
      RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;       /* enable SPI1 */
      /* set a predivision if the system clock is too high, not required for 2MHz */
      RCC->CFGR &= ~RCC_CFGR_PPRE2_Msk;
      //RCC->CFGR |= RCC_CFGR_PPRE2_DIV4;
    
      /* output setup is already done, just enable the alternate mode here */

      GPIOA->MODER &= ~GPIO_MODER_MODE7;	/* clear mode */
      GPIOA->MODER |= GPIO_MODER_MODE7_1;	/* Alternate function mode */
      GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL7_Msk;     /* clear af mode */
      GPIOA->AFR[0] |= 0 << GPIO_AFRL_AFRL7_Pos;  /* assign af mode (which is 0 for SPI) */
    
      GPIOA->MODER &= ~GPIO_MODER_MODE5;	/* clear mode */
      GPIOA->MODER |= GPIO_MODER_MODE5_1;	/* Alternate function mode */
      GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL5_Msk;     /* clear af mode */
      GPIOA->AFR[0] |= 0 << GPIO_AFRL_AFRL5_Pos;  /* assign af mode (which is 0 for SPI) */

      /* setup and enable SPI subsystem */
      /* Note: We assume SPI mode 0 for the displays (true for the most modern displays), so CPHA and CPOL are forced to 0 here. SPI mode is here: u8x8->display_info->spi_mode */
        
      SPI1->CR1 = SPI_CR1_MSTR                                                    // master
              //| SPI_CR1_BIDIMODE
              //| SPI_CR1_BIDIOE
              | SPI_CR1_SSM 
              | SPI_CR1_SSI
              //| SPI_CR1_BR_0
              //| SPI_CR1_BR_1
              //| SPI_CR1_BR_2          // speed
              //| SPI_CR1_CPHA
              //| SPI_CR1_CPOL
              ;
      SPI1->CR2 = 0;                                                            // SPI_CR2_TXDMAEN = transmit DMA enable
      SPI1->CR1 |= SPI_CR1_SPE;                                           // SPI enable
              
      u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
      break;
    case U8X8_MSG_BYTE_SET_DC:
      while ( SPI1->SR & SPI_SR_BSY )           // wait for transfer completion
          ;
      u8x8_gpio_SetDC(u8x8, arg_int);
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
      u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level);  
      u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO, u8x8->display_info->post_chip_enable_wait_ns, NULL);
      break;
    case U8X8_MSG_BYTE_END_TRANSFER:      
      while ( SPI1->SR & SPI_SR_BSY )           // wait for transfer completion
          ;
      u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO, u8x8->display_info->pre_chip_disable_wait_ns, NULL);
      u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
      break;
    default:
      return 0;
  }  
  return 1;
}
#endif
#ifdef KEKS
uint8_t u8x8_gpio_and_delay_template(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:	// called once during init phase of u8g2/u8x8
      break;							// can be used to setup pins
    case U8X8_MSG_DELAY_NANO:			// delay arg_int * 1 nano second
      break;    
    case U8X8_MSG_DELAY_100NANO:		// delay arg_int * 100 nano seconds
      break;
    case U8X8_MSG_DELAY_10MICRO:		// delay arg_int * 10 micro seconds
      break;
    case U8X8_MSG_DELAY_MILLI:			// delay arg_int * 1 milli second
      break;
    case U8X8_MSG_DELAY_I2C:				// arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
      break;							// arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
    case U8X8_MSG_GPIO_D0:				// D0 or SPI clock pin: Output level in arg_int
    //case U8X8_MSG_GPIO_SPI_CLOCK:
      break;
    case U8X8_MSG_GPIO_D1:				// D1 or SPI data pin: Output level in arg_int
    //case U8X8_MSG_GPIO_SPI_DATA:
      break;
    case U8X8_MSG_GPIO_D2:				// D2 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_D3:				// D3 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_D4:				// D4 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_D5:				// D5 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_D6:				// D6 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_D7:				// D7 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_E:				// E/WR pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_CS:				// CS (chip select) pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_DC:				// DC (data/cmd, A0, register select) pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_RESET:			// Reset pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_CS1:				// CS1 (chip select) pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_CS2:				// CS2 (chip select) pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_I2C_CLOCK:		// arg_int=0: Output low at I2C clock pin
      break;							// arg_int=1: Input dir with pullup high for I2C clock pin
    case U8X8_MSG_GPIO_I2C_DATA:			// arg_int=0: Output low at I2C data pin
      break;							// arg_int=1: Input dir with pullup high for I2C data pin
    case U8X8_MSG_GPIO_MENU_SELECT:
      u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
      break;
    case U8X8_MSG_GPIO_MENU_NEXT:
      u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
      break;
    case U8X8_MSG_GPIO_MENU_PREV:
      u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
      break;
    case U8X8_MSG_GPIO_MENU_HOME:
      u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
      break;
    default:
      u8x8_SetGPIOResult(u8x8, 1);			// default return value
      break;
  }
  return 1;
}
#endif

#endif // USE_U8G2 > 0