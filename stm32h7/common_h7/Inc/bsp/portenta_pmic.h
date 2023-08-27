/**
  ******************************************************************************
  * 
  * @file    portenta_h7.h
  * @author  Rainer, based on the work of Torsten Jaekel and Edwin Swensson
  * @brief   The portenta H7 board is powerde by a programmable power management 
  *          chip ( NXP's PM1550 ), which is normally done by the portenta bootloader. 
  *          Whenever this bootloader is erased/overwritten, the board will hang
  *          because the PMIC will not supply the 3V3 line, which is needed by most
  *          of the devices on the board, including the controller.
  *          This will be indicated by the orange "led of death" left beneath the
  *          USBC-connector. 
  *          The only way to bring back the board to work is to program the PMIC
  *          via I2C1 lines externally.
  *          
  *          So, under any circumstances, be sure that your portenta H7 boot sequence
  *          does contain the PMIC initialization code
  *
  ******************************************************************************
  */

#include "config/config.h"

int I2C_PMIC_Setup          (void);
int I2C_PMIC_Initialize     (void);
int I2C_PMIC_Reset          (void);

#if DEBUG_MODE > 0
    void I2C_PMIC_ReadOTP   (void);
    void PMIC_OTP_Dump      (void);
#endif;