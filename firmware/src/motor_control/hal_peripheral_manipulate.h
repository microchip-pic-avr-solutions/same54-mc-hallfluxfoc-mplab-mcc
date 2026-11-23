/*******************************************************************************
 Motor Control Hardware Abstraction interface file

  Company:
    Microchip Technology Inc.

  File Name:
    hal_peripheral_manipulate.h

  Summary:
    Header file for position feedback hardware abstraction

  Description:
    This file contains the function mapping and channel mapping to provide
    hardware abstraction layer for motor control algorithm
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2020 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/

#ifndef HAL_PERIPHERAL_MANIPULATE_H
#define HAL_PERIPHERAL_MANIPULATE_H

/**********************- Included Files -*******************************/
#include <stdint.h>
#include "definitions.h" 
#include "device.h"


/**********************- Data Types -*******************************/


/**********************- Macros -*******************************/
#define HALL_Get()        (((PORT_REGS->GROUP[2].PORT_IN >> 16U)) & 7U)


/**********************- Function Declarations -*******************************/
void HAL_TC0_DirDown(void);



__STATIC_FORCEINLINE void Hal_PwmEnable( void )
{
    /*Disable PWM override*/
    while ((TCC0_REGS->TCC_SYNCBUSY & (TCC_SYNCBUSY_PATT_Msk)) == TCC_SYNCBUSY_PATT_Msk)
    {
        __NOP();
    }
    TCC0_REGS->TCC_PATT = (uint16_t)0x0000;
}


__STATIC_FORCEINLINE void HAL_PwmDisable( void )
{
/** Disable PWM override*/
   /*Override all PWM outputs to low*/
    while ((TCC0_REGS->TCC_SYNCBUSY & (TCC_SYNCBUSY_PATT_Msk)) == TCC_SYNCBUSY_PATT_Msk)
    {
       __NOP();
    }
    TCC0_REGS->TCC_PATT = (uint16_t)0x00FF;
}


__STATIC_FORCEINLINE void HAL_Motor1PhaseACurrentChannelSelect( void  )
{
    ADC0_ChannelSelect( ADC_POSINPUT_AIN0, ADC_NEGINPUT_GND);
}


__STATIC_FORCEINLINE void HAL_Motor1PhaseBCurrentChannelSelect( void  )
{
    ADC1_ChannelSelect( ADC_POSINPUT_AIN0, ADC_NEGINPUT_GND);
}

__STATIC_FORCEINLINE uint32_t HAL_HallTimerGet(void)
{
    /* Write command to force COUNT register read synchronization */
    TC0_REGS->COUNT32.TC_CTRLBSET |= (uint8_t)TC_CTRLBSET_CMD_READSYNC;

    while((TC0_REGS->COUNT32.TC_SYNCBUSY & TC_SYNCBUSY_CTRLB_Msk) == TC_SYNCBUSY_CTRLB_Msk)
    {
        /* Wait for Write Synchronization */
    }

    while((TC0_REGS->COUNT32.TC_CTRLBSET & TC_CTRLBSET_CMD_Msk) != (uint8_t)(0U))
    {
        /* Wait for CMD to become zero */
    }

    /* Read current count value */
    return TC0_REGS->COUNT32.TC_COUNT;
}


__STATIC_FORCEINLINE uint32_t HAL_HallCapturedTimerGet(void)
{
    uint32_t tmp1, tmp2;
    
    tmp1 = TC0_Capture32bitChannel0Get();
    tmp2 = tmp1;  /*dummy operation */
    /* Read CC register twice to make sure that the latest value is obtained. 
     * Caution: compiler may neglect the second reading. To avoid the unwanted 
     * optimization, call a (non-inline) function twice, instead of directly 
     * execute the code twice. */
    tmp2 = TC0_Capture32bitChannel0Get(); 
    
    if(0xffffffffu == tmp2){
        return tmp1;
    } else{
        return tmp2;
    }
        
#if 0
    /* The 32-bit overflow value, 0xffffffff, will be captured even if no HALL
     * jump is detected. In this case, the actual captured value at HALL jump,
     * is switched from CCBUF register to CC register. We read CC register again
     * to obtain the actual captured value. */
    if(0xffffffff == tmp1){
        tmp2 = TC0_REGS->COUNT32.TC_CC[0];
        return tmp2;
    } else{
        return tmp1;
    }
#endif
    

}


#endif /* _HAL_PERIPHERAL_MANIPULATE_H */

/*******************************************************************************
 End of File
*/    
    