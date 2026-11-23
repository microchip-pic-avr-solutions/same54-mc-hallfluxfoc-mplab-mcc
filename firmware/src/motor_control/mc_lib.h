/*******************************************************************************
  Motor Control Hardware Abstraction interface file

  Company:
    Microchip Technology Inc.

  File Name:
    mc_lib.h

  Summary:
    Header file for motor parameters initialization.

  Description:
    Header file for motor parameters initialization.
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
#ifndef MC_LIB_H
#define MC_LIB_H


/**********************- Included Files -*******************************/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <float.h>
#include <math.h>


/**********************- Macro Definitions -*******************************/ 



/**********************- Data Types -*******************************/
typedef struct{
    /* inputs */
    float * vAlpha;                 /* imposed alpha voltage, volt */
    float * vBeta;                  /* imposed beta voltage, volt */   
    float * vAlphaZ;                /* imposed alpha voltage, volt */
    float * vBetaZ;                 /* imposed beta voltage, volt */       
    float * iAlpha;                 /* alpha current, Amps */  
    float * iBeta;                  /* beta current, Amps */  
    float * iAlphaZ;                /* alpha current, Amps */  
    float * iBetaZ;                 /* beta current, Amps */     
    /* parameters */
    bool enablePhiEst;              /* Enable the estimation of rotor flux amplitude? */
    float inductance;               /* synchronous inductance, Henry */
    float resistance;               /* phase resistance */
    float ts;                       /* sampling period */
    float gPsi;                     /* the gain to estimate total flux */
    float gPhi;                     /* the gain to estimate rotor flux amplitude */
    float psiOl;                    /* total flux amplitude during open-loop alignment */
    float phi;                      /* rotor flux amplitude */
    float phiMax;                   /* the upper bound of phiEst */
    float phiMin;                   /* the lower bound of phiEst */
    float filA;
    float filB;    
    /* intermediate variables */
    float psiAlphaEst;              /* estimated alpha total flux */
    float psiBetaEst;               /* estimated beta total flux */  
    float phiAlphaEstRaw;           /* estimated alpha rotor flux, raw value */
    float phiBetaEstRaw;            /* estimated beta rotor flux, raw value */      
    float phiEst;                   /* estimated rotor flux amplitude */
    /* outputs */
    float * phiAlphaEst;            /* estimated alpha rotor flux */
    float * phiBetaEst;             /* estimated beta rotor flux */    
} T_P_FLUX_OBSERVER;

typedef struct{
    /* inputs */
    double * pstErr;             /* actual position - estimated position */
    /* inter-mediate */
    double pst5Ini;              /* known part of 5th derivative position */ 
    double pst4Ini;              /* known part of 4th derivative position */    
    double jerkIni;              /* known part of jerk */   
    double accIni;               /* known part of acceleration */
    double spdIni;               /* known part of speed */
    double pstIni;               /* known part of position */   
    double pst5Itg;
    double pst4Itg;
    double jerkItg;    
    double accItg;
    double spdItg;
    double pstItg;
    double pst5ItgIn;
    double pst4ItgIn;
    double jerkItgIn;      
    double accItgIn;
    double spdItgIn;
    double pstItgIn;
    T_INTEGRATOR_DOUBLE pst5;    
    T_INTEGRATOR_DOUBLE pst4;
    T_INTEGRATOR_DOUBLE jerk;    
    T_INTEGRATOR_DOUBLE acc;
    T_INTEGRATOR_DOUBLE spd; 
    T_INTEGRATOR_DOUBLE pst;    
    /* parameters */
    double pG2;
    double pG3;
    double pG4;
    double pG5;
    double pG6;   
    /* outputs */
    double * pst5Est;            /* estimated 5th derivative of position */
    double * pst4Est;            /* estimated 4th derivative of position */
    double * jerkEst;            /* estimated jerk */    
    double * accEst;             /* estimated acceleration */
    double * spdEst;             /* estimated speed */
    double * pstEst;             /* estimated position */
} T_POSITION_TRACKING_PLL_DBL;

typedef struct{
    /* input */
    double * K;
    double * itgPrd;               /* integration execution frequency */    
    /* inter-mediate */
    double lambda;
    double P;
    /* outputs */
    double * pst5ItgGain;
    double * pst4ItgGain;
    double * jerkItgGain;    
    double * accItgGain;
    double * spdItgGain;
    double * pstItgGain;
    double * pG2;
    double * pG3;
    double * pG4;
    double * pG5;  
    double * pG6;      
} T_PTPLL_CAIN_CAL_DBL;

typedef struct{
    /* inputs */
    float * pstErr;             /* actual position - estimated position */
    /* inter-mediate */
    float pst5Ini;              /* known part of 5th derivative position */ 
    float pst4Ini;              /* known part of 4th derivative position */    
    float jerkIni;              /* known part of jerk */   
    float accIni;               /* known part of acceleration */
    float spdIni;               /* known part of speed */
    float pstIni;               /* known part of position */   
    float pst5Itg;
    float pst4Itg;
    float jerkItg;    
    float accItg;
    float spdItg;
    float pstItg;
    float pst5ItgIn;
    float pst4ItgIn;
    float jerkItgIn;      
    float accItgIn;
    float spdItgIn;
    float pstItgIn;
    T_INTEGRATOR pst5;    
    T_INTEGRATOR pst4;
    T_INTEGRATOR jerk;    
    T_INTEGRATOR acc;
    T_INTEGRATOR spd; 
    T_INTEGRATOR pst;    
    /* parameters */
    float pG2;
    float pG3;
    float pG4;
    float pG5;
    float pG6;   
    /* outputs */
    float * pst5Est;            /* estimated 5th derivative of position */
    float * pst4Est;            /* estimated 4th derivative of position */
    float * jerkEst;            /* estimated jerk */    
    float * accEst;             /* estimated acceleration */
    float * spdEst;             /* estimated speed */
    float * pstEst;             /* estimated position */
} T_POSITION_TRACKING_PLL_FLT;

typedef struct{
    /* input */
    float * K;
    float * itgPrd;             /* integration execution frequency */    
    /* inter-mediate */
    float lambda;
    float P;
    /* outputs */
    float * pst5ItgGain;
    float * pst4ItgGain;
    float * jerkItgGain;    
    float * accItgGain;
    float * spdItgGain;
    float * pstItgGain;
    float * pG2;
    float * pG3;
    float * pG4;
    float * pG5;  
    float * pG6;      
} T_PTPLL_CAIN_CAL_FLT;

/**********************- Variable Declarations -*******************************/



/**********************- Function Declarations -*******************************/
void MC_FluxObsEuler(T_P_FLUX_OBSERVER * ptPfo);
void MC_PtPllGainCalDbl(T_PTPLL_CAIN_CAL_DBL * ptGainCal);
void MC_PositionTrackingPllDbl(T_POSITION_TRACKING_PLL_DBL * ptPll);
void MC_PtPllGainCalFlt(T_PTPLL_CAIN_CAL_FLT * ptGainCal);
void MC_PositionTrackingPllFlt(T_POSITION_TRACKING_PLL_FLT * ptPll);


/******************************- Functions -***********************************/



#endif /* end of _MC_LIB_H */

/*******************************************************************************
 End of File
*/    
