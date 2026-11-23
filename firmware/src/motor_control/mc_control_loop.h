/*******************************************************************************
  Motor Control Loop Interrupt Handler

  Company:
    Microchip Technology Inc.

  File Name:
    mc_control_loop.h

  Summary:
    Motor control algorithm interface

  Description:
    This file contains the data structures and function prototypes of motor control loop.
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

#ifndef MC_CONTROL_LOOP_H
#define MC_CONTROL_LOOP_H

/**********************- Included Files -*******************************/
#include <stdint.h>
#include "definitions.h" 
#include "device.h"  
#include "hal_peripheral_manipulate.h"
#include "mc_library.h"
#include "mc_lib.h"


/**********************- Macro Definitions -********************************/
/* Enable motor 2 & motor 3. */
#if 0
#define MOTOR_2
#define MOTOR_3
#endif

/*-------- Encoder --------*/
/* Enable QEI and related function to obtain encoder feedbacks as reference.
 * This macro is mutual exclusive with any encoder-based applications! */
#if 0
#define ENCODER_FEEDBACK_INFORM                         
#endif
#ifdef ENCODER_FEEDBACK_INFORM
/* The FOC and speed loop use encoder feedbacks. */
#if 0
#define ENCODER_FEEDBACK_CONTROL                      
#endif
#endif

/*-------- Time measure --------*/
#if 0
#define CLOCK_MEASURE
#endif

/*-------- Phase loss detection test --------*/
#if 0
#define PHS_LOSS_TEST
#endif

/*-------- To enhance precision of fractional ePPR --------*/
#if 0
#define EPPR_ENHANCE                                        
#endif

/*-------- PWM Period --------*/
#define PER_PWM_1_MOTOR                                     (0.000050)   /* PWM period, sec */
#define PER_PWM_2_MOTOR                                     (0.000075)  /* PWM period, sec */
#define PER_PWM_3_MOTOR                                     (0.0000625) /* PWM period, sec */


/*-------- Common Parameters --------*/
#define PER_LOW_FREQ_LOOP                                   (0.001)     /* low-frequency loop period, sec */
#define PWM_CLK_FREQ                                        (120.0)     /* PWM clock frequency, MHz */

/*-------- Derived Parameters --------*/
#if (defined MOTOR_2) && (defined MOTOR_3)
    #define PER_PWM                                         PER_PWM_3_MOTOR
    /* MC_ISR_RATIO shall be consistent with the peripheral configuration.
     * For example, if the ISR is generated every 2 PWM cycles, then MC_ISR_RATIO = 2. */
    #define MC_ISR_RATIO                                    (2u)
#elif (defined MOTOR_2) || (defined MOTOR_3)
    #define PER_PWM                                         PER_PWM_2_MOTOR
    #define MC_ISR_RATIO                                    (1u)
#else
    #define PER_PWM                                         PER_PWM_1_MOTOR
    #define MC_ISR_RATIO                                    (1u)
#endif

#define PER_MC_ISR                                          ((double)MC_ISR_RATIO * PER_PWM)     /* ISR period, sec */
#define CNT_LOW_FREQ_LOOP                                   (PER_LOW_FREQ_LOOP / PER_MC_ISR + 0.5)        /* low-frequency loop period / PWM period */
#define CNT_PWM_PER                                         (uint16_t)(PER_PWM * 1000000.0 * PWM_CLK_FREQ * 0.5 + 0.5)
#define PER_PWM_US                                          (PER_PWM * 1000000.0)
#define CNT_ANG_CAL_WAIT                                    (uint32_t)(0.5 / PER_PWM + 0.5)

#ifdef EPPR_ENHANCE
    #define T_EPPR float 
    #define MODULUS MC_Modulus_Flt
#else
    #define T_EPPR int32_t 
    #define MODULUS MC_Modulus
#endif

/**********************- Data Types -*******************************/
typedef enum{
    ma_flux_foc = 0u,
    ma_hall_foc,
    ma_hall_flux_foc,
    ma_align_encoder_foc,
    ma_hall_encoder_foc        
} E_MOTOR_CONTROL_APPLICATION;

typedef enum{
    /* The following MC modes are mutual-exclusive. */
    md_alignment = 0u,   /* Only align the rotor for a given time. */
    md_open_loop,       /* Impose a rotating current vector without tracking rotor flux angle. */
    md_close_loop      /* The torque reference is given by the speed loop. */
} E_MOTOR_CONTROL_MODE;

typedef enum{
    fa_single_angle = 0u,    /* Coordinate transforms of current & voltage use the same angle. */
    fa_dual_angle,          /* Coordinate transforms of current & voltage use different angles. */
    fa_flux_single_angle    /* Coordinate transforms of current & voltage use the same angle. Sin&cos are obtained from flux components. */        
} E_FOC_ANGLE_MODE;

#if 0
typedef enum{
    ovm_disable = 0,    /* No over modulation */
    ovm_only_mode_1,    /* Over modulation only in mode 1 */
    ovm_mode_1_and_2    /* Over modulation in both mode 1 & mode 2 */
} E_OVER_MODULATION_LEVEL;

typedef enum{
    mtpa_off = 0,
    mtpa_on
} E_MTPA_FLAG;
#endif

typedef enum{
    mc_idle = 0u,
    mc_spin,
    mc_decelerate,        
    mc_fault        
} E_MOTOR_CONTROL_STATE;

typedef enum{
    mc_open_loop = 0u,
    mc_close_loop
} E_FOC_OL_STATE;

typedef enum{
    foc_low_speed_1 = 0u,
    foc_engage_pll,
    foc_low_speed_2,
    foc_high_speed
} E_FOC_CL_STATE;

typedef enum{
    cc_basic = 0u,
    cc_first_decouple,
    cc_decouple
} E_CUR_CTRL_STATE;

typedef enum{
    smo_speed_ref = 0u,
    smo_speed_fbk
} E_SMO_SPEED_STATE;

typedef enum{
    mtpa_positive_decay = 0u,
    mtpa_negative_estimate
} E_MTPA_STATE;

typedef enum{
    mc_alignment = 0u,
    mc_open_loop_acc,
    mc_open_loop_uniform,
    mc_loop_switch,
    /* For relative position sensor, two alignments are needed to guarantee a 
     * correct align angle. The 1st is "shadow alignment", the 2nd is 
     * "normal alignment". */        
    mc_shadow_alignment
} E_OPEN_LOOP_STATE;

typedef enum{
    btn_detect = 0u,
    btn_debounce        
} E_BUTTON_READ_STATE;

typedef enum{
    btn_pressed = false,
    btn_released = true
}E_BUTTON_INPUT;

typedef enum{
    hall_jump_less = 0u,
    hall_low_speed,
    hall_median_speed,        
    hall_high_speed        
} E_HALL_ANGLE_STATE;

typedef enum{
    ang_idle = 0u,
    ang_hall_wait,
    ang_hall_initial,
    ang_no_cali_yet,
    ang_hall_cali_done,
    ang_index_cali_done,
    ang_index_no_use
} E_ENC_ANGLE_STATE;

typedef struct{
    int16_t polePairs;      /* number of pole pairs */
    int32_t elePPR;         /* encoder PPR for each electric revolution */
    float ratedCurrent;     /* rated phase current peak value, Amps */
    float ratedSpeed;       /* rated mechanical speed, rad/s */
    float minSpeed;         /* minimum mechanical speed, rad/s */
    float Kt;               /* synchronized torque constant */
    float KtRlct;           /* reluctance torque constant */  
    float focSwitchSpd;     /* the speed to switch from low-speed FOC to high-speed FOC, rad/s */
    uint32_t pllStableCnt;  /* the loop count value to wait until the PLL gets stable */
} T_PMSM_PARA;   
#if 0
typedef struct{
    uint8_t state;
    bool firstEntry;
    uint32_t cnt;
} T_STATE_DATA;
#endif
typedef struct{
    bool flg;
    uint32_t cntLowFreqLoop;
    uint32_t cnt;
} T_LOOP_SYNC;

typedef struct{
    uint16_t cntStart;
    uint16_t cntEnd;
    uint16_t length;
    uint16_t lenMax;
    uint16_t lenMin;  
} T_CLOCK_MEASURE;

typedef struct{
    T_LOOP_SYNC lowFreqLoopSync;
    float angleIniTbl[4u];
    float hallAngleTbl[8u];  
    T_CLOCK_MEASURE clkLth1;
    T_CLOCK_MEASURE clkLth2;     
} T_COMMON_DATA;

typedef struct{
    float iU;
    float iV;
    float iW;
    float iD;
    float iQ;
    float iAlpha;
    float iBeta;
    float iAlphaLth;
    float iBetaLth;    
    T_ANGLE focAngle;
    T_VECTOR_PASSIVE_TRANSFORM parkTrans;
    T_CLARKE_TRANSFORM clarkeTrans;
#ifdef PHS_LOSS_TEST  /* for test */    
    float iVtest;
    float iWtest;
#endif
} T_FOC_CURRENT;

typedef struct{
    float vD;               /* per unit */
    float vQ;               /* per unit */
    float vAlpha;           /* per unit */
    float vBeta;            /* per unit */
    float vU;               /* per unit */
    float vV;               /* per unit */
    float vW;               /* per unit */
    T_ANGLE focAngle;  
    T_VECTOR_PASSIVE_TRANSFORM parkTrans;   
    T_CLARKE_TRANSFORM invClarkeTrans;    
} T_FOC_VOLTAGE;

typedef struct{
    float adcDataU;
    float adcDataV;
    float adcOffsetU;
    float adcOffsetV;
    T_FOC_CURRENT focCurrent;
    T_1ST_LPF_PT filAdcU;
    T_1ST_LPF_PT filAdcV;    
    float adc2amps;
    int16_t adcHalfCnt;
} T_CURRENT;

typedef struct{
    T_FOC_VOLTAGE focVoltage;
    T_SVPWM svpwm;
    int16_t adcDataVbus;
    float vBus;                     /* DC BUS Voltage, Volt */
    float adc2volt;
    int16_t fbCntU;                 /* detected U voltage in PWM count value */
    int16_t fbCntV;                 /* detected V voltage in PWM count value */
    int16_t fbCntW;                 /* detected W voltage in PWM count value */
    int16_t dummy;   
    uint16_t phsVoltMode;           /* Phase voltages are obtained by: 0 -- voltage references, or 1 -- HW detection. */
} T_VOLTAGE;

typedef struct{
    float iDref;
    float iQref;
    float * iDfbk;
    float * iQfbk;
    T_STATE_DATA * focState;   
    T_STATE_DATA curCtrlState;    
    bool flgDecouple;
    float compD;
    float compQ;
    T_PI_CONTROLLER iDpi;
    T_PI_CONTROLLER iQpi;
    T_FOC_DECOUPLE decouple;
    float * vDref;
    float * vQref;
} T_FOC_CURRENT_CONTROL;

typedef struct{
    float * ref;
    float * fbk;
    T_PI_CONTROLLER pi;
    float * torqueRef;
} T_SPEED_CONTROL;

typedef struct{
    float torqueRef;
    float nm2amps;
    float Kt;
    float KtRlct;
    float iDrefRaw;
    float iDref;
#if 0
    T_1ST_LPF_PT filIdRef; 
#else
    T_LINEAR_RAMP rampIdRef;
#endif
    float iQref;
    T_MTPA_ID_ESTIMATE mtpaIdEstimate;
    bool flgMtpa;
    T_STATE_DATA mtpaState;
} T_FOC_TORQUE_2_CURRENT;

#ifdef ENCODER_FEEDBACK_INFORM   
typedef struct{
    int32_t QEIcnt;
    int32_t QEIvel;
    float thIn;             /* rotor flux angle for Park transform, rad */
    float thLth;
    float thOut;            /* rotor flux angle for inverse Park transform, rad */
    float speed;            /* rotor shaft angular speed, rad/s */
    float dTh;
    float cnt2Rad;
    float cnt2RadPerSec;
    uint32_t cntPerEleRev;
    uint32_t rotorFluxAlignQeiCnt;
    int32_t speedSampleLoopCnt;
    int32_t speedSampleLoopCntMax;
} T_ROTOR_ANGLE_ENCODER_REF;
#endif

typedef struct{
    /* inputs */
    int32_t QEIcnt;
    int32_t QEIvel;
    int32_t QEIcap;             /* QEI captured counter value */
    int32_t QEIcapCorrect;      /* accepted QEIcap value */
    uint8_t hallVal;     
    /* intermediate */
    uint8_t hallValLth;      
    uint8_t hallValComb; 
    bool isHallCaliDone;
    bool isIndexDetected;
    T_EPPR angleCnt;
    int32_t accumulatedQeiCnt;  /* accumulated QEI counter value, with counting error eliminated by index signal */
    int32_t accumulatedErrorCnt;
    int32_t positionCnt;
    int32_t deltaCnt;
    int32_t speedSampleLoopCnt;     
    float hallAngle;
    float dTh;    
    float thLth;     
    T_STATE_DATA angleState;
    float angleEleErr;         /* PLL estimating error of rotor flux angle, rad */
    /* outputs */
    float thIn;                 /* rotor flux angle for Park transform, rad */
    float thOut;                /* rotor flux angle for inverse Park transform, rad */
    float speed;                /* rotor shaft angular speed, rad/s */   
    float eleSpd;               /* electric angular speed, rad/s */
    float acc;                  /* electric angular acceleration, rad/s^2 */
    float speedCnt;             /* rotor shaft angular speed obtained by QE counting, rad/s */     
    float accEleDbl;           /* electric angular acceleration estimated by the PLL, rad/s^2 */
    float spdEleDbl;           /* electric angular speed estimated by the PLL, rad/s */
    float angleEleDbl;         /* rotor flux angle estimated by the PLL, rad */
    /* parameters */
    float cnt2MechRad;
    float cnt2EleRad;
    float eleRad2Cnt;
    float cnt2MechRadPerSec;
    float cnt2EleRadPerSec;
    float oneByNumPolePairs;    
    int32_t hallOffsetCnt;
    int32_t indexAngleCnt;
    int32_t indexToleranceCnt;    
    T_EPPR ePPR;
    int32_t mPPR;
    int32_t speedSampleLoopCntMax;
    uint32_t rotorFluxAlignQeiCnt;
    E_ENC_ANGLE_STATE indexUseState;
} T_ROTOR_ANGLE_ENCODER;

typedef struct{
    /* inputs */
    uint8_t hallVal;            /* HALL value */ 
    uint32_t timer;             /* timer value */
    uint32_t timerIc;           /* the timer value latched by input capture peripheral */
    /* intermediate */
    uint8_t hallValZ;           /* the "hallVal" of the last loop cycle */      
    uint8_t hallValComb;        /* the combination of "hallVal" & "hallValZ" */
    bool isLegalHallJump;       /* Does a legal HALL jump just occurred? */
    bool isPtcHalljump;         /* Does a particular HALL jump just occurred? */
    uint16_t legalHallJumpCnt;  /* the number of HALL jumps */
    uint32_t timerZ;            /* the "timer" of the last loop cycle */ 
    uint32_t timerD;            /* timer - timerZ */
    uint32_t timerIcZ;          /* the "timerIc" of the last loop cycle */  
    uint32_t timerSinceHallJump;
    uint32_t timerBetweenHallJump; 
    uint32_t timerSincePtcHallJump;
    uint32_t timerIcPtc;
    uint32_t timerIcPtcZ;
    uint32_t timerBetweenPtcHallJump;  
    uint32_t timerTmp;
    float hallAngleRaw;         /* the angle obtained from the HALL value, rad */    
    float hallAngle;            /* the hall-angle after offset compensation, rad */
    float hallJumpAngleRaw;     /* rotor flux angle obtained from HALL jumps, rad */    
    float hallJumpAngle;        /* the hall-jump-angle after offset compensation, rad  */
    float hallJumpAngleZ;       /* the "hallJumpAngle" of the last loop cycle */
    float hallJumpAngleD;       /* hallJumpAngle - hallJumpAngleZ */ 
    float hallJumpSpdEle;       /* electric angular speed obtained by measuring the time between two HALL jumps, rad/s */   
    float hallJumpSpdSign;      /* the sign of angular speed */
    float ptcHallJumpAngle;     /* rotor flux angle obtained from particular HALL jumps, rad */
    float ptcHallJumpSpdEle;    /* electric angular speed obtained by measuring the time between two particular HALL jumps, rad/s */ 
    T_STATE_DATA hallState;
    /* parameters */
    float hallOffsetAngle;      /* the offset angle of the HALL table, rad */ 
    float oneByNumPolePair;     /* 1.0 / pole pair number */
    float spdUpThr;             /* the speed upper threshold to change state, rad/s */
    float spdLowThr;            /* the speed lower threshold to change state, rad/s */
    FLOAT timerT;               /* the period of timer clock, sec */
    uint32_t timerMax;
    float angleBetweenHallJump; /* the angle between recognized HALL jumps, rad */
    uint8_t hallJumpDtctMode;   /* 1u: each HALL jump is detected. 2u: HALL jumps are detected alternatively. */
    /* outputs */
    float angle;                /* rotor flux angle, rad */
    float speedEle;             /* electric angular speed, rad/s */
    float speedMech;            /* mechanical angular speed, rad/s */
} T_ROTOR_ANGLE_HALL;

typedef struct{
    /* inputs */
    int16_t * cntU;             /* imposed U voltage in PWM count value */
    int16_t * cntV;             /* imposed V voltage in PWM count value */
    int16_t * cntW;             /* imposed W voltage in PWM count value */
    uint16_t * phsVoltMode;      /* Phase voltages are obtained by: 0 -- voltage reference, or 1 -- HW detection. */
    float * vBus;               /* DC bus voltage, volt */    
#if 0
    float lthCntU;              /* imposed U voltage in PWM count value */
    float lthCntV;              /* imposed V voltage in PWM count value */
    float lthCntW;              /* imposed W voltage in PWM count value */   
#endif
    /* intermediate */
    float vU;                   /* imposed U voltage, volt */
    float vV;                   /* imposed V voltage, volt */
    float vW;                   /* imposed W voltage, volt */
    float vAlpha;               /* imposed alpha voltage, volt */
    float vBeta;                /* imposed beta voltage, volt */  
    float vAlphaZ1;             /* imposed alpha voltage of last sampling, volt */
    float vBetaZ1;              /* imposed beta voltage of last sampling, volt */   
    float vAlphaM1;             /* mean value of imposed alpha voltage, volt */
    float vBetaM1;              /* mean value of imposed beta voltage, volt */
    float vAlphaM2;             /* mean value of imposed alpha voltage of last sampling, volt */
    float vBetaM2;              /* mean value of imposed beta voltage of last sampling, volt */    
    float oneByPwmPrdCnt;    
    T_CLARKE_TRANSFORM voltTrans;
    /* outputs */
    float * vA;                 /* imposed alpha voltage, volt */
    float * vB;                 /* imposed beta voltage, volt */  
    float * vAz;                /* imposed alpha voltage of last sampling, volt */
    float * vBz;                /* imposed beta voltage of last sampling, volt */      
} T_IMPOSED_VOLTAGE; 

typedef struct{
    float * dir;
    float * speed;              /* rotor shaft angular speed, rad/s */
    float * olEleSpeedRef;      /* open-loop electric angular speed reference, rad/loop */
    float * w;                  /* rotor electric angular speed as SMO input, rad/s */
    float switchSpeed;          /* electric angular speed threshold - to switch from reference value to estimated value. */
    float perLoop2PerSec;
    float polePairs;            /* number of pole pairs. */
    T_STATE_DATA smoSpeedState;
} T_SMO_SPEED_INPUT;

typedef struct{
    float * dir;      
    float * rotorAngle;         /* rotor flux angle, rad */
    float bemfAngle;            /* BEMF vector angle, rad */ 
    T_ATAN actan;    
} T_BEMF_2_ROTOR_ANGLE_ATAN;

typedef struct{
    float * dAngle;             /* the angle in one loop, rad */
    float speedRaw;             /* rotor shaft angular speed, rad/s */
    float elePerLoop2MechPerSec;     
    T_1ST_LPF_PT filSpeed;   
    T_1ST_LPF_PT filSpeedBemfObs;     
} T_D_ANGLE_2_SPEED;

typedef struct{
    float * th;
} T_ATPLL;

typedef struct{  
    float vAlpha;               /* imposed alpha voltage, volt */
    float vBeta;                /* imposed beta voltage, volt */  
    float vAlphaZ;              /* imposed alpha voltage of last sampling, volt */
    float vBetaZ;               /* imposed beta voltage of last sampling, volt */       
    float eAlphaHat;            /* estimated alpha bemf */
    float eBetaHat;             /* estimated beta bemf */
    float angle;                /* rotor flux angle, rad */  
    float angleLth;             /* latched rotor flux angle, rad */     
    float angleIn;              /* the angle for Park transform */
    float angleOut;             /* the angle for inverse Park transform */
    float speed;                /* rotor shaft angular speed as the output, rad/s */ 
    float speedBemfObs;         /* rotor shaft angular speed as BEMF observer input, rad/s */     
    float w;                    /* rotor electric angular speed as BEMF observer input, rad/s */
    float dAngle;               /* angle difference */
    T_IMPOSED_VOLTAGE imposedVolt;
    T_SMO_SPEED_INPUT smoSpeedIn;
    T_BEMF_SMO smo;
    T_BEMF_2_ROTOR_ANGLE_ATAN bemf2Angle;
    T_D_ANGLE_2_SPEED dAngle2Speed;
} T_ROTOR_ANGLE_BEMF_ATAN;

typedef struct{  
    float pfoGainA;             /* coefficient A to calculate flux observer gain */
    float pfoGainB;             /* coefficient B to calculate flux observer gain */   
    float vAlpha;               /* imposed alpha voltage, volt */
    float vBeta;                /* imposed beta voltage, volt */   
    float vAlphaZ;              /* imposed alpha voltage of last sampling, volt */
    float vBetaZ;               /* imposed beta voltage of last sampling, volt */       
    float phiAlphaEst;          /* estimated alpha rotor flux */
    float phiBetaEst;           /* estimated beta rotor flux */ 
    float angleErr;             /* estimation error of rotor flux angle, rad */
    float angle;                /* estimated rotor flux angle, rad */  
    float dAngle;
    float angleLth;             /* latched rotor flux angle, rad */     
    float angleIn;              /* the angle for Park transform */
    float angleOut;             /* the angle for inverse Park transform */
    float speedEle;             /* estimated electric angular speed, rad/s */
    float speedMech;            /* estimated rotor shaft angular speed, rad/s */ 
    float accelerationEle;      /* estimated electric angular acceleration, rad/s^2 */
    float oneByNumPolePairs;     
    bool isInitialized;
    T_IMPOSED_VOLTAGE imposedVolt;
    T_P_FLUX_OBSERVER pfo;
} T_ROTOR_ANGLE_FLUX_PLL;

typedef struct{
    float K;                   /* desired estimation bandwidth, rad/s */
    float exePrd;              /* execution period of the pll */       
    T_POSITION_TRACKING_PLL_FLT pll;
    T_PTPLL_CAIN_CAL_FLT pllGC;    
} T_MOTION_ESTIMATOR_FLT;

typedef struct{
    double K;                   /* desired estimation bandwidth, rad/s */
    double exePrd;              /* execution period of the pll */       
    T_POSITION_TRACKING_PLL_DBL pll;
    T_PTPLL_CAIN_CAL_DBL pllGC;    
} T_MOTION_ESTIMATOR_DBL;

typedef struct{
    float vAlpha;
    float vBeta;
    float * iAlpha;
    float * iBeta;
    float eAlphaHat;
    float eBetaHat;
    float rotorAngle;
    T_BEMF_SMO smo;
    T_ATPLL pll;
} T_ROTOR_ANGLE_BEMF_PLL;

typedef struct{
    uint32_t alignCnt;
    uint32_t accCnt;
    uint32_t uniformCnt;
    uint32_t loopCnt;
    float alignAngle;
    float shadowAlignAngle;
    float targetSpeed;      /* electrical speed, rad/ctrlLoop */
    float speedRef;         /* electrical speed, rad/ctrlLoop */
    /* During acceleration, speedRef = dSpeedRef * loopCnt */
    float dSpeedRef;        /* electrical speed, rad/ctrlLoop */
    float iQref;
    float iDref;
    float thIn;             /* rotor flux angle for Park transform, rad */
    float thOut;            /* rotor flux angle for inverse Park transform, rad */
    T_STATE_DATA openLoopState;
} T_OPENLOOP_SPIN;

typedef struct{
    T_ANGLE angle;          /* angle from open-loop frame to close-loop frame */   
    T_VECTOR_PASSIVE_TRANSFORM iRefTrans; 
    T_VECTOR_PASSIVE_TRANSFORM vRefTrans;
} T_LOOP_SWITCH;

typedef struct{
    bool stallDetected;
    bool phaseLossDetected;
} T_FAULT_FLAGS;

typedef struct{
    T_STALL_DETECTION stallDetection;
    T_PHASE_LOSS_DETECTION phaseLossDetection;
    T_FAULT_FLAGS fltFlgs;    
} T_FAULT_DETECT;

typedef struct{
    bool gpioButton;
    bool flgMotorTrig;
    bool flgStallDtct;
    bool flgPhsLossDtct;
    int16_t adcPOT;
    E_MOTOR_CONTROL_APPLICATION flgApp;
    E_MOTOR_CONTROL_MODE flgMode;  
    E_FOC_ANGLE_MODE flgAngle;
    float speedRefPot;         /* mechanical speed reference without sign, rad/s */
    float speedRefRaw;         /* mechanical speed reference with sign but not filtered, rad/s */    
    float speedRefFil;         /* mechanical speed reference with sign and filtered, rad/s */
    float dirRef;              /* direction of speed reference */
    float cnt2radPerSec;
    float cnt2Amps;  
    float speedRefDec;          /* mechanical speed reference at deceleration, rad/s */
    float deceleratoin;         /* deceleration, rad/s/loop */
    uint32_t btnDebounceCnt;
    T_1ST_LPF_PT filSpeedRef;
    T_STATE_DATA buttonState;
} T_INSTRUCTION_POT_BUTTON;

typedef struct{
    T_PMSM_PARA motorPara;
    T_CURRENT current;
    T_VOLTAGE voltage; 
    T_ROTOR_ANGLE_ENCODER rotorAngleEncoder;
#ifdef ENCODER_FEEDBACK_INFORM       
    T_ROTOR_ANGLE_ENCODER_REF rotorAngleEncoderRef;
#endif
    T_ROTOR_ANGLE_HALL rotorAngleHall;
    T_ROTOR_ANGLE_BEMF_ATAN rotorAngleAtan;
    T_ROTOR_ANGLE_FLUX_PLL rotorAngleFluxPll;
    T_MOTION_ESTIMATOR_FLT motionEstimate;
    T_FOC_TORQUE_2_CURRENT torque2current;
    T_FOC_CURRENT_CONTROL currentControl;
    T_SPEED_CONTROL speedControl;
    T_INSTRUCTION_POT_BUTTON instruction;
    T_OPENLOOP_SPIN openloopSpin;
    T_LOOP_SWITCH loopSwitch;
    T_FAULT_DETECT faultDetect;
    T_STATE_DATA mcState;
    T_STATE_DATA focState;
} T_PMSM_FOC;



/**********************- Variable Declarations -*******************************/
extern uintptr_t dummyForMisra;
extern T_PMSM_FOC mcFoc1;


/**********************- Function Declarations -*******************************/
void MC_LowFreqTasks(void);
void MC_PeripheralInitial(void);
void MC_Initial(void);
void MC_CtrlLoop(ADC_STATUS status, uintptr_t context );
void MC_ParaConfigMotor1(T_PMSM_FOC * mcData);
void MC_PeriIniMotor1(void);
#if 0
void MC_PeripheralConnectMotor1(T_PMSM_FOC * mcData);
void MC_ParaConfigMotor2(T_PMSM_FOC * mcData);
void MC_PeripheralConnectMotor2(T_PMSM_FOC * mcData);
void MC_PeriIniMotor2(void);
void MC_ParaConfigMotor3(T_PMSM_FOC * mcData);
void MC_PeripheralConnectMotor3(T_PMSM_FOC * mcData);
void MC_PeriIniMotor3(void);
#endif

#if 0
void MC_PeripheralConnect(void);
#endif



/******************************- Functions -***********************************/
#if 0
__STATIC_INLINE void CLK_CntStartLatch(T_CLOCK_MEASURE * ptClkLth)
{
    ptClkLth->cntStart = TMR1_CounterGet();
}


__STATIC_INLINE void CLK_LengthCalculate(T_CLOCK_MEASURE * ptClkLth)
{
    /* Latch end-count. */
    ptClkLth->cntEnd = TMR1_CounterGet();  
    
    /* Calculate length. 
       The clock frequency of timer-1 is 60MHz, i.e. half of CPU. */
    ptClkLth->length = (ptClkLth->cntEnd - ptClkLth->cntStart) << 1u;
    
    if(ptClkLth->length > ptClkLth->lenMax){
        ptClkLth->lenMax = ptClkLth->length;
    }
    
    if(ptClkLth->length < ptClkLth->lenMin){
        ptClkLth->lenMin = ptClkLth->length;
    }
}
#endif

__STATIC_INLINE void CLK_DataReset(T_CLOCK_MEASURE * ptClkLth)
{
    ptClkLth->lenMax = 0u;
    ptClkLth->lenMin = 65535u;
}



#endif /* end of _MC_CONTROL_LOOP_H */

/*******************************************************************************
 End of File
*/    

