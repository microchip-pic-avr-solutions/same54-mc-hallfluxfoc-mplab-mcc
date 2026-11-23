/*******************************************************************************
  Motor Control Hardware Abstraction interface file

  Company:
    Microchip Technology Inc.

  File Name:
    mc_library.h

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
#ifndef MC_LIBRARY_H
#define MC_LIBRARY_H


/**********************- Included Files -*******************************/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <float.h>
#include <math.h>


/**********************- Macro Definitions -*******************************/ 
#define FLOAT                                   float

#define M_2PI                                   (6.28318530717958647693)
#define M_3_PI_OVER_2                           (4.71238898038468985769)
#define M_PI_OVER_6                             (M_PI / 6.0)
#define M_SQRT3                                 (1.7320508076)
#define M_1_BY_SQRT3                            (0.5773502691)
#define M_2_BY_SQRT3                            (1.1547005384)
#define M_2_BY_3                                (0.66666666666666666667)
#define M_1_BY_3                                (0.33333333333333333333)
#define M_SQRT3_BY_2                            (0.866025403788)
#define M_1_BY_SQRT2                            (0.70710678118654752440)

#define M_ATAN_A                                (0.0776509570923569)
#define M_ATAN_B                                (-0.287434475393028)
#define M_ATAN_C                                (M_PI_4 - M_ATAN_A - M_ATAN_B)

#define M_MINUS_1_BY_6                          (-1.0 / 6.0)
#define M_1_BY_120                              (1.0 / 120.0)
#define M_1_BY_24                               (1.0 / 24.0)
#define M_MINUS_1_BY_720                        (-1.0 / 720.0)

#define TABLE_SIZE                              (256)
#define ANGLE_STEP                              (float)(M_2PI / (double)TABLE_SIZE)
#define ONE_OVER_ANGLE_STEP                     (float)((double)TABLE_SIZE / M_2PI) 

#define M_3_PI_OVER_4                           (3.0 * M_PI_4)
#define M_5_PI_OVER_4                           (5.0 * M_PI_4)
#define M_7_PI_OVER_4                           (7.0 * M_PI_4)

#define DEGREE_30_RAD                           M_PI_OVER_6
#define DEGREE_60_RAD                           (M_PI_2 - M_PI_OVER_6)
#define DEGREE_90_RAD                           (M_PI_2)
#define DEGREE_120_RAD                          (M_PI_2 + M_PI_OVER_6)
#define DEGREE_150_RAD                          (M_PI - M_PI_OVER_6)
#define DEGREE_180_RAD                          (M_PI)
#define DEGREE_210_RAD                          (M_PI + M_PI_OVER_6)
#define DEGREE_240_RAD                          (DEGREE_270_RAD - M_PI_OVER_6)
#define DEGREE_270_RAD                          (M_PI + M_PI_2)
#define DEGREE_300_RAD                          (DEGREE_270_RAD + M_PI_OVER_6)
#define DEGREE_330_RAD                          (M_2PI - M_PI_OVER_6)

typedef enum{
    phase_loss_dtct_idle = 0u,
    phase_loss_dtct_work      
} E_PHASE_LOSS_DTCT_STATE;


/**********************- Data Types -*******************************/
typedef struct{
    uint8_t state;
    bool firstEntry;
    uint32_t cnt;
} T_STATE_DATA;

typedef struct{
    float a;
    float b;
    float * in;
    float * out;
} T_1ST_LPF_PT;

typedef struct{
    float * in;
    float * out;
    float step; 
} T_LINEAR_RAMP;

typedef struct{
    float * in;
    float * out;
    float maxOut;
    float minOut;
    float gain;
} T_INTEGRATOR;

typedef struct{
    double * in;
    double * out;
    double maxOut;
    double minOut;
    double gain;
} T_INTEGRATOR_DOUBLE;

typedef struct{
    float * ref;
    float * fbk;
    float integrator;
    float out;
    float maxOut;
    float pG;
    float iG;
} T_PI_CONTROLLER;

typedef struct{
    float th;               /* electric rotor flux angle, rad, 0 ~ 2pi */
    float sine;
    float cosine;
    float * ptSinTbl;  
} T_ANGLE;

typedef struct{
    float * u;
    float * v;
    float * w;
    float * alpha;
    float * beta;
} T_CLARKE_TRANSFORM;

typedef struct{
    float * x1;  /* x component under basis 1 */
    float * y1;  /* y component under basis 1 */
    float * x2;  /* x component under basis 2 */
    float * y2;  /* y component under basis 2 */
    T_ANGLE * angle;  /* angle from basis 1 to basis 2 */
} T_VECTOR_PASSIVE_TRANSFORM;

typedef struct{
    float * iDfbk;
    float * iQfbk;
    float * mechSpd;
    float lD;
    float lQ;
    float mechSpd2Bemf;
    float * compD;
    float * compQ;
} T_FOC_DECOUPLE;

typedef struct{    
    float * vU;                 /* phase U voltage, per unit */
    float * vV;                 /* phase V voltage, per unit */
    float * vW;                 /* phase W voltage, per unit */  
    float * vBus;               /* DC bus voltage, Volt */
    float svpwmU;               /* terminal U voltage, per unit */
    float svpwmV;               /* terminal V voltage, per unit */
    float svpwmW;               /* terminal W voltage, per unit */
    float maxRange;             /* the available BUS voltage, per unit */
    int16_t compareU;           /* terminal U voltage, PWM compare value */
    int16_t compareV;           /* terminal V voltage, PWM compare value */
    int16_t compareW;           /* terminal W voltage, PWM compare value */
    int16_t halfPrdCnt;         /* half PWM period count value */
    int16_t compareUpBound;     /* the upper boundary of PWM compare value */
    int16_t compareLowBound;    /* the lower boundary of PWM compare value */    
    int16_t offsetCnt;          /* the offset of PWM compare value */
    int16_t compareSign;        /* the sign of compare value */
    float vMaxDotPrdCnt;        /* max_phase_voltage * PER_cnt */
                                /* compare count value = (vRef in p.u.) * vMaxDotPrdCnt / vBus + (0.5 * prdCnt) */
} T_SVPWM; 

typedef struct{
    float a;
    float b1;
    float b2;
    float b3;   
    float boundary;            /* boundary layer of current observer */
    float m;                   /* gain of current observer */ 
    float mdbi;                /* m over boundary */    
} T_SMO_CUR_OBS_PARA;

typedef struct{
    /* inputs */
    float * v;
    float * i;
    float * eHat;    
    /* outputs */
    float * eTilde;
    /* intermediate variables */    
    float iHat; 
    float iTilde;
    /* parameters */
    T_SMO_CUR_OBS_PARA * para;
} T_SMO_CURRENT_OBSERVER;

typedef struct{
    /* inputs */  
    float * w;                      /* electric angular speed of rotor flux, rad/s */       
    float * eAlphaTilde;
    float * eBetaTilde;
    /* outputs */
    float * eAlphaHat;
    float * eBetaHat;
    /* static parameters */
    float lambda;                  /* desired eigenvalue of BEMF observer */
    float ls;                       /* phase inductance, Henry */
    float ts;                       /* sampling period */ 
    /* dynamic parameters */  
    float a1;
    float a2;    
    float b1;
    float b2;   
} T_SMO_BEMF_OBSERVER;

typedef struct{
    T_SMO_CURRENT_OBSERVER iObsAlpha;
    T_SMO_CURRENT_OBSERVER iObsBeta;
    T_SMO_CUR_OBS_PARA iObsPara; 
    T_SMO_BEMF_OBSERVER eObs;    
    /* intermediate variables */
    float eAlphaTilde;
    float eBetaTilde;          
} T_BEMF_SMO;

typedef struct{
    float * x;
    float * y;
    float * angle;
} T_ATAN;

typedef struct{
    float * iDref;
    float * iQref;
    float deltaIdC1;
    float deltaIdC2;
    float iDstep;
    float iDhigh;
    float iDlow;
} T_MTPA_ID_ESTIMATE;

typedef struct{
    /* inputs */
    float * iU;         /* phase U current, Amps */
    float * iV;         /* phase V current, Amps */
    /* parameters */
    float Ith;          /* the current threshold to detect stalling, Amps */
    /* intermediate */
    float iUfil;
    T_1ST_LPF_PT filIu;
    float iVfil;
    T_1ST_LPF_PT filIv;
    float iWfil;
    /* output */
    bool * stallDetected;
} T_STALL_DETECTION;


typedef struct{
    /* inputs */
    float * iU;         /* phase U current, Amps */
    float * iV;         /* phase V current, Amps */
    float * iW;         /* phase W current, Amps */
    float * spd;        /* mechanical (angular) speed, rad/s */
    /* parameters */
    float spdUp;
    float spdLow;
    uint32_t cntThr;    
    /* intermediate */
    float iUAbs;
    float iUAbsFil;
    T_1ST_LPF_PT filU;
    float iVAbs;
    float iVAbsFil;
    T_1ST_LPF_PT filV; 
    float iWAbs;
    float iWAbsFil;
    T_1ST_LPF_PT filW;
    float iSA;
    uint32_t cntU;
    uint32_t cntV;
    uint32_t cntW;  
    T_STATE_DATA state;
    /* output */
    bool * phaseLossDetected;    
} T_PHASE_LOSS_DETECTION;


/**********************- Variable Declarations -*******************************/
extern float sineTable[TABLE_SIZE];


/**********************- Function Declarations -*******************************/
void MC_ClarkeTransformBalanced(T_CLARKE_TRANSFORM * ptClark);
void MC_ClarkeTransform(T_CLARKE_TRANSFORM * ptClark);
void MC_InvClarkeTransform(T_CLARKE_TRANSFORM * ptClark);
void MC_ParkTransform(T_VECTOR_PASSIVE_TRANSFORM * ptVPT);
void MC_InvParkTransform(T_VECTOR_PASSIVE_TRANSFORM * ptVPT);
void MC_Svpwm(T_SVPWM * ptSvpwm);
void MC_Atan(T_ATAN * ptAtan);
void MC_CurrentObserver(T_SMO_CURRENT_OBSERVER * ptIObs);
void MC_BemfObsCoeffCal(T_SMO_BEMF_OBSERVER * ptEObs);
void MC_BemfObserver(T_SMO_BEMF_OBSERVER * ptEObs);
void MC_MptaIdEstimate(T_MTPA_ID_ESTIMATE * ptIdEst);
void MC_StallDetect(T_STALL_DETECTION * ptStlDtct);
void MC_PhaseLossDetect(T_PHASE_LOSS_DETECTION * ptPhsLossDtct);
void MC_PhsLossDtctIni(T_PHASE_LOSS_DETECTION * ptPhsLossDtct);
void MC_PhsLossDtctReset(T_PHASE_LOSS_DETECTION * ptPhsLossDtct);


/******************************- Functions -***********************************/
/* In C89, % operator is implementation defined: it can be a remainder operator or a modulo operator.
 * In C99 & C11, % operator is a remainder operator.
 * An example of remainder operation is: -5 % 4 = -1.
 * An example of modulo operation is: -5 % 4 = 3.
 * This function makes it certain to be a modulo operation. */
static inline int32_t MC_Modulus(int32_t divident, int32_t divisor)
{
    int32_t ret;
    
    /* The divisor shall be positive. */
    if(0 >= divisor){
        ret = 0;
    } else{
        ret = divident % divisor;
    }
        
    if(0 > ret){
        ret += divisor;
    }
    
    return ret;
}

#if 0  /* Disabled because MISRA-C rule 4.11 is detected by online build. */
static inline float MC_Modulus_Flt(float dividend, float divisor)
{
    float ret;
    
    /* The divisor shall be positive. */
#if 0
    if ((!isfinite(dividend)) || (!isfinite(divisor)) || (divisor <= 0.0f)){
#else
    if (0.0f >= divisor){    
#endif
        ret = 0.0f;
    } else{
        ret = fmodf(dividend, divisor);
    } 
        
    if(0.0f > ret){
        ret += divisor;
    }
    
    return ret;
}
#endif


__STATIC_INLINE float MC_Abs(float in)
{
    float out = in;
    
    if(0.0f > in){
        out = -in;
    }
    
    return out;
}


__STATIC_INLINE float MC_Sign(float in)
{
    float out = 0.0f;
    
    if(0.0f > in){
        out = -1.0f;
    } else if(0.0f < in){
        out = 1.0f;
    } else{}  /* MISRA-C compliance */
    
    return out;    
}


__STATIC_INLINE void MC_1st_Lpf(T_1ST_LPF_PT * data)
{
    *(data->out) = *(data->out) * (data->a) + *(data->in) * (data->b); 
}


__STATIC_INLINE float MC_WrapPi(float in)
{
    float out;
    float pi = (float)M_PI;
    float twoPi = (float)M_2PI;
    
    if(pi < in){
        out = in - twoPi;
    } else if(-pi > in){
        out = in + twoPi;
    } else{
        out = in;
    }
        
    return out;
}


__STATIC_INLINE double MC_WrapPiDbl(double in)
{
    double out;
    double pi = M_PI;
    double twoPi = M_2PI;
    
    if(pi < in){
        out = in - twoPi;
    } else if(-pi > in){
        out = in + twoPi;
    } else{
        out = in;
    }
        
    return out;
}


__STATIC_INLINE float MC_Wrap2Pi(float in)
{
    float out;
    float twoPi = (float)M_2PI; 
    
    if(twoPi < in){
        out = in - twoPi;
    } else if(0.0f > in){
        out = in + twoPi;
    } else{
        out = in;
    }
        
    return out;
}


__STATIC_INLINE double MC_Wrap2PiDbl(double in)
{
    double out;
    double twoPi = M_2PI; 
    
    if(twoPi < in){
        out = in - twoPi;
    } else if(0.0 > in){
        out = in + twoPi;
    } else{
        out = in;
    }
        
    return out;
}


__STATIC_INLINE int32_t MC_Wrap32(int32_t in, int32_t bound)
{
    int32_t out, tmp;
    
    tmp = bound >> 1u;
    
    if(in > tmp){
        out = in - bound;
    } else if(in < -tmp){ 
        out = in + bound; 
    } else{
        out = in;
    }    
    
    return out;
}


__STATIC_INLINE float MC_SymmetricClampF(float in, float boundary)
{
    float out;
    
    /* The "boundary" must be not negative. */
    if(in > boundary){
        out = boundary;
    } else if(in < -boundary){
        out = -boundary;
    } else{
        out = in;
    }
    
    return out;
}


__STATIC_INLINE float MC_ClampF(float in, float low, float high)
{
    float out;
    
    if(in > high){
        out = high;
    } else if(in < low){
        out = low;
    } else{
        out = in;
    }
    
    return out;
}


__STATIC_INLINE int16_t MC_PositiveClamp(int16_t in, int16_t boundary)
{
    int16_t out;
    
    /* The "boundary" must be not negative. */
    if(in > boundary){
        out = boundary;
    } else if(in < 0){
        out = 0;
    } else{
        out = in;
    }
    
    return out;
}


__STATIC_INLINE int16_t MC_Clamp(int16_t in, int16_t low, int16_t high)
{
    int16_t out;
    
    if(in > high){
        out = high;
    } else if(in < low){
        out = low;
    } else{
        out = in;
    }
    
    return out;
}


__STATIC_INLINE void MC_LinearRamp(T_LINEAR_RAMP * data)
{
    float out = *(data->out);
    float in = *(data->in);
    float err = in - out;
    float step = data->step;
    float delta;
    
    delta = MC_SymmetricClampF(err, step);
    *(data->out) = out + delta;        
}


__STATIC_INLINE void MC_Integrator(T_INTEGRATOR * ptItg)
{
    float out = *(ptItg->out);
    float in = *(ptItg->in);
    float gain = ptItg->gain;
    float max = ptItg->maxOut;
    float min = ptItg->minOut;    
    float tmp;
    
    tmp = out + (gain * in);
    
    if(tmp > max){
        out = max;
    } else if(tmp < min){
        out = min;
    } else{
        out = tmp;
    }
    
    *(ptItg->out) = out;   
}


__STATIC_INLINE void MC_IntegratorDbl(T_INTEGRATOR_DOUBLE * ptItg)
{
    double out = *(ptItg->out);
    double in = *(ptItg->in);
    double gain = ptItg->gain;
    double max = ptItg->maxOut;
    double min = ptItg->minOut;    
    double tmp;
    
    tmp = out + (gain * in);
    
    if(tmp > max){
        out = max;
    } else if(tmp < min){
        out = min;
    } else{
        out = tmp;
    }
    
    *(ptItg->out) = out;   
}


__STATIC_INLINE void MC_PiCtrl(T_PI_CONTROLLER * ptPi)
{
    float err, tmp;
    
    err = *(ptPi->ref) - *(ptPi->fbk);
    ptPi->integrator += err * ptPi->iG;
    ptPi->integrator = MC_SymmetricClampF(ptPi->integrator, ptPi->maxOut);
    tmp = (err * ptPi->pG) + ptPi->integrator;
    ptPi->out = MC_SymmetricClampF(tmp, ptPi->maxOut);
}


__STATIC_INLINE void MC_FocDecoupleCal(T_FOC_DECOUPLE * ptDecouple)
{
    float iD = *(ptDecouple->iDfbk);
    float iQ = *(ptDecouple->iQfbk);
    float spd = *(ptDecouple->mechSpd);
    float lD = ptDecouple->lD;
    float lQ = ptDecouple->lQ;
    float spd2bemf = ptDecouple->mechSpd2Bemf;
    
    /* The number of pole pairs is contained in lD, lQ, 
     * hence we use mechanical speed instead of electrical speed. */
    *(ptDecouple->compD) = -iQ * lQ * spd;
    *(ptDecouple->compQ) = (iD * lD + spd2bemf) * spd;
}


__STATIC_INLINE float MC_SinCalc(float th, float * ptSinTbl)
{
    /* Make sure "th" is in (0, 2pi). There is no additional check.
       The interpolation equation: 
       y = y0 + (y1 - y0)*((x - x0)/(x1 - x0)) */  
    int16_t y0_Index;
    int16_t y0_IndexNext;
    float x0, y0, y1, temp, sine;
    float oneByStep = ONE_OVER_ANGLE_STEP;
    float step = ANGLE_STEP;
    float tmp;
    int16_t size = TABLE_SIZE - 1;
    
    tmp = th * oneByStep;
    y0_Index = (int16_t)(tmp);
    y0_IndexNext = y0_Index + 1;
	
    if(size <= y0_Index){ 
        y0_Index = size;  
        y0_IndexNext = 0;    
    } else if(0 > y0_Index){
        y0_Index = 0;
        y0_IndexNext = 1;
    } else{}  /* MISRA-C compliance */
    
    x0 = (float)y0_Index * step;  
    temp = ((th - x0) * oneByStep);
    
    /* Find Sine */
    y0 = ptSinTbl[y0_Index];
    y1 = ptSinTbl[y0_IndexNext];     
    sine = y0 + ((y1 - y0) * temp);
    
    return sine;
}


__STATIC_INLINE float MC_CosCalc(float th, float * ptSinTbl)
{ 
    float tmp, thCos; 
    float pi_2 = (float)M_PI_2;
    float twoPi = (float)M_2PI;
     
    /* cos(th) = sin(pi/2 - th) */
    tmp = pi_2 - th;
    if(0.0f > tmp){
        thCos = tmp + twoPi;
    } else{
        thCos = tmp;
    }
    
    return MC_SinCalc(thCos, ptSinTbl);  
}
  
 
__STATIC_INLINE void MC_SinCosCalc(T_ANGLE * ptAngle)
{ 
    float th = ptAngle->th;
    float * ptSinTbl = ptAngle->ptSinTbl;
    float tmp, thCos;     
    float pi_2 = (float)M_PI_2;
    float twoPi = (float)M_2PI;
    
    /* cos(th) = sin(pi/2 - th) */
    tmp = pi_2 - th;
    if(0.0f > tmp){
        thCos = tmp + twoPi;
    } else{
        thCos = tmp;
    }
    
    ptAngle->sine = MC_SinCalc(th, ptSinTbl);
    ptAngle->cosine = MC_SinCalc(thCos, ptSinTbl);   
} 


__STATIC_INLINE float MC_SinTaylorPi4(float x)
{
    float a = 1.0f;
    float b = (float)M_MINUS_1_BY_6;
    float c = (float)M_1_BY_120;
    float xx = x * x;
    
    /* x is in radian and shall be in [-pi/4, pi/4]. */
    return(x * (a + xx * (b + xx * c)));    
}


__STATIC_INLINE float MC_CosTaylorPi4(float x)
{
    float a = 1.0f;    
    float b = -0.5f;
    float c = (float)M_1_BY_24;
    float d = (float)M_MINUS_1_BY_720;
    float xx = x * x;
    
    /* x is in radian and shall be in [-pi/4, pi/4]. */
    return(a + xx * (b + xx * (c + xx * d)));    
}


__STATIC_INLINE float MC_SinTaylorPi2(float x)
{ 
    float out;
    float pi_2 = (float)M_PI_2;
    float pi_4 = (float)M_PI_4;  
    
    /* x is in radian and shall be in [0, pi/2]. */
    if(pi_4 > x){
        out = MC_SinTaylorPi4(x);
    } else{
        out = MC_CosTaylorPi4(pi_2 - x);
    }
    
    return out;
}


__STATIC_INLINE float MC_SinTaylorPi(float x)
{ 
    float out;
    float pi_2 = (float)M_PI_2;
    float pi = (float)M_PI;
    
    /* x is in radian and shall be in [0, pi]. */
    if(pi_2 > x){
        out = MC_SinTaylorPi2(x);
    } else{
        out = MC_SinTaylorPi2(pi - x);
    }
    
    return out;
}


__STATIC_INLINE float MC_SinTaylor(float x)
{ 
    float out;
    float pi = (float)M_PI;
    float twoPi = (float)M_2PI;
    
    /* x is in radian and shall be in [0, 2pi]. */
    if(pi > x){
        out = MC_SinTaylorPi(x);
    } else{
        out = -1.0f * MC_SinTaylorPi(twoPi - x);
    }
    
    return out;
}


__STATIC_INLINE float MC_CosTaylorPi2(float x)
{ 
    float out;
    float pi_2 = (float)M_PI_2;
    float pi_4 = (float)M_PI_4;    
    
    /* x is in radian and shall be in [0, pi/2]. */
    if(pi_4 > x){
        out = MC_CosTaylorPi4(x);
    } else{
        out = MC_SinTaylorPi4(pi_2 - x);
    }
    
    return out;
}


__STATIC_INLINE float MC_CosTaylorPi(float x)
{ 
    float out;
    float pi_2 = (float)M_PI_2;
    float pi = (float)M_PI;
    
    /* x is in radian and shall be in [0, pi]. */
    if(pi_2 > x){
        out = MC_CosTaylorPi2(x);
    } else{
        out = -1.0f * MC_CosTaylorPi2(pi - x);
    }
    
    return out;
}


__STATIC_INLINE float MC_CosTaylor(float x)
{ 
    float out;
    float pi = (float)M_PI;
    float twoPi = (float)M_2PI;
    
    /* x is in radian and shall be in [0, 2pi]. */
    if(pi > x){
        out = MC_CosTaylorPi(x);
    } else{
        out = MC_CosTaylorPi(twoPi - x);
    }
    
    return out;
}


#endif /* end of _MC_LIBRARY_H */

/*******************************************************************************
 End of File
*/    
