/*******************************************************************************
  Motor Control Loop Interrupt Handler

  Company:
    Microchip Technology Inc.

  File Name:
    mc_control_loop.c

  Summary:
    This file contains the state machine and control loop of the motor control
   algorithm.

  Description:
    This file contains the control loop.
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

#include "mc_control_loop.h"


/**********************- Function Declarations -*******************************/


void MC_CtrlLoop(ADC_STATUS status, uintptr_t context); 

/* Static forward declarations for file-scope functions (MISRA-C Rule 8.4) */
static void MC_PwmDutySet(T_SVPWM * ptSvpwm);
static void MC_CommDataInitial(void);
static void MC_SpeedRefPotOlAlign(T_PMSM_FOC * mcData);
static void MC_SpeedRefPotClStart(T_PMSM_FOC * mcData);
static void MC_SpeedRefPot(T_PMSM_FOC * mcData);
static void MC_MotorTrigBtn(T_PMSM_FOC * mcData);
static void MC_DataInitial(T_PMSM_FOC * mcData);
static void MC_FluxPllReset(T_ROTOR_ANGLE_FLUX_PLL * ptFluxPll);
static void MC_DataReset(T_PMSM_FOC * mcData);
static void MC_LowFreqLoopSync(void);
static void MC_OpenLoopSttUpdate(T_PMSM_FOC * mcData);
static void MC_FocCurCtrlBasic(T_FOC_CURRENT_CONTROL * ptFocCurCtrl);
static void MC_FocCurCtrlDecouple(T_FOC_CURRENT_CONTROL * ptFocCurCtrl);
static void MC_FocCurCtrlSttUpdate(T_FOC_CURRENT_CONTROL * ptFocCurCtrl);
static void MC_FocCurCtrl(T_FOC_CURRENT_CONTROL * ptFocCurCtrl);
static void MC_FocCurCtrlOlAlign(T_FOC_CURRENT_CONTROL * ptFocCurCtrl);
static void MC_Foc(T_PMSM_FOC * mcData);
static void MC_PwmModulate(T_SVPWM * ptSvpwm);
static void MC_FocOlAlign(T_PMSM_FOC * mcData);
static void MC_SpeedCtrl(T_SPEED_CONTROL * ptSpeedCtrl);
static void MC_T2cMtpaOff(T_FOC_TORQUE_2_CURRENT * ptT2C);
static void MC_T2cMtpaOn(T_FOC_TORQUE_2_CURRENT * ptT2C);
static void MC_Torque2current(T_FOC_TORQUE_2_CURRENT * ptT2C);
static void MC_LoopSwitch(T_PMSM_FOC * mcData);
static void MC_VoltCalc(T_IMPOSED_VOLTAGE * ptVolt);
static void MC_SmoSpeedCalc(T_SMO_SPEED_INPUT * ptSmoSpeedIn);
static void MC_BemfSmo(T_BEMF_SMO * ptSmoData);
static void MC_Bemf2AngleAtan(T_BEMF_2_ROTOR_ANGLE_ATAN * ptBemf2Angle);
static void MC_Angle2Speed(T_D_ANGLE_2_SPEED * ptDAngle2Speed);
static void MC_BemfAngleAtan(T_ROTOR_ANGLE_BEMF_ATAN * ptThAtan);
static void MC_PmsmFocBemfSttUpdate(T_PMSM_FOC * mcData);
static void MC_PmsmFocFluxSttUpdate(T_PMSM_FOC * mcData);
static void MC_PmsmFocAlignEncSttUpdate(T_PMSM_FOC * mcData);
static void MC_FluxObsGainCal(T_PMSM_FOC * mcData);
static void MC_FluxPll(T_PMSM_FOC * mcData);
static void MC_FluxAnglePllOlAlign(T_PMSM_FOC * mcData);
static void MC_FluxAnglePllClStart(T_PMSM_FOC * mcData);
static void MC_PmsmFocFlux(T_PMSM_FOC * mcData);
static void MC_PmsmFocHall(T_PMSM_FOC * mcData);
static void MC_PmsmFocHallFluxAngleAssign(T_PMSM_FOC * mcData);
static void MC_PmsmFocHallFlux(T_PMSM_FOC * mcData);
static void MC_AdcOffsetCal(T_CURRENT * ptCur);
static void MC_SpeedCtrlApp(T_PMSM_FOC * mcData);
static bool MC_FaultDetection(T_PMSM_FOC * mcData);
static void MC_HallRead(uint8_t * hallVal);
static void MC_HallJumpTask(T_PMSM_FOC * mcData);
static void MC_AltHallJumpTask(T_PMSM_FOC * mcData);
static void MC_LegalHallJumpTask(T_ROTOR_ANGLE_HALL * ptThHall);
static void MC_ParticularHallJumpTask(T_ROTOR_ANGLE_HALL * ptThHall);
static float MC_HallAngleCal(uint8_t hallVal, float offset);
static void MC_HallAngleEstimateLowSpd(T_ROTOR_ANGLE_HALL * ptThHall);
static void MC_HallAngleEstimateHighSpd(T_ROTOR_ANGLE_HALL * ptThHall);
static void MC_PhaseVoltageDtct(T_VOLTAGE * ptVolt);
static void MC_HallAngle(T_PMSM_FOC * mcData);
static void MC_HallCali(T_ROTOR_ANGLE_ENCODER * ptThEnc);
static void MC_HallAngleCorrection(T_ROTOR_ANGLE_ENCODER * ptThEnc);
static void MC_AdcManipulate1(void);
static void MC_FeedbackGet1(T_PMSM_FOC * mcData);
static void MC_AdcManipulate2(void);
static void MC_BusVoltRead(T_VOLTAGE * ptVolt);
static void MC_FeedbackGet2(T_PMSM_FOC * mcData);
static void MC_DecSpeedRefCal(T_INSTRUCTION_POT_BUTTON * ptInstr);
static void MC_PeripheralEnable(T_PMSM_FOC * mcData);
static void MC_PeripheralDisable(T_PMSM_FOC * mcData);
static void MC_SpeedCtrlSttUpdate(T_PMSM_FOC * mcData);
#if 0
static void MC_MotorCtrlSttUpdate(T_PMSM_FOC * mcData);
static void MC_PmsmFocAlignEnc(T_PMSM_FOC * mcData);
static void MC_Spwm(T_SVPWM * ptSvpwm);
static void MC_PmsmFocBemf(T_PMSM_FOC * mcData);
#endif


/**********************- External Variable Declarations -*********************/


    
/**********************- Global Variables -*********************************/
static T_COMMON_DATA commData;
uintptr_t dummyForMisra;


/**********************- Functions -**********************************/
__STATIC_INLINE void MC_PwmDutySet(T_SVPWM * ptSvpwm)
{  
    bool dumm;  /* MISRA-C compliance */
    
    dumm = TCC0_PWM24bitDutySet(TCC0_CHANNEL0, (uint32_t)(ptSvpwm->compareU));
    dumm = TCC0_PWM24bitDutySet(TCC0_CHANNEL1, (uint32_t)(ptSvpwm->compareV));
    dumm = TCC0_PWM24bitDutySet(TCC0_CHANNEL2, (uint32_t)(ptSvpwm->compareW));    
}


void MC_PeripheralInitial(void)
{ 
    /*  */
    SERCOM2_USART_Enable();
    
    /*  */
    MC_PeriIniMotor1();
}


static void MC_CommDataInitial(void)
{
    T_LOOP_SYNC * ptLoopSync = &(commData.lowFreqLoopSync);
    T_CLOCK_MEASURE * ptClkLth1 = &(commData.clkLth1); 
    T_CLOCK_MEASURE * ptClkLth2 = &(commData.clkLth2); 
    float * ptHallAngle = commData.hallAngleTbl;    
    
    ptLoopSync->cntLowFreqLoop = (uint32_t)(CNT_LOW_FREQ_LOOP);
    ptLoopSync->cnt = 0u;
    ptLoopSync->flg = false;
 
    CLK_DataReset(ptClkLth1);
    CLK_DataReset(ptClkLth2);        
    
    commData.angleIniTbl[0u] = (float)M_5_PI_OVER_4;
    commData.angleIniTbl[1u] = (float)M_3_PI_OVER_4;
    commData.angleIniTbl[2u] = (float)M_7_PI_OVER_4;
    commData.angleIniTbl[3u] = (float)M_PI_4;    
    
    ptHallAngle[0u] = 0.0f;
    ptHallAngle[1u] = (float)DEGREE_90_RAD;
    ptHallAngle[2u] = (float)DEGREE_210_RAD;
    ptHallAngle[3u] = (float)DEGREE_150_RAD;
    ptHallAngle[4u] = (float)DEGREE_330_RAD;
    ptHallAngle[5u] = (float)DEGREE_30_RAD;
    ptHallAngle[6u] = (float)DEGREE_270_RAD;
    ptHallAngle[7u] = 0.0f;     
}


static void MC_SpeedRefPotOlAlign(T_PMSM_FOC * mcData)
{
    T_INSTRUCTION_POT_BUTTON * ptInstr = &(mcData->instruction); 
    T_STATE_DATA * ptFocStt = &(mcData->focState);
    T_PMSM_PARA * ptMotorPara = &(mcData->motorPara);
    T_1ST_LPF_PT * ptFilSpdRef = &(ptInstr->filSpeedRef);
    float tmp;
    
    /* The potentiometer input value is read in the ADC ISR. */
    tmp = (float)(ptInstr->adcPOT) * (ptInstr->cnt2radPerSec);
    
    if(tmp < ptMotorPara->minSpeed){
        ptInstr->speedRefPot = ptMotorPara->minSpeed;
    } else if(tmp > ptMotorPara->ratedSpeed){
        ptInstr->speedRefPot = ptMotorPara->ratedSpeed;
    } else{
        ptInstr->speedRefPot = tmp;
    }
         
    switch(ptFocStt->state){
        case mc_close_loop:
            ptInstr->speedRefRaw = (ptInstr->speedRefPot) * (ptInstr->dirRef);
            break;
        case mc_open_loop:
            /* The following assignment is redundant, because speedRefRaw is reset during motor start. */
            ptInstr->speedRefRaw = (ptMotorPara->minSpeed) * (ptInstr->dirRef);
            break;
        default: /* MISRA-C compliance */
            break;                       
    }
    
    /* LPF speedRefRaw. */
    MC_1st_Lpf(ptFilSpdRef);
}


static void MC_SpeedRefPotClStart(T_PMSM_FOC * mcData)
{
    T_INSTRUCTION_POT_BUTTON * ptInstr = &(mcData->instruction); 
    T_STATE_DATA * ptMcStt = &(mcData->mcState);
    T_PMSM_PARA * ptMotorPara = &(mcData->motorPara);
    T_1ST_LPF_PT * ptFilSpdRef = &(ptInstr->filSpeedRef);
    float tmp;
    
    /* The potentiometer input value is read in the ADC ISR. */
    tmp = (float)(ptInstr->adcPOT) * (ptInstr->cnt2radPerSec);
    
    if(tmp < ptMotorPara->minSpeed){
        ptInstr->speedRefPot = ptMotorPara->minSpeed;
    } else if(tmp > ptMotorPara->ratedSpeed){
        ptInstr->speedRefPot = ptMotorPara->ratedSpeed;
    } else{
        ptInstr->speedRefPot = tmp;
    }
         
    switch(ptMcStt->state){
        case mc_spin:
            ptInstr->speedRefRaw = (ptInstr->speedRefPot) * (ptInstr->dirRef);
            break;
        case mc_idle:
        case mc_fault:    
            /* The following assignment is redundant, because speedRefRaw is reset during motor start. */
            ptInstr->speedRefRaw = (ptMotorPara->minSpeed) * (ptInstr->dirRef);
            break;
        default: /* MISRA-C compliance */
            break;                       
    }
    
    /* LPF speedRefRaw. */
    MC_1st_Lpf(ptFilSpdRef);
}


static void MC_SpeedRefPot(T_PMSM_FOC * mcData)
{
    E_MOTOR_CONTROL_APPLICATION app = mcData->instruction.flgApp;
    
    switch(app){
        case ma_flux_foc:
            MC_SpeedRefPotOlAlign(mcData);
            break;
        case ma_hall_foc:
        case ma_hall_flux_foc:
        case ma_align_encoder_foc:
        case ma_hall_encoder_foc: 
            MC_SpeedRefPotClStart(mcData);
            break;
        default: /* MISRA-C compliance */
            break;
    }    
}


static void MC_MotorTrigBtn(T_PMSM_FOC * mcData)
{
    T_INSTRUCTION_POT_BUTTON * ptInstr = &(mcData->instruction);  
    T_STATE_DATA * ptBtnStt = &(ptInstr->buttonState);
    bool tmpBool;

    switch(ptBtnStt->state){
        case btn_detect:
            tmpBool = PORT_PinRead( PORT_PIN_PD09);
            ptInstr->gpioButton = tmpBool; 
            if(false == tmpBool){  /* The button is pressed. */
                ptInstr->flgMotorTrig = true;
                ptBtnStt->cnt = 0u;
                ptBtnStt->state = (uint8_t)(btn_debounce);
            }
            break;
        case btn_debounce:
            (ptBtnStt->cnt)++;
            if(ptBtnStt->cnt >= ptInstr->btnDebounceCnt){
                ptBtnStt->state = (uint8_t)(btn_detect);
            }
            break;
        default: /* MISRA-C compliance */
            break;
    }        
}


void MC_LowFreqTasks(void)
{
    bool * ptFlg = &(commData.lowFreqLoopSync.flg);
    
    if(true == (*ptFlg)){
        *ptFlg = false;
        MC_SpeedRefPot(&mcFoc1);  
        MC_MotorTrigBtn(&mcFoc1);
    #ifdef MOTOR_2
        MC_SpeedRefPot(&mcFoc2);   
        MC_MotorTrigBtn(&mcFoc2);
    #endif
    #ifdef MOTOR_3
        MC_SpeedRefPot(&mcFoc3); 
        MC_MotorTrigBtn(&mcFoc3);
    #endif     
    }
}


static void MC_DataInitial(T_PMSM_FOC * mcData)
{
    E_MOTOR_CONTROL_APPLICATION app = mcData->instruction.flgApp; 
    T_STATE_DATA * ptFocStt = &(mcData->focState);
    T_STATE_DATA * ptOpenLoopStt = &(mcData->openloopSpin.openLoopState);
    T_STATE_DATA * ptMcStt = &(mcData->mcState);
    T_STATE_DATA * ptBtnStt = &(mcData->instruction.buttonState);    
    T_INSTRUCTION_POT_BUTTON * ptInstr = &(mcData->instruction);
    T_CURRENT * ptCur = &(mcData->current); 
    T_FOC_CURRENT * ptFocCur = &(ptCur->focCurrent);
    T_FOC_CURRENT_CONTROL * ptFocCurCtrl = &(mcData->currentControl);
    T_VECTOR_PASSIVE_TRANSFORM * ptCurPark = &(ptFocCur->parkTrans); 
    T_CLARKE_TRANSFORM * ptCurClarke = &(ptFocCur->clarkeTrans);
    T_PI_CONTROLLER * ptIdPi = &(ptFocCurCtrl->iDpi);
    T_PI_CONTROLLER * ptIqPi = &(ptFocCurCtrl->iQpi); 
    T_FOC_DECOUPLE * ptDecouple = &(ptFocCurCtrl->decouple);      
    T_SPEED_CONTROL * ptSpeedCtrl = &(mcData->speedControl);
    T_PI_CONTROLLER * ptSpeedPi = &(ptSpeedCtrl->pi);    
    T_VOLTAGE * ptVolt = &(mcData->voltage);
    T_FOC_VOLTAGE * ptFocVolt = &(ptVolt->focVoltage);
    T_VECTOR_PASSIVE_TRANSFORM * ptVoltPark = &(ptFocVolt->parkTrans); 
    T_CLARKE_TRANSFORM * ptVoltClarke = &(ptFocVolt->invClarkeTrans);    
    T_SVPWM * ptSvpwm = &(ptVolt->svpwm);
    T_FOC_TORQUE_2_CURRENT * ptT2C = &(mcData->torque2current);
    T_ROTOR_ANGLE_BEMF_ATAN * ptThAtan = &(mcData->rotorAngleAtan);  
    T_IMPOSED_VOLTAGE * ptImposedVolt = &(ptThAtan->imposedVolt);
    T_BEMF_SMO * ptSmo = &(ptThAtan->smo);
    T_SMO_BEMF_OBSERVER * ptEObs = &(ptSmo->eObs);
    T_SMO_CURRENT_OBSERVER * ptIObsAlpha = &(ptSmo->iObsAlpha);   
    T_SMO_CURRENT_OBSERVER * ptIObsBeta = &(ptSmo->iObsBeta);    
    T_SMO_SPEED_INPUT * ptSmoSpeedIn = &(ptThAtan->smoSpeedIn);    
    T_BEMF_2_ROTOR_ANGLE_ATAN * ptBemf2Angle = &(ptThAtan->bemf2Angle);
    T_D_ANGLE_2_SPEED * ptDAngle2Speed = &(ptThAtan->dAngle2Speed);
    T_ROTOR_ANGLE_FLUX_PLL * ptFluxPll = &(mcData->rotorAngleFluxPll);
    T_IMPOSED_VOLTAGE * ptImposedVoltPfo = &(ptFluxPll->imposedVolt);    
    T_P_FLUX_OBSERVER * ptPfo = &(ptFluxPll->pfo); 
    T_MOTION_ESTIMATOR_FLT * ptMotionEst = &(mcData->motionEstimate);
    T_POSITION_TRACKING_PLL_FLT * ptPll = &(ptMotionEst->pll); 
    T_PTPLL_CAIN_CAL_FLT * ptPllGC = &(ptMotionEst->pllGC);
    T_ROTOR_ANGLE_HALL * ptThHall = &(mcData->rotorAngleHall);  
    T_STATE_DATA * ptHallStt = &(ptThHall->hallState);   
    T_ROTOR_ANGLE_ENCODER * ptThEnc = &(mcData->rotorAngleEncoder);
    T_STATE_DATA * ptEncStt = &(ptThEnc->angleState);    
    T_OPENLOOP_SPIN * ptOpenLoop = &(mcData->openloopSpin);    
    T_LOOP_SWITCH * ptLoopSwitch = &(mcData->loopSwitch); 
    T_VECTOR_PASSIVE_TRANSFORM * ptIrefTrans = &(ptLoopSwitch->iRefTrans);
    T_VECTOR_PASSIVE_TRANSFORM * ptVrefTrans = &(ptLoopSwitch->vRefTrans);
    T_FAULT_DETECT * ptFltDtct = &(mcData->faultDetect);
    T_STALL_DETECTION * ptStlDtct = &(ptFltDtct->stallDetection);
    T_PHASE_LOSS_DETECTION * ptPhsLossDtct = &(ptFltDtct->phaseLossDetection);
    float tmp;
    uint16_t tmp16;
    
    /* state machines */
    ptMcStt->cnt = 0u;
    ptMcStt->firstEntry = true;  
    ptMcStt->state = (uint8_t)(mc_idle);
    ptBtnStt->cnt = 0u;
    ptBtnStt->firstEntry = true;
    ptBtnStt->state = (uint8_t)(btn_detect);
    ptHallStt->cnt = 0u;
    ptHallStt->firstEntry = true;
    ptHallStt->state = (uint8_t)(hall_jump_less);
    ptEncStt->cnt = 0u;
    ptEncStt->firstEntry = true;
    
    /* The FOC state machine is mainly treated in DataReset() function.
     * Here it is initialized for align_encoder application. */
    ptFocStt->state = (uint8_t)(mc_open_loop); 
    /* The open-loop parameters are mainly treated in DataReset() function.
     * Here they are initialized for align_encoder application. */
    ptOpenLoopStt->state = (uint8_t)(mc_shadow_alignment);
    ptOpenLoop->thIn = ptOpenLoop->shadowAlignAngle;
    ptOpenLoop->thOut = ptOpenLoop->shadowAlignAngle;
    
    /* instructions */
    ptInstr->dirRef = 1.0f;
    ptInstr->filSpeedRef.in = &(ptInstr->speedRefRaw);
    ptInstr->filSpeedRef.out = &(ptInstr->speedRefFil);
    
    /* current */
    tmp = (float)(ptCur->adcHalfCnt);
    ptCur->adcOffsetU = tmp;
    ptCur->adcOffsetV = tmp;
    ptCur->filAdcU.in = &(ptCur->adcDataU);
    ptCur->filAdcU.out = &(ptCur->adcOffsetU);
    ptCur->filAdcV.in = &(ptCur->adcDataV);
    ptCur->filAdcV.out = &(ptCur->adcOffsetV);
    ptCurPark->x1 = &(ptFocCur->iAlpha);
    ptCurPark->y1 = &(ptFocCur->iBeta);
    ptCurPark->x2 = &(ptFocCur->iD);
    ptCurPark->y2 = &(ptFocCur->iQ);
    ptCurPark->angle = &(ptFocCur->focAngle);
    ptCurPark->angle->ptSinTbl = sineTable;
    ptCurClarke->u = &(ptFocCur->iU);
    ptCurClarke->v = &(ptFocCur->iV);
    ptCurClarke->w = &(ptFocCur->iW); 
    ptCurClarke->alpha = &(ptFocCur->iAlpha);
    ptCurClarke->beta = &(ptFocCur->iBeta);    

    /* BEMF based angle estimation */
    ptImposedVolt->voltTrans.alpha = &(ptImposedVolt->vAlpha);
    ptImposedVolt->voltTrans.beta = &(ptImposedVolt->vBeta);
    ptImposedVolt->voltTrans.u = &(ptImposedVolt->vU);
    ptImposedVolt->voltTrans.v = &(ptImposedVolt->vV);
    ptImposedVolt->voltTrans.w = &(ptImposedVolt->vW);
    ptImposedVolt->cntU = &(ptSvpwm->compareU);
    ptImposedVolt->cntV = &(ptSvpwm->compareV);
    ptImposedVolt->cntW = &(ptSvpwm->compareW);    
    ptImposedVolt->vBus = &(ptVolt->vBus);
    ptImposedVolt->vA = &(ptThAtan->vAlpha);
    ptImposedVolt->vAz = &(ptThAtan->vAlphaZ);
    ptImposedVolt->vB = &(ptThAtan->vBeta);
    ptImposedVolt->vBz = &(ptThAtan->vBetaZ);      
    ptSmoSpeedIn->olEleSpeedRef = &(ptOpenLoop->speedRef);
    ptSmoSpeedIn->speed = &(ptThAtan->speedBemfObs);
    ptSmoSpeedIn->w = &(ptThAtan->w);
    ptSmoSpeedIn->dir = &(ptInstr->dirRef);
    ptEObs->eAlphaHat = &(ptThAtan->eAlphaHat);
    ptEObs->eBetaHat = &(ptThAtan->eBetaHat);  
    ptEObs->eAlphaTilde = &(ptSmo->eAlphaTilde);
    ptEObs->eBetaTilde = &(ptSmo->eBetaTilde);    
    ptEObs->w = &(ptThAtan->w);
    ptIObsAlpha->para = &(ptSmo->iObsPara);
    ptIObsBeta->para = &(ptSmo->iObsPara);
    ptIObsAlpha->i = &(ptFocCur->iAlphaLth);
    ptIObsAlpha->v = &(ptThAtan->vAlphaZ);
    ptIObsAlpha->eHat = &(ptThAtan->eAlphaHat);
    ptIObsAlpha->eTilde = &(ptSmo->eAlphaTilde);
    ptIObsBeta->i = &(ptFocCur->iBetaLth);
    ptIObsBeta->v = &(ptThAtan->vBetaZ);
    ptIObsBeta->eHat = &(ptThAtan->eBetaHat);
    ptIObsBeta->eTilde = &(ptSmo->eBetaTilde);    
    ptBemf2Angle->dir = &(ptInstr->dirRef);
    ptBemf2Angle->rotorAngle = &(ptThAtan->angle);
    ptBemf2Angle->actan.angle = &(ptBemf2Angle->bemfAngle);
    ptBemf2Angle->actan.x = &(ptThAtan->eAlphaHat);
    ptBemf2Angle->actan.y = &(ptThAtan->eBetaHat);
    ptDAngle2Speed->dAngle = &(ptThAtan->dAngle);
    ptDAngle2Speed->filSpeed.in = &(ptDAngle2Speed->speedRaw);
    ptDAngle2Speed->filSpeed.out = &(ptThAtan->speed);
    ptDAngle2Speed->filSpeedBemfObs.in = &(ptDAngle2Speed->speedRaw);
    ptDAngle2Speed->filSpeedBemfObs.out = &(ptThAtan->speedBemfObs);
    
    /* Flux observer */
    ptImposedVoltPfo->phsVoltMode = &(ptVolt->phsVoltMode);
    ptImposedVoltPfo->voltTrans.alpha = &(ptImposedVoltPfo->vAlpha);
    ptImposedVoltPfo->voltTrans.beta = &(ptImposedVoltPfo->vBeta);
    ptImposedVoltPfo->voltTrans.u = &(ptImposedVoltPfo->vU);
    ptImposedVoltPfo->voltTrans.v = &(ptImposedVoltPfo->vV);
    ptImposedVoltPfo->voltTrans.w = &(ptImposedVoltPfo->vW);
    tmp16 = ptVolt->phsVoltMode;
    if(0u == tmp16){  /* Phase voltages are obtained by voltage references. */
        ptImposedVoltPfo->cntU = &(ptSvpwm->compareU);
        ptImposedVoltPfo->cntV = &(ptSvpwm->compareV);
        ptImposedVoltPfo->cntW = &(ptSvpwm->compareW);        
    } else{  /* Phase voltages are obtained by HW detection. */
        ptImposedVoltPfo->cntU = &(ptVolt->fbCntU);
        ptImposedVoltPfo->cntV = &(ptVolt->fbCntV);
        ptImposedVoltPfo->cntW = &(ptVolt->fbCntW);   
    }

    ptImposedVoltPfo->vBus = &(ptVolt->vBus); 
    ptImposedVoltPfo->vA = &(ptFluxPll->vAlpha);
    ptImposedVoltPfo->vAz = &(ptFluxPll->vAlphaZ);
    ptImposedVoltPfo->vB = &(ptFluxPll->vBeta);
    ptImposedVoltPfo->vBz = &(ptFluxPll->vBetaZ);    
    ptPfo->iAlpha = &(ptFocCur->iAlpha);
    ptPfo->iBeta = &(ptFocCur->iBeta);
    ptPfo->iAlphaZ = &(ptFocCur->iAlphaLth);
    ptPfo->iBetaZ = &(ptFocCur->iBetaLth);    
    ptPfo->vAlpha = &(ptFluxPll->vAlpha);
    ptPfo->vBeta = &(ptFluxPll->vBeta);
    ptPfo->vAlphaZ = &(ptFluxPll->vAlphaZ);
    ptPfo->vBetaZ = &(ptFluxPll->vBetaZ);    
    ptPfo->phiAlphaEst = &(ptFluxPll->phiAlphaEst);
    ptPfo->phiBetaEst = &(ptFluxPll->phiBetaEst);
    
    /* Pll */
    ptPll->jerkItg = 0.0f;
    ptPll->pst4Itg = 0.0f;
    ptPll->pst5Itg = 0.0f;   
    ptPll->pst5Est = NULL;
    ptPll->pst4Est = NULL;    
    ptPll->jerkEst = NULL;     
    ptPll->pst5.in = &(ptPll->pst5ItgIn);
    ptPll->pst4.in = &(ptPll->pst4ItgIn);
    ptPll->jerk.in = &(ptPll->jerkItgIn);    
    ptPll->acc.in = &(ptPll->accItgIn);
    ptPll->spd.in = &(ptPll->spdItgIn);
    ptPll->pst.in = &(ptPll->pstItgIn);
    ptPll->pst5.out = &(ptPll->pst5Itg);
    ptPll->pst4.out = &(ptPll->pst4Itg);
    ptPll->jerk.out = &(ptPll->jerkItg);     
    ptPll->acc.out = &(ptPll->accItg);
    ptPll->spd.out = &(ptPll->spdItg);
    ptPll->pst.out = &(ptPll->pstItg);    
    ptPll->pst5Ini = 0.0f;
    ptPll->pst4Ini = 0.0f;
    ptPll->jerkIni = 0.0f;      
    ptPll->accIni = 0.0f;
    ptPll->spdIni = 0.0f;
    ptPll->pstIni = 0.0f; 
    ptPll->pst5Itg = 0.0f;
    ptPll->pst4Itg = 0.0f;
    ptPll->jerkItg = 0.0f;    
    ptPll->accItg = 0.0f;
    ptPll->spdItg = 0.0f;
    ptPll->pstItg = 0.0f;    
    ptPllGC->K = &(ptMotionEst->K);
    ptPllGC->itgPrd = &(ptMotionEst->exePrd);
    ptPllGC->pst5ItgGain = &(ptPll->pst5.gain);
    ptPllGC->pst4ItgGain = &(ptPll->pst4.gain);
    ptPllGC->jerkItgGain = &(ptPll->jerk.gain);    
    ptPllGC->accItgGain = &(ptPll->acc.gain);
    ptPllGC->spdItgGain = &(ptPll->spd.gain);
    ptPllGC->pstItgGain = &(ptPll->pst.gain);
    ptPllGC->pG2 = &(ptPll->pG2);
    ptPllGC->pG3 = &(ptPll->pG3); 
    ptPllGC->pG4 = &(ptPll->pG4);
    ptPllGC->pG5 = &(ptPll->pG5);
    ptPllGC->pG6 = &(ptPll->pG6);  
    MC_PtPllGainCalFlt(ptPllGC);

    /* HALL-based angle estimation */
    if(2u == ptThHall->hallJumpDtctMode){  /* HALL jumps are detected alternatively. */
        ptThHall->angleBetweenHallJump = (float)DEGREE_120_RAD; 
    } else{  /* Each HALL jump is detected. */
        ptThHall->angleBetweenHallJump = (float)DEGREE_60_RAD; 
    }     
    ptThHall->legalHallJumpCnt = 0u;
    ptThHall->isLegalHallJump = false;
    ptThHall->isPtcHalljump = false;
    ptThHall->speedEle = 0.0f;
    ptThHall->speedMech = 0.0f; 
    ptThHall->timerSinceHallJump = ptThHall->timerMax;
    ptThHall->timerSincePtcHallJump = ptThHall->timerMax;
    ptThHall->hallVal = 0u;
    ptThHall->timer = 0u;
    ptThHall->timerIc = 0u;
    
    /* encoder */
    ptThEnc->speedSampleLoopCnt = 0u;
    ptThEnc->isHallCaliDone = false;
    ptThEnc->isIndexDetected = false;   
 
    /* application-dependent terms */
    switch(app){
        case ma_flux_foc:
            ptDecouple->mechSpd = &(ptFluxPll->speedMech);
            ptSpeedCtrl->fbk = &(ptFluxPll->speedMech);
            ptPll->pstErr = &(ptFluxPll->angleErr);    
            ptPll->accEst = &(ptFluxPll->accelerationEle);
            ptPll->spdEst = &(ptFluxPll->speedEle);    
            ptPll->pstEst = &(ptFluxPll->angle); 
            break;
        case ma_hall_foc:
        case ma_hall_flux_foc:
            ptDecouple->mechSpd = &(ptThHall->speedMech);
            ptSpeedCtrl->fbk = &(ptThHall->speedMech);
            ptPll->pstErr = &(ptFluxPll->angleErr);    
            ptPll->accEst = &(ptFluxPll->accelerationEle);
            ptPll->spdEst = &(ptFluxPll->speedEle);    
            ptPll->pstEst = &(ptFluxPll->angle);
            break;
        case ma_align_encoder_foc:
            ptDecouple->mechSpd = &(ptThEnc->speed);
            ptSpeedCtrl->fbk = &(ptThEnc->speed);
            ptPll->pstErr = &(ptThEnc->angleEleErr);    
            ptPll->accEst = &(ptThEnc->accEleDbl);
            ptPll->spdEst = &(ptThEnc->spdEleDbl);    
            ptPll->pstEst = &(ptThEnc->angleEleDbl);
            ptEncStt->state = (uint8_t)(ang_idle);    
            break;
        case ma_hall_encoder_foc: 
            ptDecouple->mechSpd = &(ptThEnc->speed);
            ptSpeedCtrl->fbk = &(ptThEnc->speed);
            ptPll->pstErr = &(ptThEnc->angleEleErr);    
            ptPll->accEst = &(ptThEnc->accEleDbl);
            ptPll->spdEst = &(ptThEnc->spdEleDbl);    
            ptPll->pstEst = &(ptThEnc->angleEleDbl);
            ptEncStt->state = (uint8_t)(ang_hall_wait);
            break;            
        default: /* MISRA-C compliance */
            ptPll->pstErr = NULL;    
            ptPll->accEst = NULL;
            ptPll->spdEst = NULL;    
            ptPll->pstEst = NULL; 
            break;
    }        
    
    /* FOC */
    ptFocCurCtrl->vDref = &(ptFocVolt->vD);
    ptFocCurCtrl->vQref = &(ptFocVolt->vQ);
    ptFocCurCtrl->iDfbk = &(ptFocCur->iD);
    ptFocCurCtrl->iQfbk = &(ptFocCur->iQ);    
    ptIdPi->ref = &(ptFocCurCtrl->iDref);
    ptIdPi->fbk = ptFocCurCtrl->iDfbk;
    ptIqPi->ref = &(ptFocCurCtrl->iQref);
    ptIqPi->fbk = ptFocCurCtrl->iQfbk;  
    ptDecouple->iDfbk = ptFocCurCtrl->iDfbk;
    ptDecouple->iQfbk = ptFocCurCtrl->iQfbk;
    ptDecouple->compD = &(ptFocCurCtrl->compD);
    ptDecouple->compQ = &(ptFocCurCtrl->compQ);    
    ptFocCurCtrl->focState = &(mcData->focState);    
    
    /* speed control */
    ptSpeedCtrl->torqueRef = &(ptT2C->torqueRef);
    ptSpeedCtrl->ref = &(ptInstr->speedRefFil);
    ptSpeedPi->ref = ptSpeedCtrl->ref;
    ptSpeedPi->fbk = ptSpeedCtrl->fbk;

#ifdef ENCODER_FEEDBACK_CONTROL   /* for test */
    T_ROTOR_ANGLE_ENCODER_REF * ptThEncRef = &(mcData->rotorAngleEncoderRef);    
    ptThEncRef->speedSampleLoopCnt = 0u;  
    ptDecouple->mechSpd = &(ptThEncRef->speed);    
    ptSpeedCtrl->fbk = &(ptThEncRef->speed);  
#endif
    
    /* current reference */
#if 0
    ptT2C->filIdRef.in = &(ptT2C->iDrefRaw);
    ptT2C->filIdRef.out = &(ptT2C->iDref);
#else
    ptT2C->rampIdRef.in = &(ptT2C->iDrefRaw);
    ptT2C->rampIdRef.out = &(ptT2C->iDref);    
#endif
    ptT2C->mtpaIdEstimate.iDref = &(ptT2C->iDref);
    ptT2C->mtpaIdEstimate.iQref = &(ptT2C->iQref);
    
    /* voltage */
    ptSvpwm->vBus = &(ptVolt->vBus);
    ptSvpwm->vU = &(ptFocVolt->vU);
    ptSvpwm->vV = &(ptFocVolt->vV);
    ptSvpwm->vW = &(ptFocVolt->vW);
    ptVoltPark->x1 = &(ptFocVolt->vD);
    ptVoltPark->y1 = &(ptFocVolt->vQ);
    ptVoltPark->x2 = &(ptFocVolt->vAlpha);
    ptVoltPark->y2 = &(ptFocVolt->vBeta);
    ptVoltPark->angle = &(ptFocVolt->focAngle);
    ptVoltPark->angle->ptSinTbl = sineTable; 
    ptVoltClarke->u = &(ptFocVolt->vU);
    ptVoltClarke->v = &(ptFocVolt->vV);
    ptVoltClarke->w = &(ptFocVolt->vW);
    ptVoltClarke->alpha = &(ptFocVolt->vAlpha);
    ptVoltClarke->beta = &(ptFocVolt->vBeta);    
    
    /* loop switch */
    ptIrefTrans->x1 = &(ptOpenLoop->iDref);
    ptIrefTrans->y1 = &(ptOpenLoop->iQref);
    ptIrefTrans->x2 = &(ptT2C->iDref);
    ptIrefTrans->y2 = &(ptT2C->iQref);

    ptIrefTrans->angle = &(ptLoopSwitch->angle);
    ptIrefTrans->angle->ptSinTbl = sineTable;
    ptVrefTrans->x1 = &(ptFocVolt->vD);
    ptVrefTrans->y1 = &(ptFocVolt->vQ);  
#if 1
    ptVrefTrans->x2 = &(ptIdPi->integrator);
    ptVrefTrans->y2 = &(ptIqPi->integrator); 
#else    
    ptVrefTrans->x2 = &(ptFocVolt->vD);
    ptVrefTrans->y2 = &(ptFocVolt->vQ);    
#endif
    ptVrefTrans->angle = &(ptLoopSwitch->angle);
    ptVrefTrans->angle->ptSinTbl = sineTable; 
    
    /* stalling detection */
    ptStlDtct->iU = &(ptFocCur->iU);
    ptStlDtct->filIu.in = ptStlDtct->iU;
    ptStlDtct->filIu.out = &(ptStlDtct->iUfil);
    ptStlDtct->iV = &(ptFocCur->iV);
    ptStlDtct->filIv.in = ptStlDtct->iV;
    ptStlDtct->filIv.out = &(ptStlDtct->iVfil);
    ptStlDtct->stallDetected = &(ptFltDtct->fltFlgs.stallDetected);
    
    /* phase loss detection */
    ptPhsLossDtct->iU = &(ptFocCur->iU);
#ifdef PHS_LOSS_TEST    
    ptPhsLossDtct->iV = &(ptFocCur->iVtest);
    ptPhsLossDtct->iW = &(ptFocCur->iWtest);     
#else 
    ptPhsLossDtct->iV = &(ptFocCur->iV);
    ptPhsLossDtct->iW = &(ptFocCur->iW);   
#endif
    ptPhsLossDtct->spd = ptSpeedCtrl->fbk;
    ptPhsLossDtct->phaseLossDetected = &(ptFltDtct->fltFlgs.phaseLossDetected);
    MC_PhsLossDtctIni(ptPhsLossDtct);
}


__STATIC_INLINE void MC_FluxPllReset(T_ROTOR_ANGLE_FLUX_PLL * ptFluxPll)
{    
    ptFluxPll->isInitialized = false; 
    ptFluxPll->speedEle = 0.0f;
    ptFluxPll->speedMech = 0.0f;
    ptFluxPll->accelerationEle = 0.0f;       
}


static void MC_DataReset(T_PMSM_FOC * mcData)
{
    E_MOTOR_CONTROL_APPLICATION app = mcData->instruction.flgApp;  
    E_MOTOR_CONTROL_MODE  mcMode = mcData->instruction.flgMode;
    T_STATE_DATA * ptFocStt = &(mcData->focState);   
    T_OPENLOOP_SPIN * ptOpenLoop = &(mcData->openloopSpin);
    T_STATE_DATA * ptOpenLoopStt = &(ptOpenLoop->openLoopState);
    T_FOC_CURRENT_CONTROL * ptFocCurCtrl = &(mcData->currentControl);    
    T_PI_CONTROLLER * ptIdPi = &(mcData->currentControl.iDpi);
    T_PI_CONTROLLER * ptIqPi = &(mcData->currentControl.iQpi);    
    T_STATE_DATA * ptCurCtrlStt = &(mcData->currentControl.curCtrlState);  
    T_FOC_DECOUPLE * ptDecouple = &(ptFocCurCtrl->decouple);      
    T_SPEED_CONTROL * ptSpeedCtrl = &(mcData->speedControl);    
    T_PI_CONTROLLER * ptSpeedPi = &(mcData->speedControl.pi);
    T_VOLTAGE * ptVolt = &(mcData->voltage);
    T_SVPWM * ptSvpwm = &(ptVolt->svpwm);
    T_INSTRUCTION_POT_BUTTON * ptInstr = &(mcData->instruction);
    T_PMSM_PARA * ptMotorPara = &(mcData->motorPara);
    T_FOC_TORQUE_2_CURRENT * ptT2C = &(mcData->torque2current);  
    T_STATE_DATA * ptMtpaStt = &(ptT2C->mtpaState);
    T_ROTOR_ANGLE_BEMF_ATAN * ptThAtan = &(mcData->rotorAngleAtan);  
    T_BEMF_SMO * ptSmo = &(ptThAtan->smo);    
    T_SMO_SPEED_INPUT * ptSmoSpeedIn = &(ptThAtan->smoSpeedIn); 
    T_STATE_DATA * ptSmoSpeedStt = &(ptSmoSpeedIn->smoSpeedState);  
    T_ROTOR_ANGLE_FLUX_PLL * ptFluxPll = &(mcData->rotorAngleFluxPll);   
    T_P_FLUX_OBSERVER * ptPfo = &(ptFluxPll->pfo);  
    T_IMPOSED_VOLTAGE * ptImposedVoltPfo = &(ptFluxPll->imposedVolt);  
    T_ROTOR_ANGLE_HALL * ptThHall = &(mcData->rotorAngleHall); 
    T_STATE_DATA * ptHallStt = &(ptThHall->hallState);    
    T_ROTOR_ANGLE_ENCODER * ptThEnc = &(mcData->rotorAngleEncoder);
    T_STATE_DATA * ptEncStt = &(ptThEnc->angleState);
    T_POSITION_TRACKING_PLL_FLT * ptPll = &(mcData->motionEstimate.pll);    
    int16_t halfPwmPrdCnt = ptVolt->svpwm.halfPrdCnt; 
    T_FAULT_DETECT * ptFltDtct = &(mcData->faultDetect);
    T_STALL_DETECTION * ptStlDtct = &(ptFltDtct->stallDetection);
    T_PHASE_LOSS_DETECTION * ptPhsLossDtct = &(ptFltDtct->phaseLossDetection);    

    /* application-dependent terms */
    switch(app){
        case ma_flux_foc:
            ptFocStt->state = (uint8_t)(mc_open_loop);
            ptOpenLoopStt->state = (uint8_t)(mc_alignment);
            ptDecouple->mechSpd = &(ptFluxPll->speedMech);
            ptSpeedCtrl->fbk = &(ptFluxPll->speedMech);
            ptOpenLoop->thIn = ptOpenLoop->alignAngle;
            ptOpenLoop->thOut = ptOpenLoop->alignAngle;
            ptInstr->speedRefRaw = (ptMotorPara->minSpeed) * (ptInstr->dirRef);
            ptInstr->speedRefFil = ptInstr->speedRefRaw;
            break;
        case ma_hall_foc:
        case ma_hall_flux_foc:
            ptFocStt->state = (uint8_t)(foc_low_speed_1);
            ptDecouple->mechSpd = &(ptThHall->speedMech);
            ptSpeedCtrl->fbk = &(ptThHall->speedMech);
            ptInstr->speedRefFil = 0.0f;
            break;
        case ma_align_encoder_foc:
            /* The FOC state does not need to be reset to open_loop. */
            /* The open loop state is reset to alignment only if it is in open loop mode. */
            if(md_open_loop == mcMode){
               ptOpenLoopStt->state = (uint8_t)(mc_alignment);
               ptEncStt->state = (uint8_t)(ang_idle);
               ptOpenLoop->thIn = ptOpenLoop->alignAngle;
               ptOpenLoop->thOut = ptOpenLoop->alignAngle;
            }
            ptDecouple->mechSpd = &(ptThEnc->speed);
            ptSpeedCtrl->fbk = &(ptThEnc->speed);
            ptInstr->speedRefFil = 0.0f;
            break;
        default: /* MISRA-C compliance */
            break;
    }  
    
    /* state machines */
    ptFocStt->cnt = 0u;
    ptFocStt->firstEntry = true;    
    ptCurCtrlStt->cnt = 0u;
    ptCurCtrlStt->firstEntry = true;
    ptCurCtrlStt->state = (uint8_t)(cc_basic);
    ptOpenLoopStt->cnt = 0u;
    ptOpenLoopStt->firstEntry = true;
    ptSmoSpeedStt->cnt = 0u;
    ptSmoSpeedStt->firstEntry = true;
    ptSmoSpeedStt->state = (uint8_t)(smo_speed_ref); 
    ptMtpaStt->cnt = 0u;
    ptMtpaStt->firstEntry = true;
    ptMtpaStt->state = (uint8_t)(mtpa_positive_decay);          
    
    /* Speed control */
    ptSpeedCtrl->ref = &(ptInstr->speedRefFil);
    ptSpeedPi->ref = ptSpeedCtrl->ref;
    
    /* PI controllers */
    ptIdPi->integrator = 0.0f;
    ptIqPi->integrator = 0.0f; 
    ptSpeedPi->integrator = 0.0f;
    ptSpeedPi->fbk = ptSpeedCtrl->fbk;
   
    /* Voltage */
    ptVolt->svpwm.compareU = halfPwmPrdCnt;
    ptVolt->svpwm.compareV = halfPwmPrdCnt;  
    ptVolt->svpwm.compareW = halfPwmPrdCnt;     
    MC_PwmDutySet(ptSvpwm); 
#if 1
    ptVolt->fbCntU = halfPwmPrdCnt;
    ptVolt->fbCntV = halfPwmPrdCnt;
    ptVolt->fbCntW = halfPwmPrdCnt;
#endif
     
    /* current reference */
    ptT2C->iDrefRaw = 0.0f;
    ptT2C->iDref = 0.0f;
    
    /* open-loop spin */
    ptOpenLoop->loopCnt = 0u;
    ptOpenLoop->speedRef = 0.0f;
    
    /* BEMF based angle estimation */
    T_IMPOSED_VOLTAGE * ptImposedVolt = &(ptThAtan->imposedVolt);    
    ptImposedVolt->vAlphaZ1 = 0.0f;
    ptImposedVolt->vBetaZ1 = 0.0f; 
    ptThAtan->speed = 0.0f;
    ptThAtan->speedBemfObs = 0.0f;
    ptThAtan->eAlphaHat = 0.0f;
    ptThAtan->eBetaHat = 0.0f;
    ptSmo->iObsAlpha.iHat = 0.0f;
    ptSmo->iObsBeta.iHat = 0.0f;
    ptSmo->eObs.a1 = 0.0f;
    ptSmo->eObs.a2 = 0.0f;
    ptSmo->eObs.b1 = 0.0f;
    ptSmo->eObs.b2 = 0.0f;    
    
    /* flux based angle estimation */
    MC_FluxPllReset(ptFluxPll);       
    ptImposedVoltPfo->vAlpha = 0.0f;
    ptImposedVoltPfo->vBeta = 0.0f;
    ptImposedVoltPfo->vAlphaM1 = 0.0f;
    ptImposedVoltPfo->vBetaM1 = 0.0f;        
    /* Reset total flux for open-loop alignment start up */
#if 0
    ptPfo->psiAlphaEst = ptPfo->psiOl; 
#else
    ptPfo->psiAlphaEst = 0.0f;    
#endif
    ptPfo->psiBetaEst = 0.0f;
    /* Reset total flux for HALL-based start up */

    /* Reset rotor flux. */
    ptPfo->phiEst = ptPfo->phi;  

    /* PLL */
    ptPll->spdItg = 0.0f;  
    ptPll->accItg = 0.0f; 
    /* Position integrator reset depends on applications. */
    
    /* HALL-based angle */
    ptHallStt->cnt = 0u;
    ptHallStt->firstEntry = true;
    ptHallStt->state = (uint8_t)(hall_jump_less); 
    ptThHall->legalHallJumpCnt = 0u;
    ptThHall->isLegalHallJump = false;
    ptThHall->isPtcHalljump = false;
    ptThHall->speedEle = 0.0f;
    ptThHall->speedMech = 0.0f; 
    ptThHall->timerSinceHallJump = ptThHall->timerMax;
    ptThHall->timerSincePtcHallJump = ptThHall->timerMax;
    ptThHall->hallVal = 0u;
    ptThHall->timer = 0u;
    ptThHall->timerIc = 0u;    
    
    /* stalling detection */
    ptStlDtct->iUfil = 0.0f;
    ptStlDtct->iVfil = 0.0f;
    *(ptStlDtct->stallDetected) = false;
    
    /* phase loss detection */
    MC_PhsLossDtctReset(ptPhsLossDtct);
}


void MC_Initial(void)
{
    MC_CommDataInitial();    
    
    MC_ParaConfigMotor1(&mcFoc1);
    MC_DataInitial(&mcFoc1);     
    MC_DataReset(&mcFoc1);   
    
#ifdef MOTOR_2    
    MC_ParaConfigMotor2(&mcFoc2);
    MC_DataInitial(&mcFoc2);    
    MC_DataReset(&mcFoc2);
#endif
    
#ifdef MOTOR_3    
    MC_ParaConfigMotor3(&mcFoc3);
    MC_DataInitial(&mcFoc3);    
    MC_DataReset(&mcFoc3);
#endif
}

#if 0
void MC_PeripheralConnect(void)
{
    MC_PeripheralConnectMotor1(&mcFoc1);
    
#ifdef MOTOR_2      
    MC_PeripheralConnectMotor2(&mcFoc2);
#endif
    
#ifdef MOTOR_3      
    MC_PeripheralConnectMotor3(&mcFoc3);   
#endif
}
#endif


static void MC_LowFreqLoopSync(void)
{
    T_LOOP_SYNC * ptLoopSync = &(commData.lowFreqLoopSync);

    (ptLoopSync->cnt)++;
    
    if(ptLoopSync->cnt >= ptLoopSync->cntLowFreqLoop){
        ptLoopSync->cnt = 0u;
        ptLoopSync->flg = true;
    }
}


__STATIC_INLINE void MC_OpenLoopSttUpdate(T_PMSM_FOC * mcData)
{
    T_STATE_DATA * ptFocStt = &(mcData->focState);  
    T_STATE_DATA * ptCurCtrlStt = &(mcData->currentControl.curCtrlState);    
    T_OPENLOOP_SPIN * ptOpenLoop = &(mcData->openloopSpin);
    T_STATE_DATA * ptOpenLoopStt = &(ptOpenLoop->openLoopState);
    #if 0
    T_PERIPHERAL * ptPeri = &(mcData->peripheral);     
    #endif
    T_ROTOR_ANGLE_ENCODER * ptThEnc = &(mcData->rotorAngleEncoder);
    T_STATE_DATA * ptEncStt = &(ptThEnc->angleState);   
    T_POSITION_TRACKING_PLL_FLT * ptPll = &(mcData->motionEstimate.pll);      
    E_MOTOR_CONTROL_MODE  mcMode = mcData->instruction.flgMode;
    T_STATE_DATA * ptMcStt = &(mcData->mcState);    
    E_MOTOR_CONTROL_APPLICATION app = mcData->instruction.flgApp;
    float dir = mcData->instruction.dirRef;
    float tmp;

    (ptOpenLoop->loopCnt)++; 
    
    /* iQref is set by the MC_ParaConfigMotorX() function. */
    switch(ptOpenLoopStt->state){
        case mc_shadow_alignment:
            /* thIn & thOut are set by DataInitial() function. */
            if(ptOpenLoop->loopCnt >= ptOpenLoop->alignCnt){
                ptOpenLoop->loopCnt = 0u;
                ptOpenLoop->thIn = ptOpenLoop->alignAngle;
                ptOpenLoop->thOut = ptOpenLoop->alignAngle;
                ptOpenLoopStt->state = (uint8_t)(mc_alignment);
            }
            break;
        case mc_alignment:
            /* thIn & thOut are set by DataReset() function. */
            if(ptOpenLoop->loopCnt >= ptOpenLoop->alignCnt){  
                #ifdef ENCODER_FEEDBACK_INFORM  
                T_ROTOR_ANGLE_ENCODER_REF * ptThEncRef = &(mcData->rotorAngleEncoderRef);
                HAL_PositionCountSet(ptThEncRef->rotorFluxAlignQeiCnt, ptPeri->qeiChannel);
                #endif
                ptOpenLoop->loopCnt = 0u;
                if(md_alignment == mcMode){
                    HAL_PwmDisable();
                    ptMcStt->state = (uint8_t)(mc_idle);
                } else if((md_open_loop != mcMode) && (ma_align_encoder_foc == app)){
                    #if 0
                    HAL_PositionCountSet(ptThEnc->rotorFluxAlignQeiCnt, ptPeri->qeiChannel);
                    HAL_IndexEventClear(ptPeri->qeiChannel);
                    #endif
                    ptFocStt->state = (uint8_t)(mc_close_loop);
                    ptCurCtrlStt->state = (uint8_t)(cc_decouple);
                    ptEncStt->state = (uint8_t)(ptThEnc->indexUseState);
                    ptPll->pstItg = 0.0f;
                    ptThEnc->angleEleDbl = ptPll->pstItg;
                    HAL_PwmDisable();
                    ptMcStt->state = (uint8_t)(mc_idle);
                } else{
                    ptOpenLoopStt->state = (uint8_t)(mc_open_loop_acc);
                    #if 0  /* to verify the encoder-based angle */
                    HAL_PositionCountSet(ptThEnc->rotorFluxAlignQeiCnt, ptPeri->qeiChannel);
                    HAL_IndexEventClear(ptPeri->qeiChannel);
                    ptEncStt->state = (uint8_t)(ptThEnc->indexUseState); 
                    ptPll->pstItg = 0.0;
                    ptThEnc->angleEleDbl = ptPll->pstItg;
                    #endif
                }                
            }
            break;
        case mc_open_loop_acc:
            tmp = (ptOpenLoop->dSpeedRef) * (float)(ptOpenLoop->loopCnt);
            ptOpenLoop->speedRef = dir * tmp;
            ptOpenLoop->thIn += ptOpenLoop->speedRef;     
            ptOpenLoop->thIn = MC_Wrap2Pi(ptOpenLoop->thIn); 
            ptOpenLoop->thOut = ptOpenLoop->thIn;
            if(tmp > ptOpenLoop->targetSpeed){
                ptOpenLoop->loopCnt = 0u;
                ptOpenLoopStt->state = (uint8_t)(mc_open_loop_uniform);
            }
            break;
        case mc_open_loop_uniform:
            ptOpenLoop->thIn += ptOpenLoop->speedRef;
            ptOpenLoop->thIn = MC_Wrap2Pi(ptOpenLoop->thIn);
            ptOpenLoop->thOut = ptOpenLoop->thIn;            
            if((ptOpenLoop->loopCnt >= ptOpenLoop->uniformCnt) && (md_open_loop != mcMode)){
                ptFocStt->firstEntry = true;
                ptFocStt->state = (uint8_t)(mc_close_loop);
            }
            break;
        default: /* MISRA-C compliance */
            break;
    }
}


__STATIC_INLINE void MC_FocCurCtrlBasic(T_FOC_CURRENT_CONTROL * ptFocCurCtrl)
{
    T_PI_CONTROLLER * ptIdPi = &(ptFocCurCtrl->iDpi);
    T_PI_CONTROLLER * ptIqPi = &(ptFocCurCtrl->iQpi); 
    
    MC_PiCtrl(ptIdPi);
    MC_PiCtrl(ptIqPi); 
    *(ptFocCurCtrl->vDref) = ptIdPi->out;
    *(ptFocCurCtrl->vQref) = ptIqPi->out;        
}


__STATIC_INLINE void MC_FocCurCtrlDecouple(T_FOC_CURRENT_CONTROL * ptFocCurCtrl)
{
    T_PI_CONTROLLER * ptIdPi = &(ptFocCurCtrl->iDpi);
    T_PI_CONTROLLER * ptIqPi = &(ptFocCurCtrl->iQpi); 
    T_FOC_DECOUPLE * ptDecouple = &(ptFocCurCtrl->decouple);
    float tmp, max;
    
    MC_FocDecoupleCal(ptDecouple);
    MC_PiCtrl(ptIdPi);
    MC_PiCtrl(ptIqPi);
    tmp = ptIdPi->out + ptFocCurCtrl->compD;
    max = ptIdPi->maxOut;
    *(ptFocCurCtrl->vDref) = MC_SymmetricClampF(tmp, max);
    tmp = ptIqPi->out + ptFocCurCtrl->compQ;
    max = ptIqPi->maxOut;
    *(ptFocCurCtrl->vQref) = MC_SymmetricClampF(tmp, max);      
}


__STATIC_INLINE void MC_FocCurCtrlSttUpdate(T_FOC_CURRENT_CONTROL * ptFocCurCtrl)
{
    T_STATE_DATA * ptFocStt = ptFocCurCtrl->focState;
    T_STATE_DATA * ptCurCtrlStt = &(ptFocCurCtrl->curCtrlState);    
    T_PI_CONTROLLER * ptIdPi = &(ptFocCurCtrl->iDpi);
    T_PI_CONTROLLER * ptIqPi = &(ptFocCurCtrl->iQpi);   
    T_FOC_DECOUPLE * ptDecouple = &(ptFocCurCtrl->decouple);
    
    switch(ptCurCtrlStt->state){
        case cc_basic:
            MC_FocCurCtrlBasic(ptFocCurCtrl);
            if(ptFocStt->state == mc_close_loop){
                ptCurCtrlStt->state = (uint8_t)(cc_first_decouple);
            }
            break;
        case cc_first_decouple:
            MC_FocCurCtrlBasic(ptFocCurCtrl);
            MC_FocDecoupleCal(ptDecouple);
            ptIdPi->integrator -= ptFocCurCtrl->compD;
            ptIqPi->integrator -= ptFocCurCtrl->compQ;
            ptCurCtrlStt->state = (uint8_t)(cc_decouple);
            break;
        case cc_decouple:
            MC_FocCurCtrlDecouple(ptFocCurCtrl);
            break;
        default: /* MISRA-C compliance */
            break;
    }
}


__STATIC_INLINE void MC_FocCurCtrl(T_FOC_CURRENT_CONTROL * ptFocCurCtrl)
{
    bool decoupleOn = ptFocCurCtrl->flgDecouple;
    
    if(true == decoupleOn){
        MC_FocCurCtrlDecouple(ptFocCurCtrl);
    } else{
        MC_FocCurCtrlBasic(ptFocCurCtrl);
    }
}

#if 0
static void MC_Spwm(T_SVPWM * ptSvpwm)
{
    float u = *(ptSvpwm->vU);
    float v = *(ptSvpwm->vV);
    float w = *(ptSvpwm->vW);
    float vBus = *(ptSvpwm->vBus);
    int16_t high = ptSvpwm->compareUpBound;
    int16_t low = ptSvpwm->compareLowBound;
    int16_t halfPrdCnt = ptSvpwm->halfPrdCnt;
    float c = ptSvpwm->vMaxDotPrdCnt;
    float pu2cnt = c / vBus;
    int16_t tmp16;    
    
    ptSvpwm->svpwmU = u;
    ptSvpwm->svpwmV = v;    
    ptSvpwm->svpwmW = w;

    tmp16 = (int16_t)(ptSvpwm->svpwmU * pu2cnt + 0.5f) + halfPrdCnt;  
    ptSvpwm->compareU = MC_Clamp(tmp16, low, high);
    tmp16 = (int16_t)(ptSvpwm->svpwmV * pu2cnt + 0.5f) + halfPrdCnt;
    ptSvpwm->compareV = MC_Clamp(tmp16, low, high);    
    tmp16 = (int16_t)(ptSvpwm->svpwmW * pu2cnt + 0.5f) + halfPrdCnt;
    ptSvpwm->compareW = MC_Clamp(tmp16, low, high);    
}
#endif

__STATIC_INLINE void MC_FocCurCtrlOlAlign(T_FOC_CURRENT_CONTROL * ptFocCurCtrl)
{
    bool decoupleOn = ptFocCurCtrl->flgDecouple;
    
    if(true == decoupleOn){
        MC_FocCurCtrlSttUpdate(ptFocCurCtrl);
    } else{
        MC_FocCurCtrlBasic(ptFocCurCtrl);
    }
}


static void MC_Foc(T_PMSM_FOC * mcData)
{
    T_CURRENT * ptCur = &(mcData->current);
    T_FOC_CURRENT * ptFocCur = &(ptCur->focCurrent);
    T_ANGLE * ptCurAngle = &(ptFocCur->focAngle);
    T_VECTOR_PASSIVE_TRANSFORM * ptCurPark = &(ptFocCur->parkTrans);
    T_FOC_CURRENT_CONTROL * ptFocCurCtrl = &(mcData->currentControl);
    T_VOLTAGE * ptVolt = &(mcData->voltage);
    T_FOC_VOLTAGE * ptFocVolt = &(ptVolt->focVoltage); 
    T_ANGLE * ptVoltAngle = &(ptFocVolt->focAngle);    
    T_VECTOR_PASSIVE_TRANSFORM * ptVoltPark = &(ptFocVolt->parkTrans);
    T_CLARKE_TRANSFORM * ptVoltClarke = &(ptFocVolt->invClarkeTrans);  
    E_MOTOR_CONTROL_APPLICATION app = mcData->instruction.flgApp; 
    E_FOC_ANGLE_MODE angleMode = mcData->instruction.flgAngle;
    T_ROTOR_ANGLE_FLUX_PLL * ptFluxPll = &(mcData->rotorAngleFluxPll);   
    float phiAlpha = ptFluxPll->phiAlphaEst;
    float phiBeta = ptFluxPll->phiBetaEst;      
    uint8_t focStt = mcData->focState.state;     
    
    switch(app){
        case ma_flux_foc:
            /*  */
            if((fa_flux_single_angle == angleMode)&&(mc_close_loop == focStt)){
                ptCurAngle->cosine = phiAlpha;
                ptCurAngle->sine = phiBeta;
                ptVoltAngle->cosine = phiAlpha;
                ptVoltAngle->sine = -phiBeta;    
            } else{
                MC_SinCosCalc(ptCurAngle);
                ptVoltAngle->cosine = ptCurAngle->cosine;
                ptVoltAngle->sine = -(ptCurAngle->sine);
            }
            MC_ParkTransform(ptCurPark);
            MC_FocCurCtrlOlAlign(ptFocCurCtrl); 
            break;
        case ma_hall_foc:
            MC_SinCosCalc(ptCurAngle);
            ptVoltAngle->cosine = ptCurAngle->cosine;
            ptVoltAngle->sine = -(ptCurAngle->sine);
            MC_ParkTransform(ptCurPark);
            MC_FocCurCtrl(ptFocCurCtrl);
            break;
        case ma_hall_flux_foc:
            if((fa_flux_single_angle == angleMode)&&(foc_low_speed_1 != focStt)&&(foc_low_speed_2 != focStt)){
                ptCurAngle->cosine = phiAlpha;
                ptCurAngle->sine = phiBeta;
                ptVoltAngle->cosine = phiAlpha;
                ptVoltAngle->sine = -phiBeta;    
            } else{
                MC_SinCosCalc(ptCurAngle);
                ptVoltAngle->cosine = ptCurAngle->cosine;
                ptVoltAngle->sine = -(ptCurAngle->sine);
            }
            MC_ParkTransform(ptCurPark);
            MC_FocCurCtrl(ptFocCurCtrl);
            break;
        case ma_align_encoder_foc:
            MC_SinCosCalc(ptCurAngle);
            ptVoltAngle->cosine = ptCurAngle->cosine;
            ptVoltAngle->sine = -(ptCurAngle->sine);
            MC_ParkTransform(ptCurPark);
            MC_FocCurCtrlOlAlign(ptFocCurCtrl); 
            break;
        default: /* MISRA-C compliance */
            break;
    }      
    
    if(fa_dual_angle == angleMode){
        MC_SinCosCalc(ptVoltAngle);
    }
    
    MC_ParkTransform(ptVoltPark);
    MC_InvClarkeTransform(ptVoltClarke);
}


__STATIC_INLINE void MC_PwmModulate(T_SVPWM * ptSvpwm)
{
#if 1
    MC_Svpwm(ptSvpwm);
#else
    MC_Spwm(ptSvpwm);    
#endif
    
    MC_PwmDutySet(ptSvpwm);
}


__STATIC_INLINE void MC_FocOlAlign(T_PMSM_FOC * mcData)
{
    T_CURRENT * ptCur = &(mcData->current);
    T_FOC_CURRENT * ptFocCur = &(ptCur->focCurrent);
    T_ANGLE * ptCurAngle = &(ptFocCur->focAngle);
    T_VECTOR_PASSIVE_TRANSFORM * ptCurPark = &(ptFocCur->parkTrans);
    T_FOC_CURRENT_CONTROL * ptFocCurCtrl = &(mcData->currentControl);
    T_VOLTAGE * ptVolt = &(mcData->voltage);
    T_FOC_VOLTAGE * ptFocVolt = &(ptVolt->focVoltage); 
    T_ANGLE * ptVoltAngle = &(ptFocVolt->focAngle);    
    T_VECTOR_PASSIVE_TRANSFORM * ptVoltPark = &(ptFocVolt->parkTrans);
    T_CLARKE_TRANSFORM * ptVoltClarke = &(ptFocVolt->invClarkeTrans);
    T_SVPWM * ptSvpwm = &(ptVolt->svpwm);    
    
    MC_SinCosCalc(ptCurAngle);
    MC_ParkTransform(ptCurPark);
    MC_FocCurCtrlOlAlign(ptFocCurCtrl);
    MC_SinCosCalc(ptVoltAngle);    
    MC_ParkTransform(ptVoltPark);
    MC_InvClarkeTransform(ptVoltClarke);
    MC_Svpwm(ptSvpwm);
}


__STATIC_INLINE void MC_SpeedCtrl(T_SPEED_CONTROL * ptSpeedCtrl)
{
    T_PI_CONTROLLER * ptPi = &(ptSpeedCtrl->pi);
    
    MC_PiCtrl(ptPi);
    *(ptSpeedCtrl->torqueRef) = ptPi->out; 
}


__STATIC_INLINE void MC_T2cMtpaOff(T_FOC_TORQUE_2_CURRENT * ptT2C)
{    
    /* Id reference */
#if 0
    T_1ST_LPF_PT * ptFilIdRef = &(ptT2C->filIdRef);    
    MC_1st_Lpf(ptFilIdRef);
#else
    T_LINEAR_RAMP * ptRampIdRef = &(ptT2C->rampIdRef);      
    MC_LinearRamp(ptRampIdRef);
#endif
    
    /* Iq reference */
    ptT2C->iQref = ptT2C->torqueRef * ptT2C->nm2amps;    
}


__STATIC_INLINE void MC_T2cMtpaOn(T_FOC_TORQUE_2_CURRENT * ptT2C)
{    
    T_LINEAR_RAMP * ptRampIdRef = &(ptT2C->rampIdRef);  
    T_MTPA_ID_ESTIMATE * ptIdEst = &(ptT2C->mtpaIdEstimate);
    T_STATE_DATA * ptMtpaStt = &(ptT2C->mtpaState);
    float kt1 = ptT2C->Kt;
    float kt2 = ptT2C->KtRlct;
    float tmp;
    
    /* Id reference */
    switch(ptMtpaStt->state){
        case mtpa_positive_decay:            
            if(FLT_EPSILON > ptT2C->iDref){
                ptMtpaStt->state = (uint8_t)(mtpa_negative_estimate);
            } else{
                MC_LinearRamp(ptRampIdRef);
            }
            break;
        case mtpa_negative_estimate:
            MC_MptaIdEstimate(ptIdEst);
            break;
        default: /* MISRA-C compliance */
            break;
    }
    
    /* Iq reference */
    tmp = kt1 + (kt2 * ptT2C->iDref);
    ptT2C->iQref = ptT2C->torqueRef / tmp;
}


__STATIC_INLINE void MC_Torque2current(T_FOC_TORQUE_2_CURRENT * ptT2C)
{    
    if(false == ptT2C->flgMtpa){
        MC_T2cMtpaOff(ptT2C);
    } else{  /* MTPA on */
        MC_T2cMtpaOn(ptT2C);
    }    
}


__STATIC_INLINE void MC_LoopSwitch(T_PMSM_FOC * mcData)
{
    T_OPENLOOP_SPIN * ptOpenLoop = &(mcData->openloopSpin);
    T_LOOP_SWITCH * ptLoopSwitch = &(mcData->loopSwitch);
    T_VECTOR_PASSIVE_TRANSFORM * ptIrefTrans = &(ptLoopSwitch->iRefTrans);
    T_VECTOR_PASSIVE_TRANSFORM * ptVrefTrans = &(ptLoopSwitch->vRefTrans);        
    T_ANGLE * ptAngle = &(ptLoopSwitch->angle);    
    T_PI_CONTROLLER * ptSpeedPi = &(mcData->speedControl.pi);
    T_PMSM_PARA * ptMotorPara = &(mcData->motorPara); 
    T_FOC_TORQUE_2_CURRENT * ptT2C = &(mcData->torque2current);   
#if 0
    T_PI_CONTROLLER * ptIdPi = &(mcData->currentControl.iDpi);
    T_PI_CONTROLLER * ptIqPi = &(mcData->currentControl.iQpi);   
    T_FOC_VOLTAGE * ptFocVolt = &(mcData->voltage.focVoltage);    
#endif
    float tmp;
    float kt1 = ptMotorPara->Kt;
    float kt2 = ptMotorPara->KtRlct; 
    float iqRef;
    float idRef;    
    
    /* Update open-loop angle reference. */
    ptOpenLoop->thIn += ptOpenLoop->speedRef;
    ptOpenLoop->thIn = MC_Wrap2Pi(ptOpenLoop->thIn);
    
    /* Calculate switch angle. */
#ifdef ENCODER_FEEDBACK_CONTROL
    T_ROTOR_ANGLE_ENCODER_REF * ptThEncRef = &(mcData->rotorAngleEncoderRef);    
    tmp = ptThEncRef->thIn - ptOpenLoop->thIn;
#else
    #if 1  /* Use flux-based angle. */
    T_ROTOR_ANGLE_FLUX_PLL * ptFluxPll = &(mcData->rotorAngleFluxPll);
    tmp = ptFluxPll->angleIn - ptOpenLoop->thIn;
    #else  /* Use BEMF-based angle. */
    T_ROTOR_ANGLE_BEMF_ATAN * ptThAtan = &(mcData->rotorAngleAtan);
    tmp = ptThAtan->angle - ptOpenLoop->thIn;
    #endif
#endif
    ptAngle->th = MC_Wrap2Pi(tmp);
    MC_SinCosCalc(ptAngle);
    
    /* Coordinate transform current reference vector; set speed control integrator. */
    MC_ParkTransform(ptIrefTrans);
    iqRef = ptT2C->iQref;
    idRef = ptT2C->iDref;
    if(false == ptT2C->flgMtpa){
        ptSpeedPi->integrator = kt1 * iqRef;
    } else{  /* MTPA on */
        ptSpeedPi->integrator = (kt1 * iqRef) + (kt2 * idRef * iqRef);
    }
        
    /* Coordinate transform voltage reference vector; set current control integrator. */
    MC_ParkTransform(ptVrefTrans); 
#if 0
    ptIdPi->out = ptFocVolt->vD;
    ptIqPi->out = ptFocVolt->vQ;
#endif    
}

#if 0
__STATIC_INLINE void MC_VoltCalc(T_IMPOSED_VOLTAGE * ptVolt)
{
    float cnt2Volt = (*(ptVolt->vBus)) * (ptVolt->oneByPwmPrdCnt);
    
    ptVolt->vU = ptVolt->lthCntU * cnt2Volt;
    ptVolt->vV = ptVolt->lthCntV * cnt2Volt;
    ptVolt->vW = ptVolt->lthCntW * cnt2Volt;

    MC_ClarkeTransform(&(ptVolt->voltTrans));
    
    /* Update cntU, cntV, cntW.
     * This update shall be after the variables are used. */
    ptVolt->lthCntU = (float)(*(ptVolt->cntU));
    ptVolt->lthCntV = (float)(*(ptVolt->cntV));
    ptVolt->lthCntW = (float)(*(ptVolt->cntW));
}
#else
__STATIC_INLINE void MC_VoltCalc(T_IMPOSED_VOLTAGE * ptVolt)
{
    float cnt2Volt = (*(ptVolt->vBus)) * (ptVolt->oneByPwmPrdCnt);
    uint16_t tmp16 = *(ptVolt->phsVoltMode);
    
    ptVolt->vAlphaZ1 = ptVolt->vAlpha;
    ptVolt->vBetaZ1 = ptVolt->vBeta;
    
    ptVolt->vU = (float)(*(ptVolt->cntU)) * cnt2Volt;
    ptVolt->vV = (float)(*(ptVolt->cntV)) * cnt2Volt;
    ptVolt->vW = (float)(*(ptVolt->cntW)) * cnt2Volt;

    MC_ClarkeTransform(&(ptVolt->voltTrans));
    
    ptVolt->vAlphaM2 = ptVolt->vAlphaM1;
    ptVolt->vBetaM2 = ptVolt->vBetaM1;      
    ptVolt->vAlphaM1 = 0.5f * (ptVolt->vAlphaZ1 + ptVolt->vAlpha);
    ptVolt->vBetaM1 = 0.5f * (ptVolt->vBetaZ1 + ptVolt->vBeta); 
    
    *(ptVolt->vA) = ptVolt->vAlphaM1;
    *(ptVolt->vB) = ptVolt->vBetaM1;
    
    if(0u == tmp16){  /* Phase voltages are obtained by voltage references. */
        *(ptVolt->vAz) = ptVolt->vAlphaM2;
        *(ptVolt->vBz) = ptVolt->vBetaM2;  
    } else{  /* Phase voltages are obtained by HW detection. */
        *(ptVolt->vAz) = ptVolt->vAlphaM1;
        *(ptVolt->vBz) = ptVolt->vBetaM1;  
    }
      
}
#endif


__STATIC_INLINE void MC_SmoSpeedCalc(T_SMO_SPEED_INPUT * ptSmoSpeedIn)
{
    float speedRef;
    float threshold = ptSmoSpeedIn->switchSpeed;
#if 0
    float dir = *(ptSmoSpeedIn->dir);
#endif
    float * w = ptSmoSpeedIn->w;
    T_STATE_DATA * ptStt = &(ptSmoSpeedIn->smoSpeedState);    
    
    switch(ptStt->state){
        case smo_speed_ref:
            speedRef = *(ptSmoSpeedIn->olEleSpeedRef) * ptSmoSpeedIn->perLoop2PerSec;
            *w = speedRef;
            if(MC_Abs(speedRef) > threshold){
                ptStt->state = (uint8_t)(smo_speed_fbk);
            }
            break;
        case smo_speed_fbk:
            speedRef = *(ptSmoSpeedIn->speed) * ptSmoSpeedIn->polePairs;
        #if 0
            if((dir > 0.0f) && (speedRef < 0.0f)){
                *w = FLT_EPSILON;
            } else if((dir < 0.0f) && (speedRef > 0.0f)){
                *w = -FLT_EPSILON;
            } else{
                *w = speedRef;
            }     
        #else
            *w = speedRef;
        #endif
            break;
        default: /* MISRA-C compliance */
            break;               
    }
}


__STATIC_INLINE void MC_BemfSmo(T_BEMF_SMO * ptSmoData)
{
    T_SMO_BEMF_OBSERVER * ptEObs = &(ptSmoData->eObs);
    
    MC_CurrentObserver(&(ptSmoData->iObsAlpha));
    MC_CurrentObserver(&(ptSmoData->iObsBeta));  
    MC_BemfObsCoeffCal(ptEObs);
    MC_BemfObserver(ptEObs);
}


__STATIC_INLINE void MC_Bemf2AngleAtan(T_BEMF_2_ROTOR_ANGLE_ATAN * ptBemf2Angle)
{
    float dir = *(ptBemf2Angle->dir);
    float tmp;
    
    MC_Atan(&(ptBemf2Angle->actan));
    tmp = ptBemf2Angle->bemfAngle - (dir * (float)M_PI_2);   
    *(ptBemf2Angle->rotorAngle) = MC_Wrap2Pi(tmp); 
}


__STATIC_INLINE void MC_Angle2Speed(T_D_ANGLE_2_SPEED * ptDAngle2Speed)
{
    float dTh = *(ptDAngle2Speed->dAngle);
    
    ptDAngle2Speed->speedRaw = dTh * (ptDAngle2Speed->elePerLoop2MechPerSec);
    MC_1st_Lpf(&(ptDAngle2Speed->filSpeed));
    MC_1st_Lpf(&(ptDAngle2Speed->filSpeedBemfObs));    
}


__STATIC_INLINE void MC_BemfAngleAtan(T_ROTOR_ANGLE_BEMF_ATAN * ptThAtan)
{
    T_BEMF_SMO * ptSmo = &(ptThAtan->smo);
    T_SMO_SPEED_INPUT * ptSmoSpeedIn = &(ptThAtan->smoSpeedIn);     
    T_IMPOSED_VOLTAGE * ptVolt = &(ptThAtan->imposedVolt);
    T_BEMF_2_ROTOR_ANGLE_ATAN * ptBemf2Angle = &(ptThAtan->bemf2Angle);
    T_D_ANGLE_2_SPEED * ptDAngle2Speed = &(ptThAtan->dAngle2Speed);
    
    /* Calculate imposed voltage. */
    MC_VoltCalc(ptVolt);  
    
    /* Calculate input speed of the SMO. */
    MC_SmoSpeedCalc(ptSmoSpeedIn);
    
    /* Estimate BEMF. */
    MC_BemfSmo(ptSmo);
    
    /* Latch rotor flux angle. */
    ptThAtan->angleLth = ptThAtan->angle;
    
    /* Calculate rotor flux angle. */
    MC_Bemf2AngleAtan(ptBemf2Angle);
    
    /* Calculate the angles for Park transform and inverse Park transform. */
    ptThAtan->dAngle = MC_WrapPi(ptThAtan->angle - ptThAtan->angleLth);
    ptThAtan->angleIn = ptThAtan->angle;
    ptThAtan->angleOut = MC_Wrap2Pi(ptThAtan->angle + ptThAtan->dAngle);
    
    /* Calculate rotor shaft angular speed. */
    MC_Angle2Speed(ptDAngle2Speed);
}


__STATIC_INLINE void MC_PmsmFocBemfSttUpdate(T_PMSM_FOC * mcData)
{
    T_STATE_DATA * ptFocStt = &(mcData->focState);
    T_ROTOR_ANGLE_BEMF_ATAN * ptThAtan = &(mcData->rotorAngleAtan);
    T_SPEED_CONTROL * ptSpeedCtrl = &(mcData->speedControl);
    T_FOC_CURRENT_CONTROL * ptFocCurCtrl = &(mcData->currentControl);    
    T_FOC_TORQUE_2_CURRENT * ptT2C = &(mcData->torque2current); 
    T_FOC_CURRENT * ptFocCur = &(mcData->current.focCurrent);    
#ifdef ENCODER_FEEDBACK_CONTROL    
    T_ROTOR_ANGLE_ENCODER_REF * ptThEncRef = &(mcData->rotorAngleEncoderRef);
#endif
    T_FOC_VOLTAGE * ptFocVolt = &(mcData->voltage.focVoltage);
    T_OPENLOOP_SPIN * ptOpenLoop = &(mcData->openloopSpin);
    
    switch(ptFocStt->state){
        case mc_open_loop:
            MC_OpenLoopSttUpdate(mcData);
            ptFocCurCtrl->iDref = ptOpenLoop->iDref;
            ptFocCurCtrl->iQref = ptOpenLoop->iQref;
            ptFocCur->focAngle.th = ptOpenLoop->thIn;
            ptFocVolt->focAngle.th = (float)M_2PI - ptOpenLoop->thOut;            
            break;
        case mc_close_loop:
            if(true == ptFocStt->firstEntry){               
                MC_LoopSwitch(mcData);
                ptSpeedCtrl->fbk = &(ptThAtan->speed);
                ptFocStt->firstEntry = false;          
            }
            MC_SpeedCtrl(ptSpeedCtrl);
            MC_Torque2current(ptT2C);
            ptFocCurCtrl->iDref = ptT2C->iDref;
            ptFocCurCtrl->iQref = ptT2C->iQref;                     
        #ifdef ENCODER_FEEDBACK_CONTROL
            ptFocCur->focAngle.th = ptThEncRef->thIn;
            ptFocVolt->focAngle.th = (float)M_2PI - ptThEncRef->thOut;
        #else
            ptFocCur->focAngle.th = ptThAtan->angleIn;
            ptFocVolt->focAngle.th = (float)M_2PI - ptThAtan->angleOut;            
        #endif
            break;
        default: /* MISRA-C compliance */
            break;
    }    
}

#if 0
static void MC_PmsmFocBemf(T_PMSM_FOC * mcData)
{
    T_ROTOR_ANGLE_BEMF_ATAN * ptThAtan = &(mcData->rotorAngleAtan);

    /* Estimate BEMF and rotor flux angle. */
    MC_BemfAngleAtan(ptThAtan);
    
    /* Update BEMF-based sensor-less FOC state. */
    MC_PmsmFocBemfSttUpdate(mcData);

    /* Execute field oriented control. */
    MC_Foc(mcData);
#if 0    
    /* Set PWM duty. */
    MC_PwmDutySet(mcData); 
#endif
}
#endif

__STATIC_INLINE void MC_PmsmFocFluxSttUpdate(T_PMSM_FOC * mcData)
{
    T_STATE_DATA * ptFocStt = &(mcData->focState);     
    T_ROTOR_ANGLE_FLUX_PLL * ptFluxPll = &(mcData->rotorAngleFluxPll);    
    T_SPEED_CONTROL * ptSpeedCtrl = &(mcData->speedControl);
    T_FOC_CURRENT_CONTROL * ptFocCurCtrl = &(mcData->currentControl);    
    T_FOC_TORQUE_2_CURRENT * ptT2C = &(mcData->torque2current); 
    T_FOC_CURRENT * ptFocCur = &(mcData->current.focCurrent);    
    T_FOC_VOLTAGE * ptFocVolt = &(mcData->voltage.focVoltage);
    T_OPENLOOP_SPIN * ptOpenLoop = &(mcData->openloopSpin);    
    
    switch(ptFocStt->state){
        case mc_open_loop:
            MC_OpenLoopSttUpdate(mcData);
            ptFocCurCtrl->iDref = ptOpenLoop->iDref;
            ptFocCurCtrl->iQref = ptOpenLoop->iQref;
            ptFocCur->focAngle.th = ptOpenLoop->thIn;
            ptFocVolt->focAngle.th = (float)M_2PI - ptOpenLoop->thOut;            
            break;
        case mc_close_loop:
            if(true == ptFocStt->firstEntry){
                MC_LoopSwitch(mcData);
                ptSpeedCtrl->fbk = &(ptFluxPll->speedMech);
                ptFocStt->firstEntry = false;          
            }
            MC_SpeedCtrl(ptSpeedCtrl);
            MC_Torque2current(ptT2C);
            ptFocCurCtrl->iDref = ptT2C->iDref;
            ptFocCurCtrl->iQref = ptT2C->iQref;                     
        #ifdef ENCODER_FEEDBACK_CONTROL
            T_ROTOR_ANGLE_ENCODER_REF * ptThEncRef = &(mcData->rotorAngleEncoderRef);
            ptFocCur->focAngle.th = ptThEncRef->thIn;
            ptFocVolt->focAngle.th = (float)M_2PI - ptThEncRef->thOut;
        #else
            ptFocCur->focAngle.th = ptFluxPll->angleIn;
            ptFocVolt->focAngle.th = (float)M_2PI - ptFluxPll->angleOut;  
        #endif
            break;
        default: /* MISRA-C compliance */
            break;
    }    
}


__STATIC_INLINE void MC_PmsmFocAlignEncSttUpdate(T_PMSM_FOC * mcData)
{
    T_STATE_DATA * ptFocStt = &(mcData->focState);     
    T_ROTOR_ANGLE_ENCODER * ptThEnc = &(mcData->rotorAngleEncoder);    
    T_SPEED_CONTROL * ptSpeedCtrl = &(mcData->speedControl);
    T_FOC_CURRENT_CONTROL * ptFocCurCtrl = &(mcData->currentControl);    
    T_FOC_TORQUE_2_CURRENT * ptT2C = &(mcData->torque2current); 
    T_FOC_CURRENT * ptFocCur = &(mcData->current.focCurrent);    
    T_FOC_VOLTAGE * ptFocVolt = &(mcData->voltage.focVoltage);
    T_OPENLOOP_SPIN * ptOpenLoop = &(mcData->openloopSpin);
    
    switch(ptFocStt->state){
        case mc_open_loop:
            MC_OpenLoopSttUpdate(mcData);
            ptFocCurCtrl->iDref = ptOpenLoop->iDref;
            ptFocCurCtrl->iQref = ptOpenLoop->iQref;
            ptFocCur->focAngle.th = ptOpenLoop->thIn;
            ptFocVolt->focAngle.th = (float)M_2PI - ptOpenLoop->thOut;            
            break;
        case mc_close_loop:
            MC_SpeedCtrl(ptSpeedCtrl);
            MC_Torque2current(ptT2C);
            ptFocCurCtrl->iDref = ptT2C->iDref;
            ptFocCurCtrl->iQref = ptT2C->iQref;                     
            ptFocCur->focAngle.th = ptThEnc->thIn;
            ptFocVolt->focAngle.th = (float)M_2PI - ptThEnc->thOut; 
            break;
        default: /* MISRA-C compliance */
            break;
    }    
}


#if 0
__STATIC_INLINE void MC_FluxObsGainCal(T_PMSM_FOC * mcData)
{
    T_ROTOR_ANGLE_FLUX_PLL * ptFluxPll = &(mcData->rotorAngleFluxPll); 
    T_P_FLUX_OBSERVER * ptPfo = &(ptFluxPll->pfo);    
    float spdRefAbs = MC_Abs(mcData->instruction.speedRefFil);
    float a = ptFluxPll->pfoGainA;
    float b = ptFluxPll->pfoGainB;
    float tmp = a * spdRefAbs + b;
    
    ptPfo->gPhi = tmp;
    ptPfo->gPsi = 2.0f * tmp;    
}
#else
__STATIC_INLINE void MC_FluxObsGainCal(T_PMSM_FOC * mcData)
{
    T_ROTOR_ANGLE_FLUX_PLL * ptFluxPll = &(mcData->rotorAngleFluxPll); 
    T_P_FLUX_OBSERVER * ptPfo = &(ptFluxPll->pfo);    
    float spdRefAbs = MC_Abs(mcData->instruction.speedRefFil);
    float minSpd = mcData->motorPara.minSpeed;
    float a = ptFluxPll->pfoGainA;
    float b = ptFluxPll->pfoGainB;
    float tmp,spd;
    
    if(spdRefAbs > minSpd){
        spd = spdRefAbs; 
    } else{
        spd = minSpd;
    }
    
    tmp = a + (b / spd);
    ptPfo->gPhi = tmp;
    ptPfo->gPsi = 2.0f * tmp;    
}
#endif


__STATIC_INLINE void MC_FluxPll(T_PMSM_FOC * mcData)
{
    T_ROTOR_ANGLE_FLUX_PLL * ptFluxPll = &(mcData->rotorAngleFluxPll);   
    T_POSITION_TRACKING_PLL_FLT * ptPll = &(mcData->motionEstimate.pll);         
    float th, sine, cosine, angle;
    uint8_t a, b, ab;
    bool tmpB;
    
    /* Initialize estimated angle. */
    if(ptFluxPll->isInitialized == false){
        tmpB = ptFluxPll->phiAlphaEst > 0.0f;
        a = (uint8_t)(tmpB); 
        tmpB = ptFluxPll->phiBetaEst > 0.0f;
        b = (uint8_t)(tmpB); 
        ab = (a * 2u) + b;  
        ptPll->pstItg = (float)(commData.angleIniTbl[ab]);
        ptFluxPll->angle = ptPll->pstItg;
        ptFluxPll->isInitialized = true;
    }
 
    /* Latch rotor flux angle. */
    th = (float)(ptFluxPll->angle);
    ptFluxPll->angleLth = th;
    
    /* Calculate angle estimation error. */
    sine = MC_SinTaylor(th);
    cosine = MC_CosTaylor(th);
    /* phiAlpha & phiBeta shall be normalized. */
    ptFluxPll->angleErr = (float)((ptFluxPll->phiBetaEst * cosine) - (ptFluxPll->phiAlphaEst * sine));
    
    /* Execute angle tracking pll. */
    MC_PositionTrackingPllFlt(ptPll);
    
    /* Wrap the angle in [0, 2pi]. */
    if(ptFluxPll->angle > (float)M_2PI){
        ptPll->pstItg = ptPll->pstItg - (float)M_2PI;
        ptFluxPll->angle = ptPll->pstItg;
    } else if(ptFluxPll->angle < 0.0f){
        ptPll->pstItg = ptPll->pstItg + (float)M_2PI;
        ptFluxPll->angle = ptPll->pstItg;
    } else {}  /* MISRA-C compliance */

    /* Calculate the angles for Park transform and inverse Park transform. */
    angle = (float)(ptFluxPll->angle);
    ptFluxPll->dAngle = MC_WrapPi(angle - ptFluxPll->angleLth);    
    ptFluxPll->angleIn = angle;
    ptFluxPll->angleOut = MC_Wrap2Pi(angle + ptFluxPll->dAngle);
    
    /* Calculate mechanical speed. */
    ptFluxPll->speedMech = (float)(ptFluxPll->speedEle) * ptFluxPll->oneByNumPolePairs;
}


/* Flux observer + PLL for open-loop alignment scenario. */
__STATIC_INLINE void MC_FluxAnglePllOlAlign(T_PMSM_FOC * mcData)
{
    T_ROTOR_ANGLE_FLUX_PLL * ptFluxPll = &(mcData->rotorAngleFluxPll); 
    T_IMPOSED_VOLTAGE * ptVolt = &(ptFluxPll->imposedVolt);     
    T_P_FLUX_OBSERVER * ptPfo = &(ptFluxPll->pfo);         
    uint8_t olStt = mcData->openloopSpin.openLoopState.state;  
 
    /* Calculate imposed voltage. */
    MC_VoltCalc(ptVolt);

    /* Calculate flux observer gains. */
    MC_FluxObsGainCal(mcData);
    
    /* Estimate rotor flux. */
    MC_FluxObsEuler(ptPfo);  
       
    /* Estimate flux angle & speed. */
    if(olStt >= mc_open_loop_uniform){
        MC_FluxPll(mcData);
    }    
}


/* Flux observer + PLL for closed-loop start scenario. */
__STATIC_INLINE void MC_FluxAnglePllClStart(T_PMSM_FOC * mcData)
{
    T_ROTOR_ANGLE_FLUX_PLL * ptFluxPll = &(mcData->rotorAngleFluxPll); 
    T_IMPOSED_VOLTAGE * ptVolt = &(ptFluxPll->imposedVolt);     
    T_P_FLUX_OBSERVER * ptPfo = &(ptFluxPll->pfo); 
    uint8_t focStt = mcData->focState.state;    
 
    /* Calculate imposed voltage. */
    MC_VoltCalc(ptVolt);

    /* Calculate flux observer gains. */
    MC_FluxObsGainCal(mcData);
    
    /* Estimate rotor flux. */
    MC_FluxObsEuler(ptPfo);  
          
    /* Estimate flux angle & speed. */
    if(focStt >= foc_engage_pll){
        MC_FluxPll(mcData);
    }    
}  


static void MC_PmsmFocFlux(T_PMSM_FOC * mcData)
{         
    /* Estimate rotor flux vector and its angle. */
    MC_FluxAnglePllOlAlign(mcData);
   
    /* Update flux-based sensor-less FOC state. */
    MC_PmsmFocFluxSttUpdate(mcData);

    /* Execute field oriented control. */
    MC_Foc(mcData);
#if 0    
    /* Set PWM duty. */
    MC_PwmDutySet(mcData); 
#endif
}

#if 0
static void MC_PmsmFocAlignEnc(T_PMSM_FOC * mcData)
{          
    /* Update state machine. */
    MC_PmsmFocAlignEncSttUpdate(mcData);

    /* Execute field oriented control. */
    MC_Foc(mcData);
#if 0    
    /* Set PWM duty. */
    MC_PwmDutySet(mcData); 
#endif
}
#endif

static void MC_PmsmFocHall(T_PMSM_FOC * mcData)
{     
    T_SPEED_CONTROL * ptSpeedCtrl = &(mcData->speedControl);
    T_FOC_CURRENT_CONTROL * ptFocCurCtrl = &(mcData->currentControl);    
    T_FOC_TORQUE_2_CURRENT * ptT2C = &(mcData->torque2current); 
    T_FOC_CURRENT * ptFocCur = &(mcData->current.focCurrent);    
    T_FOC_VOLTAGE * ptFocVolt = &(mcData->voltage.focVoltage);
    T_ROTOR_ANGLE_HALL * ptThHall = &(mcData->rotorAngleHall);    
    
    /* Control motor speed. */
    MC_SpeedCtrl(ptSpeedCtrl);
    
    /* Calculate d/q current reference. */
    MC_Torque2current(ptT2C);
    ptFocCurCtrl->iDref = ptT2C->iDref;
    ptFocCurCtrl->iQref = ptT2C->iQref;                     
    
    /* Assign FOC angle. */
    ptFocCur->focAngle.th = ptThHall->angle;
    ptFocVolt->focAngle.th = (float)M_2PI - ptThHall->angle;  
    
    /* Execute field oriented control. */
    MC_Foc(mcData);
#if 0    
    /* Set PWM duty. */
    MC_PwmDutySet(mcData);   
#endif    
}


#if 1
__STATIC_INLINE void MC_PmsmFocHallFluxAngleAssign(T_PMSM_FOC * mcData)
{
    T_STATE_DATA * ptFocStt = &(mcData->focState);  
    T_FOC_CURRENT_CONTROL * ptFocCurCtrl = &(mcData->currentControl);    
    T_FOC_DECOUPLE * ptDecouple = &(ptFocCurCtrl->decouple);      
    T_SPEED_CONTROL * ptSpeedCtrl = &(mcData->speedControl);    
    T_PI_CONTROLLER * ptSpeedPi = &(mcData->speedControl.pi);    
    T_FOC_CURRENT * ptFocCur = &(mcData->current.focCurrent);    
    T_FOC_VOLTAGE * ptFocVolt = &(mcData->voltage.focVoltage);
    T_ROTOR_ANGLE_HALL * ptThHall = &(mcData->rotorAngleHall);
    T_ROTOR_ANGLE_FLUX_PLL * ptFluxPll = &(mcData->rotorAngleFluxPll); 
#if 0
    float switchSpd = mcData->motorPara.focSwitchSpd;
#endif 
    uint32_t pllCnt = mcData->motorPara.pllStableCnt;
    float minSpd = mcData->motorPara.minSpeed;
    
    
    switch(ptFocStt->state){
        case foc_low_speed_1:  /* Both FOC angle & speed are obtained from HALL. */
            /* The speed feedback has been assigned during data initialization. */
            /* Assign FOC angle. */
            ptFocCur->focAngle.th = ptThHall->angle;
            ptFocVolt->focAngle.th = (float)M_2PI - ptThHall->angle; 
            /* Switch state. */
            if((minSpd * 0.75f) < MC_Abs(ptThHall->speedMech)){
                ptFocStt->state = (uint8_t)(foc_engage_pll);
            }
            break;
        case foc_engage_pll:  /* Both FOC angle & speed are obtained from HALL. */
            (ptFocStt->cnt)++;
            /* Assign FOC angle. */
            ptFocCur->focAngle.th = ptThHall->angle;
            ptFocVolt->focAngle.th = (float)M_2PI - ptThHall->angle; 
            /* Switch state. */
            if(pllCnt < ptFocStt->cnt){
                /* Assign speed feedback. */
                ptDecouple->mechSpd = &(ptFluxPll->speedMech);
                ptSpeedCtrl->fbk = &(ptFluxPll->speedMech);
                ptSpeedPi->fbk = ptSpeedCtrl->fbk;
                /* Switch state. */
                ptFocStt->cnt = 0u;
                ptFocStt->state = (uint8_t)(foc_low_speed_2);
            }            
            break;
        case foc_low_speed_2:  /* FOC angle is obtained from HALL; speed is obtained from flux observer. */
            /* Assign FOC angle. */
            ptFocCur->focAngle.th = ptThHall->angle;
            ptFocVolt->focAngle.th = (float)M_2PI - ptThHall->angle; 
            /* Switch state. */
            if(ptThHall->spdUpThr < MC_Abs(ptThHall->speedMech)){
                /* Switch state. */
                ptFocStt->state = (uint8_t)(foc_high_speed);
            }
            break;            
        case foc_high_speed:  /* Both FOC angle & speed are obtained from flux observer. */
            /* Assign FOC angle. */
            ptFocCur->focAngle.th = ptFluxPll->angle;
            ptFocVolt->focAngle.th = (float)M_2PI - ptFluxPll->angle; 
            /* Switch state. */ 
            if(ptThHall->spdLowThr > MC_Abs(ptFluxPll->speedMech)){
                /* Switch state. */
                ptFocStt->state = (uint8_t)(foc_low_speed_2);
            }
            break;
        default: /* MISRA-C compliance */
            break;
    }
}
#else
__STATIC_INLINE void MC_PmsmFocHallFluxAngleAssign(T_PMSM_FOC * mcData)
{
    T_STATE_DATA * ptFocStt = &(mcData->focState);  
    T_FOC_CURRENT_CONTROL * ptFocCurCtrl = &(mcData->currentControl);    
    T_FOC_DECOUPLE * ptDecouple = &(ptFocCurCtrl->decouple);      
    T_SPEED_CONTROL * ptSpeedCtrl = &(mcData->speedControl);    
    T_PI_CONTROLLER * ptSpeedPi = &(mcData->speedControl.pi);    
    T_FOC_CURRENT * ptFocCur = &(mcData->current.focCurrent);    
    T_FOC_VOLTAGE * ptFocVolt = &(mcData->voltage.focVoltage);
    T_ROTOR_ANGLE_HALL * ptThHall = &(mcData->rotorAngleHall);
    T_ROTOR_ANGLE_FLUX_PLL * ptFluxPll = &(mcData->rotorAngleFluxPll); 
#if 0
    float switchSpd = mcData->motorPara.focSwitchSpd;
#endif 
    uint32_t pllCnt = mcData->motorPara.pllStableCnt;
    
    switch(ptFocStt->state){
        case foc_low_speed_1:
            /* The speed feedback has been assigned during data initialization. */
            /* Assign FOC angle. */
            ptFocCur->focAngle.th = ptThHall->angle;
            ptFocVolt->focAngle.th = (float)M_2PI - ptThHall->angle; 
            /* Switch state. */
            if(ptThHall->spdUpThr < MC_Abs(ptThHall->speedMech)){
                ptFocStt->state = (uint8_t)(foc_engage_pll);
            }
            break;
        case foc_engage_pll:
            (ptFocStt->cnt)++;
            /* The speed feedback has been assigned during data initialization. */
            /* Assign FOC angle. */
            ptFocCur->focAngle.th = ptThHall->angle;
            ptFocVolt->focAngle.th = (float)M_2PI - ptThHall->angle; 
            
            if(pllCnt < ptFocStt->cnt){
                /* Assign speed feedback. */
                ptDecouple->mechSpd = &(ptFluxPll->speedMech);
                ptSpeedCtrl->fbk = &(ptFluxPll->speedMech);
                ptSpeedPi->fbk = ptSpeedCtrl->fbk;
                /* Switch state. */
                ptFocStt->cnt = 0u;
                ptFocStt->state = (uint8_t)(foc_high_speed);
            }            
            break;
        case foc_low_speed_2:
            /* The speed feedback has been assigned during data initialization. */
            /* Assign FOC angle. */
            ptFocCur->focAngle.th = ptThHall->angle;
            ptFocVolt->focAngle.th = (float)M_2PI - ptThHall->angle; 
            
            if(ptThHall->spdUpThr < MC_Abs(ptThHall->speedMech)){
                /* Assign speed feedback. */
                ptDecouple->mechSpd = &(ptFluxPll->speedMech);
                ptSpeedCtrl->fbk = &(ptFluxPll->speedMech);
                ptSpeedPi->fbk = ptSpeedCtrl->fbk;
                /* Switch state. */
                ptFocStt->state = (uint8_t)(foc_high_speed);
            }
            break;            
        case foc_high_speed:
            /* Assign FOC angle. */
            ptFocCur->focAngle.th = ptFluxPll->angle;
            ptFocVolt->focAngle.th = (float)M_2PI - ptFluxPll->angle; 
            
            if(ptThHall->spdLowThr > MC_Abs(ptFluxPll->speedMech)){
                /* Assign speed feedback. */
                ptDecouple->mechSpd = &(ptThHall->speedMech);
                ptSpeedCtrl->fbk = &(ptThHall->speedMech);
                ptSpeedPi->fbk = ptSpeedCtrl->fbk;
                /* Switch state. */
                ptFocStt->state = (uint8_t)(foc_low_speed_2);
            }
            break;
        default: /* MISRA-C compliance */
            break;
    }
}
#endif


static void MC_PmsmFocHallFlux(T_PMSM_FOC * mcData)
{
    T_SPEED_CONTROL * ptSpeedCtrl = &(mcData->speedControl);
    T_FOC_CURRENT_CONTROL * ptFocCurCtrl = &(mcData->currentControl);    
    T_FOC_TORQUE_2_CURRENT * ptT2C = &(mcData->torque2current);       
    
    /* Estimate rotor flux vector and its angle. */
    MC_FluxAnglePllClStart(mcData);

    /* Assign FOC angle and speed feedback. */
    MC_PmsmFocHallFluxAngleAssign(mcData);    
    
    /* Control motor speed. */
    MC_SpeedCtrl(ptSpeedCtrl);
    
    /* Calculate d/q current reference. */
    MC_Torque2current(ptT2C);
    ptFocCurCtrl->iDref = ptT2C->iDref;
    ptFocCurCtrl->iQref = ptT2C->iQref;           

    /* Execute field oriented control. */
    MC_Foc(mcData);
#if 0    
    /* Set PWM duty. */
    MC_PwmDutySet(mcData); 
#endif
}


static void MC_AdcOffsetCal(T_CURRENT * ptCur)
{
    MC_1st_Lpf(&(ptCur->filAdcU));
    MC_1st_Lpf(&(ptCur->filAdcV));    
}


__STATIC_INLINE void MC_SpeedCtrlApp(T_PMSM_FOC * mcData)
{
    E_MOTOR_CONTROL_APPLICATION app = mcData->instruction.flgApp;
    
    switch(app){
        case ma_flux_foc:
            MC_PmsmFocFlux(mcData);
            break;
        case ma_hall_foc:
            MC_PmsmFocHall(mcData);
            break;
        case ma_hall_flux_foc:
            MC_PmsmFocHallFlux(mcData);
            break;
    #if 0
        case ma_align_encoder_foc:
            MC_PmsmFocAlignEnc(mcData);
            break;
    #endif
        default: /* MISRA-C compliance */
            break;
    }
}


__STATIC_INLINE bool MC_FaultDetection(T_PMSM_FOC * mcData)
{
    bool fault = false;
    bool stlDtct = mcData->instruction.flgStallDtct;
    bool phsLssDtct = mcData->instruction.flgPhsLossDtct;
    T_FAULT_DETECT * ptFltDtct = &(mcData->faultDetect);
    T_FAULT_FLAGS * ptFltFlgs = &(ptFltDtct->fltFlgs);
    T_STALL_DETECTION * ptStlDtct = &(ptFltDtct->stallDetection);
    T_PHASE_LOSS_DETECTION * ptPhsLossDtct = &(ptFltDtct->phaseLossDetection);    
    
    
    if(true == stlDtct){
        MC_StallDetect(ptStlDtct);
    }
    
    if(true == phsLssDtct){
        MC_PhaseLossDetect(ptPhsLossDtct);
    }    
        
    fault = (ptFltFlgs->stallDetected) || (ptFltFlgs->phaseLossDetected);
    return fault;
}


__STATIC_INLINE void MC_HallRead(uint8_t * hallVal)
{
#if 1
    * hallVal = (uint8_t)(HALL_Get()); 
#else
    * hallVal = PDEC_HALLPatternGet();
#endif
}


__STATIC_INLINE void MC_HallJumpTask(T_PMSM_FOC * mcData)
{
    T_ROTOR_ANGLE_HALL * ptThHall = &(mcData->rotorAngleHall);            
    
    ptThHall->isLegalHallJump = true; 
    ptThHall->legalHallJumpCnt += 1u;
    
    switch(ptThHall->hallValComb){
        case 0x51:
        case 0x15:
            ptThHall->hallJumpAngleRaw = (float)DEGREE_60_RAD;
            ptThHall->isPtcHalljump = true;
            break;
        case 0x13:
        case 0x31:
            ptThHall->hallJumpAngleRaw = (float)DEGREE_120_RAD;
            break;
        case 0x32:
        case 0x23:
            ptThHall->hallJumpAngleRaw = (float)DEGREE_180_RAD;
            break;
        case 0x26:
        case 0x62:
            ptThHall->hallJumpAngleRaw = (float)DEGREE_240_RAD;
            break;
        case 0x64:
        case 0x46:
            ptThHall->hallJumpAngleRaw = (float)DEGREE_300_RAD;
            break;
        case 0x45:
        case 0x54:
            ptThHall->hallJumpAngleRaw = 0.0f;
            break;            
        default:  /* illegal HALL jump */
            ptThHall->isLegalHallJump = false;
            ptThHall->legalHallJumpCnt -= 1u;
            break;               
    }    
    
    /* Prevent integer overflow. */
    if(3u < ptThHall->legalHallJumpCnt){
        ptThHall->legalHallJumpCnt = 3u;
    }
}


__STATIC_INLINE void MC_AltHallJumpTask(T_PMSM_FOC * mcData)
{
    T_ROTOR_ANGLE_HALL * ptThHall = &(mcData->rotorAngleHall);            
    
    ptThHall->isLegalHallJump = true; 
    ptThHall->legalHallJumpCnt += 1u;
    
    switch(ptThHall->hallValComb){
#if 0
        case 0x51:
#endif
        case 0x15:
            ptThHall->hallJumpAngleRaw = (float)DEGREE_60_RAD;
            ptThHall->isPtcHalljump = true;
            break;
        case 0x13:
#if 0
        case 0x31:
#endif
            ptThHall->hallJumpAngleRaw = (float)DEGREE_120_RAD;
            ptThHall->isPtcHalljump = true;
            break;
#if 0
        case 0x32:
#endif
        case 0x23:
            ptThHall->hallJumpAngleRaw = (float)DEGREE_180_RAD;
            break;
        case 0x26:
#if 0
        case 0x62:
#endif
            ptThHall->hallJumpAngleRaw = (float)DEGREE_240_RAD;
            break;
#if 0
        case 0x64:
#endif
        case 0x46:
            ptThHall->hallJumpAngleRaw = (float)DEGREE_300_RAD;
            break;
        case 0x45:
#if 0
        case 0x54:
#endif
            ptThHall->hallJumpAngleRaw = 0.0f;
            break;            
        default:  /* illegal HALL jump */
            ptThHall->isLegalHallJump = false;
            ptThHall->legalHallJumpCnt -= 1u;
            break;               
    }    
    
    /* Prevent integer overflow. */
    if(3u < ptThHall->legalHallJumpCnt){
        ptThHall->legalHallJumpCnt = 3u;
    }
}


__STATIC_INLINE void MC_LegalHallJumpTask(T_ROTOR_ANGLE_HALL * ptThHall)
{
    uint32_t tMax = ptThHall->timerMax;
    float tmp;
    FLOAT t;

    /* Calculate HALL jump angle. */
    ptThHall->hallJumpAngleZ = ptThHall->hallJumpAngle;
    tmp = ptThHall->hallJumpAngleRaw + ptThHall->hallOffsetAngle;
    ptThHall->hallJumpAngle = MC_Wrap2Pi(tmp);
    
    /* Read the timer value captured by the input capture. */
    ptThHall->timerIcZ = ptThHall->timerIc;
    ptThHall->timerIc = ptThHall->timerTmp;
    
    /* Calculate the timer count between two neighboring HALL jumps */
    if(tMax > ptThHall->timerSinceHallJump){
        /* The timer is counting down, not up. */
        ptThHall->timerBetweenHallJump = ptThHall->timerIcZ - ptThHall->timerIc;
    } else{
        ptThHall->timerBetweenHallJump = tMax;
    }
    
    /* Calculate rotor speed. */
    if((tMax > ptThHall->timerBetweenHallJump) && (0u != ptThHall->timerBetweenHallJump)){
        tmp = ptThHall->hallJumpAngle - ptThHall->hallJumpAngleZ;
        ptThHall->hallJumpAngleD = MC_WrapPi(tmp);
        t = (FLOAT)(ptThHall->timerBetweenHallJump) * ptThHall->timerT;
        ptThHall->hallJumpSpdEle = (float)((FLOAT)(ptThHall->hallJumpAngleD) / t);
        ptThHall->hallJumpSpdSign = MC_Sign(ptThHall->hallJumpSpdEle);
    } else{
        ptThHall->hallJumpSpdEle = 0.0f;
    }
}


__STATIC_INLINE void MC_ParticularHallJumpTask(T_ROTOR_ANGLE_HALL * ptThHall)
{
    uint32_t tMax = ptThHall->timerMax;
    FLOAT m2Pi = (FLOAT)M_2PI;
    FLOAT t;

    /* Calculate particular HALL jump angle. */
    ptThHall->ptcHallJumpAngle = ptThHall->hallJumpAngle;
    
    /* Read the timer value captured by the input capture. */
    ptThHall->timerIcPtcZ = ptThHall->timerIcPtc;
    ptThHall->timerIcPtc = ptThHall->timerIc;   
    
    /* Calculate the timer count between two neighboring HALL jumps */
    if(tMax > ptThHall->timerSincePtcHallJump){
        /* The timer is counting down, not up. */
        ptThHall->timerBetweenPtcHallJump = ptThHall->timerIcPtcZ - ptThHall->timerIcPtc;
    } else{
        ptThHall->timerBetweenPtcHallJump = tMax;
    }    
    
    /* Calculate rotor speed. */
    if((tMax > ptThHall->timerBetweenPtcHallJump) && (0u != ptThHall->timerBetweenPtcHallJump)){
        t = (FLOAT)(ptThHall->timerBetweenPtcHallJump) * ptThHall->timerT;
        ptThHall->ptcHallJumpSpdEle = ptThHall->hallJumpSpdSign * (float)(m2Pi / t);
    } else{
        ptThHall->ptcHallJumpSpdEle = 0.0f;
    }   
}


__STATIC_INLINE float MC_HallAngleCal(uint8_t hallVal, float offset)
{
    float raw = commData.hallAngleTbl[hallVal];
    float tmp = raw + offset; 
    
    return MC_Wrap2Pi(tmp);   
}


__STATIC_INLINE void MC_HallAngleEstimateLowSpd(T_ROTOR_ANGLE_HALL * ptThHall)
{
    FLOAT spd = (FLOAT)(ptThHall->speedEle);
    FLOAT tmpDbl;
    float tmp;
    FLOAT degMax = (FLOAT)(ptThHall->angleBetweenHallJump);   
    
    tmpDbl = spd * (FLOAT)(ptThHall->timerSinceHallJump) * ptThHall->timerT;

    if(degMax < tmpDbl){
        tmp = (float)degMax;
    } else if(-degMax > tmpDbl){
        tmp = (float)(-degMax);
    } else{
        tmp = (float)tmpDbl;
    }    
    
    ptThHall->angle = MC_Wrap2Pi(ptThHall->hallJumpAngle + tmp);
}


__STATIC_INLINE void MC_HallAngleEstimateHighSpd(T_ROTOR_ANGLE_HALL * ptThHall)
{
    FLOAT spd = (FLOAT)(ptThHall->speedEle);
    FLOAT tmpDbl;
    float tmp;
    FLOAT m2Pi = (FLOAT)(M_2PI);
    
    tmpDbl = spd * (FLOAT)(ptThHall->timerSincePtcHallJump) * ptThHall->timerT;
    
    if(m2Pi < tmpDbl){
        tmp = (float)m2Pi;
    } else if(-m2Pi > tmpDbl){
        tmp = (float)(-m2Pi);
    } else{
        tmp = (float)tmpDbl;
    }
    
    ptThHall->angle = MC_Wrap2Pi(ptThHall->ptcHallJumpAngle + tmp);
}


__STATIC_INLINE void MC_PhaseVoltageDtct(T_VOLTAGE * ptVolt)
{  
    /* Read CC registers twice to get the latest value. */
    ptVolt->dummy = (int16_t)(TC2_Capture16bitChannel0Get());
    ptVolt->fbCntU = (int16_t)(TC2_Capture16bitChannel0Get());
    ptVolt->dummy = (int16_t)(TC3_Capture16bitChannel0Get());
    ptVolt->fbCntV = (int16_t)(TC3_Capture16bitChannel0Get());
    ptVolt->dummy = (int16_t)(TC4_Capture16bitChannel0Get());
    ptVolt->fbCntW = (int16_t)(TC4_Capture16bitChannel0Get());  
}


__STATIC_INLINE void MC_HallAngle(T_PMSM_FOC * mcData)
{
    T_ROTOR_ANGLE_HALL * ptThHall = &(mcData->rotorAngleHall);     
    T_STATE_DATA * hallStt = &(ptThHall->hallState);
 
#if 1      
    /* Read inputs. */
    ptThHall->hallValZ = ptThHall->hallVal;
    ptThHall->hallValComb = ptThHall->hallVal << 4;
    MC_HallRead(&(ptThHall->hallVal));
    ptThHall->timerTmp = HAL_HallCapturedTimerGet();  /* Frequently read CCx register, to relieve load of the single buffer. */      
    ptThHall->hallValComb = ptThHall->hallValComb | ptThHall->hallVal; 
    ptThHall->timerZ = ptThHall->timer;
    ptThHall->timer = HAL_HallTimerGet();
    ptThHall->timerD = ptThHall->timerZ - ptThHall->timer;  /* The timer is counting down, not up. */
#endif
    
    /* Do tasks if a HALL jump occurs. */
    if(ptThHall->hallValZ != ptThHall->hallVal){ 
        if(2u == ptThHall->hallJumpDtctMode){  /* HALL jumps are detected alternatively. */
            MC_AltHallJumpTask(mcData); 
        } else{  /* Each HALL jump is detected. */
            MC_HallJumpTask(mcData); 
        }                       
    }
    
    /* Do tasks if a legal HALL jump occurs. */
    if(true == ptThHall->isLegalHallJump){
        MC_LegalHallJumpTask(ptThHall);
    }
    
    /* Calculate the timer count since last HALL jump. */
    if(true == ptThHall->isLegalHallJump){
        /* The timer is counting down, not up. */
        ptThHall->timerSinceHallJump = ptThHall->timerIc - ptThHall->timer;
    } else if(ptThHall->timerSinceHallJump < ptThHall->timerMax){       
        ptThHall->timerSinceHallJump += ptThHall->timerD;
    } else if(ptThHall->timerSinceHallJump >= ptThHall->timerMax){
        ptThHall->hallJumpSpdEle = 0.0f;
        ptThHall->hallJumpSpdSign = 0.0f;
    } else{}  /* MISRA-C compliance */
    ptThHall->isLegalHallJump = false;

    /* Do tasks if a particular HALL jump occurs. */
    if(true == ptThHall->isPtcHalljump){
       MC_ParticularHallJumpTask(ptThHall);
    }
    
    /* Calculate the timer count since last particular HALL jump. */
    if(true == ptThHall->isPtcHalljump){
        /* The timer is counting down, not up. */
        ptThHall->timerSincePtcHallJump = ptThHall->timerIcPtc - ptThHall->timer;
    } else if(ptThHall->timerSincePtcHallJump < ptThHall->timerMax){
        ptThHall->timerSincePtcHallJump += ptThHall->timerD;
    } else{}  /* MISRA-C compliance */
    ptThHall->isPtcHalljump = false;    
    
    /* State machine */
    switch(hallStt->state){
        case hall_jump_less:                      
            if(2u <= ptThHall->legalHallJumpCnt){
                #if 1
                hallStt->state = (uint8_t)(hall_median_speed);
                #else
                hallStt->state = (uint8_t)(hall_low_speed); 
                #endif
            } else{
                /* The speed outputs has been initialized as 0. */
                /* Take hallAngle as the angle output. */
                ptThHall->hallAngle = MC_HallAngleCal(ptThHall->hallVal, ptThHall->hallOffsetAngle);
                ptThHall->angle = ptThHall->hallAngle;  
            }
            break;
        case hall_low_speed:
            /* Assign rotor speed. */
            ptThHall->speedEle = ptThHall->hallJumpSpdEle;
            ptThHall->speedMech = ptThHall->speedEle * ptThHall->oneByNumPolePair;
            /* Take hallAngle as the angle output. */
            ptThHall->hallAngle = MC_HallAngleCal(ptThHall->hallVal, ptThHall->hallOffsetAngle);
            ptThHall->angle = ptThHall->hallAngle;     
            /* Change state. */
            if(MC_Abs(ptThHall->speedMech) > ptThHall->spdLowThr){
                hallStt->state = (uint8_t)(hall_median_speed);
            }
            break;
        case hall_median_speed:
            /* Assign rotor speed. */
            ptThHall->speedEle = ptThHall->hallJumpSpdEle;
            ptThHall->speedMech = ptThHall->speedEle * ptThHall->oneByNumPolePair;
            /* Estimate rotor flux angle. */
            MC_HallAngleEstimateLowSpd(ptThHall);    
            /* Change state. */
            if(MC_Abs(ptThHall->speedMech) > ptThHall->spdUpThr){
                hallStt->state = (uint8_t)(hall_high_speed);
            }
            #if 0
            else if(MC_Abs(ptThHall->speedMech) < ptThHall->spdLowThr){
                hallStt->state = (uint8_t)(hall_low_speed);
            } else{}  /* MISRA-C compliance */
            #endif
            break;            
        case hall_high_speed:
            /* Assign rotor speed. */
            ptThHall->speedEle = ptThHall->ptcHallJumpSpdEle;
            ptThHall->speedMech = ptThHall->speedEle * ptThHall->oneByNumPolePair;
            /* Estimate rotor flux angle. */
            MC_HallAngleEstimateHighSpd(ptThHall);   
            /* Change state. */
            if(MC_Abs(ptThHall->speedMech) < ptThHall->spdLowThr){
                hallStt->state = (uint8_t)(hall_median_speed);
            }
            break;
        default: /* MISRA-C compliance */
            break;
    }
}


__STATIC_INLINE void MC_HallCali(T_ROTOR_ANGLE_ENCODER * ptThEnc)
{
    /* Calculate angleCnt. */
    ptThEnc->isHallCaliDone = true;
    
    switch(ptThEnc->hallValComb){
        case 0x51:
        case 0x15:
            ptThEnc->hallAngle = (float)DEGREE_60_RAD;
            break;
        case 0x13:
        case 0x31:
            ptThEnc->hallAngle = (float)DEGREE_120_RAD;
            break;
        case 0x32:
        case 0x23:
            ptThEnc->hallAngle = (float)DEGREE_180_RAD;
            break;
        case 0x26:
        case 0x62:
            ptThEnc->hallAngle = (float)DEGREE_240_RAD;
            break;
        case 0x64:
        case 0x46:
            ptThEnc->hallAngle = (float)DEGREE_300_RAD;
            break;
        case 0x45:
        case 0x54:
            ptThEnc->hallAngle = 0.0f;
            break;            
        default: /* MISRA-C compliance */
            ptThEnc->isHallCaliDone = false;
            break;               
    }        
}


__STATIC_INLINE void MC_HallAngleCorrection(T_ROTOR_ANGLE_ENCODER * ptThEnc)
{
    int32_t tmp32;
    int32_t ePPR = (int32_t)(ptThEnc->ePPR);
    float tmp;
    
    tmp = (ptThEnc->hallAngle) * (ptThEnc->eleRad2Cnt) + 0.5f;
    tmp32 = (int32_t)(tmp);
    tmp32 += ptThEnc->hallOffsetCnt;
    
    if(tmp32 >= ePPR){
        tmp32 = tmp32 - ePPR;
    }
    
    ptThEnc->angleCnt = (T_EPPR)tmp32;      
}

#if 0
__STATIC_INLINE void MC_EncoderPll(T_PMSM_FOC * mcData)
{
    T_ROTOR_ANGLE_ENCODER * ptThEnc = &(mcData->rotorAngleEncoder);    
    T_POSITION_TRACKING_PLL * ptPll = &(mcData->motionEstimate.pll);         
    
    /* Calculate angle estimation error. */
    ptThEnc->angleEleErr = MC_WrapPiDbl((double)(ptThEnc->thIn) - ptThEnc->angleEleDbl);
    
    /* Execute angle tracking pll. */
    MC_PositionTrackingPll(ptPll);
    
    /* Wrap the angle in [0, 2pi]. */
    if(ptThEnc->angleEleDbl > M_2PI){
        ptPll->pstItg = ptPll->pstItg - M_2PI;
        ptThEnc->angleEleDbl = ptPll->pstItg;
    } else if(ptThEnc->angleEleDbl < 0.0){
        ptPll->pstItg = ptPll->pstItg + M_2PI;
        ptThEnc->angleEleDbl = ptPll->pstItg;
    }
    
    /* Calculate mechanical speed. */
    ptThEnc->speed = (float)(ptThEnc->spdEleDbl) * ptThEnc->oneByNumPolePairs;
    ptThEnc->acc = (float)(ptThEnc->accEleDbl) * ptThEnc->oneByNumPolePairs; 
}


__STATIC_INLINE void MC_EncoderAngle(T_PMSM_FOC * mcData)
{
    T_PERIPHERAL * ptPeri = &(mcData->peripheral);      
    T_ROTOR_ANGLE_ENCODER * ptThEnc = &(mcData->rotorAngleEncoder); 
    T_STATE_DATA * ptAngStt = &(ptThEnc->angleState);      
    T_MOTION_ESTIMATOR * ptMotionEst = &(mcData->motionEstimate);   
    T_POSITION_TRACKING_PLL * ptPtPll = &(ptMotionEst->pll);       
    int32_t tmp32, capDiff, angleCntlth;
    T_EPPR ePPR = ptThEnc->ePPR;
    int32_t mPPR = ptThEnc->mPPR;    
    T_POS_CH_E qeiCh = ptPeri->qeiChannel;    
    float tmp;  
    bool tmpBl;
 
    /* Read HALL & encoder signals. */
    ptThEnc->hallValLth = ptThEnc->hallVal;
    ptThEnc->hallValComb = ptThEnc->hallVal << 4;
    MC_HallRead(ptPeri, &(ptThEnc->hallVal));
    ptThEnc->hallValComb = ptThEnc->hallValComb | ptThEnc->hallVal;
    ptThEnc->QEIcnt = (int32_t)HAL_PositionCountGet(qeiCh); 
#if 1  /* Calculate speed by counting QE number. */
    (ptThEnc->speedSampleLoopCnt)++;
    if(ptThEnc->speedSampleLoopCnt >= ptThEnc->speedSampleLoopCntMax){
        ptThEnc->speedSampleLoopCnt = 0u;
        ptThEnc->QEIvel = (int32_t)HAL_VelocityCountGet(qeiCh);
        ptThEnc->speedCnt = (float)(ptThEnc->QEIvel) * ptThEnc->cnt2MechRadPerSec;
    }    
#endif
    
    /* Encoder-based rotor angle */
    switch(ptAngStt->state){
        case ang_idle:
            break;
        case ang_hall_wait:
            (ptAngStt->cnt)++;
            if(CNT_ANG_CAL_WAIT <= ptAngStt->cnt){
               ptAngStt->cnt = 0u;
               ptAngStt->state = (uint8_t)(ang_hall_initial); 
            }
            break;
        case ang_hall_initial:
            /* Calculate angleCnt according to HALL value. And set QEI counter. */
            ptThEnc->hallAngle = commData.hallAngleTbl[ptThEnc->hallVal];
            MC_HallAngleCorrection(ptThEnc);
            HAL_PositionCountSet((int32_t)(ptThEnc->angleCnt), qeiCh); 
            ptAngStt->state = (uint8_t)(ang_no_cali_yet);
            break;
        case ang_no_cali_yet:
            /* Latch counter value. */
            angleCntlth = (int32_t)(ptThEnc->angleCnt);              
            #if 1
            /* Calculate angleCnt according to QEI counter. */
            ptThEnc->angleCnt = MODULUS((T_EPPR)(ptThEnc->QEIcnt), ePPR);
            #else
            /* Calculate angleCnt according to HALL value. And set QEI counter. 
               This is necessary because some HALL sensor's output is not ready @ ang_hall_initial state. */
            ptThEnc->hallAngle = commData.hallAngleTbl[ptThEnc->hallVal];
            MC_HallAngleCorrection(ptThEnc);
            HAL_PositionCountSet((int32_t)(ptThEnc->angleCnt), qeiCh);            
            #endif                                
            /* Detect HALL jump. */
            if(ptThEnc->hallValLth != ptThEnc->hallVal){
                MC_HallCali(ptThEnc);
            }
            if(ptThEnc->isHallCaliDone == true){
                /* Set position counter. */                
                MC_HallAngleCorrection(ptThEnc);
                tmp32 = (int32_t)(ptThEnc->angleCnt) - angleCntlth;  
                ptThEnc->deltaCnt = MC_Wrap32(tmp32, (int32_t)ePPR);   
                tmp32 = ptThEnc->QEIcnt + ptThEnc->deltaCnt;
                HAL_PositionCountSet((uint32_t)tmp32, qeiCh);
                /* Update estimated position. */
                tmp = ptThEnc->deltaCnt * ptThEnc->cnt2MechRad;
                ptPtPll->pstIni = MC_Wrap2PiDbl(ptPtPll->pstIni + (double)tmp);
                ptThEnc->angleEleDbl = MC_Wrap2PiDbl(ptThEnc->angleEleDbl + (double)tmp);
                /* Switch state. */
                ptAngStt->state = (uint8_t)(ptThEnc->indexUseState);
            }  
            /* Detect index event. */
            ptThEnc->isIndexDetected = HAL_IndexEventCheck(qeiCh);
            if(ptThEnc->isIndexDetected == true){
                ptThEnc->QEIcap = HAL_CapturedPisitionGet(qeiCh);
                ptThEnc->QEIcapCorrect = ptThEnc->QEIcap;
                ptAngStt->state = (uint8_t)(ang_index_cali_done);
            }             
            break;
        case ang_hall_cali_done:
            /* Calculate angleCnt according to QEI counter. */
            ptThEnc->angleCnt = MODULUS((T_EPPR)(ptThEnc->QEIcnt), ePPR);
            /* Detect index event. */
            ptThEnc->isIndexDetected = HAL_IndexEventCheck(qeiCh);
            if(ptThEnc->isIndexDetected == true){
                ptThEnc->QEIcap = HAL_CapturedPisitionGet(qeiCh);
                ptThEnc->QEIcapCorrect = ptThEnc->QEIcap;
                ptAngStt->state = (uint8_t)(ang_index_cali_done);
            } 
            break;
        case ang_index_cali_done:
            ptThEnc->QEIcap = HAL_CapturedPisitionGet(qeiCh);
            capDiff = ptThEnc->QEIcap - ptThEnc->QEIcapCorrect;
            tmp32 = MC_Modulus(capDiff, mPPR);
            if(tmp32 > (mPPR>>1)){
                tmp32 = tmp32 - mPPR;
            }
            /* Check if the captured counter value is reasonable or not. */
            if((tmp32 < ptThEnc->indexToleranceCnt) && (-tmp32 < ptThEnc->indexToleranceCnt)){
                /* Accept the captured counter value. */
                ptThEnc->QEIcapCorrect = ptThEnc->QEIcap;
                /* Update estimated position. */   
                ptThEnc->deltaCnt = -tmp32;
                tmp = ptThEnc->deltaCnt * ptThEnc->cnt2MechRad;
                ptPtPll->pstIni = MC_Wrap2PiDbl(ptPtPll->pstIni + (double)tmp);
                ptThEnc->angleEleDbl = MC_Wrap2PiDbl(ptThEnc->angleEleDbl + (double)tmp);
                }
            tmp32 = ptThEnc->QEIcnt - ptThEnc->QEIcapCorrect + ptThEnc->indexAngleCnt;
            ptThEnc->angleCnt = MODULUS((T_EPPR)tmp32, ePPR);
            break;
        case ang_index_no_use:
            /* Calculate angleCnt according to QEI counter. */
            ptThEnc->angleCnt = MODULUS((T_EPPR)(ptThEnc->QEIcnt), ePPR);
            /* Optional: read index event to confirm its position. */
            tmpBl = HAL_IndexEventCheck(qeiCh);
            ptThEnc->isIndexDetected= tmpBl; 
            if(true == tmpBl){
                HAL_IndexEventClear(qeiCh);
            }
            break;
        default: /* MISRA-C compliance */
            break;
    }
    
    /* Calculate flux angles for Park transform & inverse Park transform. */
    ptThEnc->thLth = ptThEnc->thIn;
    ptThEnc->thIn = (float)(ptThEnc->angleCnt) * ptThEnc->cnt2EleRad;
    tmp = (ptThEnc->thIn * 2.0f) - ptThEnc->thLth;
    ptThEnc->thOut = MC_Wrap2Pi(tmp);
    
    /* Estimate rotor angular speed. */
    if(ang_idle != (ptAngStt->state)){
        MC_EncoderPll(mcData);
    }
    
}
#endif


__STATIC_INLINE void MC_AdcManipulate1(void)
{
    /** Set Potentiometer and bus voltage channels */
    ADC0_ChannelSelect( ADC_POSINPUT_AIN12, ADC_NEGINPUT_GND);  /* potentiometer */
    ADC1_ChannelSelect( ADC_POSINPUT_AIN2, ADC_NEGINPUT_GND);  /* BUS voltage */

    /** Start software conversion */
#if 0
    ADC0_InterruptsClear(ADC_STATUS_MASK);
    ADC0_InterruptsDisable( ADC_STATUS_RESRDY );
#endif
    ADC0_ConversionStart();   
}


__STATIC_INLINE void MC_FeedbackGet1(T_PMSM_FOC * mcData)
{    
    T_CURRENT * ptCur = &(mcData->current); 
    T_FOC_CURRENT * ptFocCur = &(ptCur->focCurrent);  
    T_CLARKE_TRANSFORM * ptCurClarke = &(ptFocCur->clarkeTrans);   
    E_MOTOR_CONTROL_APPLICATION app = mcData->instruction.flgApp;

    T_VOLTAGE * ptVolt = &(mcData->voltage);
    int16_t tmp16;   
    
#ifdef ENCODER_FEEDBACK_INFORM   
    float tmp;     
    T_ROTOR_ANGLE_ENCODER_REF * ptThEncRef = &(mcData->rotorAngleEncoderRef);      
    /* Encoder. */
    ptThEncRef->QEIcnt = (int32_t)HAL_PositionCountGet(ptPeri->qeiChannel);   
    (ptThEncRef->speedSampleLoopCnt)++;
    if(ptThEncRef->speedSampleLoopCnt >= ptThEncRef->speedSampleLoopCntMax){
        ptThEncRef->speedSampleLoopCnt = 0u;
        ptThEncRef->QEIvel = (int32_t)HAL_VelocityCountGet(ptPeri->qeiChannel);
        ptThEncRef->speed = (float)(ptThEncRef->QEIvel) * ptThEncRef->cnt2RadPerSec;
    }
    ptThEncRef->thLth = ptThEncRef->thIn;
    ptThEncRef->thIn = (float)(ptThEncRef->QEIcnt) * ptThEncRef->cnt2Rad;
    tmp = (ptThEncRef->thIn * 2.0f) - ptThEncRef->thLth;
    ptThEncRef->thOut = MC_Wrap2Pi(tmp); 
#endif 

    /** Read phase currents  */
    tmp16 = (int16_t)ADC0_ConversionResultGet();
    ptCur->adcDataU = (float)tmp16;
    tmp16 = (int16_t)ADC1_ConversionResultGet();
    ptCur->adcDataV = (float)tmp16;  

    MC_AdcManipulate1();
    
    /* Current calculation */  
    ptFocCur->iU = (ptCur->adcOffsetU - ptCur->adcDataU) * ptCur->adc2amps;    
    ptFocCur->iV = (ptCur->adcOffsetV - ptCur->adcDataV) * ptCur->adc2amps;
    ptFocCur->iW = -(ptFocCur->iU) - ptFocCur->iV;
#ifdef PHS_LOSS_TEST  /* for test */
    ptFocCur->iVtest = -(ptFocCur->iU);
    ptFocCur->iWtest = 0.0f;    
#endif
    ptFocCur->iAlphaLth = ptFocCur->iAlpha;
    ptFocCur->iBetaLth = ptFocCur->iBeta;
    MC_ClarkeTransformBalanced(ptCurClarke); 
    
    switch(app){
        case ma_hall_foc:
        case ma_hall_flux_foc:
            MC_HallAngle(mcData);
            break;
        case ma_flux_foc:
            if(1u == (ptVolt->phsVoltMode)){  /* Phase voltages are obtained by HW detection. */
                MC_PhaseVoltageDtct(ptVolt);
            }           
        #if 0
            T_ROTOR_ANGLE_HALL * ptThHall = &(mcData->rotorAngleHall);
            /* (optional) Read HALL value to verify HALL tables. */
            MC_HallRead(&(ptThHall->hallVal));
            MC_HallAngle(mcData);
        #endif
            break;
        #if 0
        case ma_align_encoder_foc:
        case ma_hall_encoder_foc:
            MC_EncoderAngle(mcData);
            break;
        #endif
        default: /* MISRA-C compliance */
            break;
    }       
}


__STATIC_INLINE void MC_AdcManipulate2(void)
{
    /** Set phase A and phase B current channels */
    ADC0_ChannelSelect( ADC_POSINPUT_AIN0, ADC_NEGINPUT_GND);
    ADC1_ChannelSelect( ADC_POSINPUT_AIN0, ADC_NEGINPUT_GND);

#if 0
    /** ADC interrupt enable  */
    ADC0_InterruptsClear(ADC_STATUS_MASK);
    ADC0_InterruptsEnable( ADC_STATUS_RESRDY );
#endif
}


__STATIC_INLINE void MC_BusVoltRead(T_VOLTAGE * ptVolt)
{      
    ptVolt->adcDataVbus = (int16_t)ADC1_ConversionResultGet();
    ptVolt->vBus = (float)(ptVolt->adcDataVbus) * ptVolt->adc2volt;    
}


__STATIC_INLINE void MC_FeedbackGet2(T_PMSM_FOC * mcData)
{
    T_VOLTAGE * ptVolt = &(mcData->voltage);      
    T_INSTRUCTION_POT_BUTTON * ptInstr = &(mcData->instruction); 

    /** Read DC bus voltage */
    MC_BusVoltRead(ptVolt);

    /** Read potentiometer input */
    ptInstr->adcPOT = (int16_t)ADC0_ConversionResultGet();
    
    MC_AdcManipulate2();

#if 0
    T_ROTOR_ANGLE_HALL * ptThHall = &(mcData->rotorAngleHall); 
    ptThHall->timerIc = HAL_HallCapturedTimerGet(); 
#endif
}


__STATIC_INLINE void MC_DecSpeedRefCal(T_INSTRUCTION_POT_BUTTON * ptInstr)
{
    float spd = ptInstr->speedRefDec;
    float step = ptInstr->deceleratoin;
    
    if(step < spd){
        ptInstr->speedRefDec = spd - step;
    } else if(-step > spd){
        ptInstr->speedRefDec = spd + step;
    } else{}  /* MISRA-C compliance */
} 


__STATIC_INLINE void MC_PeripheralEnable(T_PMSM_FOC * mcData)
{
    uint16_t phsVltMd = mcData->voltage.phsVoltMode;
    
    Hal_PwmEnable();
    
    if(1u == phsVltMd){
        TC2_CaptureStart();
        TC3_CaptureStart();
        TC4_CaptureStart(); 
    }     
}


__STATIC_INLINE void MC_PeripheralDisable(T_PMSM_FOC * mcData)
{
    uint16_t phsVltMd = mcData->voltage.phsVoltMode;
    
    HAL_PwmDisable();
    
    if(1u == phsVltMd){
        TC2_CaptureStop();
        TC3_CaptureStop();
        TC4_CaptureStop(); 
    }     
}


static void MC_SpeedCtrlSttUpdate(T_PMSM_FOC * mcData) 
{
    T_INSTRUCTION_POT_BUTTON * ptInstr = &(mcData->instruction); 
    T_STATE_DATA * ptMcStt = &(mcData->mcState);
    T_CURRENT * ptCur = &(mcData->current);  
    T_VOLTAGE * ptVolt = &(mcData->voltage); 
    T_SPEED_CONTROL * ptSpeedCtrl = &(mcData->speedControl);
    T_PI_CONTROLLER * ptSpeedPi = &(mcData->speedControl.pi);
    bool * ptTrig = &(ptInstr->flgMotorTrig);
    T_SVPWM * ptSvpwm = &(mcData->voltage.svpwm); 
    float minSpd = mcData->motorPara.minSpeed;
    float spd = MC_Abs(*(mcData->speedControl.fbk));
    bool fault; 

    switch(ptMcStt->state){
        case mc_idle:
            MC_AdcOffsetCal(ptCur);
            if(true == *ptTrig){
                *ptTrig = false;
                ptInstr->dirRef *= -1.0f;                
                MC_DataReset(mcData);
                MC_PeripheralEnable(mcData);                
                ptMcStt->state = (uint8_t)(mc_spin);
            }           
            break;
        case mc_spin:
            fault = MC_FaultDetection(mcData);
        #ifndef PHS_LOSS_TEST
            if(true == fault){
                MC_PeripheralDisable(mcData);
                LED_01_Set();
                ptMcStt->state = (uint8_t)(mc_fault);
                /* Do not reset the data, keep them for tuning. */
            #if 0  /* Disable the break for MISRA-C compliance. */
                break;
            #endif
            } 
        #endif
            MC_SpeedCtrlApp(mcData);
            MC_BusVoltRead(ptVolt);
            MC_PwmModulate(ptSvpwm);
            if(true == *ptTrig){
                *ptTrig = false;
                ptInstr->speedRefDec = ptInstr->speedRefFil;
                ptSpeedCtrl->ref = &(ptInstr->speedRefDec);
                ptSpeedPi->ref = ptSpeedCtrl->ref;
                ptMcStt->state = (uint8_t)(mc_decelerate);
                /* Do not reset the data, keep them for tuning. */
            }             
            break;  
        case mc_decelerate:
            fault = MC_FaultDetection(mcData);
        #ifndef PHS_LOSS_TEST
            if(true == fault){
                MC_PeripheralDisable(mcData);
                LED_01_Set();
                ptMcStt->state = (uint8_t)(mc_fault);
                /* Do not reset the data, keep them for tuning. */
            #if 0  /* Disable the break for MISRA-C compliance. */
                break;
            #endif
            } 
        #endif  
            MC_DecSpeedRefCal(ptInstr); 
            MC_SpeedCtrlApp(mcData);
            MC_BusVoltRead(ptVolt);
            MC_PwmModulate(ptSvpwm);
            if((spd < minSpd) || (true == *ptTrig)){
                *ptTrig = false;
                MC_PeripheralDisable(mcData);
                ptMcStt->state = (uint8_t)(mc_idle);
                /* Do not reset the data, keep them for tuning. */
            } 
            break;
        case mc_fault:
            if(true == *ptTrig){
                *ptTrig = false;
                LED_01_Clear();
                ptMcStt->state = (uint8_t)(mc_idle);
                /* Do not reset the data, keep them for tuning. */
            } 
            break;
        default: /* MISRA-C compliance */
            break;  
    }
}

#if 0
static void MC_MotorCtrlSttUpdate(T_PMSM_FOC * mcData)
{
    E_MOTOR_CONTROL_APPLICATION app = mcData->instruction.flgApp;
    
    switch(app){
        case ma_flux_foc:
        case ma_hall_foc:
        case ma_hall_flux_foc:
        case ma_align_encoder_foc:
        case ma_hall_encoder_foc:
            MC_SpeedCtrlSttUpdate(mcData);
            break;
        default: /* MISRA-C compliance */
            break;
    }
}
#endif

void MC_CtrlLoop(ADC_STATUS status, uintptr_t context )
{
    /* ADC interrupt disable */
    ADC0_InterruptsDisable(ADC_STATUS_RESRDY);
    ADC0_InterruptsClear(ADC_STATUS_MASK);    
         
    /* Get feedback information. */    
    MC_FeedbackGet1(&mcFoc1);

    /* Update motor control state. */    
    MC_SpeedCtrlSttUpdate(&mcFoc1);
    
    /* Increase loop counting number for low frequency loop. */
    MC_LowFreqLoopSync();
      
    /* Read potentiometer & BUS voltage. */
    MC_FeedbackGet2(&mcFoc1);    
    
    /* Update X2Cscope. */
    X2Cscope_Update();   
    
    /* ADC interrupt enable */
    ADC0_InterruptsClear(ADC_STATUS_MASK);
    ADC0_InterruptsEnable(ADC_STATUS_RESRDY);
}




/*************************- End of File -************************************/

