/*******************************************************************************
  Motor Control Hardware Abstraction interface file

  Company:
    Microchip Technology Inc.

  File Name:
    mc_motor_1.c

  Summary:
    Motor parameters initialization.

  Description:
    Motor parameters initialization.
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

#include "mc_motor_1.h"
#include "definitions.h"                // SYS function prototypes
#include "device.h"

static void MC_FaultIsr_Motor_1(uint32_t status, uintptr_t context);


T_PMSM_FOC mcFoc1;


void MC_ParaConfigMotor1(T_PMSM_FOC * mcData)
{
    T_PMSM_PARA * ptMotorPara = &(mcData->motorPara);
    T_FOC_CURRENT_CONTROL * ptFocCurCtrl = &(mcData->currentControl);    
    T_PI_CONTROLLER * ptIdPi = &(ptFocCurCtrl->iDpi);
    T_PI_CONTROLLER * ptIqPi = &(ptFocCurCtrl->iQpi);
    T_FOC_DECOUPLE * ptDecouple = &(ptFocCurCtrl->decouple);    
    T_PI_CONTROLLER * ptSpeedPi = &(mcData->speedControl.pi);
    T_CURRENT * ptCur = &(mcData->current);
    T_VOLTAGE * ptVolt = &(mcData->voltage);
    T_OPENLOOP_SPIN * ptOpenLoop = &(mcData->openloopSpin);
    T_SVPWM * ptSvpwm = &(ptVolt->svpwm);
    T_FOC_TORQUE_2_CURRENT * ptT2C = &(mcData->torque2current);
    T_INSTRUCTION_POT_BUTTON * ptInstr = &(mcData->instruction);
    T_ROTOR_ANGLE_BEMF_ATAN * ptThAtan = &(mcData->rotorAngleAtan);
    T_BEMF_SMO * ptSmo = &(ptThAtan->smo);
    T_SMO_CUR_OBS_PARA * ptIObsPara = &(ptSmo->iObsPara);
    T_SMO_BEMF_OBSERVER * ptEObs = &(ptSmo->eObs);
    T_SMO_SPEED_INPUT * ptSmoSpeedIn = &(ptThAtan->smoSpeedIn);
    T_ROTOR_ANGLE_FLUX_PLL * ptFluxPll = &(mcData->rotorAngleFluxPll);  
    T_P_FLUX_OBSERVER * ptPfo = &(ptFluxPll->pfo); 
    T_ROTOR_ANGLE_ENCODER * ptThEnc = &(mcData->rotorAngleEncoder);     
    T_MOTION_ESTIMATOR_FLT * ptMotionEst = &(mcData->motionEstimate);   
    T_POSITION_TRACKING_PLL_FLT * ptPll = &(ptMotionEst->pll);    
    T_ROTOR_ANGLE_HALL * ptThHall = &(mcData->rotorAngleHall);
    T_FAULT_DETECT * ptFltDtct = &(mcData->faultDetect);
    T_STALL_DETECTION * ptStlDtct = &(ptFltDtct->stallDetection);
    T_PHASE_LOSS_DETECTION * ptPhsLossDtct = &(ptFltDtct->phaseLossDetection);
    int16_t tmp16;
    float tmp;

    /* motor */
    ptMotorPara->ratedSpeed = RATED_SPEED_RAD_S;
    ptMotorPara->minSpeed = MIN_SPEED_RAD_S;
    ptMotorPara->ratedCurrent = (float)(RATED_CURRENT);
    ptMotorPara->Kt = TORQUE_CONSTANT;
    ptMotorPara->KtRlct = MTPA_TORQUE_C2;
#if 0
    ptMotorPara->focSwitchSpd = FOC_SWITCH_SPD_RAD_S;
#endif
    ptMotorPara->pllStableCnt = PLL_STABLE_CNT;
    
    /* current */
    ptCur->adcHalfCnt = ADC_FULL_CNT >> 1u;
    ptCur->adc2amps = ADC_2_AMPS;
    ptCur->filAdcU.a = ADC_OFFSET_FIL_A;
    ptCur->filAdcU.b = ADC_OFFSET_FIL_B;
    ptCur->filAdcV.a = ADC_OFFSET_FIL_A;
    ptCur->filAdcV.b = ADC_OFFSET_FIL_B;    
    
    /* voltage */
    ptVolt->adc2volt = ADC_2_VOLT;
    ptVolt->phsVoltMode = PHASE_VOLT_MODE;
    tmp16 = (int16_t)CNT_PWM_PER;
    ptSvpwm->halfPrdCnt = tmp16 >> 1u;
    ptSvpwm->vMaxDotPrdCnt = (float)(MAX_PHASE_VOLTAGE) * (float)tmp16;
    ptSvpwm->compareSign = COMPARE_VALUE_SIGN;
    if(1 == COMPARE_VALUE_SIGN){
        ptSvpwm->compareUpBound = tmp16 - PWM_CNT_MARGIN;  /* boundary count value for the minimum lower-side on time */ 
        ptSvpwm->compareLowBound = 0;         
    } else if (-1 == COMPARE_VALUE_SIGN){
        ptSvpwm->compareUpBound = tmp16;  
        ptSvpwm->compareLowBound = PWM_CNT_MARGIN;  /* boundary count value for the minimum lower-side on time */
    } else {}
    ptSvpwm->offsetCnt = (int16_t)PWM_CNT_OFFSET;
    ptSvpwm->maxRange = (float)AVAILABLE_BUS_VOLTAGE_PU;
    
    /* BEMF based angle estimation */
#if PHASE_VOLT_MODE == 0u
    ptThAtan->imposedVolt.oneByPwmPrdCnt = (float)COMPARE_VALUE_SIGN / (float)tmp16;
#else
    ptThAtan->imposedVolt.oneByPwmPrdCnt = 1.0f / (float)tmp16; 
#endif
    ptThAtan->dAngle2Speed.elePerLoop2MechPerSec = ELE_PER_LOOP_2_MECH_PER_SEC;
    ptThAtan->dAngle2Speed.filSpeed.a = SPEED_FBK_FIL_A;
    ptThAtan->dAngle2Speed.filSpeed.b = SPEED_FBK_FIL_B;
    ptThAtan->dAngle2Speed.filSpeedBemfObs.a = SPEED_BEMF_OBS_FIL_A;
    ptThAtan->dAngle2Speed.filSpeedBemfObs.b = SPEED_BEMF_OBS_FIL_B;    
    ptIObsPara->boundary = (float)SMO_BOUNDARY_I;
    ptIObsPara->m = (float)SMO_M;
    ptEObs->lambda = (float)SMO_LAMBDA;
    ptEObs->ls = (float)SMO_LS;
    ptEObs->ts = (float)SMO_PER_LOOP;
    ptIObsPara->mdbi = (float)(SMO_M / SMO_BOUNDARY_I);
    tmp = SMO_EXP_MINUS_RSLST;
    ptIObsPara->a = tmp;
    ptIObsPara->b1 = (float)((tmp - 1.0) / SMO_RS);
    ptIObsPara->b2 = -(ptIObsPara->b1);
    ptIObsPara->b3 = (float)(ptIObsPara->b2 * SMO_LS); 
    ptSmoSpeedIn->perLoop2PerSec = PER_LOOP_2_PER_SEC;
    ptSmoSpeedIn->switchSpeed = SMO_SWITCH_SPEED_RAD_S;
    ptSmoSpeedIn->polePairs = (float)NUM_POLE_PAIRS;

    /* Flux observer */
#if PHASE_VOLT_MODE == 0u 
    ptFluxPll->imposedVolt.oneByPwmPrdCnt = (float)COMPARE_VALUE_SIGN / (float)tmp16;
#else
    ptFluxPll->imposedVolt.oneByPwmPrdCnt = 1.0f / (float)tmp16;
#endif
    ptFluxPll->oneByNumPolePairs = (float)(1.0 / NUM_POLE_PAIRS);
#if 0
    tmp = (float)((PFO_GAMMA_MIN - PFO_GAMMA_MAX) * PFO_PER_LOOP / (RATED_SPEED_RAD_S - MIN_SPEED_RAD_S));
    ptFluxPll->pfoGainA = tmp;
    ptFluxPll->pfoGainB = (float)(PFO_GAMMA_MAX * PFO_PER_LOOP - tmp * MIN_SPEED_RAD_S);
#else
    tmp = (float)((PFO_GAMMA_MAX * MIN_SPEED_RAD_S - PFO_GAMMA_MIN * RATED_SPEED_RAD_S) * PFO_PER_LOOP / (MIN_SPEED_RAD_S - RATED_SPEED_RAD_S));
    ptFluxPll->pfoGainA = tmp;
    ptFluxPll->pfoGainB = (float)((PFO_GAMMA_MAX * PFO_PER_LOOP - tmp) * MIN_SPEED_RAD_S);    
#endif
    ptPfo->inductance = (float)(PFO_LS * PFO_G);
    ptPfo->resistance = (float)PFO_RS;
    ptPfo->ts = (float)(PFO_PER_LOOP * PFO_G);   
    ptPfo->psiOl = (float)(PFO_VIRTUAL_PSI_OL * PFO_G);
    ptPfo->phi = (float)(PFO_PHI * PFO_G);
    ptPfo->phiMax = PFO_PHI_MAX;
    ptPfo->phiMin = PFO_PHI_MIN;
    ptPfo->enablePhiEst = PFO_PHI_EST;
    ptPfo->filA = PFO_FIL_A;
    ptPfo->filB = PFO_FIL_B;    
  
    /* encoder */
    ptThEnc->cnt2MechRad = QEI_CNT_2_MECH_RAD;
    ptThEnc->cnt2EleRad = QEI_CNT_2_ELE_RAD;
    ptThEnc->eleRad2Cnt = QEI_ELE_RAD_2_CNT;
    ptThEnc->cnt2MechRadPerSec = QEI_CNT_2_MECH_RAD_S;
    ptThEnc->cnt2EleRadPerSec = QEI_CNT_2_ELE_RAD_S;
    ptThEnc->speedSampleLoopCntMax = CNT_SPEED_SAMPLE;
#ifdef EPPR_ENHANCE
    ptThEnc->ePPR = (float)ENC_CNT_ELE_REV;
#else
    ptThEnc->ePPR = (int32_t)(ENC_CNT_ELE_REV + 0.5);
#endif
    ptThEnc->mPPR = (int32_t)ENC_CNT_MECH_REV;
    ptThEnc->hallOffsetCnt = HALL_OFFSET_CNT;
    ptThEnc->indexAngleCnt = INDEX_ANGLE_CNT;
    ptThEnc->indexToleranceCnt = INDEX_TOLERANCE_CNT;
    ptThEnc->rotorFluxAlignQeiCnt = ROTOR_FLUX_ALIGN_QEI_CNT;  
    ptThEnc->oneByNumPolePairs = (float)(1.0 / NUM_POLE_PAIRS);
    ptThEnc->indexUseState = INDEX_USE_STATE;
    
    /* Angle tracking pll */
    ptMotionEst->K = (float)PLL_K;
    ptMotionEst->exePrd = (float)PLL_PER_LOOP;    
    ptPll->pst5.maxOut = FLT_MAX;
    ptPll->pst5.minOut = -FLT_MAX; 
    ptPll->pst4.maxOut = FLT_MAX;
    ptPll->pst4.minOut = -FLT_MAX; 
    ptPll->jerk.maxOut = FLT_MAX;
    ptPll->jerk.minOut = -FLT_MAX;     
    ptPll->acc.maxOut = FLT_MAX;
    ptPll->acc.minOut = -FLT_MAX;    
    ptPll->spd.maxOut = FLT_MAX;
    ptPll->spd.minOut = -FLT_MAX; 
    ptPll->pst.maxOut = FLT_MAX;
    ptPll->pst.minOut = -FLT_MAX;     
 
    /* HALL based angle estimation */
    ptThHall->oneByNumPolePair = HALL_1_BY_NUM_POLE_PAIR;
    ptThHall->timerMax = HALL_TIMER_MAX;
    ptThHall->hallOffsetAngle = (float)HALL_OFFSET_ANGLE;
    ptThHall->timerT = HALL_TIMER_CLK_PER;
    ptThHall->spdUpThr = HALL_UP_SPD_THR_RAD_S;
    ptThHall->spdLowThr = HALL_LOW_SPD_THR_RAD_S;
    ptThHall->hallJumpDtctMode = HALL_JUMP_DETECT_MODE;
    
    /* FOC */
    ptFocCurCtrl->flgDecouple = FOC_DECOUPLE;
    ptDecouple->lD = (float)(D_INDUCTANCE * NUM_POLE_PAIRS / MAX_PHASE_VOLTAGE);
    ptDecouple->lQ = (float)(Q_INDUCTANCE * NUM_POLE_PAIRS / MAX_PHASE_VOLTAGE); 
    ptDecouple->mechSpd2Bemf = (float)(MECH_BEMF_CONST_Vpeak_PHASE_RAD_PER_SEC / MAX_PHASE_VOLTAGE);    
    
    /* PI controllers */
    ptIdPi->pG = CURRENT_P_D;
    ptIdPi->iG = CURRENT_I_D;
    ptIdPi->maxOut = MAX_VOLTAGE_VECTOR_AMPLITUDE_PU;
    ptIqPi->pG = CURRENT_P_Q;
    ptIqPi->iG = CURRENT_I_Q;
    ptIqPi->maxOut = MAX_VOLTAGE_VECTOR_AMPLITUDE_PU;
    ptSpeedPi->pG = SPEED_P;
    ptSpeedPi->iG = SPEED_I;
    ptSpeedPi->maxOut = SPEED_OUT_MAX;    
    
    /* open-loop spin */
    ptOpenLoop->alignAngle = (float)OPEN_LOOP_ALIGN_ANGLE;
    ptOpenLoop->shadowAlignAngle = (float)OPEN_LOOP_SHADOW_ALIGN_ANGLE;    
    ptOpenLoop->alignCnt = ALIGN_TIME_CNT;
    ptOpenLoop->accCnt = OPEN_LOOP_ACC_TIME_CNT;
    ptOpenLoop->uniformCnt = OPEN_LOOP_UNIFORM_TIME_CNT;        
    ptOpenLoop->iQref = (float)ALIGN_CURRENT_REF;
    ptOpenLoop->iDref = 0.0f;
    ptOpenLoop->targetSpeed = (float)(MIN_ELE_SPEED_RAD_PER_LOOP);
    ptOpenLoop->dSpeedRef = D_MIN_ELE_SPEED;
    
    /* torque to current */
    ptT2C->nm2amps = NM_2_AMPS;
    ptT2C->flgMtpa = MC_MTPA;
    ptT2C->Kt = MTPA_TORQUE_C1;
    ptT2C->KtRlct = MTPA_TORQUE_C2;
    ptT2C->mtpaIdEstimate.deltaIdC1 = MTPA_DELTA_ID_C1;
    ptT2C->mtpaIdEstimate.deltaIdC2 = MTPA_DELTA_ID_C2;
    ptT2C->mtpaIdEstimate.iDstep = ID_REF_RAMP_STEP;  
    ptT2C->mtpaIdEstimate.iDhigh = 0.0f;
    ptT2C->mtpaIdEstimate.iDlow = (float)(-RATED_CURRENT);
    ptT2C->rampIdRef.step = ID_REF_RAMP_STEP;
    
    /* instructions */
    ptInstr->flgApp = MC_APP;
    ptInstr->flgMode = MC_MODE;
    ptInstr->flgAngle = FOC_ANGLE_MODE;
    ptInstr->flgStallDtct = STALL_DETECTION;
    ptInstr->flgPhsLossDtct = PHS_LOSS_DETECTION;
#if 1
    ptInstr->cnt2radPerSec = ADC_CNT_2_RAD_S;
#else  /* test for ultra low speed reference */
    ptInstr->cnt2radPerSec = 0.5f * ADC_CNT_2_RAD_S;
#endif
    ptInstr->filSpeedRef.a = POT_FIL_A;
    ptInstr->filSpeedRef.b = POT_FIL_B;
    ptInstr->btnDebounceCnt = BUTTON_DEBOUNCE_CNT;
    ptInstr->deceleratoin = (float)(DECELERATION_RAD_S_PER_LOOP);
    
    /* stalling detection */
    ptStlDtct->Ith = STALL_I_THRESHOLD;
    ptStlDtct->filIu.a = STALL_FIL_A;
    ptStlDtct->filIu.b = STALL_FIL_B;
    ptStlDtct->filIv.a = STALL_FIL_A;
    ptStlDtct->filIv.b = STALL_FIL_B;
    
    /* phase loss detection */
    ptPhsLossDtct->spdUp = PHS_LOSS_SPD_UP;
    ptPhsLossDtct->spdLow = PHS_LOSS_SPD_LOW;
    ptPhsLossDtct->cntThr = PHS_LOSS_CNT_THR;
    ptPhsLossDtct->filU.a = PHS_LOSS_FIL_A;
    ptPhsLossDtct->filU.b = PHS_LOSS_FIL_B;
    ptPhsLossDtct->filV.a = PHS_LOSS_FIL_A;
    ptPhsLossDtct->filV.b = PHS_LOSS_FIL_B;
    ptPhsLossDtct->filW.a = PHS_LOSS_FIL_A;
    ptPhsLossDtct->filW.b = PHS_LOSS_FIL_B;
    
#ifdef ENCODER_FEEDBACK_INFORM     
    T_ROTOR_ANGLE_ENCODER_REF * ptThEncRef = &(mcData->rotorAngleEncoderRef);     
    /* encoder */
    ptThEncRef->cnt2Rad = QEI_CNT_2_RAD;
    ptThEncRef->cnt2RadPerSec = QEI_CNT_2_RAD_S;
    ptThEncRef->speedSampleLoopCntMax = CNT_SPEED_SAMPLE;
    ptThEncRef->rotorFluxAlignQeiCnt = ROTOR_FLUX_ALIGN_QEI_CNT;
    ptThEncRef->cntPerEleRev = ENC_CNT_ELE_REV - 1u;
#endif
}

#if 0
void MC_PeripheralConnectMotor1(T_PMSM_FOC * mcData)
{
    T_PERIPHERAL * ptPeri = &(mcData->peripheral);
    
    ptPeri->iUchannel = ADCHS_CH0;
    ptPeri->iVchannel = ADCHS_CH3;
    ptPeri->vBusChannel = ADCHS_CH21;
    ptPeri->potChannel = ADCHS_CH20;
    ptPeri->vUchannel = MCPWM_CH_1;
    ptPeri->vVchannel = MCPWM_CH_2;
    ptPeri->vWchannel = MCPWM_CH_3;
    ptPeri->faultIntSource = INT_SOURCE_PWM1;
    ptPeri->qeiChannel = qei_ch_1;
    ptPeri->buttonPin = GPIO_BUTTON1_PIN;
    ptPeri->ledPin = LED_1_PIN;
#if 1
    ptPeri->hallA = HALL_A_1_PIN;
    ptPeri->hallB = HALL_B_1_PIN;
    ptPeri->hallC = HALL_C_1_PIN;    
#else
    ptPeri->ptHallAread = HAL_HallA1Read;
    ptPeri->ptHallBread = HAL_HallB1Read;
    ptPeri->ptHallCread = HAL_HallC1Read;
#endif
    ptPeri->iCapA.ptBufferRead = ICAP3_CaptureBufferRead;
    ptPeri->iCapA.ptIsBufferNonempty = ICAP3_CaptureStatusGet;
    ptPeri->iCapB.ptBufferRead = ICAP2_CaptureBufferRead;
    ptPeri->iCapB.ptIsBufferNonempty = ICAP2_CaptureStatusGet;
    ptPeri->iCapC.ptBufferRead = ICAP1_CaptureBufferRead;
    ptPeri->iCapC.ptIsBufferNonempty = ICAP1_CaptureStatusGet; 
    ptPeri->ptIcapTimerRead = TMR2_CounterGet;
}
#endif

static void MC_FaultIsr_Motor_1(uint32_t status, uintptr_t context)
{
    T_STATE_DATA * ptMcStt = &(mcFoc1.mcState);
    
    HAL_PwmDisable(); 
    ptMcStt->state = (uint8_t)(mc_fault);
    LED_01_Set();   
}


void MC_PeriIniMotor1(void)
{
    /** ADC end of conversion interrupt generation for FOC control */
    ADC0_InterruptsDisable( ADC_STATUS_RESRDY );
    ADC0_InterruptsClear(ADC_STATUS_MASK);

    /** Enable ADC interrupt for field oriented control */
    ADC0_CallbackRegister(  (ADC_CALLBACK)MC_CtrlLoop, (uintptr_t)dummyForMisra );
    ADC0_InterruptsEnable( ADC_STATUS_RESRDY );
    ADC0_Enable(); 
    
    /* Select ADC channels for phase currents. */
    ADC0_ChannelSelect( ADC_POSINPUT_AIN0, ADC_NEGINPUT_GND);  /* phase A */
    ADC1_ChannelSelect( ADC_POSINPUT_AIN0, ADC_NEGINPUT_GND);  /* phase B */

#ifdef MCLV  
    TCC0_REGS->TCC_WEXCTRL = 0U;
    /* Select output matrix mode. */
    TCC0_REGS->TCC_WEXCTRL = TCC_WEXCTRL_OTMX(0UL);
    /* Set dead time as 0.5us. */
    TCC0_REGS->TCC_WEXCTRL |= TCC_WEXCTRL_DTIEN0_Msk | TCC_WEXCTRL_DTIEN1_Msk | TCC_WEXCTRL_DTIEN2_Msk | TCC_WEXCTRL_DTIEN3_Msk
 	 	 | TCC_WEXCTRL_DTLS(60UL) | TCC_WEXCTRL_DTHS(60UL);
#endif
    
    /** Enable interrupt for fault detection */
    TCC0_PWMCallbackRegister( (TCC_CALLBACK)MC_FaultIsr_Motor_1, (uintptr_t)dummyForMisra );

    /** Enable PWM interrupt to detect fault */
    NVIC_EnableIRQ(TCC0_OTHER_IRQn);

    /** Enables PWM channels. */
    TCC0_PWMStart( );

    /** Disable PWM output */
    HAL_PwmDisable();

    /* Set TC direction as counting down, because in counting up mode, 0 cannot be captured. */
    HAL_TC0_DirDown();
    
    /* Start TC0 for HALL capture. */
    TC0_CaptureStart();
            
    /* Start PDEC in HALL mode. */
    PDEC_HALLStart();
  
#if 0
    /* Start TCs for phase voltage capture. */
    TC2_CaptureStart();
    TC3_CaptureStart();
    TC4_CaptureStart();
#endif    
}
/*******************************************************************************
 End of File
*/ 
