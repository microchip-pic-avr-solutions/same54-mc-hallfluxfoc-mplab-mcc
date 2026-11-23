/*******************************************************************************
  Motor Control Hardware Abstraction interface file

  Company:
    Microchip Technology Inc.

  File Name:
    mc_motor_1.h

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
#ifndef MC_MOTOR_1_H
#define MC_MOTOR_1_H


/**********************- Included Files -*******************************/
#include <math.h>
#include "mc_control_loop.h"
#include "mc_library.h"


/**********************- Macro Definitions -*******************************/ 
/*---- Select board ----*/
#define MCLV  /* MCHV Or MCLV */ 


/*---- Select motor ----*/
#ifdef MCHV
    #define LEADSHINE_EL5  /* LEADSHINE_EL5 or JINGWEI_1 */  
#endif
#ifdef MCLV
    #define ACT_57BLF02  /* LONG_HURST, ACT_57BLF02, or SDCQ_200W */    
#endif


/*---- Board parameters ----*/
#ifdef MCHV
#define DC_BUS_VOLTAGE                                      (324.0)  /* Volt */
#define ADC_FULL_CNT                                        (4096)
#define SHUNT_RESISTANCE                                    (0.003)  /* Ohm */
#define CURRENT_CIRCUIT_GAIN                                (25.0)
#define ADC_POWER_VOLTAGE                                   (3.3)  /* Volt */
#define VOLTAGE_SAMPLE_GAIN                                 (137.36)  /* DC_BUS_VOLTAGE / ADC pin voltage */
#define PHASE_VOLT_MODE                                     (0u)  /* Phase voltages are obtained by: 0 -- voltage reference, or 1 -- HW detection. */
#endif

#ifdef MCLV
#define DC_BUS_VOLTAGE                                      (24.0)  /* Volt */
#define ADC_FULL_CNT                                        (4096)
#define SHUNT_RESISTANCE                                    (0.003)  /* Ohm */
#define CURRENT_CIRCUIT_GAIN                                (24.95)
#define ADC_POWER_VOLTAGE                                   (3.3)  /* Volt */
#define VOLTAGE_SAMPLE_GAIN                                 (23.0)  /* DC_BUS_VOLTAGE / ADC pin voltage */
#define PHASE_VOLT_MODE                                     (0u)  /* Phase voltages are obtained by: 0 -- voltage reference, or 1 -- HW detection. */
#endif


/*---- Motor-dependent parameters ----*/ 
#ifdef  LEADSHINE_EL5   
/* Motor parameters */    
#define PHASE_RESISTANCE                                    (1.42)  /* Ohm, alt: 1.70578 */
#define Q_INDUCTANCE                                        (0.00270)  /* Henry */
#define D_INDUCTANCE                                        (0.00225)  /* Henry */
#define INERTIA                                             (0.00005)  /* Kg.m^2 */  
#define BEMF_CONST_V_PEAK_LL_KRPM_MECH                      (43.98)  /* line voltage peak value, V/KRPM */
#define NUM_POLE_PAIRS                                      (5.0)
#define RATED_SPEED_RPM                                     (3000.0)  /* RPM */
#define MIN_SPEED_RPM                                       (400.0)  /* the minimum speed for the closed-loop FOC to work, RPM */
#define RATED_CURRENT                                       (4.242)  /* phase current peak value, Amps */
/* Position sensor */
#define ENCODER_PULSES_PER_REV                              (2500)
#define INDEX_IMV_VALUE                                     (0u)  /* IMV field value for QEI peripheral */
#define INDEX_ANGLE_CNT                                     (0) /* the rotor angle when index event is detected, cnt */
#define INDEX_TOLERANCE_CNT                                 (20) /* the allowed difference between successive QEI counter value captured at index pulses */
#define INDEX_USE_STATE                                     (ang_index_no_use)  /* Option 1 -- ang_hall_cali_done: index is used to calibrate rotor angle. Option 2 -- ang_index_no_use: index is not used. */
#define HALL_OFFSET_ANGLE                                   (DEGREE_150_RAD) /* the offset angle of the HALL-angle table, rad */
#define HALL_JUMP_DETECT_MODE                               (1u)  /* 1u: each HALL jump is detected. 2u: HALL jumps are detected alternatively. */
/* Control parameters */
#define ALIGN_CURRENT_REF                                   (0.5)  /* current reference of the open-loop alignment stage, Amps */
#define ALIGN_TIME                                          (0.001)  /* length of the open-loop alignment stage, Sec */
#define OPEN_LOOP_ACC_TIME                                  (0.4)  /* length of the open-loop acceleration stage, Sec */
#define OPEN_LOOP_UNIFORM_TIME                              (0.1)  /* length of the open-loop uniform speed stage, Sec */
#define WC_SPEED_LOOP                                       (200.0)  /* desired speed control bandwidth, rad/s */
#define DAMP_SPEED_LOOP                                     (1.0)  /* damper ratio of speed control */
#define WC_CURRENT_LOOP_D                                   (4000.0)  /* desired d-axis current control bandwidth, rad/s */
#define WC_CURRENT_LOOP_Q                                   (4000.0)  /* desired q-axis current control bandwidth, rad/s */
#define RAMP_SLOPE_ID_REF                                   (0.1)  /* ramp slope to calculate Id reference, Amps/sec */
#define FOC_DECOUPLE                                        (false)  /* Enable FOC decouple function. */
#define HALL_UP_SPD_THR_RPM                                 (1000.0)  /* the speed upper threshold to change HALL calculation state, RPM */
#define HALL_LOW_SPD_THR_RPM                                (400.0)  /* the speed lower threshold to change HALL calculation state, RPM */
#if 0
#define FOC_SWITCH_SPD_RPM                                  (1000.0)  /* the speed to switch from low-speed FOC to high-speed FOC, RPM */
#endif
#define CNT_SPEED_SAMPLE                                    (20)  /* speed sample window length / current loop period */
/* BEMF sliding mode observer */
#define SMO_BOUNDARY_I                                      (0.5)  /* boundary layer of current observer, Amps */
#define SMO_M                                               (0.8 * SMO_BOUNDARY_I / SMO_PER_LOOP)  /* Gain of current observer. The coefficient is recommended to be in [0.5, 1]. */
#define SMO_LAMBDA                                          (-1000.0)  /* desired eigenvalue of BEMF observer, rad/s */
#define FIL_T_SPEED_FBK                                     (0.001)  /* time constant of the LPF to calculate speed feedback, sec */
#define FIL_T_SPEED_BEMF_OBS                                (0.01)  /* time constant of the LPF to calculate speed input of the BEMF observer, sec */
/* Flux observer */
#define PFO_GAMMA_MIN                                       (1000.0)  /* the minimum gain of the flux observer */
#define PFO_GAMMA_MAX                                       (4000.0)  /* the maximum gain of the flux observer */ 
#define PFO_PHI_EST                                         (true)  /* Enable rotor flux amplitude estimation. */
#define PFO_WC_PHI_EST                                      (200.0)  /* flux amplitude estimation bandwidth, rad/s */
/* Angle tracking pll */
#define PLL_K                                               (2000.0)  /* desired estimation bandwidth, rad/s */
/* MTPA */
#define MC_MTPA                                             (false)  /* Enable MPTA function. */
#define MTPA_W_D                                            (10.0)  /* desired converge rate of Id reference, rad/s */
/* Stalling detection */
#define STALL_DETECTION                                     (true)  /* Enable stalling detection. */
#define STALL_T                                             (5.0)  /* the desired time to detect stalling, sec */
#define STALL_I                                             (RATED_CURRENT * M_1_BY_SQRT2)  /* the allowed current amplitude of each phase when the angular speed is zero, Amps */
/* Phase loss detection */
#define PHS_LOSS_DETECTION                                  (false)  /* Enable phase loss detection. */
#define PHS_LOSS_MIN_SPD_RPM_MECH                           (10.0)  /* minimum mechanical angular speed to enable phase loss detection, RPM */
#define PHS_LOSS_K                                          (2.0)   /* a user-defined parameter, suggested to be >=1 */  
#endif
    
#ifdef  JINGWEI_1  
/* Motor parameters */   
#define PHASE_RESISTANCE                                    (1.75)  /* Ohm */
#define Q_INDUCTANCE                                        (0.0059)  /* Henry */
#define D_INDUCTANCE                                        (0.0054)  /* Henry */
#define INERTIA                                             (0.216)  /* Kg.m^2 */  
#define BEMF_CONST_V_PEAK_LL_KRPM_MECH                      (104.9585)  /* line voltage peak value, V/KRPM */
#define NUM_POLE_PAIRS                                      (2.0)
#define RATED_SPEED_RPM                                     (2000.0)  /* RPM */
#define MIN_SPEED_RPM                                       (200.0)  /* the minimum speed for the closed-loop FOC to work, RPM */
#define RATED_CURRENT                                       (5.0)  /* phase current peak value, Amps, 12.5 */
/* Position sensor */
#define ENCODER_PULSES_PER_REV                              (2500)
#define INDEX_IMV_VALUE                                     (0u)  /* IMV field value for QEI peripheral */
#define INDEX_ANGLE_CNT                                     (1833) /* the rotor angle when index event is detected, cnt, 330 degree */
#define INDEX_TOLERANCE_CNT                                 (20) /* the allowed difference between successive QEI counter value captured at index pulses */
#define INDEX_USE_STATE                                     (ang_hall_cali_done)  /* Option 1 -- ang_hall_cali_done: index is used to calibrate rotor angle. Option 2 -- ang_index_no_use: index is not used. */
#define HALL_OFFSET_ANGLE                                   (DEGREE_330_RAD) /* the offset angle of the HALL-angle table, rad */
#define HALL_JUMP_DETECT_MODE                               (1u)  /* 1u: each HALL jump is detected. 2u: HALL jumps are detected alternatively. */
/* Control parameters */
#define ALIGN_CURRENT_REF                                   (0.5)  /* current reference of the open-loop alignment stage, Amps */
#define ALIGN_TIME                                          (1.0)  /* length of the open-loop alignment stage, Sec */
#define OPEN_LOOP_ACC_TIME                                  (2.0)  /* length of the open-loop acceleration stage, Sec */
#define OPEN_LOOP_UNIFORM_TIME                              (1.0)  /* length of the open-loop uniform speed stage, Sec */
#define WC_SPEED_LOOP                                       (200.0)  /* desired speed control bandwidth, rad/s */
#define DAMP_SPEED_LOOP                                     (1.0)  /* damper ratio of speed control */
#define WC_CURRENT_LOOP_D                                   (4000.0)  /* desired d-axis current control bandwidth, rad/s */
#define WC_CURRENT_LOOP_Q                                   (4000.0)  /* desired q-axis current control bandwidth, rad/s */
#define RAMP_SLOPE_ID_REF                                   (0.2)  /* ramp slope to calculate Id reference, Amps/sec */
#define FOC_DECOUPLE                                        (false)  /* Enable FOC decouple function. */
#define HALL_UP_SPD_THR_RPM                                 (1600.0)  /* the speed upper threshold to change HALL calculation state, RPM */
#define HALL_LOW_SPD_THR_RPM                                (1000.0)  /* the speed lower threshold to change HALL calculation state, RPM */
#if 0
#define FOC_SWITCH_SPD_RPM                                  (1000.0)  /* the speed to switch from Hall-FOC to sensor-less FOC, RPM */
#endif
#define CNT_SPEED_SAMPLE                                    (60)  /* speed sample window length / current loop period */
/* BEMF sliding mode observer */
#define SMO_BOUNDARY_I                                      (1.5)  /* boundary layer of current observer, Amps */
#define SMO_M                                               (0.8 * SMO_BOUNDARY_I / SMO_PER_LOOP)  /* Gain of current observer. The coefficient is recommended to be in [0.5, 1]. */
#define SMO_LAMBDA                                          (-1000.0)  /* desired eigenvalue of BEMF observer, rad/s */
#define FIL_T_SPEED_FBK                                     (0.001)  /* time constant of the LPF to calculate speed feedback, sec */
#define FIL_T_SPEED_BEMF_OBS                                (0.01)  /* time constant of the LPF to calculate speed input of the BEMF observer, sec */
/* Flux observer */
#define PFO_GAMMA_MIN                                       (2000.0)  /* the minimum gain of the flux observer */
#define PFO_GAMMA_MAX                                       (2000.0)  /* the maximum gain of the flux observer */
#define PFO_PHI_EST                                         (true)  /* Enable rotor flux amplitude estimation. */
#define PFO_WC_PHI_EST                                      (5000.0)  /* flux amplitude estimation bandwidth, rad/s */
/* Angle tracking pll */
#define PLL_K                                               (2000.0)  /* desired estimation bandwidth, rad/s */
/* MTPA */
#define MC_MTPA                                             (false)  /* Enable MPTA function. */
#define MTPA_W_D                                            (10.0)  /* desired converge rate of Id reference, rad/s */
/* Stalling detection */
#define STALL_DETECTION                                     (false)  /* Enable stalling detection. */
#define STALL_T                                             (5.0)  /* the desired time to detect stalling, sec */
#define STALL_I                                             (RATED_CURRENT * M_1_BY_SQRT2)  /* the allowed current amplitude of each phase when the angular speed is zero, Amps */
/* Phase loss detection */
#define PHS_LOSS_DETECTION                                  (false)  /* Enable phase loss detection. */
#define PHS_LOSS_MIN_SPD_RPM_MECH                           (10.0)  /* minimum mechanical angular speed to enable phase loss detection, RPM */
#define PHS_LOSS_K                                          (2.0)   /* a user-defined parameter, suggested to be >=1 */  
#endif    

#ifdef  LONG_HURST   
/* Motor parameters */    
#define PHASE_RESISTANCE                                    (0.34)  /* Ohm */
#define Q_INDUCTANCE                                        (0.000324)  /* Henry */
#define D_INDUCTANCE                                        (0.000309)  /* Henry */
#define INERTIA                                             (0.000018)  /* Kg.m^2 */  
#define BEMF_CONST_V_PEAK_LL_KRPM_MECH                      (1.2 * 6.2)  /* line voltage peak value, V/KRPM */
#define NUM_POLE_PAIRS                                      (5.0)
#define RATED_SPEED_RPM                                     (3000.0)  /* RPM */
#define MIN_SPEED_RPM                                       (50.0)  /* the minimum speed for the closed-loop FOC to work, RPM */
#define RATED_CURRENT                                       (4.0)  /* phase current peak value, Amps */
/* Position sensor */
#define ENCODER_PULSES_PER_REV                              (250)
#define INDEX_IMV_VALUE                                     (0u)  /* IMV field value for QEI peripheral */
#define INDEX_ANGLE_CNT                                     (0) /* the rotor angle when index event is detected, cnt */
#define INDEX_TOLERANCE_CNT                                 (20) /* the allowed difference between successive QEI counter value captured at index pulses */
#define INDEX_USE_STATE                                     (ang_index_no_use)  /* Option 1 -- ang_hall_cali_done: index is used to calibrate rotor angle. Option 2 -- ang_index_no_use: index is not used. */
#define HALL_OFFSET_ANGLE                                   (DEGREE_30_RAD) /* the offset angle of the HALL-angle table, rad */
#define HALL_JUMP_DETECT_MODE                               (1u)  /* 1u: each HALL jump is detected. 2u: HALL jumps are detected alternatively. */
/* Control parameters */
#define ALIGN_CURRENT_REF                                   (0.5)  /* current reference of the open-loop alignment stage, Amps */
#define ALIGN_TIME                                          (0.1)  /* length of the open-loop alignment stage, Sec */
#define OPEN_LOOP_ACC_TIME                                  (0.2)  /* length of the open-loop acceleration stage, Sec */
#define OPEN_LOOP_UNIFORM_TIME                              (0.1)  /* length of the open-loop uniform speed stage, Sec */
#define WC_SPEED_LOOP                                       (100.0)  /* desired speed control bandwidth, rad/s */
#define DAMP_SPEED_LOOP                                     (1.0)  /* damper ratio of speed control */
#define WC_CURRENT_LOOP_D                                   (4000.0)  /* desired d-axis current control bandwidth, rad/s */
#define WC_CURRENT_LOOP_Q                                   (4000.0)  /* desired q-axis current control bandwidth, rad/s */
#define RAMP_SLOPE_ID_REF                                   (0.1)  /* ramp slope to calculate Id reference, Amps/sec */
#define FOC_DECOUPLE                                        (false)  /* Enable FOC decouple function. */
#define HALL_UP_SPD_THR_RPM                                 (1000.0)  /* the speed upper threshold to change HALL calculation state, RPM */
#define HALL_LOW_SPD_THR_RPM                                (400.0)  /* the speed lower threshold to change HALL calculation state, RPM */
#if 0
#define FOC_SWITCH_SPD_RPM                                  (400.0)  /* the speed to switch from low-speed FOC to high-speed FOC, RPM */
#endif
#define CNT_SPEED_SAMPLE                                    (20)  /* speed sample window length / current loop period */
/* BEMF sliding mode observer */
#define SMO_BOUNDARY_I                                      (0.5)  /* boundary layer of current observer, Amps */
#define SMO_M                                               (0.8 * SMO_BOUNDARY_I / SMO_PER_LOOP)  /* Gain of current observer. The coefficient is recommended to be in [0.5, 1]. */
#define SMO_LAMBDA                                          (-1000.0)  /* desired eigenvalue of BEMF observer, rad/s */
#define FIL_T_SPEED_FBK                                     (0.001)  /* time constant of the LPF to calculate speed feedback, sec */
#define FIL_T_SPEED_BEMF_OBS                                (0.01)  /* time constant of the LPF to calculate speed input of the BEMF observer, sec */
/* Flux observer */
#define PFO_GAMMA_MIN                                       (1000.0)  /* the minimum gain of the flux observer */
#define PFO_GAMMA_MAX                                       (4000.0)  /* the maximum gain of the flux observer */ 
#define PFO_PHI_EST                                         (false)  /* Enable rotor flux amplitude estimation. */
#define PFO_WC_PHI_EST                                      (200.0)  /* flux amplitude estimation bandwidth, rad/s */
/* Angle tracking pll */
#define PLL_K                                               (1000.0)  /* desired estimation bandwidth, rad/s */
/* MTPA */
#define MC_MTPA                                             (false)  /* Enable MPTA function. */
#define MTPA_W_D                                            (10.0)  /* desired converge rate of Id reference, rad/s */
/* Stalling detection */
#define STALL_DETECTION                                     (true)  /* Enable stalling detection. */
#define STALL_T                                             (5.0)  /* the desired time to detect stalling, sec */
#define STALL_I                                             (RATED_CURRENT * M_1_BY_SQRT2)  /* the allowed current amplitude of each phase when the angular speed is zero, Amps */
/* Phase loss detection */
#define PHS_LOSS_DETECTION                                  (true)  /* Enable phase loss detection. */
#define PHS_LOSS_MIN_SPD_RPM_MECH                           (10.0)  /* minimum mechanical angular speed to enable phase loss detection, RPM */
#define PHS_LOSS_K                                          (2.0)   /* a user-defined parameter, suggested to be >=1 */  
#endif

#ifdef ACT_57BLF02
/* Motor parameters */    
#define PHASE_RESISTANCE                                    (0.205)  /* Ohm */
#define Q_INDUCTANCE                                        (0.000167)  /* Henry 0.000183 */
#define D_INDUCTANCE                                        (0.000112)  /* Henry 0.000140*/
#define INERTIA                                             (0.000017)  /* Kg.m^2 */  
#define BEMF_CONST_V_PEAK_LL_KRPM_MECH                      (6.3)  /* line voltage peak value, V/KRPM */
#define NUM_POLE_PAIRS                                      (4.0)
#define RATED_SPEED_RPM                                     (3000.0)  /* RPM */
#define MIN_SPEED_RPM                                       (50.0)  /* the minimum speed for the closed-loop FOC to work, RPM */
#define RATED_CURRENT                                       (11.0)  /* phase current peak value, Amps */
/* Position sensor */
#define ENCODER_PULSES_PER_REV                              (250)
#define INDEX_IMV_VALUE                                     (0u)  /* IMV field value for QEI peripheral */
#define INDEX_ANGLE_CNT                                     (0) /* the rotor angle when index event is detected, cnt */
#define INDEX_TOLERANCE_CNT                                 (20) /* the allowed difference between successive QEI counter value captured at index pulses */
#define INDEX_USE_STATE                                     (ang_index_no_use)  /* Option 1 -- ang_hall_cali_done: index is used to calibrate rotor angle. Option 2 -- ang_index_no_use: index is not used. */
#define HALL_OFFSET_ANGLE                                   (DEGREE_210_RAD) /* the offset angle of the HALL-angle table, rad */
#define HALL_JUMP_DETECT_MODE                               (1u)  /* 1u: each HALL jump is detected. 2u: HALL jumps are detected alternatively. */
/* Control parameters */
#define ALIGN_CURRENT_REF                                   (1.0)  /* current reference of the open-loop alignment stage, Amps */
#define ALIGN_TIME                                          (0.001)  /* length of the open-loop alignment stage, Sec */
#define OPEN_LOOP_ACC_TIME                                  (0.2)  /* length of the open-loop acceleration stage, Sec */
#define OPEN_LOOP_UNIFORM_TIME                              (0.1)  /* length of the open-loop uniform speed stage, Sec */
#define WC_SPEED_LOOP                                       (400.0)  /* desired speed control bandwidth, rad/s */
#define DAMP_SPEED_LOOP                                     (1.0)  /* damper ratio of speed control */
#define WC_CURRENT_LOOP_D                                   (4000.0)  /* desired d-axis current control bandwidth, rad/s */
#define WC_CURRENT_LOOP_Q                                   (4000.0)  /* desired q-axis current control bandwidth, rad/s */
#define RAMP_SLOPE_ID_REF                                   (0.1)  /* ramp slope to calculate Id reference, Amps/sec */
#define FOC_DECOUPLE                                        (false)  /* Enable FOC decouple function. */
#define HALL_UP_SPD_THR_RPM                                 (1000.0)  /* the speed upper threshold to change HALL calculation state, RPM */
#define HALL_LOW_SPD_THR_RPM                                (400.0)  /* the speed lower threshold to change HALL calculation state, RPM */
#if 0
#define FOC_SWITCH_SPD_RPM                                  (400.0)  /* the speed to switch from low-speed FOC to high-speed FOC, RPM */
#endif
#define CNT_SPEED_SAMPLE                                    (20)  /* speed sample window length / current loop period */
/* BEMF sliding mode observer */
#define SMO_BOUNDARY_I                                      (0.5)  /* boundary layer of current observer, Amps */
#define SMO_M                                               (0.8 * SMO_BOUNDARY_I / SMO_PER_LOOP)  /* Gain of current observer. The coefficient is recommended to be in [0.5, 1]. */
#define SMO_LAMBDA                                          (-1000.0)  /* desired eigenvalue of BEMF observer, rad/s */
#define FIL_T_SPEED_FBK                                     (0.001)  /* time constant of the LPF to calculate speed feedback, sec */
#define FIL_T_SPEED_BEMF_OBS                                (0.01)  /* time constant of the LPF to calculate speed input of the BEMF observer, sec */
/* Flux observer */
#define PFO_GAMMA_MIN                                       (1000.0)  /* the minimum gain of the flux observer */
#define PFO_GAMMA_MAX                                       (4000.0)  /* the maximum gain of the flux observer */ 
#define PFO_PHI_EST                                         (true)  /* Enable rotor flux amplitude estimation. */
#define PFO_WC_PHI_EST                                      (1000.0)  /* flux amplitude estimation bandwidth, rad/s */
/* Angle tracking pll */
#define PLL_K                                               (2000.0)  /* desired estimation bandwidth, rad/s */
/* MTPA */
#define MC_MTPA                                             (false)  /* Enable MPTA function. */
#define MTPA_W_D                                            (10.0)  /* desired converge rate of Id reference, rad/s */
/* Stalling detection */
#define STALL_DETECTION                                     (false)  /* Enable stalling detection. */
#define STALL_T                                             (5.0)  /* the desired time to detect stalling, sec */
#define STALL_I                                             (RATED_CURRENT * M_1_BY_SQRT2)  /* the allowed current amplitude of each phase when the angular speed is zero, Amps */
/* Phase loss detection */
#define PHS_LOSS_DETECTION                                  (false)  /* Enable phase loss detection. */
#define PHS_LOSS_MIN_SPD_RPM_MECH                           (10.0)  /* minimum mechanical angular speed to enable phase loss detection, RPM */
#define PHS_LOSS_K                                          (2.0)   /* a user-defined parameter, suggested to be >=1 */  
#endif

#ifdef  SDCQ_200W  
/* Motor parameters */   
#define PHASE_RESISTANCE                                    (0.055)  /* Ohm */
#define Q_INDUCTANCE                                        (0.0001025)  /* Henry */
#define D_INDUCTANCE                                        (0.000095)  /* Henry */
#define INERTIA                                             (0.000028)  /* Kg.m^2 */  
#define BEMF_CONST_V_PEAK_LL_KRPM_MECH                      (5.8)  /* line voltage peak value, V/KRPM */
#define NUM_POLE_PAIRS                                      (5.0)
#define RATED_SPEED_RPM                                     (3000.0)  /* RPM */
#define MIN_SPEED_RPM                                       (210.0)  /* the minimum speed for the closed-loop FOC to work, RPM */
#define RATED_CURRENT                                       (5.0)  /* phase current peak value, Amps, 12.5 */
/* Position sensor */
#define ENCODER_PULSES_PER_REV                              (2500)
#define INDEX_IMV_VALUE                                     (0u)  /* IMV field value for QEI peripheral */
#define INDEX_ANGLE_CNT                                     (1833) /* the rotor angle when index event is detected, cnt, 330 degree */
#define INDEX_TOLERANCE_CNT                                 (20) /* the allowed difference between successive QEI counter value captured at index pulses */
#define INDEX_USE_STATE                                     (ang_hall_cali_done)  /* Option 1 -- ang_hall_cali_done: index is used to calibrate rotor angle. Option 2 -- ang_index_no_use: index is not used. */
#define HALL_OFFSET_ANGLE                                   (DEGREE_330_RAD) /* the offset angle of the HALL-angle table, rad */
#define HALL_JUMP_DETECT_MODE                               (1u)  /* 1u: each HALL jump is detected. 2u: HALL jumps are detected alternatively. */
/* Control parameters */
#define ALIGN_CURRENT_REF                                   (1.5)  /* current reference of the open-loop alignment stage, Amps */
#define ALIGN_TIME                                          (1.0)  /* length of the open-loop alignment stage, Sec */
#define OPEN_LOOP_ACC_TIME                                  (2.0)  /* length of the open-loop acceleration stage, Sec */
#define OPEN_LOOP_UNIFORM_TIME                              (1.0)  /* length of the open-loop uniform speed stage, Sec */
#define WC_SPEED_LOOP                                       (200.0)  /* desired speed control bandwidth, rad/s */
#define DAMP_SPEED_LOOP                                     (1.0)  /* damper ratio of speed control */
#define WC_CURRENT_LOOP_D                                   (10000.0)  /* desired d-axis current control bandwidth, rad/s */
#define WC_CURRENT_LOOP_Q                                   (10000.0)  /* desired q-axis current control bandwidth, rad/s */
#define RAMP_SLOPE_ID_REF                                   (0.2)  /* ramp slope to calculate Id reference, Amps/sec */
#define FOC_DECOUPLE                                        (false)  /* Enable FOC decouple function. */
#define HALL_UP_SPD_THR_RPM                                 (600.0)  /* the speed upper threshold to change HALL calculation state, RPM */
#define HALL_LOW_SPD_THR_RPM                                (200.0)  /* the speed lower threshold to change HALL calculation state, RPM */
#if 0
#define FOC_SWITCH_SPD_RPM                                  (400.0)  /* the speed to switch from Hall-FOC to sensor-less FOC, RPM */
#endif
#define CNT_SPEED_SAMPLE                                    (60)  /* speed sample window length / current loop period */
/* BEMF sliding mode observer */
#define SMO_BOUNDARY_I                                      (1.5)  /* boundary layer of current observer, Amps */
#define SMO_M                                               (0.8 * SMO_BOUNDARY_I / SMO_PER_LOOP)  /* Gain of current observer. The coefficient is recommended to be in [0.5, 1]. */
#define SMO_LAMBDA                                          (-1000.0)  /* desired eigenvalue of BEMF observer, rad/s */
#define FIL_T_SPEED_FBK                                     (0.001)  /* time constant of the LPF to calculate speed feedback, sec */
#define FIL_T_SPEED_BEMF_OBS                                (0.01)  /* time constant of the LPF to calculate speed input of the BEMF observer, sec */
/* Flux observer */
#define PFO_GAMMA_MIN                                       (1000.0)  /* the minimum gain of the flux observer */
#define PFO_GAMMA_MAX                                       (4000.0)  /* the maximum gain of the flux observer */
#define PFO_PHI_EST                                         (true)  /* Enable rotor flux amplitude estimation. */
#define PFO_WC_PHI_EST                                      (200.0)  /* flux amplitude estimation bandwidth, rad/s */
/* Angle tracking pll */
#define PLL_K                                               (2000.0)  /* desired estimation bandwidth, rad/s */
/* MTPA */
#define MC_MTPA                                             (false)  /* Enable MPTA function. */
#define MTPA_W_D                                            (10.0)  /* desired converge rate of Id reference, rad/s */
/* Stalling detection */
#define STALL_DETECTION                                     (false)  /* Enable stalling detection. */
#define STALL_T                                             (5.0)  /* the desired time to detect stalling, sec */
#define STALL_I                                             (RATED_CURRENT * M_1_BY_SQRT2)  /* the allowed current amplitude of each phase when the angular speed is zero, Amps */
/* Phase loss detection */
#define PHS_LOSS_DETECTION                                  (false)  /* Enable phase loss detection. */
#define PHS_LOSS_MIN_SPD_RPM_MECH                           (10.0)  /* minimum mechanical angular speed to enable phase loss detection, RPM */
#define PHS_LOSS_K                                          (2.0)   /* a user-defined parameter, suggested to be >=1 */  
#endif    


/*---- Motor-independent parameters ----*/
/* Control options */
#define MC_APP                                              (ma_flux_foc)
#define MC_MODE                                             (md_close_loop)
#define FOC_ANGLE_MODE                                      (fa_single_angle)
#define COMPARE_VALUE_SIGN                                  (-1)  /* 1: big compare value leads to big PWM duty cycle. Or -1: big compare value leads to small PWM duty cycle. 
                                                                    Suppose phase shunt resistors are at lower bridge arms. If ADC is triggered near TOP of PWM counter, then the sign is 1. 
                                                                    If ADC is triggered near BOTTOM of PWM counter, then the sign is -1. */
#define MC_OVM_LEVEL                                        (0u)
/* Control parameters */
#define PER_CURRENT_LOOP                                    PER_MC_ISR   /* current loop period, sec */
#define PER_SPEED_LOOP                                      PER_CURRENT_LOOP  /* speed loop period, sec */
#define PER_STALL_DETECTION                                 PER_CURRENT_LOOP  /* stalling detection period, sec */
#define PER_PHS_LOSS_DETECTION                              PER_CURRENT_LOOP  /* phase loss detection period, sec */
#define FIL_T_ADC_OFFSET                                    (0.5)  /* time constant of the LPF to calculate ADC offset, sec */
#define FIL_T_POT                                           (0.5)  /* time constant of the POT signal LPF, sec */
#define DECELERATE_TIME                                     (5.0) /* deceleration time from rated angular speed to 0, sec */
#if 0
#define DECELERATE_MIN_SPEED                                (2.0 * MIN_SPEED_RAD_S)  /* The angular speed to end deceleration and stop PWM output. Big coefficient may be needed if the speed feedback is HALL-based. */
#endif
#define MIN_SHUNT_TIME                                      (1.5)  /* minimum time window for shunt current sampling, uSec */
#define PWM_DEAD_TIME                                       (1.0)  /* PWM dead time, uSec */       
#define OPEN_LOOP_ALIGN_ANGLE                               (M_3_PI_OVER_2)  /* Rad */
#define ROTOR_FLUX_ALIGN_ANGLE                              (0.0)       /* Rad */
#define OPEN_LOOP_SHADOW_ALIGN_ANGLE                        (0.0)       /* Rad */
#define BUTTON_DEBOUNCE_TIME                                (0.5)       /* Sec */
/* BEMF sliding mode observer */
#define SMO_PER_LOOP                                        PER_CURRENT_LOOP    /* smo loop period, sec */
#define SMO_RS                                              PHASE_RESISTANCE 
#define SMO_LS                                              Q_INDUCTANCE  
#define SMO_P                                               NUM_POLE_PAIRS 
#define SMO_SWITCH_SPEED_RATIO                              (0.95)      /* ratio = switch speed / minimum speed, shall be close to and less than 1. */
                                                                        /* The SMO uses estimated speed if speed reference > switch speed. */
/* Flux observer */
#define PFO_PER_LOOP                                        PER_CURRENT_LOOP    /* pfo loop period, sec */
#define PFO_LS                                              Q_INDUCTANCE 
#define PFO_RS                                              PHASE_RESISTANCE
#define PFO_PHI_MAX                                         (5.0f)
#define PFO_PHI_MIN                                         (0.1f)
/* Angle tracking pll */
#define PLL_PER_LOOP                                        PER_CURRENT_LOOP  /* pll loop period, sec */
/* HALL-based angle */
#define HALL_TIMER_MAX                                      (0x7FFFFFFFu)
#define HALL_TIMER_CLK_FRQ                                  (60.0)  /* input capture timer clock frequency, MHz */    
/* MTPA */
#define MTPA_PER_LOOP                                       PER_CURRENT_LOOP    /* mtpa loop period, sec */


/*---- Derived parameters ----*/
#define MIN_SHUNT_CNT                                       (int16_t)((MIN_SHUNT_TIME * PWM_CLK_FREQ * 0.5) + 0.5) 
#define PWM_DEAD_CNT                                        (int16_t)((PWM_DEAD_TIME * PWM_CLK_FREQ) + 0.5)
#define PWM_CNT_MARGIN                                      (MIN_SHUNT_CNT + PWM_DEAD_CNT)
#define PWM_CNT_OFFSET                                      (-1 * COMPARE_VALUE_SIGN * ((PWM_CNT_MARGIN) >> 1))
#define MAX_VBUS_UTILIZATION_RATIO                          ((PER_PWM_US - MIN_SHUNT_TIME - 2.0 * PWM_DEAD_TIME) / PER_PWM_US)
#define AVAILABLE_BUS_VOLTAGE                               (MAX_VBUS_UTILIZATION_RATIO * DC_BUS_VOLTAGE)

#if 0
#define MAX_PHASE_VOLTAGE                                   (DC_BUS_VOLTAGE / M_SQRT3)
#else
#define MAX_PHASE_VOLTAGE                                   (AVAILABLE_BUS_VOLTAGE * 2.0 / M_PI)
#endif

#if (MC_OVM_LEVEL == 0u)  /* No over modulation */
    #define MAX_VOLTAGE_VECTOR_AMPLITUDE_PU                 (0.9068f)
#elif (MC_OVM_LEVEL == 1u)  /* Over modulation only in mode 1 */
    #define MAX_VOLTAGE_VECTOR_AMPLITUDE_PU                 (0.9517f)
#else /* MC_OVM_LEVEL == 2u. Over modulation in both mode 1 & mode 2 */
    #define MAX_VOLTAGE_VECTOR_AMPLITUDE_PU                 (0.9999f)
#endif

#define AVAILABLE_BUS_VOLTAGE_PU                            (AVAILABLE_BUS_VOLTAGE / MAX_PHASE_VOLTAGE)
#define CURRENT_P_D                                         (float)(WC_CURRENT_LOOP_D * D_INDUCTANCE / MAX_PHASE_VOLTAGE)
#define CURRENT_I_D                                         (float)(WC_CURRENT_LOOP_D * PHASE_RESISTANCE * PER_CURRENT_LOOP / MAX_PHASE_VOLTAGE)
#define CURRENT_P_Q                                         (float)(WC_CURRENT_LOOP_Q * Q_INDUCTANCE / MAX_PHASE_VOLTAGE)
#define CURRENT_I_Q                                         (float)(WC_CURRENT_LOOP_Q * PHASE_RESISTANCE * PER_CURRENT_LOOP / MAX_PHASE_VOLTAGE)
#define MECH_BEMF_CONST_Vpeak_PHASE_RAD_PER_SEC             (float)(BEMF_CONST_V_PEAK_LL_KRPM_MECH / M_SQRT3 * 60.0 / M_2PI / 1000.0)
#define TORQUE_CONSTANT                                     (float)(1.5 * MECH_BEMF_CONST_Vpeak_PHASE_RAD_PER_SEC)
#define ELE_BEMF_CONST_Vpeak_PHASE_RAD_PER_SEC              (MECH_BEMF_CONST_Vpeak_PHASE_RAD_PER_SEC / NUM_POLE_PAIRS) 
#define INERTIA_OVER_KT                                     (float)((double)INERTIA / (double)TORQUE_CONSTANT)
#define NM_2_AMPS                                           (float)(1.0 / TORQUE_CONSTANT)
#define RATED_TORQUE_SPMSM                                  (float)(RATED_CURRENT * TORQUE_CONSTANT)
#define MTPA_DELTA_L                                        (Q_INDUCTANCE - D_INDUCTANCE)   
#define MTPA_PSI_R                                          ELE_BEMF_CONST_Vpeak_PHASE_RAD_PER_SEC  /* rotor linkage vector amplitude (amplitude invariant transformation) */
#define MTPA_PSI_OVER_DELTA_L                               (MTPA_PSI_R / MTPA_DELTA_L)
#define MTPA_PSI_OVER_DELTA_L_SQ                            (MTPA_PSI_OVER_DELTA_L * MTPA_PSI_OVER_DELTA_L)  
#define MTPA_TORQUE_C1                                      TORQUE_CONSTANT
#define MTPA_TORQUE_C2                                      (float)(-1.5 * NUM_POLE_PAIRS * MTPA_DELTA_L)  /* negative */
#define MTPA_8_RATED_I_SQ                                   (8.0 * RATED_CURRENT * RATED_CURRENT)
#define MTPA_RATED_ID_SQRT_PART                             (sqrt(MTPA_PSI_OVER_DELTA_L_SQ + MTPA_8_RATED_I_SQ))
#define MTPA_RATED_ID                                       (0.25 * (MTPA_PSI_OVER_DELTA_L - MTPA_RATED_ID_SQRT_PART))  /* negative */
#define MTPA_RATED_IQ_SQ                                    (MTPA_RATED_ID * (MTPA_RATED_ID - MTPA_PSI_OVER_DELTA_L))
#define MTPA_RATED_TORQUE_ID_PART                           (MTPA_TORQUE_C1 + MTPA_TORQUE_C2 * MTPA_RATED_ID)
#define MTPA_RATED_TORQUE_SQ                                (MTPA_RATED_IQ_SQ * MTPA_RATED_TORQUE_ID_PART * MTPA_RATED_TORQUE_ID_PART)
#define MTPA_RATED_TORQUE                                   (float)(sqrt(MTPA_RATED_TORQUE_SQ))
#define MTPA_DELTA_ID_C2                                    (float)(-MTPA_PER_LOOP * MTPA_W_D)  /* negative */
#define MTPA_DELTA_ID_C1                                    (float)(MTPA_PER_LOOP * MTPA_W_D * MTPA_DELTA_L / MTPA_PSI_R)
#define PFO_PHI                                             ELE_BEMF_CONST_Vpeak_PHASE_RAD_PER_SEC  /* rotor linkage vector amplitude (amplitude invariant transformation) */
#define PFO_G                                               (1.0 / PFO_PHI)  /* scale factor for the flux */
#define PFO_DELTA_L                                         (D_INDUCTANCE - Q_INDUCTANCE)
#define PFO_VIRTUAL_PHI_OL                                  (float)(PFO_PHI + PFO_DELTA_L * ALIGN_CURRENT_REF) /* virtual rotor flux amplitude during open-loop alignment */
#define PFO_VIRTUAL_PSI_OL                                  (float)(PFO_VIRTUAL_PHI_OL + (PFO_LS * ALIGN_CURRENT_REF))  /* virtual total flux amplitude during open-loop alignment */
#define PFO_T_PHI_EST                                       (1.0 / PFO_WC_PHI_EST)
#define PFO_FIL_B                                           (float)(PFO_PER_LOOP / PFO_T_PHI_EST)
#define PFO_FIL_A                                           (float)(1.0 - PFO_FIL_B) 
#define STALL_FIL_B                                         (float)(PER_STALL_DETECTION / STALL_T)
#define STALL_FIL_A                                         (float)(1.0 - STALL_FIL_B)
#define STALL_I_THRESHOLD                                   (float)(STALL_I * 0.63212)
#define PHS_LOSS_MIN_SPD_RAD_SEC_MECH                       (PHS_LOSS_MIN_SPD_RPM_MECH * M_2PI / 60.0)
#define PHS_LOSS_MIN_SPD_RAD_SEC_ELE                        (float)(PHS_LOSS_MIN_SPD_RAD_SEC_MECH * NUM_POLE_PAIRS)
#define PHS_LOSS_T_FIL                                      (float)(PHS_LOSS_K / PHS_LOSS_MIN_SPD_RAD_SEC_ELE)
#define PHS_LOSS_T_THR                                      (PHS_LOSS_T_FIL * M_2PI)
#define PHS_LOSS_CNT_THR                                    (uint32_t)((PHS_LOSS_T_THR / PER_PHS_LOSS_DETECTION) + 0.5)
#define PHS_LOSS_FIL_B                                      (float)(PER_PHS_LOSS_DETECTION / PHS_LOSS_T_FIL)
#define PHS_LOSS_FIL_A                                      (float)(1.0 - PHS_LOSS_FIL_B) 
#define PHS_LOSS_SPD_LOW                                    (float)(PHS_LOSS_MIN_SPD_RAD_SEC_MECH)
#define PHS_LOSS_SPD_UP                                     (float)(2.0f * PHS_LOSS_SPD_LOW)

#if 0  /* output in Amps */
    #define SPEED_P                                             (float)(INERTIA_OVER_KT * WC_SPEED_LOOP * DAMP_SPEED_LOOP)
    #define SPEED_I                                             (float)(INERTIA_OVER_KT * WC_SPEED_LOOP * WC_SPEED_LOOP * 0.2 * PER_SPEED_LOOP)
    #define SPEED_OUT_MAX                                       RATED_CURRENT 
#else  /* output in Nm */
    #define SPEED_P                                             (float)(INERTIA * WC_SPEED_LOOP * DAMP_SPEED_LOOP)
    #define SPEED_I                                             (float)(INERTIA * WC_SPEED_LOOP * WC_SPEED_LOOP * 0.2 * PER_SPEED_LOOP)
    #if (MC_MTPA == true) 
        #define SPEED_OUT_MAX                                       MTPA_RATED_TORQUE 
    #else
        #define SPEED_OUT_MAX                                       RATED_TORQUE_SPMSM
    #endif
#endif
#define ADC_2_AMPS                                          (float)(ADC_POWER_VOLTAGE / (SHUNT_RESISTANCE * CURRENT_CIRCUIT_GAIN * (double)ADC_FULL_CNT))
#define ADC_2_VOLT                                          (float)(ADC_POWER_VOLTAGE / (double)ADC_FULL_CNT * VOLTAGE_SAMPLE_GAIN)
#define ADC_OFFSET_FIL_B                                    (float)(PER_CURRENT_LOOP / FIL_T_ADC_OFFSET)
#define ADC_OFFSET_FIL_A                                    (float)(1.0 - ADC_OFFSET_FIL_B)
#define POT_FIL_B                                           (float)(PER_LOW_FREQ_LOOP / FIL_T_POT)
#define POT_FIL_A                                           (float)(1.0 - POT_FIL_B)
#define ID_REF_RAMP_STEP                                    (float)(RAMP_SLOPE_ID_REF * PER_CURRENT_LOOP)
#define SPEED_FBK_FIL_B                                     (float)(SMO_PER_LOOP / FIL_T_SPEED_FBK)
#define SPEED_FBK_FIL_A                                     (float)(1.0 - SPEED_FBK_FIL_B) 
#define SPEED_BEMF_OBS_FIL_B                                (float)(SMO_PER_LOOP / FIL_T_SPEED_BEMF_OBS)
#define SPEED_BEMF_OBS_FIL_A                                (float)(1.0 - SPEED_BEMF_OBS_FIL_B) 
#define BUTTON_DEBOUNCE_CNT                                 (uint32_t)(BUTTON_DEBOUNCE_TIME / PER_LOW_FREQ_LOOP + 0.5)
#define RATED_SPEED_RAD_S                                   (float)(RATED_SPEED_RPM * M_2PI / 60.0)
#define MIN_SPEED_RAD_S                                     (float)(MIN_SPEED_RPM * M_2PI / 60.0)
#define FRQ_CURRENT_LOOP                                    (1.0 / PER_CURRENT_LOOP)
#define ALIGN_TIME_CNT                                      (uint32_t)(ALIGN_TIME * FRQ_CURRENT_LOOP + 0.5)     
#define OPEN_LOOP_ACC_TIME_CNT                              (uint32_t)(OPEN_LOOP_ACC_TIME * FRQ_CURRENT_LOOP + 0.5)   
#define OPEN_LOOP_UNIFORM_TIME_CNT                          (uint32_t)(OPEN_LOOP_UNIFORM_TIME * FRQ_CURRENT_LOOP + 0.5)   
#define MIN_ELE_SPEED_RAD_PER_LOOP                          (MIN_SPEED_RAD_S * NUM_POLE_PAIRS * PER_CURRENT_LOOP)
#define D_MIN_ELE_SPEED                                     (float)(MIN_ELE_SPEED_RAD_PER_LOOP / OPEN_LOOP_ACC_TIME_CNT)
#define DECELERATE_TIME_CNT                                 (DECELERATE_TIME * FRQ_CURRENT_LOOP)
#define DECELERATION_RAD_S_S                                (RATED_SPEED_RAD_S / DECELERATE_TIME)
#define DECELERATION_RAD_S_PER_LOOP                         (DECELERATION_RAD_S_S * PER_CURRENT_LOOP)

#define ADC_CNT_2_RAD_S                                     (float)(RATED_SPEED_RAD_S / (double)ADC_FULL_CNT)
#define ENC_CNT_MECH_REV                                    ((double)ENCODER_PULSES_PER_REV * 4.0)
#define ENC_CNT_ELE_REV                                     (ENC_CNT_MECH_REV / NUM_POLE_PAIRS)
#define SPEED_SAMPLE_WINDOW                                 (PER_CURRENT_LOOP * (double)CNT_SPEED_SAMPLE)    /* speed sample window length, sec */
#define MECH_RAD_2_QEI_CNT                                  (float)(ENC_CNT_MECH_REV / M_2PI)
#define QEI_CNT_2_MECH_RAD                                  (float)(M_2PI / ENC_CNT_MECH_REV)  
#define QEI_CNT_2_ELE_RAD                                   (float)(M_2PI / ENC_CNT_ELE_REV)
#define QEI_ELE_RAD_2_CNT                                   (float)(ENC_CNT_ELE_REV / M_2PI)
#define QEI_CNT_2_MECH_RAD_S                                (float)(QEI_CNT_2_MECH_RAD / SPEED_SAMPLE_WINDOW)
#define QEI_CNT_2_ELE_RAD_S                                 (float)(QEI_CNT_2_ELE_RAD / SPEED_SAMPLE_WINDOW)
#define HALL_OFFSET_CNT                                     (int32_t)((HALL_OFFSET_ANGLE / M_2PI * ENC_CNT_ELE_REV) + 0.5) /* the offset angle of the HALL-angle table, cnt */

#define QEI_CNT_2_RAD                                       (float)(M_2PI / ENC_CNT_ELE_REV)
#define QEI_CNT_2_RAD_S                                     (float)(QEI_CNT_2_RAD / SPEED_SAMPLE_WINDOW / NUM_POLE_PAIRS)
#define ROTOR_FLUX_ALIGN_QEI_CNT                            (uint32_t)((ROTOR_FLUX_ALIGN_ANGLE * ENC_CNT_ELE_REV / M_2PI) + 0.5)
#define ELE_PER_LOOP_2_MECH_PER_SEC                         (float)(FRQ_CURRENT_LOOP / NUM_POLE_PAIRS)
#define PER_LOOP_2_PER_SEC                                  (float)(FRQ_CURRENT_LOOP)
#define SMO_RSLST                                           (SMO_RS / SMO_LS * SMO_PER_LOOP)
#define SMO_EXP_MINUS_RSLST                                 (float)(exp(-SMO_RSLST))
#define SMO_SWITCH_SPEED_RAD_S                              (float)(SMO_SWITCH_SPEED_RATIO * MIN_SPEED_RAD_S * NUM_POLE_PAIRS)
#define HALL_1_BY_NUM_POLE_PAIR                             (float)(1.0 / NUM_POLE_PAIRS)
#define HALL_TIMER_CLK_PER                                  (FLOAT)(1.0 / (HALL_TIMER_CLK_FRQ * 1000000.0))
#define HALL_UP_SPD_THR_RAD_S                               (float)(HALL_UP_SPD_THR_RPM * M_2PI / 60.0)
#define HALL_LOW_SPD_THR_RAD_S                              (float)(HALL_LOW_SPD_THR_RPM * M_2PI / 60.0)
#if 0
#define FOC_SWITCH_SPD_RAD_S                                (float)(FOC_SWITCH_SPD_RPM * M_2PI / 60.0)
#endif
#define PLL_STABLE_CNT                                      (uint32_t)(5.0 / PLL_K / PLL_PER_LOOP)



/**********************- Function Declarations -*******************************/
#if 0
void MC_ParaConfigMotor1(T_PMSM_FOC * mcData);
void MC_PeriIniMotor1(void);
void MC_PeripheralConnectMotor1(T_PMSM_FOC * mcData);
#endif


/**********************- Variable Declarations -*******************************/
#if 0
extern T_PMSM_FOC mcFoc1;
#endif

#endif /* end of _MC_MOTOR_1_H */

/*******************************************************************************
 End of File
*/    
