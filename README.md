<!-- Please do not change this logo with link -->

<a target="_blank" href="https://www.microchip.com/" id="top-of-page">
   <picture>
      <source media="(prefers-color-scheme: light)" srcset="images/mchp_logo_light.png" width="350">
      <source media="(prefers-color-scheme: dark)" srcset="images/mchp_logo_dark.png" width="350">
      <img alt="Microchip Technologies Inc." src="https://www.microchip.com/content/experience-fragments/mchp/en_us/site/header/master/_jcr_content/root/responsivegrid/header/logo.coreimg.100.300.png/1605828081463/microchip.png">
   </picture>
</a>

# SAME54 Flux Observer and HALL-based FOC Demo Applications

## Objective
This example is an MPLAB X project implementing three different field oriented control (FOC) applications for a PMSM. The FOC applications include:
- HALL-based FOC
- Flux-based FOC
- HALL-flux-based FOC

## Hardware Used
- [SAME54 DIM](https://www.microchip.com/en-us/development-tool/EV24X01A)
- [MCHV-230V-1.5KW](https://www.microchip.com/en-us/development-tool/ev78u65a) or 
- [MCLV-48V-300W](https://www.microchip.com/en-us/development-tool/EV18H47A)  

## Software Used
-  [MPLAB® X IDE 5.30 or newer](http://www.microchip.com/mplab/mplab-x-ide)
-  [MPLAB® XC32 4.60 or a newer compiler](http://www.microchip.com/mplab/compilers)
-  [MPLAB® Code Configurator (MCC) 3.95.0 or newer](https://www.microchip.com/mplab/mplab-code-configurator)

## Directory structure
- *README.md*
    - this file
- *firmware*
    - *flux_hall_foc.X*
        - the MPLAB X project folder
    - *src*
        - source files of the project
- *docs* 
    - the tuning guide of the project

## Peripherals used
- ADC: to read analog signals such as phase current and BUS voltage
- TCC: to generate PWM waveforms driving motors
- TC: to capture duty cycle of the PWM waveforms
- UART: to implement communication for X2Cscope tools 

## Release Information of Version 1.70
- Added the ACT_57BLF02 motor
- For HALL-flux-based FOC at low angular speed, use flux-based speed feedback instead of HALL-based speed feedback. By doing this, the update rate of speed feedback increases, thus speed control performance is enhanced. 
- Added an option to measure phase voltage by hardware to increase the accuracy at low angular speed. With the hardware-measurement, performance of the flux observer at low angular speed is enhanced.

## Release Information of Version 1.50
- Added the support for MCLV platform
- In HALL-flux-based FOC, improved angle/speed assignment logic

## Release Information of Version 1.00
- HALL-based FOC
    - The rotor flux angle/angular velocity are estimated by exploiting HALL signals
    - At low (angular) velocity, the estimated velocity is updated at the jump point of each phase's HALL signal to make the refreshing rate as high as possible  
    - At high velocity, since the refreshing rate is high enough, the estimated velocity is updated at the jump point of a particular phase's HALL signal to suppress fluctuations   
- Flux-based FOC
    - The rotor flux vector is estimated by a nonlinear observer which does not need angular velocity information
    - The rotor flux amplitude can be estimated if the rotor flux amplitude is not precisely known
    - The rotor flux angle/angular velocity are estimated by a 3-pole trajectory estimator
    - The open-closed loop switch is done by an instananeous reference frame transformation, so that the open-loop stage is minimized and the stability is enhanced    
- HALL-flux-based FOC 
    - At low velocity, the rotor flux angle/angular velocity are estimated by exploiting HALL signals
    - At high velocity, the rotor flux vector is estimated by the nonlinear observer and the rotor flux angle/angular velocity are estimated by the 3-pole trajectory estimator
- The software is thoroughly reconstructed to provide simplified data structure, control flow and file structure
- The parameters of the motor and its control are allocated in a dedicated header file so that they can be modified easily
- The d-q axes crossover compensation for current control is implemented as an optional feature
- MTPA is offered as an optional feature
- Optional stalling detection
- Optional phase loss detection

## Software Introduction, Operation, Project Configuration, Tuning Method
- Please refer to the tuning guide for detailed information about software introduction, operation, project configuration and tuning method. 



