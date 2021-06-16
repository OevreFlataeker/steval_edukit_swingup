Demo project for the STEVAL-EDUKIT pendulum system.
This code tries to swing-up the pendulum up into the upright position
From there the existing demo code from ST to can keep it upright (Integration not shown here)
 
Explanation of overall setup:
The integration of the rotary encoder to read the pendulum angle complicates the setup as the encoder is read using TIM3 in quadrature encoder mode (i.e. the external signal of the encoder 
"drives" the timer's counter forwards and backwards). Unfortunately, the IHM01A1 code expects to get the PWM signal to drive the motor from TIM3 as well (via PC7, alternate function TIM3_CH2) 
but both things are of course not possible at once.
 
Thus a trick is used here:
We use TIM2 to create the actual PWM signal and once the signal is output (on this unconnected pin) we copy/replay the wave form MANUALLY through the desired pin PC7, we we in turn defines as GPIO.
 
See "void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)" in stm32f4xx_hal_msp.c for details how this is done.
Unfortunately this also requires patching the file x_nucleo_ihm01a1_stm32f4xx.h of the BSP in order to switch TIM3 and TIM2 on all relevant code portions of the motor driver's code itself, because we must let the 
BSP code generate the PWM via TIM2 as well. This is a little bit unfortunate but as this project directly imported the relevant files from the BSP and did not link them, no original files of the BSP are modified at least.

For example (experts from x_nucleo_ihm01a1_stm32f4xx.h after change)

[...]
/// Timer used for PWM1
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM1      (TIM2) // Was TIM3

/// Timer used for PWM2
#define BSP_MOTOR_CONTROL_BOARD_TIMER_PWM2      (TIM3) // Was TIM2
[...]

All peripherals are configured using CubeMX integration/.ioc file. However the IHM01A1 code seems to do this on it's own as well (ref file x_nucleo_ihm01a1_stm32f4xx.L6474_Board_SpiInit() and L6474_Board_PwmInit()).
Presumable the functions HAL_TIM_MspPostInit(), HAL_TIM_PWM_MspInit(), HAL_SPI_MspInit(), ... in file stm32f4xx_hal_msp.c are not really needed therefore.
If this assumption holds, they will be removed in a later version.

How is the swing-up performed?

The swing-up is done by following a very simple idea. Using the rotary encoder the bottom crossing (i.e when the pendulum is pointing straight down) is detected. This happens when
the sign of the encoder's value flips from + to - and v.v. If this happens, the bool zero_crossing is set and the main loop handles this by moving the motor in that instant a tiny bit in the forward
or backward direction. This gets a little bit more energy into the pendulum on each zero crossing which helps it to get higher and higher.
 
The current version still has some shortcomings:
- Swing up takes rather long (it works better/faster when patched into the ST provided firmware due to the overall motor parameters used there)
- Crossing the upright position confuses the sign-change detector and no bottom crossing is recognized anymore, in other words: Swing-up only works once. Needs (simple) fixing

The latest version of the ST demo code can be downloaded here: 

https://www.st.com/content/st_com/en/campaigns/educationalplatforms.html
https://www.st.com/content/st_com/en/campaigns/educationalplatforms/motorcontrol-edu.html --> Resources

Changelog:
20210601: ST released the student curriculum files with lots of tutorials.
If you play around with the ST provided firmware make sure you use the version 2.0 of the firmware from the download link above which has a very stable algorithm to keep the pendulum upright. The firmware download
on the product page itself still is an old version.

20210604: Rewrite of this code with further explanations what's going on

 The swing-up code can be easily integrated into the ST provided sources (not done here)

20210616: v3 of the EDUKIT firmware has been published which incorporates the essence of the swing-up code.
Please refer to version 3.0 of the firmware available from the project page at: 
https://drive.google.com/file/d/1iIQZ4iUZDyy4EnBHb67mchZSPRWiUIK8/view
https://drive.google.com/file/d/1blLjVmdHWpM5Fxm3sqMHc_Uhsn8pHlkY/view
