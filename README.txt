 Demo project for the STEVAL-EDUKIT pendulum system.
 This code tries to swing up the pendulum up into the upright position to let then
 take over the existing demo code from ST to keep it upright.
 
 Explanation of overall setup:
 The integration of the rotary encoder to read the pendulum angle complicates the vanilla setup as the encoder
 is read using TIM3 in quadrature encoder mode (i.e. the external signal of the encoder "drives" the timer forwards and backwards).
 Unfortunately, the IHM01A1 code expects to get the PWM signal to drive the motor from TIM3 (via PC7, alternate function TIM3_CH2) but both things are of course not possible at once.
 
 Thus a trick is used here:
 We use TIM2 to create the actual PWM signal and once the signal is output we copy/replay the wave form MANUALLY through the desired pin PC7.
 
 See void "HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)" in stm32f4xx_hal_msp.c for details how this is done.
 Unfortunately this also requires "patching" the file x_nucleo_ihm01a1_stm32f4xx.h of the BSP in order to switch TIM3 and TIM2 on all relevant code portions.
 
 All peripherals are configured using CubeMX integration/.ioc file. However the IHM01A1 code seems to do this on it's own (ref file x_nucleo_ihm01a1_stm32f4xx.L6474_Board_SpiInit() and L6474_Board_PwmInit())
 Presumable the functions HAL_TIM_MspPostInit(), HAL_TIM_PWM_MspInit(), HAL_SPI_MspInit(), ... in file stm32f4xx_hal_msp.c are not really needed therefore.
 If this assumption holds, they will be removed in a later version.
 
 The current version manages to have the pendulum swing up for a little bit over the horizontal plane but fails to get it any higher.
 I presume this is due to the rotary motion counter-acting against the forward/backward acceleration of the motor. Both need to be smoothly synchronized.