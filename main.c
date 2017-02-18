#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_rtc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32_ub_lis302dl.h"
#include <math.h>
#include <stdio.h>


void Delay(__IO uint32_t nCount);

int main(void)
{
    /*Structures used in the configuration*/
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable TIM4 Clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* Enable GPIOD Pins that are used for on board LED's */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  //Enabled GPIOB we are going to use PB6 which is linked to TIM4_CH1 according to the
  //documentation
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  /* Initialise  pins 13, 14 and 15 D - relating to on board LED's*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  // boutton PA0
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;		  // we want to configure PA0
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 	  // we want it to be an input
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   // this sets the pin type to push / pull (as opposed to open drain)
 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;   // this enables the pulldown resistor --> we want to detect a high level
 	GPIO_Init(GPIOA, &GPIO_InitStructure);



  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);


  /* Setup PWM */
  uint16_t PrescalerValue = 0;
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 21000000) - 1;

  /* Setup timer defaults */
  TIM_TimeBaseStructure.TIM_Period = 665;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* Configure timer for PWM - channel 1*/
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  //notice the number 1 in TIM_OC1Init
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* Configure timer for PWM - channel 2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* Configure timer for PWM - channel 3*/
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* Configure timer for PWM - channel 4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM4, ENABLE);

  /* Start timer*/
  TIM_Cmd(TIM4, ENABLE);

  //config();

    SystemInit();

    UB_LIS302_Init(SCALE_8G); // init


  int speed = 0;
  int speed1 = 0;
  int speed2 = 0;
   int speed3 = 0;

  int Kp=3;
  float Kd=0.2;

  int erreur;
  int an_erreur=0;
  int U=0;
  //for this timer configuration a CCR (power) value of 700 yields approximately 3V on the pin (and is the max)

  while(1)  //Loop forever
    		  {
  	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0))
  	{
  		while(1)  //Loop forever
  		  {
	  UB_LIS302_Read(); // read LIS302
      //printf("%f",LIS302.y_achse);

      // pin PD12 et PD13 --------> Avance Segway------> PD12 : IN1 , PD13 : IN3 (L293D)
      // pin PD14 et PD15 --------> Recule Segway------> PD14 : IN2 , PD15 : IN4 (L293D)

	  TIM4->CCR1 = speed;//CCR1 controls channel 1    pin PD12
      TIM4->CCR2 = speed1;//CCR2 controls channel 1    PIN PD13
      TIM4->CCR3 = speed2;//CCR3 controls channel 1   pin PD14
      TIM4->CCR4 = speed3;//CCR4 controls channel 1   pin PD15
      //speed1=600;
      //speed2=600;
     erreur = LIS302.y_achse;

      U=(int) Kp*erreur+Kd*(erreur-an_erreur);
      an_erreur=erreur;
      if (U<0)
      U=-U;
      if (erreur>0){
      speed1=U;
      speed2=U;
      speed3=0;
      speed=0;
      }
      else {
      speed1=0;
      speed2=0;
      speed3=U;
      speed=U;
      }
      Delay(80000);
      if(LIS302.y_achse>80) {
       speed=fabs(LIS302.y_achse)*4;
       speed3=fabs(LIS302.y_achse)*4;
       speed1=0;
       speed2=0;
      }
      if(LIS302.y_achse<-20) {
       speed1=fabs(LIS302.y_achse+40)*5.5;
       speed2=fabs(LIS302.y_achse+40)*5.5;
       speed3=0;
       speed=0;

    }
      if(LIS302.y_achse<80 && LIS302.y_achse>-20) {
    	  speed1=0;
    	       speed2=0;
    	       speed3=0;
    	       speed=0;


      }


}
}
}
}
/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */



void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}
