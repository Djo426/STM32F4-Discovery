#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_rtc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32_ub_lis302dl.h"
#include "stm32f4xx_usart.h"
#include "math.h"
#include "stm32_ub_led.h"


      // pin PD12 et PD13 --------> Avance Segway------> PD12 : IN1 , PD13 : IN3 (L293D)
      // pin PD14 et PD15 --------> Recule Segway------> PD14 : IN2 , PD15 : IN4 (L293D)


void Delay(__IO uint32_t nCount);

void TurnLeft(void){
	UB_Led_On(LED_GREEN);
}

void TurnRight(void){
	UB_Led_On(LED_RED);
}

void Avance(void){
	UB_Led_On(LED_BLUE);
}
void Recule(void){
	UB_Led_On(LED_ORANGE);
}


void Usart3_Config(void){
	//USART1 (PA9/PA10)
	//USART3 (PB10/11 and PC10/11)
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	// Configure USART TX/RX as Alternate function
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	// Connect USART pins to AF
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3);

    USART_InitTypeDef USART_InitStructure;

  /* USARTx configuration ------------------------------------------------------*/
  /* USARTx configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART3, &USART_InitStructure);

  USART_Cmd(USART3, ENABLE);
}
int putchar(USART_TypeDef* USARTx, int c){
	while (USART_GetFlagStatus(USARTx , USART_FLAG_TXE) == RESET);
	USART_SendData(USARTx,c);
	//USART1 ->DR = (c & 0xff);
return 0;
}

int getchar(USART_TypeDef* USARTx){
	while (USART_GetFlagStatus(USARTx , USART_FLAG_RXNE) == RESET);
	return USART_ReceiveData(USARTx);
}

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
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   // this sets the pin type to push / pull (as opposed to open drain)
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;   // this enables the pulldown resistor --> we want to detect a high level
    GPIO_Init(GPIOA, &GPIO_InitStructure);



  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);


  /* Setup PWM */
  uint16_t PrescalerValue = 0;
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 41000000) - 1;

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
    UB_Led_Init();
    UB_LIS302_Init(SCALE_8G); // init


  double speed = 0;
  double speed1 = 0;
  double speed2 = 0;
  double speed3 = 0;
  //bluetouth
  Usart3_Config();
  	char a;
  while(1)  //Loop forever
            {
     if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0))
     {
        while(1)  //Loop forever

        	a=getchar(USART3);


      // read LIS302



     TIM4->CCR1 = speed;//CCR1 controls channel 1    pin PD12
      TIM4->CCR2 = speed1;//CCR2 controls channel 1    PIN PD13
      TIM4->CCR3 = speed2;//CCR3 controls channel 1   pin PD14
      TIM4->CCR4 = speed3;//CCR4 controls channel 1   pin PD15

     // Delay(80000);
      UB_LIS302_Read();

      if(LIS302.y_achse>250 && LIS302.y_achse<300) {



             speed=800;
             speed1=800;
             speed2=0;
             speed3=0;
            }

      if(LIS302.y_achse>200 && LIS302.y_achse<250) {



             speed=750;
             speed1=750;
             speed2=0;
             speed3=0;
            }

      if(LIS302.y_achse>150 && LIS302.y_achse<200) {



             speed=700;
             speed1=700;
             speed2=0;
             speed3=0;
            }

      if(LIS302.y_achse>100 && LIS302.y_achse<150) {



             speed=600;
             speed1=600;
             speed2=0;
             speed3=0;
            }

      if(LIS302.y_achse>75 && LIS302.y_achse<100) {



             speed=500;
             speed1=500;
             speed2=0;
             speed3=0;
            }

      if(LIS302.y_achse>50  && LIS302.y_achse<75) {



             speed=350;
             speed1=350;
             speed2=0;
             speed3=0;
            }

      if(LIS302.y_achse>30 && LIS302.y_achse<50) {



             speed=100;
             speed1=100;
             speed2=0;
             speed3=0;
            }

      if(LIS302.y_achse>10 && LIS302.y_achse<30) {



                          speed1=60;
                          speed=60;
                          speed2=0;
                          speed3=0;
                         }




      if(LIS302.y_achse<-250 && LIS302.y_achse>-300) {



               speed2=800;
               speed3=800;
               speed=0;
               speed1=0;
              }

        if(LIS302.y_achse<-200 && LIS302.y_achse>-250) {



               speed2=750;
               speed3=750;
               speed=0;
               speed1=0;
              }

        if(LIS302.y_achse<-150 && LIS302.y_achse>-200) {



               speed2=700;
               speed3=700;
               speed=0;
               speed1=0;
              }

        if(LIS302.y_achse<-100 && LIS302.y_achse>-150) {



               speed2=670;
               speed3=670;
               speed=0;
               speed1=0;
              }

        if(LIS302.y_achse<-75 && LIS302.y_achse>-100) {



               speed2=500;
               speed3=500;
               speed1=0;
               speed=0;
              }

        if(LIS302.y_achse<-50  && LIS302.y_achse>-75) {



               speed2=350;
               speed3=350;
               speed1=0;
               speed=0;
              }

        if(LIS302.y_achse<-30 && LIS302.y_achse>-50) {



               speed2=150;
               speed3=150;
               speed1=0;
               speed=0;
              }

        if(LIS302.y_achse<-10 && LIS302.y_achse>-30) {



                      speed2=100;
                      speed3=100;
                      speed1=0;
                      speed=0;
                     }
       // SystemCoreClock--;
    }/*else if (LIS302.y_achse<3 && LIS302.y_achse>-3)

   {
       speed=0;
       speed1=0;
       speed2=0;
       speed3=0;
    }*/

      if(a=='L'){
          		TurnLeft();
          		speed=600;
          	}

          	if(a=='R'){
          		TurnRight();
          		speed2=300;
          	}
          	if(a=='U'){
          		Avance();
          		speed=300;
          		speed1=300;
          	}
          	if(a=='D'){

          		Recule();
          		speed2=300;
          		speed3=300;
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

