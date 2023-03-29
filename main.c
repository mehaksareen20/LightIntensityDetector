

#include "main.h"

#include "stm32f4xx_hal.h"
#include "Board_GLCD.h"
#include "GLCD_Config.h"
#define USE_GLCD        1 

uint32_t adcValue = 0;
uint16_t LED_Toggle = 0;


extern GLCD_FONT GLCD_Font_6x8;
extern GLCD_FONT GLCD_Font_16x24;
#ifdef RTE_CMSIS_RTOS_RTX
extern   uint32_t               os_time;
uint32_t HAL_GetTick (void) {
  return os_time;
}
#endif

//osthreaddef(ui_thread,  osprioritynormal, 1u, null);
//osthreaddef(can_thread, osprioritynormal, 1u, null);

//osthreadid ui_thread_id;
//osthreadid can_thread_id;

/*----------------------------------------------------------------------------
 * SystemClock_Config: System Clock Configuration
 *----------------------------------------------------------------------------*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the
     device is clocked below the maximum system frequency (see datasheet). */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

static void Error_Handler (void) {
  while (1);
}

void LED_Init(void)
{
		__HAL_RCC_GPIOH_CLK_ENABLE() ;// Clock enable for GPIO port H //We will study more about this later
	  __HAL_RCC_GPIOG_CLK_ENABLE() ;// Clock enable for GPIO port H //We will study more about this later		
		__HAL_RCC_GPIOI_CLK_ENABLE() ;// Clock enable for GPIO port H //We will study more about this later
		
		
	    GPIO_InitTypeDef GPIO_InitStruct;
	    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_7|GPIO_PIN_6;// More pins can be added using |
	    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
		  GPIOH->BSRR=0xCC0000;
		  GPIO_InitStruct.Pin = GPIO_PIN_10;// More pins can be added using |
	    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
			 GPIOI->BSRR=0x4000000;
		
			GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_6;// More pins can be added using |
	    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
			GPIOG->BSRR=0x1C00000;
}




void Check_Light_Intensity(void)
{
	if((adcValue < 0x200) && (adcValue > 0x000)) 
	{
				
		GLCD_ClearScreen        ();
		GLCD_DrawString        (0, 6*24, "    LEVEL - 3     ");
	}
	else if((adcValue  > 0x200) && (adcValue  < 0x700))
	{

		GLCD_ClearScreen        ();
		GLCD_DrawString        (0, 6*24, "    LEVEL - 2     ");
	}
	else if((adcValue > 0x700) && (adcValue < 0xE00))
	{
		
		GLCD_ClearScreen        ();
		GLCD_DrawString        (0, 6*24, "    LEVEL - 1     ");
	}
	else if(adcValue > 0xE00)
	{
		
		GLCD_ClearScreen        ();
		GLCD_DrawString        (0, 6*24, "LEVEL - 0     ");
	}
		
}
uint8_t toggle=0;
uint8_t tog=0;
void TIM2_IRQHandler(void){
	tog=~tog;
}
	
void ADC_IRQHandler (void)
{toggle=~toggle;
	ADC3->SR = ~2; //Clear EOC Flag  
	adcValue=(ADC3->DR); //read ADC value 
	if ((TIM2->SR &0x0001)!=0)
	{
		TIM2->SR &=~(1<<0);
		
	}  
	
	Check_Light_Intensity();
	
} 



void ADC_Initialize(void) 
{
	RCC->APB2ENR|=(1UL<<10); //Enable ADC3 clock     
	RCC->AHB1ENR|=(1UL<<5);//Enable GPIOF Clock     
	GPIOF->MODER|=(3UL<<2*7);//PF7 in Analog mode     
	ADC3->SQR1=0;     
	ADC3->SQR2=0; 	
	ADC3->SQR3=(5UL<<0); //SQ1 channel selection     
	
	ADC3->SMPR1=   0;       
	ADC3->SMPR2= (7UL<<3*5);;//time=480 cycles         
	ADC3->CR2 &=~2; //Single conversion mode  
         ADC3->CR2|=3UL<<28;	//RISING AND FALLING EDGE DETECTION
	ADC3->CR2|=3UL<<24;	//Timer 2 - CC2
	ADC3->CR1|=(1UL<<5);//Enable EOC IRQ     
	ADC3->CR2|=(1UL<<0);// ADC Enable     
	NVIC_EnableIRQ(ADC_IRQn);   
	
}

void TIM2_Initialize (void){
	const uint16_t PSC_val =  8400; //prescalar value
  const uint16_t ARR_val= 5000;//1000;//ARR value 
	RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;//enable timer clock
  	TIM2->PSC=PSC_val-1;//comparison of ARR an Timer cont will take 1 cycle hence count-1 has to be uploaded for both PSC & ARR
	TIM2->ARR=ARR_val-1;//
	TIM2->CR1=(1UL<<0);//set commanf register 
TIM2->CCMR1|=(3UL<<12);//OCIREF toggles which serves as trigger for ADC
TIM2->CCER|=(1UL<<4);//CC2E set// refer programmers manual
	
}


int main (void) {

  HAL_Init();                           // Initialize the HAL Library
  SystemClock_Config();                 // Configure the System Clock
  SystemCoreClockUpdate();              // Update system clock
	GLCD_Initialize         ();
  GLCD_SetBackgroundColor (GLCD_COLOR_BLUE);
  GLCD_SetForegroundColor (GLCD_COLOR_WHITE);
  GLCD_ClearScreen        ();
  GLCD_SetFont            (&GLCD_Font_16x24);
  GLCD_DrawString         (0, 0*24, "                   ");
  GLCD_DrawString         (0, 1*24, "                   ");
  GLCD_DrawString         (0, 2*24, "                ");
	GLCD_DrawString        (0, 3*24, "    INTENSITY LEVEL     ");
	ADC_Initialize(); 
	TIM2_Initialize();
  HAL_PWR_EnableSleepOnExit();


  while (1) {
    
  }  
}
