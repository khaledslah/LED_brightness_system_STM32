
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#include "my.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define ReadPort(a, b) HAL_GPIO_ReadPin(a, b)
#define WrietPort(a, b) HAL_GPIO_WritePin(a, b)

//global variables
char keyflg = 0;
char keyflg_n = 0;
char keyPressed = 0;
char keyState = 0;
int timeCnt;
int cnt = 0;

int g_FadePeriod[PWM_CNT];
double g_Brightness[PWM_CNT];
int g_SetBrightness[PWM_CNT];
double g_fadeStep[PWM_CNT];

int g_status = STATE_INIT;

int g_t1Tic = 0;
int g_tmr_1s= 0;

int keyPressCnt = 0;
int keyTempCnt = 0;
int release_count = 500;
//my functions

void Flash_Write_Start (uint32_t Flash_Address, uint32_t Flash_Data)
{
	 HAL_FLASH_Unlock();
   FLASH_EraseInitTypeDef f;
	 f.TypeErase = FLASH_TYPEERASE_PAGES;
	 f.PageAddress = 0x08007000;
	 f.NbPages = 1;
	 uint32_t PageError = 0;
	 HAL_FLASHEx_Erase(&f, &PageError);	
   HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)0x08007000, Flash_Data);
   HAL_FLASH_Lock();
	

}
uint32_t Flash_Read(uint32_t Fash_Address){
	uint32_t data = *(__IO uint32_t*)(0x08007000);
	return data;
	
}
uint32_t g_level = 0;
void keyProc()
{
		switch(keyState)
		{
		
			 case KEYSTATE_NONE://turn off
				for(int i = 0; i < PWM_CNT; i++)
				{
					LedOn_FadeOnTime(i, 0, 500);
				}
				g_status = STATE_INIT;			
			break;
			case KEYSTATE_FIRST://on first click rise from 0 to 100% (50% duty cycle) over 3 second transition so it rises like the sun.
				for(int i = 0; i < PWM_CNT; i++)
				{
					LedOn_FadeOnTime(i, MAX_BRIGHT, 1500);					
				}
				g_status = STATE_FIRST;
				g_level = 100;
			break;
			case KEYSTATE_SECOND://On second touch reduce from 100% to 65% over 1 second transition.
				for(int i = 0; i < PWM_CNT; i++)
				{
					LedOn_FadeOnTime(i, ceil(60.0f*MAX_BRIGHT/100), 500);
				}
				g_status = STATE_SECOND;
				g_level = 60;
			break;
			case KEYSTATE_THIRD://On third tough reduce from 65% to 35% over1 second transition.
				for(int i = 0; i < PWM_CNT; i++)
				{
					LedOn_FadeOnTime(i, ceil(35.0f*MAX_BRIGHT/100), 500);
				}
				g_status = STATE_THIRD;
				g_level = 35;
			break;
			case KEYSTATE_FOURTH://On fourth touch reduce from 36% to 15% over 1 second transition.
				for(int i = 0; i < PWM_CNT; i++)
				{
					LedOn_FadeOnTime(i, ceil(15.0f*MAX_BRIGHT/100), 500);
				}
				g_status = STATE_FOURTH;
				g_level = 15;				
			break;
			case KEYSTATE_FADEON:
				keyState = KEYSTATE_NONE;
				for(int i = 0; i < PWM_CNT; i++)
				{
					double brightness = ceil(5.0f*MAX_BRIGHT/100);
					g_SetBrightness[i] = brightness;
					g_Brightness[i] = brightness;
					//SetBrightness(i, brightness);
					SetRealBrightness(i, brightness);
				}
				for(int i = 0; i < PWM_CNT; i++)
				{
					LedOn_FadeOnTime(i, MAX_BRIGHT ,900000);//60 min
				}
				g_status = STATE_FADEON;
			break;
			case KEYSTATE_FADEOFF:
				keyState = KEYSTATE_NONE;
				for(int i = 0; i < PWM_CNT; i++)
				{
					LedOn_FadeOnTime(i, 0, 900000);//60 min
				}					
				g_status = STATE_FADEOFF;
			
/*				for(int i = 0; i < PWM_CNT; i++)
				{
					g_SetBrightness[i] = 100;
					g_Brightness[i] = 100;
					//SetBrightness(i, brightness);
					SetRealBrightness(i, 100);
					
					LedOn_FadeOnTime(i, 0, 30000);//60 min
				}		
*/				
				g_status = STATE_FADEOFF;
			break;
			case KEYSTATE_FADEOFF_NEXT:
				keyState = KEYSTATE_NONE;
				for(int i = 0; i < PWM_CNT; i++){
					LedOn_FadeOnTime(i, 0 ,1500);
				}
				g_status = STATE_FADEOFF_NEXT;
				break;
			
			case KEYSTATE_FADEON_DEMO:
				for(int i = 0; i < PWM_CNT; i++)
				{
				/*	double brightness = ceil(5.0f*MAX_BRIGHT/100);
					g_SetBrightness[i] = brightness;
					g_Brightness[i] = brightness;
					//SetBrightness(i, brightness);
					SetRealBrightness(i, brightness);
					*/
					g_SetBrightness[i] = 0;
					g_Brightness[i] = 0;
					//SetBrightness(i, brightness);
					SetRealBrightness(i, 0);
				}
				for(int i = 0; i < PWM_CNT; i++)
				{
					LedOn_FadeOnTime(i, MAX_BRIGHT ,3500);//60 min
				}
				g_status = STATE_FADEON_DEMO;
				keyState = KEYSTATE_NONE;
			break;
			case KEYSTATE_FADEOFF_DEMO:
				for(int i = 0; i < PWM_CNT; i++)
				{
					LedOn_FadeOnTime(i, 0, 3500);//60 min
				}					
				g_status = STATE_FADEOFF_DEMO;
				keyState = KEYSTATE_NONE;
			break;
		}
}

void keyCheck()
{
	HAL_Delay(1);
}

int getLogBright(double bn)
{
	int nret = 0;
	//LOG(1001- B2)/LOG(MAX_BRIGHTNESS)
	//double ret = log10(MAX_BRIGHT - bn)/log10(MAX_BRIGHT)*MAX_BRIGHT;
	double ret;
	if(bn > 1)
		ret = log(bn+1)/log(MAX_BRIGHT)*MAX_BRIGHT;
	else
		ret = 0;
	nret = (int)rint(ret);
	return nret;
}
int tempppp = 0;
void SetRealBrightness(int ch, double bn)
{
	int brightness = (int)rint(bn);//getLogBright(bn);
//	int brightness = getLogBright(bn);
	tempppp = brightness;
	switch(ch)
	{
		case PWM_CH0:
			htim3.Instance->CCR4 = brightness;
		break;
		case PWM_CH1:
			htim2.Instance->CCR1 = brightness;
		break;
		case PWM_CH2:
			htim3.Instance->CCR2 = brightness;
		break;
		case PWM_CH3:
			htim3.Instance->CCR3 = brightness;
		break;
		case PWM_CH4:
			htim2.Instance->CCR4 = brightness;
		break;
		case PWM_CH5:
			htim2.Instance->CCR3 = brightness;
		break;
		case PWM_CH6:
			htim2.Instance->CCR2 = brightness;
		break;
		case PWM_CH7:
			htim3.Instance->CCR1 = brightness;
		break;
	}
}
void setBrightnessAll(double bn)
{
	int brightness = (int)rint(bn);
	if(brightness < MIN_BRIGHT) brightness = 0;
//	brightness = 40;

			htim3.Instance->CCR4 = brightness;

			htim2.Instance->CCR1 = brightness;

			htim3.Instance->CCR2 = brightness;

			htim3.Instance->CCR3 = brightness;

			htim2.Instance->CCR4 = brightness;

			htim2.Instance->CCR3 = brightness;

			htim2.Instance->CCR2 = brightness;
			htim3.Instance->CCR1 = brightness;

}
	int nRet = 0;
int GetBrightness(int channel)
{

	switch(channel)
	{
		case PWM_CH0:
			nRet = htim3.Instance->CCR4;
		break;
		case PWM_CH1:
			nRet = htim2.Instance->CCR1;
		break;
		case PWM_CH2:
			nRet = htim3.Instance->CCR2;
		break;
		case PWM_CH3:
			nRet = htim3.Instance->CCR3;
		break;
		case PWM_CH4:
			nRet = htim2.Instance->CCR4;
		break;
		case PWM_CH5:
			nRet = htim2.Instance->CCR3;
		break;
		case PWM_CH6:
			nRet = htim2.Instance->CCR2;
		break;
		case PWM_CH7:
			nRet = htim3.Instance->CCR1;
		break;
	}
	return nRet;
}

double absf(double val)
{
	return (val >= 0)?val:(-1.0f)*val;
}
void LedControlProcTest()
{
	g_tmr_1s++; 
	if(g_Brightness[0] > TRANSIT_BRIGHT2)
			g_tmr_1s %= 15;
	else
		g_tmr_1s %= 13;
	
	if(g_tmr_1s == 0)
	{
		for(int i= 0; i < PWM_CNT; i++)
		{
			g_Brightness[i]--; 
			//SetBrightness(i, g_Brightness[i]);
			SetRealBrightness(i, g_Brightness[i]);
		}
	}
}
void LedControlProc() //2ms
{
//	if(g_status == STATE_FADEOFF && g_Brightness[0] - TRANSIT_BRIGHT1 < 0 && g_Brightness[0] >= 0)
//	{
//		LedControlProcTest();
//	}else	
	{
		for(int i = 0; i < PWM_CNT; i++)
		{
			if(g_FadePeriod[i] > 0 )
			{
				g_FadePeriod[i]--;
//				if(g_FadePeriod[i] > 0)
//					g_Brightness[i] = 1.0f*g_SetBrightness[i];
//				else
				if(1.0f*g_Brightness[i] + g_fadeStep[i] > 0)
					g_Brightness[i] = 1.0f*g_Brightness[i] + g_fadeStep[i];
				else
					g_Brightness[i] = 0;
				//SetBrightness(i, rint(g_Brightness[i]));
				SetRealBrightness(i, g_Brightness[i]);
				
				if ((g_Brightness[0] < 240 ) &&(g_status == STATE_FADEOFF)){
					keyState = KEYSTATE_FADEOFF_NEXT;
					keyProc();
				}else if ((g_status == STATE_FADEON) &&(g_Brightness[0] < MAX_BRIGHT_FULL)){
					keyState = KEYSTATE_FIRST;
				}
			}else{
				if (g_status == STATE_FADEON_DEMO){					
					keyState = KEYSTATE_FADEOFF_DEMO;
					keyProc();
				}else if (g_status == STATE_FADEOFF_DEMO){
					keyState = KEYSTATE_FADEON_DEMO;
					keyProc();
				}
			}
		}	
	}
}

double getFadeStep(int fromBright, int toBright, int period_ms)
{
	double ret = 0.0f;
		ret = 1.0f*(toBright - fromBright)/period_ms;
	return ret;
}

void LedOn_FadeOnTime(int pwmChannel, int brightness, int fadePeriod_ms)
{
	if(pwmChannel >= PWM_CNT) return;
	g_SetBrightness[pwmChannel] = brightness;
	int curBright = GetBrightness(pwmChannel);
	g_fadeStep[pwmChannel] = getFadeStep(curBright, brightness, fadePeriod_ms);
	g_FadePeriod[pwmChannel] = fadePeriod_ms;
}
void LedOn_FadeOnTime2(int pwmChannel, int fadePeriod_ms)
{
	if(pwmChannel >= PWM_CNT) return;
	g_SetBrightness[pwmChannel] = 0;
	int curBright = GetBrightness(pwmChannel);
	g_fadeStep[pwmChannel] = 1;
	g_FadePeriod[pwmChannel] = 15;
}
void LedOn_FadeOnTime3(int pwmChannel, int fadePeriod_ms)
{
	if(pwmChannel >= PWM_CNT) return;
	g_SetBrightness[pwmChannel] = 0;
	int curBright = GetBrightness(pwmChannel);
	g_fadeStep[pwmChannel] = 1;
	g_FadePeriod[pwmChannel] = 10;
}

void LedOn_FadeOnTime60(int pwmChannel, int fadePeriod_ms)
{
	if(pwmChannel >= PWM_CNT) return;
	g_SetBrightness[pwmChannel] = MAX_BRIGHT;
	int curBright = 0;
	g_fadeStep[pwmChannel] = getFadeStep(curBright, MAX_BRIGHT, fadePeriod_ms);
	g_FadePeriod[pwmChannel] = fadePeriod_ms;
}

void LedOn_FadeOffTime60(int pwmChannel, int fadePeriod_ms)
{
	if(pwmChannel >= PWM_CNT) return;
	g_SetBrightness[pwmChannel] = 0;
	int curBright = MAX_BRIGHT;
	g_fadeStep[pwmChannel] = getFadeStep(curBright, 0, fadePeriod_ms);
	g_FadePeriod[pwmChannel] = fadePeriod_ms;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t isBootOn = 0;
	uint8_t tempState = 0;
	int i;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	 MX_RTC_Init();
	 MX_TIM1_Init();
	 MX_TIM2_Init();
  MX_TIM3_Init();
  
  
 
  /* USER CODE BEGIN 2 */
	uint32_t level = 0;
	level = Flash_Read(0);
	
	if (level != 0){
		if (level > 100) level = 100;
		if (level == 100) keyState = KEYSTATE_FIRST;
		if (level == 60)  keyState = KEYSTATE_SECOND;
		if (level == 35)  keyState = KEYSTATE_THIRD;
		if (level == 15)  keyState = KEYSTATE_FOURTH;		
		
		for(i = 0; i < PWM_CNT; i++)
		{
				double brightness = ceil(level*MAX_BRIGHT/100);
				g_SetBrightness[i] = brightness;
				g_Brightness[i] = brightness;
				//SetBrightness(i, brightness);
			  SetRealBrightness(i, brightness);
		}
		isBootOn = 1;
	}else{
		keyState = 0;
		setBrightnessAll(0);		
		for(i = 0; i < PWM_CNT; i++)
		{
			double brightness = 0;
				g_SetBrightness[i] = brightness;
				g_Brightness[i] = brightness;	
		}		
		isBootOn = 0;		
	}
//	HAL_TIM_Base_Start_IT(&htim1);
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	for(int i = 0; i < PWM_CNT; i++)
	{
		g_FadePeriod[i] = 0;
		g_fadeStep[i] = 0.0f;
	}
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
			int curBright;
			keyCheck();
		
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
			if (g_status != STATE_FADE_HOLD)				LedControlProc();			
			char key = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);		
		
			if (!key){
				release_count = 0;
			}else{
				release_count ++;
				if (release_count > 250) release_count = 250;
			}
			if (release_count < 250){				
				keyTempCnt ++;
				if (keyTempCnt == 1250){
						curBright					 = GetBrightness(0);	
						if (curBright != 0){
							setBrightnessAll(MAX_BRIGHT);
							HAL_Delay(10);
							setBrightnessAll(0);						
							HAL_Delay(10);
							setBrightnessAll(curBright);												
						}
						tempState = 1;
				}else if (keyTempCnt == 1750){
						curBright					 = GetBrightness(0);						
						setBrightnessAll(MAX_BRIGHT);
						HAL_Delay(10);
						setBrightnessAll(0);						
						HAL_Delay(10);
						setBrightnessAll(curBright);
						tempState = 2;
				}else if (keyTempCnt == 3250){
						curBright					 = GetBrightness(0);						
						setBrightnessAll(MAX_BRIGHT);
						HAL_Delay(150);
						setBrightnessAll(0);						
						HAL_Delay(150);
						setBrightnessAll(MAX_BRIGHT);
						HAL_Delay(150);
						setBrightnessAll(0);						
						HAL_Delay(150);
						setBrightnessAll(MAX_BRIGHT);
						HAL_Delay(150);
						setBrightnessAll(0);						
						HAL_Delay(150);					
						setBrightnessAll(curBright);
						tempState = 3;
						if (!isBootOn){
							Flash_Write_Start(0,g_level);
							isBootOn = 1;
						}else{
							Flash_Write_Start(0,0);
							isBootOn = 0;
						}
				}
			}else{				
				if (tempState == 1){
						if(g_Brightness[0] < MIN_BRIGHT)						keyState = KEYSTATE_FADEON; 
						else						keyState = KEYSTATE_FADEOFF;
						keyProc(); 
				}else if (tempState == 2){
						keyState = KEYSTATE_FADEON_DEMO;							
						keyProc();  
				}else if (tempState == 3){					
				}else if(keyTempCnt > 10){
							if ((g_status == STATE_FADEON_DEMO) ||(g_status == STATE_FADEOFF_DEMO))
							{
								g_status = STATE_FADE_HOLD;
							}else{
								keyState++;
								keyState %= KEYSTATE_FADEON;
							//	keyState = KEYSTATE_FADEOFF;
								keyProc();
							}
				}
				tempState = 0;
				keyTempCnt = 0;
			}
			
			
	}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	

  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 40;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 60000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	
  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
