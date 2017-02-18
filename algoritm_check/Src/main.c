/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include <cmath> 
#include <stdlib.h>
#include <stdint.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void bubbleSort(uint16_t A[],uint16_t B[], uint32_t n);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
const uint16_t chislo_piks=513;
	uint16_t w,j=0;
	uint64_t k,n;
	int64_t temp1[chislo_piks],temp2[chislo_piks]=0;
	uint16_t center_toc[chislo_piks],center_toc_drob[chislo_piks],center_toc_sort[chislo_piks],center_toc_sort_drob[chislo_piks],center_itog,center_itog1;
	float  center[chislo_piks];
	float bufert[chislo_piks]=
{0,-0.01074,-0.00302,-0.00170,0.01020,0.00514,-0.00032,0.00737,-0.00583,-0.00146,0.01143,0.02582,0.06522,0.15473,0.26374,0.33576,0.28628,0.18362,
0.08062,0.03088,0.00296,0.00441,0.00455,-0.00504,0.00088,0.00898,0.00063,-0.00173,0.00262,-0.00105,0.00598,0.01661,0.06101,0.15456,0.27469,
0.33411,0.30021,0.16899,0.07651,0.02580,0.00066,-0.00635,0.00356,0.00345,0.00163,0.00222,-0.00490,-0.00280,-0.00055,-0.00337,0.00535,0.03067,
0.05840,0.15561,0.26122,0.33911,0.28549,0.18157,0.07556,0.02735,0.00542,0.00058,0.01667,-0.00354,-0.00026,0.01509,-0.00645,0.00311,-0.00600,
0.00629,0.00533,0.02103,0.05955,0.15335,0.27531,0.31878,0.28175,0.18273,0.07536,0.02281,-0.00306,-0.00426,-0.00218,-0.00407,-0.00487,-0.00237,
-0.00530,0.00755,-0.00551,0.01072,0.00138,0.00920,0.05764,0.15722,0.27297,0.33247,0.29255,0.18395,0.07083,0.02120,-0.00494,-0.00170,0.00543,
0.00404,0.01287,0.00094,-0.01213,0.00950,0.00726,-0.00278,0.00474,0.02175,0.06328,0.15588,0.26640,0.33136,0.29143,0.17595,0.08704,0.02451,
0.00921,-0.00398,-0.00596,-0.00050,-0.00142,0.00672,0.00341,0.00304,-0.00808,-0.00495,0.00462,0.01541,0.06837,0.15951,0.26377,0.33649,0.29878,
0.18220,0.08817,0.03045,0.00656,0.00576,0.00118,-0.00233,0.00202,0.00020,-0.00205,0.00996,0.00129,0.01585,0.00182,0.02054,0.07001,0.16732,
0.26874,0.32713,0.29545,0.17297,0.07750,0.02600,0.01373,0.00891,-0.00006,0.00520,-0.00181,0.00372,0.01077,0.00089,-0.00164,-0.00232,0.00752,
0.02489,0.07307,0.15227,0.26795,0.32802,0.30000,0.17997,0.07638,0.03676,0.00991,0.00486,-0.00556,0.00245,0.00293,-0.00614,-0.00127,-0.00227,
0.00302,-0.00517,0.00961,0.02740,0.05941,0.15192,0.26988,0.33834,0.29387,0.18263,0.07632,0.02317,0.00880,0.01029,0.00329,-0.00678,0.00244,
-0.00052,-0.00137,-0.00123,-0.00490,-0.00131,-0.00092,0.01532,0.06177,0.15295,0.26582,0.33072,0.29113,0.18297,0.08625,0.02796,0.00374,0.00419,
-0.00129,-0.01001,-0.01320,-0.00964,-0.00133,-0.00348,-0.00400,0.00072,-0.00022,0.01382,0.06454,0.15637,0.28073,0.33327,0.29126,0.17716,0.07794,
0.02419,-0.00452,-0.00062,0.00431,-0.00320,0.00278,0.00558,0.00606,0.00437,-0.01266,-0.00268,0.00856,0.01959,0.06114,0.15443,0.27339,0.33244,
0.28667,0.17938,0.07787,0.02952,0.00767,0.00391,0.00092,-0.00203,0.00034,0.00122,-0.00786,0.00101,0.00527,-0.00012,0.01233,0.02052,0.05875,
0.15110,0.26549,0.33293,0.29518,0.17875,0.08162,0.03434,0.01059,0.00547,-0.00449,-0.00300,0.00136,-0.00057,-0.00914,-0.00851,0.00582,-0.00377,
0.01518,0.00487,0.06364,0.15197,0.26278,0.33385,0.28647,0.18523,0.07942,0.01525,-0.00828,-0.00077,0.00653,0.00103,0.00031,0.00387,-0.00784,
0.00802,-0.00498,0.00071,0.00144,0.01821,0.05018,0.16054,0.27010,0.34288,0.29890,0.18046,0.07376,0.01469,0.00545,0.00206,0.00597,-0.00534,
0.00447,0.00040,-0.00435,0.00436,0.00154,-0.00083,0.01404,0.02079,0.06049,0.15353,0.26752,0.33255,0.29349,0.17700,0.07456,0.02650,0.01595,
-0.00090,-0.00560,-0.00133,-0.00272,-0.00184,0.00267,-0.00073,-0.00329,-0.00200,-0.00395,0.01253,0.05317,0.14907,0.26093,0.32760,0.28924,0.17364,
0.07893,0.02639,-0.00006,0.00709,-0.00883,-0.00220,-0.00835,-0.00194,-0.00744,-0.00323,-0.00412,-0.00617,-0.00812,0.02055,0.07236,0.16447,0.26426,
0.33131,0.28832,0.18082,0.07387,0.01368,0.00515,-0.00606,-0.01061,0.01336,-0.00131,0.00227,0.00671,-0.00947,0.00237,0.00332,0.00395,0.01825,
0.05815,0.15603,0.26225,0.32709,0.29502,0.18988,0.07687,0.02163,0.00765,0.00686,0.00553,0.00153,-0.00357,0.00354,-0.00045,0.00775,-0.00787,
0.00129,0.01074,0.01458,0.05760,0.14672,0.26547,0.33045,0.29171,0.17543,0.07950,0.02108,0.00328,-0.00389,-0.00150,0.02008,0.01916,0.00645,
0.00443,-0.00718,-0.00324,-0.00299,0.00650,0.02008,0.06172,0.15005,0.26183,0.33160,0.29734,0.18280,0.08006,0.02817,0.00864,-0.00826,0.00020,
-0.00239,-0.01144,-0.00747,-0.00180,0.00465,-0.00124,0.00197,-0.00156,0.00524,0.06164,0.14662,0.27363,0.34133,0.28885,0.18710,0.08440,0.01651,
0.00517,0.00230,0.00303,0.00381,0.00379,0.00381,-0.00176,0.00214,-0.01262,-0.00608,0.01025,0.01365,0.06296,0.16045,0.26203,0.32618,0.28817,
0.18287,0.07789,0.01897,-0.00349,-0.00125,-0.00115,-0.00300,-0.00147,-0.00616,0.00309,-0.01223,0.00086,0.00426,-0.00129,0.01705,0.06276,0.14851,
0.26696,0.33389,0.28567,0.18509,0.08519,0.02628,0.00544,-0.00572,0.00287,-0.00683,0.00895,0.00296,0.00149,-0.00221,-0.01224,-0.00672,0.00205,
0.01357,0.00416,
};


		uint64_t buffer_2[chislo_piks];
uint32_t buffer_3[chislo_piks];
int32_t  fpg6[chislo_piks],fpg7[chislo_piks],fpg8[chislo_piks];
uint32_t tt,mm,hh,ll=0;

	
/* USER CODE END 0 */
 uint16_t chislo_piks_var=  chislo_piks;
int64_t fpg,fpg1,fpg2,fpg3,fpg4=0;
	div_t fpg5;
float fpg_2=0;
int flagg=1;
uint32_t count=1;
uint32_t count_koord=0;
uint32_t poloshene_pikov[50];
uint32_t poloshene_pikov_drob[50];
int main(void)
{
bufert[5]=bufert[5]+0;

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */

	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
fpg=1053;
fpg1=4;
fpg2=236;


		for(j=0;j<chislo_piks_var;j++)
		{
			if(bufert[j]<0)
				bufert[j]=0;
	buffer_2[j]=round(bufert[j]*1000000);
		}	
	
	
  while (1)
  {
		
//fpg3=fpg/4;
fpg2=(fpg%236)*10000/236;
	
	
		
		
						tt=HAL_RCC_GetHCLKFreq();
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
		
//	HAL_GPIO_WritePin( GPIOA,  GPIO_PIN_4, GPIO_PIN_SET);	
		
				for(j=0;j<chislo_piks_var;j++)
				{
				buffer_3[j]=buffer_2[j]*j;
				}		
			for(w=1;w<chislo_piks_var-17;w++)
			{
			if((w==1))
			{
			for(j=1;j<18;j++)
			{
			k=k+buffer_3[j];
			n=n+buffer_2[j];	
			}
			temp1[1]=k/n;
			temp2[1]=(k%n)*10000/n;

			}
			 else
				{
			k=k+buffer_3[w+16]-buffer_3[w-1];
			n=n+buffer_2[w+16]-buffer_2[w-1];	
		temp1[w]=k/n;
		temp2[w]=(k%n)*10000/n;		
//				fpg6[w]=temp2[w]-temp2[w-1];
					if(((temp1[w]-temp1[w-1])<2)&&(abs((temp2[w]-temp2[w-1]))<2000))	//0.05
			{			
			center_toc[w]=	(temp1[w]+temp1[w-1])/2;
			center_toc_drob[w]=(temp2[w]+temp2[w-1])/2;
			
			}
			else
			{	
				center_toc[w]=0;
			}	
			center_toc_sort[w]=center_toc[w];		
			center_toc_sort_drob[w]=center_toc_drob[w];	
	
			}
			
	}

n=0;k=0;

	
bubbleSort (center_toc_sort,center_toc_sort_drob,chislo_piks_var);
	poloshene_pikov[1]=0;
	poloshene_pikov_drob[1]=0;
	
	for(int y =2;y<chislo_piks;y++)
	{
		
		if(center_toc_sort[y]>0)
		{
			fpg6[y]=center_toc_sort[y]-center_toc_sort[y+1];
		if(abs(center_toc_sort[y]-center_toc_sort[y+1])<3)
		{
			
		poloshene_pikov[count]=poloshene_pikov[count]+center_toc_sort[y];
			poloshene_pikov_drob[count]=poloshene_pikov_drob[count]+center_toc_sort_drob[y];
			count_koord=count_koord+1;
		}
		else
		{
		poloshene_pikov[count]=poloshene_pikov[count]+center_toc_sort[y];
			poloshene_pikov_drob[count]=poloshene_pikov_drob[count]+center_toc_sort_drob[y];
			
			count_koord=count_koord+1;
			
			poloshene_pikov[count]=poloshene_pikov[count]/count_koord;
			poloshene_pikov_drob[count]=poloshene_pikov_drob[count]/count_koord;
			count=count+1;
			count_koord=0;
		}
		}
	}
	
	
	
	
	
	mm=DWT->CYCCNT;
		hh=tt/mm;
	


  }

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_ActivateOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif


 void bubbleSort(uint16_t A[],uint16_t B[], uint32_t n)
	 {
 int i,found;                                // Количество обменов
      do {                                     // Повторять просмотр...
      found =0;
      for (i=0; i<n-1; i++)
                if (A[i] > A[i+1]) {        // Сравнить соседей
           uint32_t cc = A[i]; A[i]=A[i+1]; A[i+1]=cc;
						uint32_t dd = B[i]; B[i]=B[i+1]; B[i+1]=dd;			
           found++;                        // Переставить соседей
           }
								
								
      } while(found !=0); 
	 }               //...пока есть перестановки



/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
