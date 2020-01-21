/**
 ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  * Author             : Ashish Kumar Verma 
  * Date               : 21-01-2020
  * Contact            : Ashish2614@gmail.com
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
#include "math.h"

/* USER CODE BEGIN Includes */

#define BMP180_ADDR 0xEE                    // 7-bit address
#define BMP180_REG_CONTROL 0xF4
#define BMP180_REG_RESULT 0xF6
#define BMP180_COMMAND_TEMPERATURE 0x2E
#define BMP180_COMMAND_PRESSURE 0x34       //0x34 0x74 0xB4 0xF4
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
char begin(void);
char readCompData(char address);
double calculatePressure(double up,double T);
double calculateTemperature(double ut);
char measureParameters(double *P, double *T);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint16_t unSignedIntTempVar,AC4,AC5,AC6;
int16_t signedIntTempVar,AC1,AC2,AC3,VB1,VB2,MB,MC,MD;
char buf[20];
double c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2;

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	HAL_UART_Transmit(&huart1,(uint8_t*)"REBOOT\r\n",sizeof("REBOOT\r\n"),100);
	if (begin())
			HAL_UART_Transmit(&huart1,(uint8_t*)"BMP180 init success\r\n",sizeof("BMP180 init success\r\n"),100);
  else{
		HAL_UART_Transmit(&huart1,(uint8_t*)"BMP180 init fail\r\n\n",sizeof("BMP180 init fail\r\n\n"),100);
    while(1);
  }
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    double T,P,p0,A;

    if (measureParameters(&P, &T) != 0){

				HAL_UART_Transmit(&huart1,(uint8_t*)"temperature: ",sizeof("temperature: "),100);
			  sprintf(buf,"%0.2lf",T);
        HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
			  HAL_UART_Transmit(&huart1,(uint8_t*)" deg C, ",sizeof(" deg C, "),100);
        sprintf(buf,"%0.2lf",(9.0/5.0)*T+32.0);
				HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
			  HAL_UART_Transmit(&huart1,(uint8_t*)" deg F\r\n",sizeof(" deg F\r\n"),100);
      
				HAL_UART_Transmit(&huart1,(uint8_t*)"absolute pressure: ",sizeof("absolute pressure: "),100);
			  sprintf(buf,"%0.2lf",P);
        HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
			  HAL_UART_Transmit(&huart1,(uint8_t*)" mb, ",sizeof(" mb, "),100);
        sprintf(buf,"%0.2lf",P*0.0295333727);
				HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
			  HAL_UART_Transmit(&huart1,(uint8_t*)" inHg\r\n",sizeof(" inHg\r\n"),100);
			
        A = 44330.0*(1-pow(P/1013.25,1/5.255));
			
				HAL_UART_Transmit(&huart1,(uint8_t*)"computed altitude: ",sizeof("computed altitude: "),100);
			  sprintf(buf,"%0.2lf",A);
        HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
			  HAL_UART_Transmit(&huart1,(uint8_t*)" meters, ",sizeof(" meters, "),100);
        sprintf(buf,"%0.2lf",A*3.28084);
				HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
			  HAL_UART_Transmit(&huart1,(uint8_t*)" feet\r\n",sizeof(" feet\r\n"),100);
			  HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n\n",sizeof("\r\n\n"),100);
    }
    else
				HAL_UART_Transmit(&huart1,(uint8_t*)"error retrieving pressure measurement\n",sizeof("error retrieving pressure measurement\n"),100);
		    HAL_Delay(5000);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
char begin(){
  double c3,c4,b1;

  if(readCompData(0xAA))
    AC1=signedIntTempVar;
		HAL_UART_Transmit(&huart1,(uint8_t*)"AC1: ",sizeof("AC1: "),100);
		sprintf(buf,"%d\r\n",AC1);
		HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
		
	if(readCompData(0xAC))
    AC2=signedIntTempVar;
		HAL_UART_Transmit(&huart1,(uint8_t*)"AC2: ",sizeof("AC2: "),100);
		sprintf(buf,"%d\r\n",AC2);
		HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
		
	if(readCompData(0xAE))
    AC3=signedIntTempVar;
		HAL_UART_Transmit(&huart1,(uint8_t*)"AC3: ",sizeof("AC3: "),100);
		sprintf(buf,"%d\r\n",AC3);
		HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
		
	if(readCompData(0xB0))
    AC4=unSignedIntTempVar;
		HAL_UART_Transmit(&huart1,(uint8_t*)"AC4: ",sizeof("AC4: "),100);
		sprintf(buf,"%d\r\n",AC4);
		HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
		
  if(readCompData(0xB2))
    AC5=unSignedIntTempVar;
		HAL_UART_Transmit(&huart1,(uint8_t*)"AC5: ",sizeof("AC5: "),100);
		sprintf(buf,"%d\r\n",AC5);
		HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
		
  if(readCompData(0xB4))
    AC6=unSignedIntTempVar;
		HAL_UART_Transmit(&huart1,(uint8_t*)"AC6: ",sizeof("AC6: "),100);
		sprintf(buf,"%d\r\n",AC6);
		HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
		
  if(readCompData(0xB6))
    VB1=signedIntTempVar;
		HAL_UART_Transmit(&huart1,(uint8_t*)"VB1: ",sizeof("VB1: "),100);
		sprintf(buf,"%d\r\n",VB1);
		HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
		
  if(readCompData(0xB8))
    VB2=signedIntTempVar;
		HAL_UART_Transmit(&huart1,(uint8_t*)"VB2: ",sizeof("VB2: "),100);
		sprintf(buf,"%d\r\n",VB2);
		HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
		
  if(readCompData(0xBA))
    MB=signedIntTempVar;
		HAL_UART_Transmit(&huart1,(uint8_t*)"MB: ",sizeof("MB: "),100);
		sprintf(buf,"%d\r\n",MB);
		HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
		
  if(readCompData(0xBC))
    MC=signedIntTempVar;
		HAL_UART_Transmit(&huart1,(uint8_t*)"MC: ",sizeof("MC: "),100);
		sprintf(buf,"%d\r\n",MC);
		HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
		
  if(readCompData(0xBE))
    MD=signedIntTempVar;
		HAL_UART_Transmit(&huart1,(uint8_t*)"MD: ",sizeof("MD: "),100);
		sprintf(buf,"%d\r\n",MD);
		HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
		
		c3 = 160.0 * pow(2,-15) * AC3;
		c4 = pow(10,-3) * pow(2,-15) * AC4;
		b1 = pow(160,2) * pow(2,-30) * VB1;
		c5 = (pow(2,-15) / 160) * AC5;
		c6 = AC6;
		mc = (pow(2,11) / pow(160,2)) * MC;
		md = MD / 160.0;
		x0 = AC1;
		x1 = 160.0 * pow(2,-13) * AC2;
		x2 = pow(160,2) * pow(2,-25) * VB2;
		y0 = c4 * pow(2,15);
		y1 = c4 * c3;
		y2 = c4 * b1;
		p0 = (3791.0 - 8.0) / 1600.0;
		p1 = 1.0 - 7357.0 * pow(2,-20);
		p2 = 3038.0 * 100.0 * pow(2,-36);
		
  return(1);
}
char readCompData(char address){
  unsigned char data[2];
	if(HAL_I2C_IsDeviceReady(&hi2c1,BMP180_ADDR,5,100))
		{
	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
		 HAL_Delay(100);
    }
  if((HAL_I2C_Master_Transmit(&hi2c1,BMP180_ADDR,(uint8_t*)&address,1,100))== HAL_OK)
	{
			HAL_I2C_Master_Receive(&hi2c1,BMP180_ADDR,(uint8_t*)&data,2,100);
			signedIntTempVar = (int16_t)((data[0]<<8)|data[1]);
			unSignedIntTempVar = (((uint16_t)data[0]<<8)|(uint16_t)data[1]);
			return(1);
	}
	signedIntTempVar= unSignedIntTempVar= 0;
  return(0);
  }


char writeBytes(unsigned char *values, char length){

	if((HAL_I2C_Master_Transmit(&hi2c1,BMP180_ADDR,values,length,100))== HAL_OK)
    return(1);
  else
    return(0);
} 


char measureParameters(double *P, double *T){
  unsigned char data[3],delay1;
  double up,ut;
	uint8_t cmd;
	
	if(HAL_I2C_IsDeviceReady(&hi2c1,BMP180_ADDR,5,100)){
			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
			 HAL_Delay(50);
			}
		
			data[0] = BMP180_REG_CONTROL;
			data[1] = BMP180_COMMAND_TEMPERATURE;
			writeBytes(data, 2);
			HAL_Delay(6);
      cmd = BMP180_REG_RESULT;
	if((HAL_I2C_Master_Transmit(&hi2c1,BMP180_ADDR,(uint8_t*)&cmd,1,100)) == HAL_OK)
{
			HAL_I2C_Master_Receive(&hi2c1,BMP180_ADDR,(uint8_t*)data,2,100);
	    ut = (data[0] * 256.0) + data[1];
	
			HAL_UART_Transmit(&huart1,(uint8_t*)"ut: ",sizeof("ut: "),100);
			sprintf(buf,"%lf\r\n",ut);
			HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
	
			*T= calculateTemperature(ut);
			HAL_UART_Transmit(&huart1,(uint8_t*)"T: ",sizeof("T: "),100);
			sprintf(buf,"%lf\r\n",*T);
			HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
			
}
 else
	 return(0);
 
			data[0] = BMP180_REG_CONTROL;
			data[1] = BMP180_COMMAND_PRESSURE;
			delay1 = 27;                              //5,8,14,26
			writeBytes(data, 2);
			HAL_Delay(delay1);
      cmd =	BMP180_REG_RESULT;		
	if((HAL_I2C_Master_Transmit(&hi2c1,BMP180_ADDR,(uint8_t*)&cmd,1,100))==HAL_OK)
{
			HAL_I2C_Master_Receive(&hi2c1,BMP180_ADDR,(uint8_t*)data,3,100);
			up = (data[0] * 256.0) + data[1] + (data[2]/256.0);
			HAL_UART_Transmit(&huart1,(uint8_t*)"up: ",sizeof("up: "),100);
			sprintf(buf,"%lf\r\n",up);
			HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
			
			*P = calculatePressure(up,*T);
			
			HAL_UART_Transmit(&huart1,(uint8_t*)"P: ",sizeof("P: "),100);
			sprintf(buf,"%lf\r\n",*P);
			HAL_UART_Transmit(&huart1,(uint8_t*)buf,sizeof(buf),100);
			
			return(1);
}
  
  return(0);
}

double calculateTemperature(double ut){
  double T,a;
  a = c5 * (ut - c6);
  T = a + (mc / (a + md));

  return T;
}
double calculatePressure(double up,double T){
  double P;
  double s,x,y,z;

  s = T - 25.0;
  x = (x2 * pow(s,2)) + (x1 * s) + x0;
  y = (y2 * pow(s,2)) + (y1 * s) + y0;
  z = (up - x) / y;
  P = (p2 * pow(z,2)) + (p1 * z) + p0;

  return P;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

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

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
