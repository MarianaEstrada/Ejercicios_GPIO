# Ejercicios GPIO

## Semáforo

Se va realizar un semáforo con las siguientes caracteristicas:

![Semaforo](https://github.com/MarianaEstrada/Ejercicios_GPIO/blob/master/Imagenes/Semaforo.PNG)

A continuación se presenta el código implementado: 
~~~
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  * 
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define LEDDELAY_1    3700000 //10seg
#define LEDDELAY_2    740000  //2seg
#define LEDDELAY_3    5200000  // 14seg
#define LEDDELAY_4    1000000   //3 seg

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int a;
	a=0;
	// Se van a activar los puertos A
		RCC->AHB2ENR = 0x00000001;

		//Se van a declarar cuales son las entradas y las salidas del sistema

		// Para el banco A se configuran PA5, PA6 y PA7
		GPIOA->MODER &= 0xABFFFFFF;		// Se realiza el reset
		GPIOA->MODER &= 0xFFFF57FF;		// Se escribe 01 en los pines 5,6 y 7 ya que son salidas

		//Inicialización de los valores
		// Para el banco A
		GPIOA->ODR |= 0x00E0;			//0x00E0 se pone 1 a los bits 5,6,7


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  // Primero se va a encender los LED ubicado en el pin A5
	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_1){
	  a=a+1;}
	  // Segundo se van a encender los LED’s ubicados en los pines A5,B5,B6
	  a=0;
	  GPIOA ->ODR = 0x0060;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  // Tercero se van a encender los LED’s ubicados en los pines A5,A6,A7,B5,B6
      a=0;
	  GPIOA ->ODR = 0x0080;
	  while(a< LEDDELAY_3){
	  a=a+1;}
	  // Cuarto se van a apagar los LED’s ubicados en los pines A6 y A7.
	  a=0;
	  GPIOA ->ODR = 0x0040;
	  while(a< LEDDELAY_4){
	  a=a+1;}


    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

~~~

## Código morse

Se va a realizar el abcedario en código morse.

![CM](https://github.com/MarianaEstrada/Ejercicios_GPIO/blob/master/Imagenes/CM.PNG)

Donde :

* 1 seg para cambio de símbolo
* 2seg para una línea
* 3 seg para un punto
* 4 seg para cambio de letra

A continuación se presenta el código implementado: 
~~~
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define LEDDELAY_1    370000 //1seg espera simb
#define LEDDELAY_2    740000 // 2seg linea
#define LEDDELAY_3    1000000// 3seg punto
#define LEDDELAY_4    1400000 //4seg espera letra

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int a;
	a=0;
	// Se van a activar los puertos A
		RCC->AHB2ENR = 0x00000001;

		//Se van a declarar cuales son las entradas y las salidas del sistema

		// Para el banco A se configuran PA5
		GPIOA->MODER &= 0xABFFFFFF;		// Se realiza el reset
		GPIOA->MODER &= 0xFFFFF4FF;		// Se escribe 01 en el pin 5
		//Inicialización de los valores
		// Para el banco A
		GPIOA->ODR |= 0x0020;			//0x0020 se pone 1 a el bit 5


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  //Letra A
	   a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra B

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra C

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra D

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra E

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra F

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra G

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra H

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra I

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra J

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra K

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra L

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

		a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra M

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra N

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra O

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra P

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra Q

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra R

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}


	  //Letra S

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra T

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra U

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra V

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra W

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra X

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra Y

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}

	  //Letra Z

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0020;
	  while(a< LEDDELAY_3){
	  a=a+1;}

	  a=0;
	  GPIOA ->ODR = 0x0000;
	  while(a< LEDDELAY_4){
	  a=a+1;}
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

~~~

## Feliz navidad

Se van a realizar las siguientes secuencias de luces:

### Secuencia 1.

![sec1](https://github.com/MarianaEstrada/Ejercicios_GPIO/blob/master/Imagenes/sec1.PNG)

### Secuencia 2.

![sec2](https://github.com/MarianaEstrada/Ejercicios_GPIO/blob/master/Imagenes/sec2.PNG)

### Secuencia 3.

![sec_3_b](https://github.com/MarianaEstrada/Ejercicios_GPIO/blob/master/Imagenes/sec_3_b.PNG)

~~~
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define LEDDELAY_1    500000
#define LEDDELAY_2    1000000

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int a;
	a=0;
	// Se van a activar los puertos A,B,C
		RCC->AHB2ENR = 0x00000007;

		//Se van a declarar cuales son las entradas y las salidas del sistema

		// Para el banco A se configuran PA5, PA6 y PA7
		GPIOA->MODER &= 0xABFFFFFF;		// Se realiza el reset
		GPIOA->MODER &= 0xFFFF57FF;		// Se escribe 01 en los pines 5,6 y 7 ya que son salidas
		//Para el banco B se configuran PB5 y PB6
		GPIOB->MODER &=  0xFFFFFEBF;		// Se realiza el reset
		GPIOB->MODER &= 0xFFFFD7FF;		// Se escribe 01 en los pines 5,6 ya que son salidas
		//Para el banco C se configuran PC7 y PC9
		GPIOC->MODER &=  0xFFFFFFFF;		// Se realiza el reset
		GPIOC->MODER &= 0xFFF77FFF;		// Se escribe 01 en los pines 7,9 ya que son salidas



		//Inicialización de los valores
		// Para el banco A
		GPIOA->ODR |= 0x00E0;			//0x00E0 se pone 1 a los bits 5,6,7

		// Para el banco B
		GPIOB->ODR |= 0x0060;			//0x0060 se pone 1 en los bits 5 y 6

		// Para el banco C
		GPIOC->ODR |= 0x0280;			//0x0280 se pone 1 en los bits 5 y 6


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  //SECUENCIA 1
	  // Primero se va a encender los LED ubicado en el pin A5
	  a=0;
	  GPIOA ->ODR = 0x0020;
	  GPIOB ->ODR = 0x0000;
	  GPIOC->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}
	  // Segundo se van a encender los LED’s ubicados en los pines A5,B5,B6
	  a=0;
	  GPIOA ->ODR = 0x0020;
	  GPIOB ->ODR = 0x0060;
	  GPIOC->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  // Tercero se van a encender los LED’s ubicados en los pines A5,A6,A7,B5,B6
      a=0;
	  GPIOA ->ODR = 0x00E0;
	  GPIOB ->ODR = 0x0060;
	  GPIOC->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  // Cuarto se van a encender los LED’s ubicados en los pines A5,A6,A7,B5,B6,C7,C9
      a=0;
	  GPIOA ->ODR = 0x00E0;
	  GPIOB ->ODR = 0x0060;
	  GPIOC->ODR = 0x0280;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  // Quinto se van a apagar los LED’s ubicados en los pines C7 y C9.
      a=0;
	  GPIOA ->ODR = 0x00E0;
	  GPIOB ->ODR = 0x0060;
	  GPIOC->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  // Cuarto se van a apagar los LED’s ubicados en los pines A6 y A7.
	  a=0;
	  GPIOA ->ODR = 0x0020;
	  GPIOB ->ODR = 0x0060;
	  GPIOC->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}
	  // Quinto se van a apagar los LED’s ubicados en los pines B5,B6
	  a=0;
	  GPIOA ->ODR = 0x0020;
	  GPIOB ->ODR = 0x0000;
	  GPIOC->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}
	  // Sexto se va a apagar el LED ubicado en el pin A5
	  a=0;
	  GPIOA ->ODR = 0x0000;
	  GPIOB ->ODR = 0x0000;
	  GPIOC->ODR = 0x0000;
	  while(a< LEDDELAY_2){
	  a=a+1;}


	 //Secuencia 2


	  // Primero se va a encender los LED ubicado en el pin C9
	  a=0;
	  GPIOA ->ODR = 0x0000;
	  GPIOB ->ODR = 0x0000;
	  GPIOC ->ODR = 0x0200;
	  while(a< LEDDELAY_1){
	  a=a+1;}
	  // Segundo  se va a encender los LED ubicado en el pin A7
	  a=0;
	  GPIOA ->ODR = 0x0080;
	  GPIOB ->ODR = 0x0000;
	  GPIOC ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  // Tercero  se va a encender los LED ubicado en el pin B6
		a=0;
	  GPIOA ->ODR = 0x0000;
	  GPIOB ->ODR = 0x0040;
	  GPIOC ->ODR = 0x0000;
	   while(a< LEDDELAY_1){
	  a=a+1;}
	  // Cuarto  se va a encender los LED ubicado en el pin A5
	  a=0;
	  GPIOA ->ODR = 0x0020;
	  GPIOB ->ODR = 0x0000;
	  GPIOC ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}
	  // Quinto  se va a encender los LED ubicado en el pin B5
	  a=0;
	  GPIOA ->ODR = 0x0000;
	  GPIOB ->ODR = 0x0020;
	  GPIOC ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  // Sexto  se va a encender los LED ubicado en el pin A6
	  a=0;
	  GPIOA ->ODR = 0x0040;
	  GPIOB ->ODR = 0x0000;
	  GPIOC ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	// Séptimo se va a encender los LED ubicado en el pin C7
		a=0;
	  GPIOA ->ODR = 0x0000;
	  GPIOB ->ODR = 0x0000;
	  GPIOC ->ODR = 0x0080;
	   while(a< LEDDELAY_1){
	  a=a+1;}

	  // Octavo se va a encender los LED ubicado en el pin A6
	  a=0;
	  GPIOA ->ODR = 0x0040;
	  GPIOB ->ODR = 0x0000;
	  GPIOC ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  // Noveno  se va a encender los LED ubicado en el pin B5
	  a=0;
	  GPIOA ->ODR = 0x0000;
	  GPIOB ->ODR = 0x0020;
	  GPIOC ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  // Dècimo  se va a encender los LED ubicado en el pin A5
	  a=0;
	  GPIOA ->ODR = 0x0020;
	  GPIOB ->ODR = 0x0000;
	  GPIOC ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  // Once  se va a encender los LED ubicado en el pin B6
		a=0;
	  GPIOA ->ODR = 0x0000;
	  GPIOB ->ODR = 0x0040;
	  GPIOC ->ODR = 0x0000;
	   while(a< LEDDELAY_1){
	  a=a+1;}

	  // Doce  se va a encender los LED ubicado en el pin A7
	  a=0;
	  GPIOA ->ODR = 0x0080;
	  GPIOB ->ODR = 0x0000;
	  GPIOC ->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  // Trece se va a encender los LED ubicado en el pin C9
	  a=0;
	  GPIOA ->ODR = 0x0000;
	  GPIOB ->ODR = 0x0000;
	  GPIOC ->ODR = 0x0200;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  //Tiempo de espera entre secuencias
	  a=0;
	  GPIOA ->ODR = 0x0000;
	  GPIOB ->ODR = 0x0000;
	  GPIOC->ODR = 0x0000;
	  while(a< LEDDELAY_2){
	  a=a+1;}

	  //Secuencia 3

	  // Primero se va a encender los LED ubicado en el pin A5,A6,A7
	  a=0;
	  GPIOA ->ODR = 0x00E0;
	  GPIOB ->ODR = 0x0000;
	  GPIOC->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  // Segundo se van a encender los LED’s ubicados en los pines B5,B6,C7,C9
	  a=0;
	  GPIOA ->ODR = 0x0000;
	  GPIOB ->ODR = 0x0060;
	  GPIOC->ODR = 0x0280;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  // Tercero se va a encender los LED ubicado en el pin A5,A6,A7
	  a=0;
	  GPIOA ->ODR = 0x00E0;
	  GPIOB ->ODR = 0x0000;
	  GPIOC->ODR = 0x0000;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  // Cuarto se van a encender los LED’s ubicados en los pines B5,B6,C7,C9
	  a=0;
	  GPIOA ->ODR = 0x0000;
	  GPIOB ->ODR = 0x0060;
	  GPIOC->ODR = 0x0280;
	  while(a< LEDDELAY_1){
	  a=a+1;}

	  // Tiempo de espera entre secuencias
      a=0;
	  GPIOA ->ODR = 0x0000;
	  GPIOB ->ODR = 0x0000;
	  GPIOC->ODR = 0x0000;
	  while(a< LEDDELAY_2){
	  a=a+1;}

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

~~~
