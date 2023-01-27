/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h" //potreban za sscanf, itoa, printf funkcije
#include "ssd1306.h"
#include "fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _DEBUG //ispis na serial 2 prema potrebi
#define NL_Char '\n' //potrebno za prepoznati kraj poruke
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t GPSbuffer[82] = {0}; //za spemanje podataka s GPSa u buffer
uint8_t buffer_index = 0;
uint8_t setspeed = 50, speedkmh = 0; //zadana brzina i brzina
uint32_t current = 0, previous = 0; //za koristenje internog brojaca
uint8_t BzCnt, Buzz = 0; //zujalica
char strSetSpeed[4], strSpeed[4]; //stringovi za ispis brzine i limita brzine
uint8_t	newdata = 0; //provjera jesu li stigli novi podatci na UART1
uint8_t br = 255; //parametar za osvjetljenje
uint8_t hud = 0;
uint8_t turned = 0;
uint8_t HUDon[2] = {0x00, 0xC8};
uint8_t HUDoff[2] = {0x00, 0xC0};

//Varijable za spremanje podataka iscitanih s GPSa
char time[11];
char status;
float latitude;
char lat_dir;
float longitude;
char lon_dir;
float speedknts;
float course;
char date[7];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
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
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_I2C3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &GPSbuffer[buffer_index], 1); //pokrece UART interrupt, potrebno staviti prije inicijalizacije ekrana inace ne radi
  ssd1306_Init(&hi2c3);	//inicijalizacija OLED ekrana na I2C3
  HAL_TIM_Base_Start_IT(&htim16); //pokretanje timera 16 sa interrupt funkcijom za zujalicu
  previous = HAL_GetTick(); //za delay bez zaustavljanja procesora (umjesto interrupta)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  void setbrightness(uint8_t setbr){
  	  uint8_t contrast[3] = {0x00, 0x81, setbr};
  	  HAL_I2C_Master_Transmit(&hi2c3, SSD1306_I2C_ADDR, contrast, 3, 20); //preko I2C na mem. polje 0x00 saljemo 2 bita za osvjetljenje
  }

   while (1)
  {
	   if(newdata == 1){ //primili smo poruku koja zavrsava na /n
		   sscanf(&GPSbuffer, "$GNRMC,%10s,%c,%f,%c,%f,%c,%f,%f,%6s",
				   time, &status, &latitude, &lat_dir, &longitude, &lon_dir, &speedknts, &course, date); //pretvara string u varijable
		   speedkmh = speedknts*1.852; //cvorovi u km/h i float->int
		   HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		   if(hud == 0){ //ako ekran nije zrcaljen
			   HAL_ADC_Start(&hadc1);
			   HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			   br = HAL_ADC_GetValue(&hadc1); //iscitaj osvjetljenje
			   if(br < 10){
				   br=10;
			   }
		   	   setbrightness(br); // podesi jacinu svjetla
		   }
		   newdata = 0; //primili smo podatke, ne ocitavaj vise do novih podataka
		#ifdef _DEBUG //ispis podataka s GPSa po potrebi
		   printf("Time: %s  ", time);
		   printf("Status: %c  ", status);
		   printf("Latitude: %f %c  ", latitude, lat_dir);
		   printf("Longitude: %f %c  ", longitude, lon_dir);
		   printf("Speed: %i  ", speedkmh);
		   printf("Date: %s ", date);
		   printf("Light: %d ", br);
		   printf("SetSpeed: %i\r\n", setspeed);
		   printf("%s \r\n", GPSbuffer);
		#endif
	   }

	   itoa(setspeed, strSetSpeed, 10); // int -> char - zbog ispisa na OLED
	   if(hud == 0 && turned == 1){
		   HAL_I2C_Master_Transmit(&hi2c3, SSD1306_I2C_ADDR, HUDon, 2, 10); //preko I2C saljemo 0xC0 poruku za normalan ekran
	   }else if(hud == 1 && turned == 1){
		   HAL_I2C_Master_Transmit(&hi2c3, SSD1306_I2C_ADDR, HUDoff, 2, 10); //preko I2C saljemo 0xC8 poruku za normalan ekran
		   setbrightness(255);
	   }
	   turned = 0;


	   if(status == 'A'){ //ako primamo ispravne poruke i GPS signal
		   if(speedkmh > setspeed+5 && setspeed != 0){ //ako je brzina za 5 veca od dozvoljene aktiviraj zujalicu (3 beepa)
			   Buzz = 1;
		   }
		   if(speedkmh < setspeed-5){ //ako brzina padne ispod  dozvoljenje - 5 resetiraj zujalicu i dozvoli ponovno bipanje
			   BzCnt = 0;
			   Buzz = 0;
		   }
		  //Ispis na ekran
		   ssd1306_Fill(Black);
		   ssd1306_SetCursor(0, 5);
		   ssd1306_WriteString("Brzina:", Font_11x18, White);
		   itoa(speedkmh, strSpeed, 10); //int -> string
		   ssd1306_WriteString(strSpeed, Font_16x26, White);

		   ssd1306_SetCursor(0, 36);
		   ssd1306_WriteString("Limit: ", Font_11x18, White);
		   ssd1306_WriteString(strSetSpeed, Font_16x26, White);
		   ssd1306_UpdateScreen(&hi2c3);

	   }else{ //ako nema GPS signala

		   current = HAL_GetTick(); //trenutno vrijeme
		   speedknts = 0;
		   Buzz = 0; //iskljucenje zujalice za svaki slucaj
		   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0); //u slucaju da izgubimo GPS signal u trenutku zvucne signalizacije da ne ostane ukljucena
		   if (current - previous < 800) { //interni brojac, koristimo umjesto timera i delaya, ako nije prošlo 800ms prikaži "Cekam satelite"
			   ssd1306_SetCursor(35, 0);
			   ssd1306_WriteString("Cekam", Font_11x18, White);
			   ssd1306_SetCursor(32, 20);
			   ssd1306_WriteString("signal", Font_11x18, White);
			   ssd1306_SetCursor(5, 45);
			   ssd1306_WriteString("Limit:", Font_11x18, White);
			   ssd1306_WriteString(strSetSpeed, Font_11x18, White);
			   ssd1306_UpdateScreen(&hi2c3);
		   }else if(current - previous < 1000){ //između 800 i 1000 ms
			   ssd1306_Fill(Black);
			   ssd1306_SetCursor(5, 45);
			   ssd1306_WriteString("Limit:", Font_11x18, White);
			   ssd1306_WriteString(strSetSpeed, Font_11x18, White);
			   ssd1306_UpdateScreen(&hi2c3);
		   }else{
			   previous = HAL_GetTick(); //ako je prošlo 1000 ms, updejtaj previous na trenutno stanje internog brojaca
		   }
	   }
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//UART interrupt funkcija
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //primljen je novi znak
{
	if (GPSbuffer[buffer_index] == NL_Char){ //ako je primljen new line simbol
		newdata = 1; //primili smo cijelu re�?enicu
		buffer_index = 0; //resetiraj poziciju indexa
	}else{ // digni index za 1 za iduci znak
		buffer_index++;
	}
	HAL_UART_Receive_IT(&huart1, &GPSbuffer[buffer_index], 1); //pononvo pokreni interrupt
}

//interrupt funkcija kod pritiska tipke
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if ((GPIO_Pin == GPIO_PIN_0 || GPIO_Pin == GPIO_PIN_1)){ //ako je okinut jedan od interrupta
		HAL_TIM_Base_Start_IT(&htim6); //pokrece timer 55ms
	}

}

//Interrupt funkcija za timer koja se pozove kada timer odbroji
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim16 ){ //provjera koji timer je okinuo -> za zujalicu, 3 beepa
		if(BzCnt < 6 && Buzz == 1){
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
			BzCnt++;
		}
		if (BzCnt > 5 || Buzz == 0){
			Buzz = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0); //u slucaju da zbog necega izade iz gornje petlje dok je zujalica aktivna
		}
	}
	if (htim == &htim6 ){ //provjera koji timer je okinuo -> timer 6, 55ms za debouncing
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1){ //stisnuta samo tipka 1
			setspeed = setspeed + 5; //povecaj limit za 5
			if(setspeed > 250){setspeed = 250;} //ako je brzina veca, limitiraj na 250
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 1){ //stisnuta samo tipka 2
			if(setspeed < 5){
				setspeed = 0; //ako je brzina manja od 5, vrati na 0
			}else{
				setspeed = setspeed - 5;
			}
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0){ //stisnute obje tipke
			if(hud == 0){
				hud = 1;
			}else if(hud == 1){
				hud = 0;
			}
			turned = 1;
		}
		HAL_TIM_Base_Stop_IT(&htim6); //zaustavi timer
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
