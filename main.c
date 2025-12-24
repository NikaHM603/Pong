
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "crc.h"
#include "dac.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "lcd.h"
#include "lcd_demo.h"
#include "ugui.h"
#include "lcd_ugui.h"
#include "XPT2046_touch.h"
#include "joystick.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pong_width 320
#define pong_height 240
#define paddle_width 10
#define paddle_height 60
#define zogica_size 5
#define zogica_speed_start 3.0f
#define paddle_speed 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float zogica_x = 160.0f;
float zogica_y = 120.0f;
float zogica_dx = 2.0f;
float zogica_dy = 1.5f;

//sredinska crtkana crta
int razpolovisce = 90;
int st_crt = 5;

//ploscice
int paddle_x_leva = 20;
int paddle_y_leva = 90;
int paddle_x_desna = pong_width - paddle_width - 20;
int paddle_y_desna = 90;


uint8_t score_L = 0;
uint8_t score_D = 0;
uint8_t zogica = 90;

uint8_t nizi_L = 0;
uint8_t nizi_D = 0;

const int max_tock_na_niz = 3; // 4 tocke na niz
const int max_nizov = 4; // 4 nizi
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void zogica_narisi(void);
int zogica_reset(void);
int preveri_zmago(void);
float narisi_score(void);
void povecaj_score(int igralec);
void score_reset(void);
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
	coord_t joystick_raw={0,0}, joystick_new={0,0}, joystick_prev={0,0};
		joystick_t joystick;
		char MSG[100]={0};
		uint16_t touch_x = 0, touch_y = 0;

		char str[50];
		float bitrate;


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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_FMC_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_QUADSPI1_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM20_Init();
  MX_ADC3_Init();
  MX_DAC1_Init();
  MX_DAC2_Init();
  MX_FDCAN2_Init();
  MX_I2C1_Init();
  MX_TIM15_Init();
  MX_USART3_UART_Init();
  MX_USB_Device_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  MX_ADC4_Init();	//bug workaround: https://community.st.com/s/question/0D50X0000BVnBhASQV/bug-report-dma-and-adc-initialization-order-changed-in-stm32f4-hal-v1241-causing-incorrect-adc-operation
/*
  for (uint8_t i=0;i<3;i++)
  {
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOF, LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOF, LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin, GPIO_PIN_RESET);
  }
*/
  LCD_Init();
  LCD_UG_init();
/*
  LCD_Intro_LogoSlide(140,200);
  bitrate = DrawColors(0,0,80);

  UG_SetForecolor(C_WHITE);
  UG_FontSelect(&FONT_8X12);
  sprintf(str,"%.2f MB/s, %.0f fps",bitrate/(1024*1024),bitrate/(100.0*180*2));
  UG_PutString(5,105,str);

  // to mi deli
  UG_SetForecolor(C_WHITE);
  UG_FontSelect(&FONT_16X26);
  UG_PutString(5,205,"To mi deli, Borut!");
*/

  joystick_init(&joystick);
  HAL_ADC_Start_DMA(&hadc4, &joystick_raw, 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


//leva ploscica
  	int paddle_x_leva = 20;
  	int paddle_y_leva = 90; // Sredina zaslona: 240/2 - 60/2 = 90
  	UG_FillFrame(paddle_x_leva, paddle_y_leva, paddle_x_leva + paddle_width - 1, paddle_y_leva + paddle_height - 1, C_WHITE); // LEVA ploščica: (20, 90), (29, 149)

//desna ploscica
  	int paddle_x_desna = pong_width - paddle_width - 20;
  	int paddle_y_desna = 90;
  	UG_FillFrame(paddle_x_desna, paddle_y_desna, paddle_x_desna + paddle_width - 1, paddle_y_desna + paddle_height - 1, C_WHITE); // DESNA ploščica: (), ()299, 149


// ŽOGICA
uint8_t zogica = 90;
UG_FillCircle(160, 120, 5, C_WHITE ); // x, y, radij, barva

	narisi_score();
  	zogica_reset();


  	/* USER CODE BEGIN 2 */
  	  LCD_Init();
  	  LCD_UG_init();
  	  joystick_init(&joystick);
  	  HAL_ADC_Start_DMA(&hadc4, &joystick_raw, 2);

  	  // --- ZAČETNI ZASLON ---
  	  draw_title_screen();

  	  // Čakaj, da uporabnik pritisne tipko OK (PC15, kot v tvoji funkciji zogica_reset)
  	  // Dodamo še preprosto utripanje besedila "PRITISNI OK"
  	  uint32_t last_blink = HAL_GetTick();
  	  uint8_t text_visible = 1;

  	  while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_SET)
  	  {
  	      if (HAL_GetTick() - last_blink > 500) {
  	          text_visible = !text_visible;
  	          last_blink = HAL_GetTick();
  	          UG_FontSelect(&FONT_16X26);
  	          if (text_visible) {
  	              UG_SetForecolor(C_WHITE);
  	              UG_PutString(45, 190, "PRITISNI OK");
  	          } else {
  	              UG_FillFrame(45, 190, 275, 220, C_BLACK); // Prekrij tekst s črno
  	          }
  	      }
  	  }

  	  // Ko je gumb pritisnjen, počisti zaslon in začni igro
  	  UG_FillFrame(0, 0, 319, 239, C_BLACK);
  	  /* USER CODE END 2 */

  while (1)
  {

	  joystick_get(&joystick_raw, &joystick_new, &joystick);

	  // ---------- LEVA PLOŠČICA (JOYSTICK) ----------

	  int old_y_leva = paddle_y_leva;

	  if (joystick_new.y < -40) paddle_y_leva -= paddle_speed;
	  if (joystick_new.y >  40) paddle_y_leva += paddle_speed;

	  // Omeji Y LEVE ploščice
	  if (paddle_y_leva < 0) paddle_y_leva = 0; //Preprečimo izhod zogice zgoraj

	  if (paddle_y_leva > pong_height - paddle_height) paddle_y_leva = pong_height - paddle_height; //Preprecimo izhod zogice spodaj

	  // Če se je Y spremenil, izbriši staro in nariši novo ploščico
	  if (old_y_leva != paddle_y_leva) //Ploščico pobrišeš in ponovno narišeš samo, če se je premaknila
	  {
	      UG_FillFrame(paddle_x_leva, old_y_leva, paddle_x_leva + paddle_width - 1, old_y_leva + paddle_height - 1, C_BLACK);
	      UG_FillFrame(paddle_x_leva, paddle_y_leva, paddle_x_leva + paddle_width - 1, paddle_y_leva + paddle_height - 1, C_WHITE);
	  }

	  // ODBOJ OD L
	  if (zogica_x - zogica_size <= (paddle_x_leva + paddle_width))
	  {
	      // Preverimo še, če je žogica po Y med vrhom in dnom ploščice
	      if (zogica_y >= paddle_y_leva && zogica_y <= (paddle_y_leva + paddle_height))
	      {
	          zogica_dx = -zogica_dx;          // Obrni smer v desno
	          zogica_x = (paddle_x_leva + paddle_width) + zogica_size; // Prepreči "zatikanje" žogice v ploščici
	      }
	  }

	  // ---------- DESNA PLOŠČICA (TIPKE) ----------
	  int old_y_desna = paddle_y_desna;

	  if (!HAL_GPIO_ReadPin(BTN_UP_GPIO_Port, BTN_UP_Pin)) //če je pritisnjeno, vrne 0, negacija ! pa to spremeni v "resnično"
	      paddle_y_desna -= paddle_speed; // premik ploščice navzgor

	  if (!HAL_GPIO_ReadPin(BTN_DOWN_GPIO_Port, BTN_DOWN_Pin))
	      paddle_y_desna += paddle_speed; // premik ploscice navzdol

	  // Omeji Y DESNE ploščice
	  if (paddle_y_desna < 0)
	      paddle_y_desna = 0;
	  else if (paddle_y_desna > pong_height - paddle_height)
	      paddle_y_desna = pong_height - paddle_height;

	  // Če se je Y spremenil, izbriši staro in nariši novo ploščico
	  if (old_y_desna != paddle_y_desna)
	  {
	      UG_FillFrame(paddle_x_desna, old_y_desna, paddle_x_desna + paddle_width - 1, old_y_desna + paddle_height - 1, C_BLACK);
	      UG_FillFrame(paddle_x_desna, paddle_y_desna, paddle_x_desna + paddle_width - 1, paddle_y_desna + paddle_height - 1, C_WHITE);
	  }

	  // Preverimo, če je žogica po X prišla do desne ploščice (x = 290)
	  if (zogica_x + zogica_size >= paddle_x_desna)
	  {
	      // Preverimo še, če je žogica po Y med vrhom in dnom ploščice
	      if (zogica_y >= paddle_y_desna && zogica_y <= (paddle_y_desna + paddle_height))
	      {
	          zogica_dx = -zogica_dx;          // Obrni smer v levo
	          zogica_x = paddle_x_desna - zogica_size; // Prepreči "zatikanje"
	      }
	  }

	     HAL_Delay(20);
	     zogica_narisi();
	   }


  while(1) {

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  //LEDs and KEYs
//	 HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, !HAL_GPIO_ReadPin(BTN_OK_GPIO_Port, BTN_OK_Pin));
//	 HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, !HAL_GPIO_ReadPin(BTN_DOWN_GPIO_Port, BTN_DOWN_Pin));
//	 HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, !HAL_GPIO_ReadPin(BTN_RIGHT_GPIO_Port, BTN_RIGHT_Pin));
//	 HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, !HAL_GPIO_ReadPin(BTN_UP_GPIO_Port, BTN_UP_Pin));
//	 HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, !HAL_GPIO_ReadPin(BTN_LEFT_GPIO_Port, BTN_LEFT_Pin));
//	 HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, !HAL_GPIO_ReadPin(BTN_ESC_GPIO_Port, BTN_ESC_Pin));
//	 HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, !HAL_GPIO_ReadPin(JOY_BTN_GPIO_Port, JOY_BTN_Pin));
}
  /* USER CODE END 3 */


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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

//--------------------------------------FUNKCIJE--------------------------------------------------

/* USER CODE BEGIN 4 */

void narisi_sredino(void) {
	uint8_t w = 240 / (2 * st_crt - 1 );
	uint8_t y;
		for(uint8_t i = 1; i <= (2 * st_crt); i++) {
			y = (i - 1) * w;
			if (i % 2 != 0) {
				UG_FillFrame(159, y, 161, y+w, C_CHOCOLATE);// x, y, h, w, barva
			}
			else {
				UG_FillFrame(159, y, 161, y+w, C_BLACK);
			}
		}
}


void zogica_narisi(void) {
    // 1. POBRIŠI staro žogico
    UG_FillCircle((int16_t)zogica_x, (int16_t)zogica_y, zogica_size, C_BLACK);

    narisi_sredino();
    if (zogica_y < 50) narisi_score();

    // 3. PREMIK
    zogica_x += zogica_dx;
    zogica_y += zogica_dy;

    // 4. LOGIKA ODBOJEV (Z deltami)
    float delta = 1.0f;

    // Odboj od sten (zgoraj/spodaj)
    if(zogica_y <= zogica_size || zogica_y >= (240 - zogica_size)){
        zogica_dy = -zogica_dy;
    }

    // Odboj od LEVE ploščice
    if (zogica_x - zogica_size <= (paddle_x_leva + paddle_width + delta)) {
        if (zogica_y >= (paddle_y_leva - delta) && zogica_y <= (paddle_y_leva + paddle_height + delta)) {
            zogica_dx = -zogica_dx;
            zogica_x = (paddle_x_leva + paddle_width) + zogica_size + 1; // "Odlepi" žogico od ploščice
        }
    }

    // Odboj od DESNE ploščice
    if (zogica_x + zogica_size >= (paddle_x_desna - delta)) {
        if (zogica_y >= (paddle_y_desna - delta) && zogica_y <= (paddle_y_desna + paddle_height + delta)) {
            zogica_dx = -zogica_dx;
            zogica_x = paddle_x_desna - zogica_size - 1; // "Odlepi" žogico od ploščice
        }
    }

    // 5. TOČKE
    if(zogica_x < 0) { povecaj_score(1); return; }
    if(zogica_x > 320) { povecaj_score(0); return; }

    // 6. NARIŠI novo pozicijo
    UG_FillCircle((int16_t)zogica_x, (int16_t)zogica_y, zogica_size, C_WHITE);
}

//-----------------------------------------------------------------------------------------------

int zogica_reset(void) {
	zogica_x = 160.0f;
	zogica_y = 120.0f;

	narisi_score();
	narisi_sredino();
	UG_FillCircle(160, 120, 5, C_WHITE);

	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_SET) {
	}

	UG_FillFrame(100, 160, 220, 180, C_BLACK); // Ko pritisneš OK, pobriši napis in nadaljuj

	zogica_dx = (zogica_dx > 0) ? -2.0f : 2.0f;
	zogica_dy = 1.5f;

	return 1;

}

//-----------------------------------------------------------------------------------------------

int preveri_zmago(void) {
    // Ta funkcija vrne 1, če je nekdo dosegel 5 točk (rezultat 4)
    if (score_L > 3 || score_D > 3) {
        return 1; // Konec iteracije!
    }
    return 0; // Igra se nadaljuje
}

//-----------------------------------------------------------------------------------------------

float narisi_score(void) {
	char score_buffer[16]; // Začasni prostor za pretvorbo številk v tekst

	UG_FontSelect(&FONT_16X26);
	UG_SetForecolor(C_WHITE);
	UG_SetBackcolor(C_BLACK);

	UG_FillFrame(95, 5, 135, 40, C_BLACK); // PObrišemo staro območje točk, da se št. ne prekrivajo
	sprintf(score_buffer, "%d", score_L);
	UG_PutString(100, 10, score_buffer);

	UG_FillFrame(200, 5, 240, 40, C_BLACK);
	sprintf(score_buffer, "%d", score_D);
	UG_PutString(207, 10, score_buffer);

	if (score_L > 4 || score_D > 4) {
	        return 1;
	    }
	    return 0;
	}

//-----------------------------------------------------------------------------------------------

void povecaj_score(int igralec) {
	if(igralec == 0) score_L++;
	else score_D++;

	if (score_L > max_tock_na_niz || score_D > max_tock_na_niz) {
		if(score_L > 3) nizi_L++;
		else nizi_D++;

		score_L = 0;
		score_D = 0;

	if (nizi_L >= max_nizov || nizi_D >= max_nizov) {
		nizi_L = 0;
		nizi_D = 0;
		// Tukaj pride utripanje luck
	}

	HAL_Delay(1000);
	}
	narisi_score();
	zogica_reset();

}

//-----------------------------------------------------------------------------------------------

void score_reset(void) {
	if(score_L > 3 || score_D > 3) {
		score_L = 0;
		score_D = 0;

		narisi_score();

		HAL_Delay(500);

		zogica_reset();
	}
}

//-----------------------------------------------------------------------------------------------
/* USER CODE BEGIN 4 */

void draw_title_screen(void) {
    // 1. Počisti zaslon na črno
    UG_FillFrame(0, 0, 319, 239, C_BLACK);

    // 2. Nariši retro okvir (dvojna črta)
    UG_DrawFrame(5, 5, 314, 234, C_WHITE);
    UG_DrawFrame(8, 8, 311, 231, C_WHITE);

    // 3. Naslov PONG
    UG_FontSelect(&FONT_32X53); // Uporabi največji font, ki ga imaš (npr. 22X36 ali 32X53)
    UG_SetForecolor(C_WHITE);
    UG_SetBackcolor(C_BLACK);
    UG_PutString(95, 50, "PONG");

    // 4. Dekoracija: Majhni palici in žogica
    UG_FillFrame(20, 100, 30, 160, C_LIME);      // Leva dekorativna palica
    UG_FillFrame(290, 80, 300, 140, C_LIME);     // Desna dekorativna palica
    UG_FillCircle(160, 130, 5, C_WHITE);         // Žogica na sredini

    // 5. Avtor / Verzija
    UG_FontSelect(&FONT_8X12);
    UG_PutString(110, 110, "RETRO C-EDITION");

    // 6. Navodilo (spodaj)
    UG_FontSelect(&FONT_16X26);
    UG_PutString(45, 190, "PRITISNI OK");
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
