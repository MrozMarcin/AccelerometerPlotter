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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "iis3dwb_reg.h"
#include "stdlib.h"
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
static int16_t data_raw_acceleration[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

const uint16_t f_signal = 1000;
const uint16_t n_samples = 10000;
const uint16_t f_sample = 10000;
const double amplitude = 1;
const double pi = 3.14159265358979323846;
	
//double time[n_samples];	
double x[n_samples];
double y[n_samples];
double z[n_samples];

long long no = 0;
#define DATA_LEN 25 
char data_string[DATA_LEN];

uint32_t delta_t_s;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void SWO_ITM_enable(void);
bool itoaBase10(int32_t num, char *str, size_t length);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  stmdev_ctx_t dev_ctx;
volatile bool getMeas = false;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	SWO_ITM_enable();

//	for (uint32_t i = 0; i < n_samples; i++)
//	{
////		time[i] = i*(1.0/f_sample);
//		x[i] = sin(i*(1.0/f_sample) * f_signal * 2 * pi) * amplitude;;
//		y[i] = 1 + sin(i*(1.0/f_sample) * f_signal * 2 * pi) * amplitude;;
//		z[i] = 2 + sin(i*(1.0/f_sample) * f_signal * 2 * pi) * amplitude;;
//	}
	
//	while(1)
//	{
//		for (uint32_t i = 0; i < n_samples; i++)
//		{		
//			sprintf(data_string, "%lld;%f;%f;%f\n", no, x[i], y[i], z[i]);
//			HAL_UART_Transmit(&huart1, (uint8_t *)data_string, strlen(data_string), 100);
//			no++;
//		}
//	}

	
  stmdev_ctx_t dev_ctx;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hspi1;

	do{  
		/* Wait sensor boot time */
		HAL_Delay(10);
		/* Check device ID */
		iis3dwb_device_id_get(&dev_ctx, &whoamI);
	}while(whoamI != IIS3DWB_ID);

  /* Restore default configuration */
  iis3dwb_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    iis3dwb_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  iis3dwb_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  iis3dwb_xl_data_rate_set(&dev_ctx, IIS3DWB_XL_ODR_26k7Hz);
  /* Set full scale */
  iis3dwb_xl_full_scale_set(&dev_ctx, IIS3DWB_2g);
  /* Configure filtering chain(No aux interface)
   * Accelerometer low pass filter path
   */
  iis3dwb_xl_filt_path_on_out_set(&dev_ctx, IIS3DWB_HP_ODR_DIV_10);
	
	iis3dwb_pin_int1_route_t temp;
	temp.drdy_xl = 1;
	iis3dwb_pin_int1_route_set(&dev_ctx, &temp);
	iis3dwb_data_ready_mode_set(&dev_ctx, IIS3DWB_DRDY_PULSED);
	
	uint8_t reg;
	uint32_t str_len = 0;
	memset(data_string, 0x00, DATA_LEN);
	HAL_Delay(10);
	
	/* Enable INT1 interrupt */
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//		/* Read output only if new xl value is available */
//		iis3dwb_xl_flag_data_ready_get(&dev_ctx, &reg);
//		if (reg) {
//			/* Read acceleration field data */
//			iis3dwb_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
////			acceleration_mg[0] = iis3dwb_from_fs2g_to_mg(data_raw_acceleration[0]);
////			acceleration_mg[1] = iis3dwb_from_fs2g_to_mg(data_raw_acceleration[1]);
////			acceleration_mg[2] = iis3dwb_from_fs2g_to_mg(data_raw_acceleration[2]);
//			memset(data_string, '0x0A', strlen(data_string));
//			_sprintf(data_string, "0;%i;%i;%i\n", data_raw_acceleration[0], data_raw_acceleration[1], data_raw_acceleration[2]);
//			HAL_UART_Transmit(&huart1, (uint8_t *)data_string, strlen(data_string), 10);
//			no++;
//		}
		
		char num[10];
			
		if(getMeas == true)
		{
			iis3dwb_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
			memset(data_string, 0x00, DATA_LEN);
			_sprintf(data_string, "0;%hx;%hx;%hx\n", (int16_t)data_raw_acceleration[0], (int16_t)data_raw_acceleration[1], (int16_t)data_raw_acceleration[2]);		
//			memset(data_string, 0x00, sizeof(data_string));
//			strcat(data_string, "0;");
//			itoaBase10(data_raw_acceleration[0], num, 7);
//			strcat(data_string, num);
//			strcat(data_string, ";");
//			memset(num, 0x00, sizeof(num));
//			itoaBase10(data_raw_acceleration[1], num, 7);
//			strcat(data_string, num);
//			strcat(data_string, ";");
//			memset(num, 0x00, sizeof(num));
//			itoaBase10(data_raw_acceleration[2], num, 7);
//			strcat(data_string, num);
//			strcat(data_string, "\n");
//			memset(num, 0x00, sizeof(num));
			HAL_UART_Transmit_IT(&huart1, (uint8_t *)data_string, strlen(data_string));
			
			no++;
			getMeas = false;
		}

//		iis3dwb_temp_flag_data_ready_get(&dev_ctx, &reg);

//		if (reg) {
//			/* Read temperature data */
//			memset(&data_raw_temperature, 0x00, sizeof(int16_t));
//			iis3dwb_temperature_raw_get(&dev_ctx, &data_raw_temperature);
//			temperature_degC = iis3dwb_from_lsb_to_celsius(data_raw_temperature);
//			printf("Temperature [degC]:%6.2f\r\n", temperature_degC);
//		}
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* USER CODE BEGIN 4 */
/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	HAL_StatusTypeDef errorcode = HAL_OK;
  if (handle == &hspi1) 
	{		
		SPI1_IIS3DWB_CS_GPIO_Port->BSRR = (uint32_t) SPI1_IIS3DWB_CS_Pin << 16U;
    HAL_SPI_Transmit(handle, &reg, 1, 10);
    HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 10);
		SPI1_IIS3DWB_CS_GPIO_Port->BSRR = (uint32_t) SPI1_IIS3DWB_CS_Pin;
  }
	return errorcode;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_StatusTypeDef errorcode = HAL_OK;
  if (handle == &hspi1) 
	{
    /* Read command */
    reg |= 0x80;
		SPI1_IIS3DWB_CS_GPIO_Port->BSRR = (uint32_t) SPI1_IIS3DWB_CS_Pin << 16U;
    HAL_SPI_Transmit(handle, &reg, 1, 10);
    HAL_SPI_Receive(handle, bufp, len, 10);
		SPI1_IIS3DWB_CS_GPIO_Port->BSRR = (uint32_t) SPI1_IIS3DWB_CS_Pin;
  }
  return errorcode;
}

static void SWO_ITM_enable(void)
{
	extern uint32_t SystemCoreClock;
  /*
    This functions recommends system speed of 400000000Hz and will
    use SWO clock speed of 2000000Hz

    # GDB OpenOCD commands to connect to this:
    monitor tpiu config internal - uart off 400000000
    monitor itm port 0 on

    Code Gen Ref: https://gist.github.com/mofosyne/178ad947fdff0f357eb0e03a42bcef5c
  */

  /* Setup SWO and SWO funnel (Note: SWO_BASE and SWTF_BASE not defined in stm32h743xx.h) */
  // DBGMCU_CR : Enable D3DBGCKEN D1DBGCKEN TRACECLKEN Clock Domains
  DBGMCU->CR =  DBGMCU_CR_DBG_CKD3EN | DBGMCU_CR_DBG_CKD1EN | DBGMCU_CR_DBG_TRACECKEN; // DBGMCU_CR
  // SWO_LAR & SWTF_LAR : Unlock SWO and SWO Funnel
  *((uint32_t *)(0x5c003fb0)) = 0xC5ACCE55; // SWO_LAR
  *((uint32_t *)(0x5c004fb0)) = 0xC5ACCE55; // SWTF_LAR
  // SWO_CODR  : 480000000Hz -> 2000000Hz
  // Note: SWOPrescaler = ((sysclock_Hz / SWOSpeed_Hz) - 1) --> 0x0000c7 = 199 = (480000000 / 2000000) - 1)
//	/* J-link */
//  *((uint32_t *)(0x5c003010)) = ((SystemCoreClock /  4484581) - 1); // SWO_CODR
	/* St-link */
  *((uint32_t *)(0x5c003010)) = ((SystemCoreClock /  12000000) - 1); // SWO_CODR
  // SWO_SPPR : (2:  SWO NRZ, 1:  SWO Manchester encoding)
  *((uint32_t *)(0x5c0030f0)) = 0x00000002; // SWO_SPPR
  // SWTF_CTRL : enable SWO
  *((uint32_t *)(0x5c004000)) |= 0x1; // SWTF_CTRL

  /* SWO GPIO Pin Setup */
  //RCC_AHB4ENR enable GPIOB clock
  *(__IO uint32_t*)(0x580244E0) |= 0x00000002;
  // Configure GPIOB pin 3 as AF
  *(__IO uint32_t*)(0x58020400) = (*(__IO uint32_t*)(0x58020400) & 0xffffff3f) | 0x00000080;
  // Configure GPIOB pin 3 Speed
  *(__IO uint32_t*)(0x58020408) |= 0x00000080;
  // Force AF0 for GPIOB pin 3
  *(__IO uint32_t*)(0x58020420) &= 0xFFFF0FFF;
}

#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
  if (DEMCR & TRCENA) {
    while (ITM_Port32(0) == 0);
    ITM_Port8(0) = ch;
  }
  return(ch);
}

/*
  Modified from:
  https://gist.github.com/madex/c5cd5c6a23965a845d6e

  This only works for up to 9 digits and only for base 10 numbers,
  but no division is used and this method is very fast.
*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

bool itoaBase10(int32_t num, char *str, size_t length)
{
    /* output is at least sign char, one digit and a terminating NUL */
    if (!str || length <= 2) {
        return false;
    }

    if (!num) {
        /* special-case zero (which would be skipped as a leading 0) */
        strcpy(str, " 0");
        return true;
    }

    static const uint32_t subtractands[] =  {
        1000000000,
        100000000,
        10000000,
        1000000,
        100000,
        10000,
        1000,
        100,
        10,
        1,
        0
    };

    /* last possible position for NUL */
    char const *const last_pos = str + length - 1;
    /* work with a positive version of num */
    uint32_t u = num < 0 ? -(uint32_t)num : (uint32_t)num;

    /* write sign character */
    *str++ = num < 0 ? '-' : '+';

    /* skip leading zeros */
    uint32_t const *sub = subtractands;
    while (u < *sub) {
        ++sub;
    }

    /* write the digits */
    while (*sub)  {
        if (str >= last_pos) {
            /* no space for NUL */
            return false;
        }
        char n = '0';
        while (u >= *sub) { u -= *sub; ++n; }
        *str++ = n;
        ++sub;
    }

    *str = '\0';
    return true;
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
