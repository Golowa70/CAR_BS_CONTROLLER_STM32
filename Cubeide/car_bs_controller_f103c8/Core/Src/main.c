/* USER CODE BEGIN Header */
/**
  ******************************************************************************



  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "u8g2.h"
#include "w25qxx.h"
//#include "stdio.h"
#include "stdbool.h"
#include "button.h"
#include "mini-printf.h"
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
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
u8g2_t u8g2;

//*********** Setpoints variables *********************************************************************

typedef struct SetpointsStruct { // структура для уставок

  uint8_t pump_T_off;
  uint8_t pump_out_mode;  		// Режим(выкл, авто, вкл).
  uint8_t conv_U_off;    		// дробное со смещённой вправо точкой 12.7в = 127,  13.2в =132 и т.д.
  uint8_t conv_T_U_off;  		// задержка отключения по низкому напряжению U_off
  uint8_t conv_U_on;     		// напряжение включения (порог)
  uint8_t conv_T_IGN_off;
  uint8_t conv_out_mode;
  uint8_t fridge_U_off;
  uint8_t fridge_T_U_off;
  uint8_t fridge_U_on;
  uint8_t fridge_T_IGN_off;
  uint8_t fridge_Temp_on;
  uint8_t fridge_Temp_off;
  uint8_t fridge_out_mode;
  uint8_t water_sens_min;
  uint8_t water_sens_max;
  uint8_t water_sens_correction;
  uint8_t water_tank_capacity;
  uint8_t shutdown_delay;
  uint8_t U_correction;
  uint8_t brightness;
  uint8_t logo;
  uint8_t inside_sensor_ID;
  uint8_t outside_sensor_ID;
  uint8_t fridge_sensor_ID;

}default_setpoints_data;

//union
union {

  struct SetpointsStruct setpoints_data;
  uint8_t SetpointsArray[MENU_SETPOINTS_NUM_ITEMS];

}SetpointsUnion; //

//********** end setpoints variables ******************************************************************

//*********** Main data *******************************************************************************
struct MyData
{
  float outside_temperature;      //  наружная температура
  float inside_temperature;       // температура внутри
  float fridge_temperature;        // температура третьего датчика(пока не используется)
  uint16_t res_sensor_resistance;    // сопротивление резистивного датчика
  uint8_t battery_voltage;          // напряжени бортсети ( например 124 это 12.4в)
  uint8_t sensors_supply_voltage;   // напряжение питания датчиков 5в
  uint8_t water_level_liter;      // уровень воды в литрах
  uint8_t error_code;

  bool door_switch_state;         // состояние концевика задней двери
  bool proximity_sensor_state;    // состояние датчика приближения
  bool ignition_switch_state;     // состояние входа зажигания
  bool converter_output_state;    // состояние выхода управления инвертором 12/220в
  bool fridge_output_state;        //состояние выхода освещения
  bool pump_output_state;         //состояние выхода насоса
  bool sensors_supply_output_state; // состояние выхода управления питанием сенсоров 5в
  bool main_supply_output_state;  // состояние выхода управления общим питанием 7.5в
  bool flag_system_started;

} main_data;

//****** end main data **************************************************************

//********** MENU VARIABLES *********************************************************
  /* массивы строк с именами пунктов меню просмотра параметров */
  const char p0[]  = "Battery V";
  const char p1[]  = "Water level";
  const char p2[]  = "Temp outside";
  const char p3[]  = "Temp inside";
  const char p4[]  = "Temp fridge";
  const char p5[]  = "Sensors V";
  const char p6[]  = "RS Resistance";
  const char p7[]  = "Door";
  const char p8[]  = "Prx sensor";
  const char p9[]  = "IGN";
  const char p10[]  = "Conv relay";
  const char p11[]  = "Fridge relay";
  const char p12[]  = "Pump relay";
  const char p13[]  = "Error code";
  const char p14[]  = "Spare";


/*Массив ссылок на имена пунктов  меню просмотра параметров, обращение к названию пунктов по их номеру*/
const char* const parameters_names[]  =
{
  p0,p1,p2,p3,p4,
  p5,p6,p7,p8,p9,
  p10,p11,p12,p13,p14
};

/* массивы строк с именами пунктов меню настроек */
	const char i0[] = "Pump T off";
	const char i1[] = "Pump out mode";
	const char i2[] = "Conv U off";
	const char i3[] = "Conv T U off";
	const char i4[] = "Conv U on";
	const char i5[] = "Conv T IGN off";
	const char i6[] = "Conv out mode";
	const char i7[] = "Fridge U off";
	const char i8[] = "Fridge T U off";
	const char i9[] = "Fridge U on";
	const char i10[] = "Fridge T IGN off";
	const char i11[] = "Fridge temp on";
	const char i12[] = "Fridge temp off";
	const char i13[] = "Fridge out mode";
	const char i14[] = "Water sens min";
	const char i15[] = "Water sens max";
	const char i16[] = "Water sens corr";
	const char i17[] = "Water tank cap";
	const char i18[] = "Shutdown delay";
	const char i19[] = "U correction";
	const char i20[] = "Brightness";
	const char i21[] = "Logo";
	const char i22[] = "Inside sid";
	const char i23[] = "Outside sid";
	const char i24[] = "Fridge sid";

  /*Массив ссылок на имена пунктов  меню настроек, обращение к названию пунктов по их номеру*/
  const char* const setpoints_menu_names[]  =
  {
    i0, i1, i2, i3, i4,
	i5, i6, i7, i8, i9,
	i10, i11, i12, i13, i14,
	i15, i16, i17, i18, i19,
	i20, i21, i22, i23, i24
  };

  //Массив минимальных значений параметров
  uint8_t param_range_min[MENU_SETPOINTS_NUM_ITEMS]  =
  {
    3,0,90,1,100,

  };

  //Массив максимальных значений параметров
  uint8_t param_range_max[MENU_SETPOINTS_NUM_ITEMS]  =
  {
   20,2,130,240,180,

  };

  uint8_t menu_current_item = 0; // Переменная указатель на текущий пункт меню
  uint8_t menu_current_page = 0;

  enum Menus{
	  MENU_MAIN_VIEW,MENU_PARAM_VIEW,MENU_SETPOINTS,MENU_SETPOINTS_EDIT_MODE,MENU_LOGO_VIEW,MENUS_MAX
  };

  enum Menus menu_mode = MENU_MAIN_VIEW;


  //****** DISPLAY VARIABLES ********************************************************
  uint8_t display_height;
  uint8_t display_width;
  uint8_t display_num_lines;

  //****** Errors ***************************************************************************

 //******** BUTTONS ******************************************************
  enum Buttons{
  	BUTTON_UP,BUTTON_DOWN,BUTTON_ENTER,BUTTON_ESC,MAX_BUTTONS
  };

   enum ButtonHandlerMode{
  	NAVI_MODE,EDIT_MODE
  };

   enum ButtonHandlerMode BtnHandlerMode = NAVI_MODE;

   static uint16_t btn_state = 0;
   uint8_t btnStatesArray[MAX_BUTTONS] = {0,};
   Button_Struct_t Button_A;
   Button_Struct_t Button_B;
   Button_Struct_t Button_C;
   Button_Struct_t Button_D;

  //********* OTHER VARIABLES *********************************************************

	uint8_t main_process_step = 0;
	uint32_t  time_b = 0;
	bool flag_blink = false; // флаг для мигания чего либо на экране
	uint8_t imageBuff[IMAGE_BUFFER_SIZE] = {0,};
	bool flag_mb_connected = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

//DISPLAY FUNCTION
uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8,
		U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
		U8X8_UNUSED void *arg_ptr);
void u8g_port_delay_10us(uint8_t us);
void u8g_port_delay_100ns(uint8_t ns);
void u8g_port_delay_ns(uint8_t ns);

//MENU FUNCTIONS
void fnPrintSelectionFrame(uint8_t item_pointer);
void printMenuSetpoints(void);
void fnPrintMenuItemName(uint8_t _num_item, uint8_t _num_line, const char* const* _names);
void fnPrintMenuSetpointsItemVal(uint8_t num_item, uint8_t num_line);
void fnPrintMenuParamView(void);
void fnPrintMainView(void);
void fnMenuProcess(void);

//BUTTON FUNCTIONS
static uint16_t fnGetPressKey(void);
uint32_t Button_Get_Tick(void);
uint8_t Button_A_Read(void);
uint8_t Button_B_Read(void);
uint8_t Button_C_Read(void);
uint8_t Button_D_Read(void);

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

//FLASH INIT
  	  W25qxx_Init();
	//W25qxx_EraseChip();
	//W25qxx_EraseBlock(0); // 65536 байт
	//W25qxx_EraseSector(0); // 4096 байт

//DISPLAY INIT
	u8g2_Setup_st7565_nhd_c12864_f(&u8g2, U8G2_R2, u8x8_byte_4wire_hw_spi,
			u8g2_gpio_and_delay_stm32);
	u8g2_InitDisplay(&u8g2); 	 // send init sequence to the display, display is in sleep mode after this
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	u8g2_SetContrast(&u8g2, 250);
	u8g2_ClearDisplay(&u8g2);
	u8g2_SetFont(&u8g2, u8g2_font_courB18_tr);
	u8g2_DrawStr(&u8g2, 20, 30, "Hello!");
	u8g2_SendBuffer(&u8g2);
	HAL_Delay(1000);
	display_height = u8g2_GetDisplayHeight(&u8g2);
	display_width = u8g2_GetDisplayWidth(&u8g2);
	display_num_lines = display_height / (LCD_FONT_HIGHT + LCD_LINE_SPACER);

//BUTTONS INIT
	Button_A.Button_Init = NULL; // инициализация кнопки
	Button_A.Button_Read = Button_A_Read;
	Button_A.Callback = NULL; //    NULL; /** without callback */
	Button_Add(&Button_A);

	Button_B.Button_Init = NULL;
	Button_B.Button_Read = Button_B_Read;
	Button_B.Callback = NULL; //    NULL; /** without callback */
	Button_Add(&Button_B);

	Button_C.Button_Init = NULL;
	Button_C.Button_Read = Button_C_Read;
	Button_C.Callback = NULL; //    NULL; /** without callback */
	Button_Add(&Button_C);

	Button_D.Button_Init = NULL;
	Button_D.Button_Read = Button_D_Read;
	Button_D.Callback = NULL; //    NULL; /** without callback */
	Button_Add(&Button_D);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if((HAL_GetTick() - time_b) > BLINK_INTERVAL) // интервал 500мс
	   {
		  flag_blink = !flag_blink;
		  time_b = HAL_GetTick();
	   }


	  fnMenuProcess();

	  btn_state = fnGetPressKey();// опрос кнопок

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W25Q_CS_GPIO_Port, W25Q_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RESET_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : W25Q_CS_Pin */
  GPIO_InitStruct.Pin = W25Q_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W25Q_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RESET_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_ENTER_Pin BUTTON_ESC_Pin BUTTON_UP_Pin BUTTON_DOWN_Pin */
  GPIO_InitStruct.Pin = BUTTON_ENTER_Pin|BUTTON_ESC_Pin|BUTTON_UP_Pin|BUTTON_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


//****************************************************************************

//функции задержек для работы библиотеки дисплея
void u8g_port_delay_ns(uint8_t ns) {
	// Core @72 MHZ: 14ns per instruction.
	// __NOP(); is direct "nop;" instruction to cpu.
	// Divide ns / 28 (extra instruction for jump back to beginning of the loop) for loop cycles.
	for (uint8_t i = 0; i < (ns / 28); i++) {
		__NOP();
	}
}

void u8g_port_delay_100ns(uint8_t ns) {
	// Same as in u8g_hw_port_delay_ns function.
	// 100 / 28 = 3.57;
	for (uint16_t i = 0; i < (ns * 3.57); i++) {
		__NOP();
	}
}

void u8g_port_delay_10us(uint8_t us) {
	// Same as in u8g_hw_port_delay_ns function.
	// 3.57 * 100 ? 357;
	for (uint16_t i = 0; i < (us * 357); i++) {
		__NOP();
	}
}
//************************************************************************

// функция обработки задержек и управления gpio для работы библиотеки дисплея
uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8,
		U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
		U8X8_UNUSED void *arg_ptr) {

	switch (msg) {

		case U8X8_MSG_GPIO_AND_DELAY_INIT:
		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, RESET);
		break;

		case U8X8_MSG_DELAY_NANO:
		u8g_port_delay_ns(arg_int);
		break;

		case U8X8_MSG_DELAY_100NANO:
		u8g_port_delay_100ns(arg_int);
		break;

		case U8X8_MSG_DELAY_10MICRO:
		u8g_port_delay_10us(arg_int);
		break;

		case U8X8_MSG_DELAY_MILLI:
		HAL_Delay(arg_int);
		break;

		case U8X8_MSG_GPIO_RESET:
		if (arg_int)
		HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, SET);
		else
		HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, RESET);
		break;
		default:
		return 0;//A message was received which is not implemented, return 0 to indicate an error
	}

	return 1; // command processed successfully.
}
//***************************************************************************************************

// функция для работы библиотеки дисплея по SPI
uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr) {

	switch (msg) {
	case U8X8_MSG_BYTE_SEND:
		HAL_SPI_Transmit(&hspi1, (uint8_t*) arg_ptr, arg_int, 100);
		break;

	case U8X8_MSG_BYTE_INIT:
		break;

	case U8X8_MSG_BYTE_SET_DC:
		 HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, arg_int);
		break;

	case U8X8_MSG_BYTE_START_TRANSFER:
		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, RESET);
		break;

	case U8X8_MSG_BYTE_END_TRANSFER:
		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, SET);
		break;

	default:
		return 0;
	}
	return 1;
}
//*****************************************************************************************

//Функция печати имени пункта меню из progmem (общая для всех меню) --------------
void fnPrintMenuItemName(uint8_t _num_item, uint8_t _num_line, const char* const* _names) {

  char buffer[32] = {0,};                            // Буфер на полную строку
  uint8_t i = 0;                                     // Переменная - счетчик

  const char * ptr = _names[_num_item];			// Получаем указатель на первый символ строки


  do {                                            // Начало цикла
    buffer[i] = *ptr;        					  // Прочитать в буфер один символ из PGM и подвинуть указатель на 1
    i++;
    ptr++;
  } while (i<ITEM_MAX_CHARS);                     // Если это не конец строки - вернуться в начало цикла


  /*
   while (pgm_read_byte(ptr) != NULL) {           // всю строку до нулевого символа
      buffer[i++] = (char)(pgm_read_byte(ptr));   // выводим
      ptr++;                                      // следующий символ
    }
 */

  u8g2_SetFont(&u8g2,u8g2_font_6x12_tr);
  u8g2_DrawStr(&u8g2,3,(_num_line*12)-1,buffer); // Вывод готовой строки
  u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tr);

}
//*******************************************************************************************************************

//----------- Функция печати рамки навигации по меню -------------------
void fnPrintSelectionFrame(uint8_t item_pointer) {

  uint8_t n = 0;


  if(item_pointer < display_num_lines)n = item_pointer;
  else n = item_pointer % display_num_lines;

  if(menu_mode == MENU_SETPOINTS_EDIT_MODE){


    if(flag_blink)u8g2_DrawFrame(&u8g2,0, n*(LCD_FONT_HIGHT + LCD_LINE_SPACER)+2, display_width-2, (LCD_FONT_HIGHT + LCD_LINE_SPACER));
    else{
      u8g2_SetDrawColor(&u8g2,0);
      u8g2_DrawFrame(&u8g2,0, n*(LCD_FONT_HIGHT + LCD_LINE_SPACER)+2, display_width-2, (LCD_FONT_HIGHT + LCD_LINE_SPACER));
      u8g2_SetDrawColor(&u8g2,1);
    }
  }
  else{
	  u8g2_DrawFrame(&u8g2,0, n*(LCD_FONT_HIGHT + LCD_LINE_SPACER)+2, display_width-2, (LCD_FONT_HIGHT + LCD_LINE_SPACER));
  }

}
//********************************************************************************************************************

//--------- Функция вывода меню уставок ------------------------------------------------
void printMenuSetpoints(void){

  u8g2_ClearBuffer(&u8g2);				//
  u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tr);

  for (uint8_t i = 0; i < display_num_lines; i++) {   // Цикл, выводящий пункты на дисплей

    fnPrintMenuItemName(i+(menu_current_page*display_num_lines), i+1, setpoints_menu_names); // Выводим название пункта
    fnPrintMenuSetpointsItemVal(i+(menu_current_page*display_num_lines), i+1); // Выводим значение пункта меню уставок
  }

  //рисуем рамку
  fnPrintSelectionFrame(menu_current_item);

  //рисуем боковой скролл бар
  uint8_t scroll_bar_height = display_height/(MENU_SETPOINTS_NUM_ITEMS/display_num_lines);
  u8g2_DrawVLine(&u8g2,127, menu_current_page*scroll_bar_height, scroll_bar_height);

  u8g2_SendBuffer(&u8g2);
}
//*************************************************************************************************************

//Функция печати значения пункта меню уставок ---------------------------------
void fnPrintMenuSetpointsItemVal(uint8_t num_item, uint8_t num_line){

  //если все параметры одного типа то можно выводить через массив
  //snprintf(buffer,sizeof(buffer), "%d", SetpointsUnion.SetpointsArray[num_item]);
  //u8g2_DrawStr(&u8g2,98,(num_line*12)-2,buffer);

  char buffer[10] = {0,};
  uint8_t float_m, float_n; // переменные для разбития числа на целую и дробную часть

	switch (num_item) {
	case 0:
		snprintf(buffer, sizeof(buffer), "%ds",
				SetpointsUnion.SetpointsArray[num_item]);
		break;

	case 1:

		switch (SetpointsUnion.SetpointsArray[num_item]) {
		case OFF_MODE:
			snprintf(buffer, sizeof(buffer), "off");
			break;
		case ON_MODE:
			snprintf(buffer, sizeof(buffer), "on");
			break;
		case AUTO_MODE:
			snprintf(buffer, sizeof(buffer), "auto");
			break;
		default:
			break;
		}

		break;

	case 2:
		float_m = SetpointsUnion.SetpointsArray[num_item];
		float_n = float_m % 10;
		float_m = float_m / 10;
		snprintf(buffer, sizeof(buffer), "%d.%d", float_m, float_n);
		break;

	case 3:
		snprintf(buffer, sizeof(buffer), "%ds",
				SetpointsUnion.SetpointsArray[num_item]);
		break;

	case 4:
		float_m = SetpointsUnion.SetpointsArray[num_item];
		float_n = float_m % 10;
		float_m = float_m / 10;
		snprintf(buffer, sizeof(buffer), "%d.%d", float_m, float_n);
		break;

	case 5:
		snprintf(buffer, sizeof(buffer), "%dm",
				SetpointsUnion.SetpointsArray[num_item]);
		break;

	case 6:
		switch (SetpointsUnion.SetpointsArray[num_item]) {
		case OFF_MODE:
			snprintf(buffer, sizeof(buffer), "off");
			break;
		case ON_MODE:
			snprintf(buffer, sizeof(buffer), "on");
			break;
		case AUTO_MODE:
			snprintf(buffer, sizeof(buffer), "auto");
			break;
		default:
			break;
		}
		break;

	case 7:
		float_m = SetpointsUnion.SetpointsArray[num_item];
		float_n = float_m % 10;
		float_m = float_m / 10;
		snprintf(buffer, sizeof(buffer), "%d.%d", float_m, float_n);
		break;

	case 8:
	  	snprintf(buffer,sizeof(buffer), "%ds", SetpointsUnion.SetpointsArray[num_item]);
	  	break;

	case 9:
		float_m = SetpointsUnion.SetpointsArray[num_item];
		float_n = float_m % 10;
		float_m = float_m / 10;
		snprintf(buffer, sizeof(buffer), "%d.%d", float_m, float_n);
		break;

	case 10:
	  	snprintf(buffer,sizeof(buffer), "%dm", SetpointsUnion.SetpointsArray[num_item]);
	      break;

	case 11:
	  	snprintf(buffer,sizeof(buffer), "%dC", SetpointsUnion.SetpointsArray[num_item]);
	    break;

	case 12:
	  	snprintf(buffer,sizeof(buffer), "%dC", SetpointsUnion.SetpointsArray[num_item]);
	    break;

	case 13:
		switch (SetpointsUnion.SetpointsArray[num_item]) {
		case OFF_MODE:
			snprintf(buffer, sizeof(buffer), "off");
			break;
		case ON_MODE:
			snprintf(buffer, sizeof(buffer), "on");
			break;
		case AUTO_MODE:
			snprintf(buffer, sizeof(buffer), "auto");
			break;
		default:
			break;
		}
	    break;

	case 14:
		snprintf(buffer,sizeof(buffer), "%dR", SetpointsUnion.SetpointsArray[num_item]);
		break;

	case 15:
	  	snprintf(buffer,sizeof(buffer), "%dR", SetpointsUnion.SetpointsArray[num_item]);
	    break;

	case 16:
	  	snprintf(buffer,sizeof(buffer), "%d", SetpointsUnion.SetpointsArray[num_item]);
	    break;

	case 17:
	  	snprintf(buffer,sizeof(buffer), "%dL", SetpointsUnion.SetpointsArray[num_item]);
	    break;

	case 18:
	  	snprintf(buffer,sizeof(buffer), "%dh", SetpointsUnion.SetpointsArray[num_item]);
	    break;

	case 19:
	  	snprintf(buffer,sizeof(buffer), "%d", SetpointsUnion.SetpointsArray[num_item]);
	    break;

	case 20:
	  	snprintf(buffer,sizeof(buffer), "%d", SetpointsUnion.SetpointsArray[num_item]);
	    break;

	case 21:
	  	snprintf(buffer,sizeof(buffer), "%d", SetpointsUnion.SetpointsArray[num_item]);
	    break;

	case 22:
	  	snprintf(buffer,sizeof(buffer), "%d", SetpointsUnion.SetpointsArray[num_item]);
	    break;

	case 23:
	  	snprintf(buffer,sizeof(buffer), "%d", SetpointsUnion.SetpointsArray[num_item]);
	    break;

	case 24:
	  	snprintf(buffer,sizeof(buffer), "%d", SetpointsUnion.SetpointsArray[num_item]);
	    break;

	default:
		break;
	}

   u8g2_DrawStr(&u8g2,102,(num_line*12),buffer);

 }
//*********************************************************************************************************************


//Функция печати значения пункта меню просмотра параметров ------------------------------
void fnPrintMenuParamItemVal(uint8_t num_item, uint8_t num_line){

  char buffer[10] = {0,};
  int float_m, float_n; // переменные для разбития числа на целую и дробную часть

  switch (num_item)
  {
  case 0:
    float_m = (int)(main_data.battery_voltage * 10);
    float_n = float_m%10;
    float_m = float_m/10;
    snprintf(buffer,sizeof(buffer),"%d.%dv",float_m, float_n);
    break;

  case 1:
    snprintf(buffer,sizeof(buffer),"%uL", main_data.water_level_liter);
    break;

  case 2:
    float_m = (int)(main_data.outside_temperature * 10);
    float_n = float_m%10;
    float_m = float_m/10;
    snprintf(buffer,sizeof(buffer),"%d.%dC",float_m, float_n);
    break;

  case 3:
    float_m = (int)(main_data.inside_temperature * 10);
    float_n = float_m%10;
    float_m = float_m/10;
    snprintf(buffer,sizeof(buffer),"%d.%dC",float_m, float_n);
    break;

  case 4:
    float_m = (int)(main_data.fridge_temperature * 10);
    float_n = float_m%10;
    float_m = float_m/10;
    snprintf(buffer,sizeof(buffer),"%d.%dC",float_m, float_n);
    break;

  case 5:
    float_m = (int)(main_data.sensors_supply_voltage * 10);
    float_n = float_m%10;
    float_m = float_m/10;
    snprintf(buffer,sizeof(buffer),"%d.%dv",float_m, float_n);
    break;

  case 6:
    if(main_data.res_sensor_resistance <= MAX_RESISTANCE)snprintf(buffer,sizeof(buffer),"%d",main_data.res_sensor_resistance);
    else snprintf(buffer,sizeof(buffer),"xxx");
    break;

  case 7:
    snprintf(buffer,sizeof(buffer),"%1u", main_data.door_switch_state);

    break;

  case 8:
    snprintf(buffer,sizeof(buffer),"%1u", main_data.proximity_sensor_state);
    break;
  case 9:
    snprintf(buffer,sizeof(buffer),"%1u", main_data.ignition_switch_state);
    break;

  case 10:
	snprintf(buffer,sizeof(buffer),"%1u", main_data.converter_output_state);
    break;

  case 11:
    snprintf(buffer,sizeof(buffer),"%1u", main_data.fridge_output_state);
    break;

  case 12:
    snprintf(buffer,sizeof(buffer),"%1u", main_data.pump_output_state);
    break;

  case 13:
	snprintf(buffer,sizeof(buffer),"%1u", main_data.error_code);
    break;

  case 14:

    break;

  default:
    break;
  }

  u8g2_DrawStr(&u8g2,98,(num_line*12),buffer);
}
//*************************************************************************************************************

//Функция вывода меню параметров  ---------------------------------------------
void fnPrintMenuParamView(void){

	u8g2_ClearBuffer(&u8g2);				//
	u8g2_SetFont(&u8g2,u8g2_font_ncenB08_tr);

  for (uint8_t i = 0; i < display_num_lines; i++) {   // Цикл, выводящий пункты на дисплей

    fnPrintMenuItemName(i+(menu_current_page*display_num_lines), i+1, parameters_names); // Выводим название пункта
    fnPrintMenuParamItemVal(i+(menu_current_page*display_num_lines), i+1); // Выводим значение пункта меню уставок
  }

  //рисуем боковой скролл бар
    uint8_t scroll_bar_height = display_height/(MENU_PARAM_VIEW_NUM_ITEMS/display_num_lines);
    u8g2_DrawVLine(&u8g2,127, menu_current_page*scroll_bar_height, scroll_bar_height);

    u8g2_SendBuffer(&u8g2);

}
//*******************************************************************************************************************

//Функция вывода главного экрана -----------------------------------------------
void fnPrintMainView(void){

  char buffer[20] = {0,};
  uint8_t float_m, float_n; // переменные для разбития числа на целую и дробную часть

  u8g2_ClearBuffer(&u8g2);					//


  u8g2_SetFont(&u8g2,u8g2_font_5x7_tr);

  u8g2_DrawBox(&u8g2,98,1,31,8);
  u8g2_DrawBox(&u8g2,98,11,31,8);
  u8g2_DrawBox(&u8g2,98,21,31,8);

  u8g2_SetDrawColor(&u8g2,0);

  float_m = (uint8_t)(main_data.battery_voltage * 10);
  float_n = float_m%10;
  float_m = float_m/10;
  snprintf(buffer,sizeof(buffer),"%d.%dv",float_m, float_n);
  u8g2_DrawStr(&u8g2,102, 8, buffer);

  snprintf(buffer,sizeof(buffer),"> %dC", (int)main_data.inside_temperature);
  u8g2_DrawStr(&u8g2, 98, 18, buffer);

  snprintf(buffer,sizeof(buffer),"< %dC", (int)main_data.outside_temperature);
  u8g2_DrawStr(&u8g2, 98, 28, buffer);

  u8g2_SetDrawColor(&u8g2,1);

  if(main_data.pump_output_state){
	u8g2_DrawBox(&u8g2,64,1,21,8);
	u8g2_SetDrawColor(&u8g2,0);
	u8g2_DrawStr(&u8g2,65, 8, "PUMP");
	u8g2_SetDrawColor(&u8g2,1);
  }
  else{
	u8g2_SetDrawColor(&u8g2,0);
	u8g2_DrawBox(&u8g2,64,1,21,8);
	u8g2_SetDrawColor(&u8g2,0);
  }

  if(main_data.converter_output_state){
	u8g2_DrawBox(&u8g2,64,11,21,8);
	u8g2_SetDrawColor(&u8g2,0);
	u8g2_DrawStr(&u8g2,65, 18, "CONV");
	u8g2_SetDrawColor(&u8g2,1);
  }
  else{
	u8g2_SetDrawColor(&u8g2,0);
	u8g2_DrawBox(&u8g2,64,11,21,8);
	u8g2_SetDrawColor(&u8g2,1);
  }

  if(main_data.fridge_output_state){
	u8g2_DrawBox(&u8g2,64,21,21,8);
	u8g2_SetDrawColor(&u8g2,0);
	u8g2_DrawStr(&u8g2,65, 28, "FRDG");
	u8g2_SetDrawColor(&u8g2,1);
  }
  else{
	u8g2_SetDrawColor(&u8g2,0);
	u8g2_DrawBox(&u8g2,64,21,21,8);
	u8g2_SetDrawColor(&u8g2,1);
  }

  if(main_data.error_code){
	u8g2_DrawBox(&u8g2,1,1,16,8);
	u8g2_SetDrawColor(&u8g2,0);
	u8g2_DrawStr(&u8g2,2, 8, "ERR");
	u8g2_SetDrawColor(&u8g2,1);
  }
  else{
	u8g2_SetDrawColor(&u8g2,0);
	u8g2_DrawBox(&u8g2,1,1,16,8);
	u8g2_SetDrawColor(&u8g2,1);
  }

  if(flag_mb_connected){
	u8g2_DrawBox(&u8g2,19,1,11,8);
	u8g2_SetDrawColor(&u8g2,0);
	u8g2_DrawStr(&u8g2,20, 8, "MB");
	u8g2_SetDrawColor(&u8g2,1);
  }
  else{
	u8g2_SetDrawColor(&u8g2,0);
	u8g2_DrawBox(&u8g2,19,1,16,8);
    u8g2_SetDrawColor(&u8g2,1);
  }

  snprintf(buffer,sizeof(buffer),"%d L",main_data.water_level_liter);
  u8g2_SetFont(&u8g2, u8g2_font_ncenB18_tr);	//
  u8g2_DrawStr(&u8g2,55, 55, buffer);

  W25qxx_ReadBytes(imageBuff, (FLASH_SECTOR_SIZE*2), 1024);
  u8g2_DrawXBM(&u8g2,5, 12, 64, 55, imageBuff);

  u8g2_SendBuffer(&u8g2);
}
//***********************************************************************************************************

//Menu -----------------
void fnMenuProcess(void){

    //определение текущей страницы меню
    if(menu_current_item < display_num_lines) menu_current_page = 0;
    else if(menu_current_item < display_num_lines*2)menu_current_page = 1 ;
    else if(menu_current_item < display_num_lines*3)menu_current_page = 2 ;
    else if(menu_current_item < display_num_lines*4)menu_current_page = 3 ;
    else if(menu_current_item < display_num_lines*5)menu_current_page = 4 ;
    else if(menu_current_item < display_num_lines*6)menu_current_page = 5 ;
    else if(menu_current_item < display_num_lines*7)menu_current_page = 6 ;

    switch (menu_mode)
    {
      case MENU_MAIN_VIEW:

        fnPrintMainView();

        if(btn_state == BTN_ENTER_LONG_PRESS){
          menu_mode = MENU_SETPOINTS;
          menu_current_item = 0;
         // tone(BUZZER,500,200);
        }

        if(btn_state == BTN_ENTER){
          menu_mode = MENU_PARAM_VIEW;
          menu_current_item = 0;
        }
        break;

      case MENU_PARAM_VIEW:

        fnPrintMenuParamView();

        if ((btn_state == BTN_UP) || (btn_state == BTN_UP_LONG_PRESS)) {         // Если кнопку нажали или удерживают
          menu_current_item = constrain(menu_current_item - display_num_lines , 0, MENU_PARAM_VIEW_NUM_ITEMS - 1); // Двигаем указатель в пределах дисплея
         // if(SetpointsUnion.setpoints_data.debug_key== DEBUG_KEY_1)Serial.println(menu_current_item);
        }

        if ((btn_state == BTN_DOWN) || (btn_state == BTN_DOWN_LONG_PRESS)) {
          menu_current_item = constrain(menu_current_item + display_num_lines, 0, MENU_PARAM_VIEW_NUM_ITEMS - 1);
         // if(SetpointsUnion.setpoints_data.debug_key== DEBUG_KEY_1)Serial.println(menu_current_item);
        }

        if(btn_state == BTN_ENTER){
          menu_mode = MENU_LOGO_VIEW;
          menu_current_item = 0;
        }

        break;

      case MENU_SETPOINTS:

        printMenuSetpoints();

        if ((btn_state == BTN_DOWN) || (btn_state == BTN_DOWN_LONG_PRESS)) {         // Если кнопку нажали или удерживают
          menu_current_item = constrain(menu_current_item + 1, 0, MENU_SETPOINTS_NUM_ITEMS - 1); // Двигаем указатель в пределах дисплея
          //Serial.println(menu_current_item);
        }

        if ((btn_state == BTN_UP) || (btn_state == BTN_UP_LONG_PRESS)) {
          menu_current_item = constrain(menu_current_item - 1, 0, MENU_SETPOINTS_NUM_ITEMS - 1);
          //if(SetpointsUnion.setpoints_data.debug_key== DEBUG_KEY_1)Serial.println(menu_current_item);
        }

        if(btn_state == BTN_ENTER)menu_mode = MENU_SETPOINTS_EDIT_MODE;

        if(btn_state == BTN_ESC){
          menu_mode = MENU_MAIN_VIEW;
          menu_current_item = 0;
          //tone(BUZZER,500,200);
        }

        break;

      case MENU_SETPOINTS_EDIT_MODE:

        printMenuSetpoints();

        if ((btn_state == BTN_UP) || (btn_state == BTN_UP_LONG_PRESS)){
          SetpointsUnion.SetpointsArray[menu_current_item] = constrain(SetpointsUnion.SetpointsArray[menu_current_item]+1,param_range_min[menu_current_item],param_range_max[menu_current_item]);
         // Serial.println(SetpointsUnion.SetpointsArray[menu_current_item]);
        }

        if ((btn_state == BTN_DOWN) || (btn_state == BTN_DOWN_LONG_PRESS)){
          SetpointsUnion.SetpointsArray[menu_current_item] = constrain(SetpointsUnion.SetpointsArray[menu_current_item]-1,param_range_min[menu_current_item],param_range_max[menu_current_item]);
         // if(SetpointsUnion.setpoints_data.debug_key== DEBUG_KEY_1)Serial.println(SetpointsUnion.SetpointsArray[menu_current_item]);
        }

        if(btn_state == BTN_ENTER){
          //выход с сохранением в flash
          //tone(BUZZER,500,200);
          menu_mode = MENU_SETPOINTS;
        }

        if(btn_state == BTN_ESC){
		  //выход без сохранения
		  menu_mode = MENU_SETPOINTS;
		}

        break;

      case MENU_LOGO_VIEW:

        switch (SetpointsUnion.setpoints_data.logo)
        {
        case 0:
          u8g2_ClearBuffer(&u8g2);
          W25qxx_ReadBytes(imageBuff, IMAGE_LOGO_FK, 1024);
          u8g2_DrawXBM(&u8g2,33,5, 64, 55, imageBuff);
          u8g2_SendBuffer(&u8g2);
          break;

        case 1:
          u8g2_ClearBuffer(&u8g2);
          //
          u8g2_SendBuffer(&u8g2);
          break;

        case 2:
          u8g2_ClearBuffer(&u8g2);
          //
          u8g2_SendBuffer(&u8g2);
          break;

		default:
			u8g2_ClearBuffer(&u8g2);
			//пусто
			u8g2_SendBuffer(&u8g2);
			break;
        }


        if(btn_state == BTN_ENTER){
          menu_mode = MENU_MAIN_VIEW;
          menu_current_item = 0;
        }

        break;

      default:

      break;

    }
  //end menu
}

//***************************************************************************************************************

//
static uint16_t fnGetPressKey(void){

 	static uint16_t key_pressed;

 	 //считываем состояние кнопок и заносим в массив
 		  btnStatesArray[BUTTON_UP] = Button_Get_Clicked_Count(&Button_A);  //
 		  btnStatesArray[BUTTON_DOWN] = Button_Get_Clicked_Count(&Button_B);
 		  btnStatesArray[BUTTON_ENTER] = Button_Get_Clicked_Count(&Button_C);
 		  btnStatesArray[BUTTON_ESC] = Button_Get_Clicked_Count(&Button_D);


 	if(btnStatesArray[BUTTON_UP] == 1)key_pressed |= BTN_UP;
 		else key_pressed &= ~BTN_UP;

 	if(btnStatesArray[BUTTON_DOWN] == 1) key_pressed |= BTN_DOWN;    //
 		else key_pressed &= ~BTN_DOWN;

 	if(Button_Get_Status(&Button_A) == Button_Long_Pressed)key_pressed |= BTN_UP_LONG_PRESS;
 	 	else key_pressed &= ~BTN_UP_LONG_PRESS;

	if(Button_Get_Status(&Button_B) == Button_Long_Pressed)key_pressed |= BTN_DOWN_LONG_PRESS; //
		else key_pressed &= ~BTN_DOWN_LONG_PRESS;

 	if(btnStatesArray[BUTTON_ENTER] == 1) key_pressed |= BTN_ENTER;    //
 		else key_pressed &= ~BTN_ENTER;

 	if(btnStatesArray[BUTTON_ESC] == 1) key_pressed |= BTN_ESC;  //
 		else key_pressed &= ~BTN_ESC;

 	if(btnStatesArray[BUTTON_ENTER] == 255) key_pressed |= BTN_ENTER_LONG_PRESS;  //
 		else key_pressed &= ~BTN_ENTER_LONG_PRESS;

 	if(btnStatesArray[BUTTON_ESC] == 255) key_pressed |= BTN_ESC_LONG_PRESS;  //
 		else key_pressed &= ~BTN_ESC_LONG_PRESS;

 	return key_pressed;
 }

//*********************************************************************************

//
uint32_t Button_Get_Tick(void)
{
    return HAL_GetTick();
}
//**********************************************************************************

//
uint8_t Button_A_Read(void)
{
	return HAL_GPIO_ReadPin(BUTTON_UP_GPIO_Port, BUTTON_UP_Pin) ? 0 : 1;
}
//**********************************************************************************

//
uint8_t Button_B_Read(void)
{
	return HAL_GPIO_ReadPin(BUTTON_DOWN_GPIO_Port, BUTTON_DOWN_Pin) ? 0 : 1;
}
//*********************************************************************************

//
uint8_t Button_C_Read(void)
{
	return HAL_GPIO_ReadPin(BUTTON_ENTER_GPIO_Port, BUTTON_ENTER_Pin) ? 0 : 1;
}
//****************************************************************************************

//
uint8_t Button_D_Read(void)
{
	return HAL_GPIO_ReadPin(BUTTON_ESC_GPIO_Port, BUTTON_ESC_Pin) ? 0 : 1;
}
//***********************************************************************************
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

