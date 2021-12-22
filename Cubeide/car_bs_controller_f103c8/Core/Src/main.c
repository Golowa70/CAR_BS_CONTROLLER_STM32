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
//#include "stdint.h".h"
#include "button.h"
#include "mini-printf.h"
#include "OneWire.h"
#include "TIMERS.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
u8g2_t u8g2;

//*********** Setpoints variables *********************************************************************

struct SetpointsStruct { // структура для уставок

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
  uint8_t lcd_light_T_off;
  uint8_t logo;
  uint8_t inside_sensor_ID;
  uint8_t outside_sensor_ID;
  uint8_t fridge_sensor_ID;

  uint16_t magic_key;
  uint16_t crc;

}default_setpoints_data;


//union
union {

  struct SetpointsStruct setpoints_data;
  uint8_t SetpointsArray[MENU_SETPOINTS_NUM_ITEMS+4]; // (+4) - два байта magic key + два байта контрольной суммы

}SetpointsUnion; //

//********** end setpoints variables ******************************************************************

//*********** Main data *******************************************************************************
struct MyData
{
  float outside_temperature;      	//  наружная температура
  float inside_temperature;       	// температура внутри
  float fridge_temperature;        	// температура третьего датчика(пока не используется)
  float battery_voltage;          // напряжени бортсети ( например 124 это 12.4в)
  float sensors_supply_voltage;   // напряжение питания датчиков 5в
  uint16_t res_sensor_resistance;   // сопротивление резистивного датчика
  uint8_t water_level_liter;      	// уровень воды в литрах
  uint8_t error_code;
  uint8_t btn_state;

  bool door_switch_state;         	// состояние концевика задней двери
  bool proximity_sensor_state;    	// состояние датчика приближения
  bool ignition_switch_state;     	// состояние входа зажигания
  bool converter_output_state;    	// состояние выхода управления инвертором 12/220в
  bool fridge_output_state;       	//состояние выхода освещения
  bool pump_output_state;         	//состояние выхода насоса
  bool sensors_supply_output_state; // состояние выхода управления питанием сенсоров 5в
  bool main_supply_output_state;  	// состояние выхода управления общим питанием 7.5в
  bool lcd_light_output_state;
  bool flag_system_started;
  bool flag_setpoints_read_success;

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
	const char i20[] = "Light T off";
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
	PUMP_T_OFF_MIN,
	PUMP_OUT_MODE_MIN,
	CONV_U_OFF_MIN,
	CONV_T_U_OFF_MIN,
	CONV_U_ON_MIN,
	CONV_T_IGN_OFF_MIN,
	CONV_OUT_MODE_MIN,
	FRIDGE_U_OFF_MIN,
	FRIDGE_T_U_OFF_MIN,
	FRIDGE_U_ON_MIN,
	FRIDGE_T_IGN_OFF_MIN,
	FRIDGE_TEMP_ON_MIN,
	FRIDGE_TEMP_OFF_MIN,
	FRIDGE_OUT_MODE_MIN,
	WATER_SENS_MIN_MIN,
	WATER_SENS_MAX_MIN,
	WATER_SENS_CORR_MIN,
	WATER_TANK_CAP_MIN,
	SHUTDOWN_DELAY_MIN,
	U_CORR_MIN,
	LIGHT_T_OFF_MIN,
	LOGO_MIN,
	INSIDE_SENS_ID_MIN,
	OUTSIDE_SENS_ID_MIN,
	FRIDGE_SENS_ID_MIN
  };

  //Массив максимальных значений параметров
  uint8_t param_range_max[MENU_SETPOINTS_NUM_ITEMS]  =
  {
	PUMP_T_OFF_MAX,
	PUMP_OUT_MODE_MAX,
	CONV_U_OFF_MAX,
	CONV_T_U_OFF_MAX,
	CONV_U_ON_MAX,
	CONV_T_IGN_OFF_MAX,
	CONV_OUT_MODE_MAX,
	FRIDGE_U_OFF_MAX,
	FRIDGE_T_U_OFF_MAX,
	FRIDGE_U_ON_MAX,
	FRIDGE_T_IGN_OFF_MAX,
	FRIDGE_TEMP_ON_MAX,
	FRIDGE_TEMP_OFF_MAX,
	FRIDGE_OUT_MODE_MAX,
	WATER_SENS_MIN_MAX,
	WATER_SENS_MAX_MAX,
	WATER_SENS_CORR_MAX,
	WATER_TANK_CAP_MAX,
	SHUTDOWN_DELAY_MAX,
	U_CORR_MAX,
	LIGHT_T_OFF_MAX,
	LOGO_MAX,
	INSIDE_SENS_ID_MAX,
	OUTSIDE_SENS_ID_MAX,
	FRIDGE_SENS_ID_MAX

  };

  uint8_t menu_current_item = 0; // Переменная указатель на текущий пункт меню
  uint8_t menu_current_page = 0;

  enum Menus{
	  MENU_MAIN_VIEW,MENU_PARAM_VIEW,MENU_SETPOINTS,MENU_SETPOINTS_EDIT_MODE,MENU_LOGO_VIEW, MENU_MESSAGE_VIEW, MENUS_MAX
  };

  enum Menus menu_mode = MENU_MAIN_VIEW;


  //****** DISPLAY VARIABLES ********************************************************
  uint8_t display_height;
  uint8_t display_width;
  uint8_t display_num_lines;


 //******** BUTTONS ******************************************************
  enum Buttons{
  	BUTTON_UP,BUTTON_DOWN,BUTTON_ENTER,BUTTON_ESC,MAX_BUTTONS
  };


   uint8_t btnStatesArray[MAX_BUTTONS] = {0,};
   Button_Struct_t Button_A;
   Button_Struct_t Button_B;
   Button_Struct_t Button_C;
   Button_Struct_t Button_D;

  //********* ADC ****************************************************************
   volatile uint16_t adc_source_value[ADC_CHANELS] = {0,};
   volatile bool flag_adc_complet = false;


  //********* OTHER VARIABLES *********************************************************
	uint8_t main_process_step = 0;
	uint32_t  time_b = 0;
	bool flag_blink = false; // флаг для мигания чего либо на экране
	uint8_t imageBuff[IMAGE_BUFFER_SIZE] = {0,};
	bool flag_mb_connected = 0;
	uint32_t sys_timer = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
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

//ADC
uint16_t fnEmaFilterBatVolt(uint16_t new_value);
uint16_t fnEmaFilterSensVolt(uint16_t new_value);
uint16_t fnEmaFilterResSens(uint16_t new_value);

uint8_t map(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max);

//
void fnPumpControl(struct MyData *data, struct SetpointsStruct *setpoints);
void fnWaterLevelControl(struct MyData *data, struct SetpointsStruct *setpoints);
void fnConverterControl(struct MyData *data, struct SetpointsStruct *setpoints);
void fnFridgeControl(struct MyData *data, struct SetpointsStruct *setpoints);
void fnMainPowerControl(struct MyData *data, struct SetpointsStruct *setpoints);
void fnInputsUpdate(void);
uint8_t fnDebounce(uint8_t sample);
void fnOutputsUpdate(struct MyData *data);
void fnDisplayLightControl(struct MyData *data, struct SetpointsStruct *setpoints);
uint16_t fnCrc16Calc(uint8_t *Data, uint8_t count);
void fnCheckSavedSetpoints(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern float Temp[MAXDEVICES_ON_THE_BUS];
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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

//DEFAULT SETPOINTS
  default_setpoints_data.pump_T_off = 5;
  default_setpoints_data.pump_out_mode = AUTO_MODE;
  default_setpoints_data.conv_U_off = 115;
  default_setpoints_data.conv_T_U_off = 5;
  default_setpoints_data.conv_U_on = 128;
  default_setpoints_data.conv_T_IGN_off = 120;
  default_setpoints_data.conv_out_mode = AUTO_MODE;
  default_setpoints_data.fridge_U_off = 110;
  default_setpoints_data.fridge_T_U_off = 3;
  default_setpoints_data.fridge_U_on = 130;
  default_setpoints_data.fridge_T_IGN_off = 60;
  default_setpoints_data.fridge_Temp_on = 12;
  default_setpoints_data.fridge_Temp_off = 7;
  default_setpoints_data.fridge_out_mode = AUTO_MODE;
  default_setpoints_data.water_sens_min = 0;
  default_setpoints_data.water_sens_max = 190;
  default_setpoints_data.water_tank_capacity = 100;
  default_setpoints_data.shutdown_delay = 4;
  default_setpoints_data.U_correction = 127;
  default_setpoints_data.lcd_light_T_off = 5;
  default_setpoints_data.logo = 0;
  default_setpoints_data.inside_sensor_ID = 1;
  default_setpoints_data.outside_sensor_ID = 2;
  default_setpoints_data.inside_sensor_ID = 3;
  default_setpoints_data.magic_key = MAGIC_KEY;

//FLASH INIT
  	  W25qxx_Init();
	//W25qxx_EraseChip();
	//W25qxx_EraseBlock(0); // 65536 байт
	//W25qxx_EraseSector(0); // 4096 байт

//ADC INIT
  	HAL_ADCEx_Calibration_Start(&hadc1);
  	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_source_value, ADC_CHANELS	); // DMA считывает ADC_CHANELS значений и инициирует прерывание


//DISPLAY INIT
	u8g2_Setup_st7565_nhd_c12864_f(&u8g2, U8G2_R2, u8x8_byte_4wire_hw_spi,
			u8g2_gpio_and_delay_stm32);
	u8g2_InitDisplay(&u8g2); 	 // send init sequence to the display, display is in sleep mode after this
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	u8g2_SetContrast(&u8g2, 250);
	u8g2_ClearDisplay(&u8g2);
	//u8g2_SetFont(&u8g2, u8g2_font_courB18_tr);
	u8g2_ClearBuffer(&u8g2);
	W25qxx_ReadBytes(imageBuff, IMAGE_LOGO_3, 1024);
	u8g2_DrawXBM(&u8g2,33,5, 64, 55, imageBuff);
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

//READ SETPOINTS FROM FLASH W25Q
	fnCheckSavedSetpoints();

//ONE WIRE INIT
	get_ROMid();

//TIMERS INIT
	InitGTimers();
	//StartGTimer(TIMER_CONV_U_OFF);
	StartGTimer(TIMER_PRX_SENS_FEEDBACK);
	StartGTimer(TIMER_TEMP_SENS_UPDATE);

//GET TEMPERATURE
	get_Temperature();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //blink
	  if((HAL_GetTick() - time_b) > BLINK_INTERVAL) // интервал 500мс
	   {
		  flag_blink = !flag_blink;
		  time_b = HAL_GetTick();
	   }


	  fnMenuProcess();
	  main_data.btn_state = fnGetPressKey();// опрос кнопок
	  ProcessTimers(&sys_timer); //

	  fnInputsUpdate();
	  fnPumpControl(&main_data, &SetpointsUnion.setpoints_data);
	  fnWaterLevelControl(&main_data, &SetpointsUnion.setpoints_data);
	  fnConverterControl(&main_data, &SetpointsUnion.setpoints_data);
	  fnFridgeControl(&main_data, &SetpointsUnion.setpoints_data);
	  fnMainPowerControl(&main_data, &SetpointsUnion.setpoints_data);
	  fnDisplayLightControl(&main_data, &SetpointsUnion.setpoints_data);
	  fnOutputsUpdate(&main_data);

	  if (GetGTimer(TIMER_TEMP_SENS_UPDATE) >= TEMP_SENS_UPDATE_PERIOD) {
		//  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  get_Temperature();
		  main_data.inside_temperature = Temp[0];
		  main_data.outside_temperature = Temp[1];
		  main_data.fridge_temperature = Temp[2];
		  StartGTimer(TIMER_TEMP_SENS_UPDATE);
	  }

//---------------------------------------------------------------------------------------
	  if (flag_adc_complet == true) // если сработало прерывание от DMA АЦП
	  		{
	  			flag_adc_complet = false;
	  			main_data.battery_voltage = fnEmaFilterBatVolt(adc_source_value[0]) * ADC_REFERENCE * ADC_BAT_VOLT_DIVIDER; 		// значение напряжения после фильтрации
	  			main_data.sensors_supply_voltage = (fnEmaFilterSensVolt(adc_source_value[1]) * ADC_REFERENCE);
	  			main_data.res_sensor_resistance = (fnEmaFilterResSens(adc_source_value[2]) / ADC_RES_SENS_DIVIDER);

	  			for (uint8_t i = 0; i < ADC_CHANELS; i++) {	// обнуляем массив со значениями от АЦП
	  				adc_source_value[i] = 0;
	  			}

	  			HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adc_source_value, ADC_CHANELS); // перезапускаем DMA
	  		}

//------------------------------------------------------------------------------------------------------------
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, W25Q_CS_Pin|LCD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RESET_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_DC_Pin|SENS_SUPPLY_Pin|BUZZER_Pin|CONV_OUTPUT_Pin
                          |PUMP_OUTPUT_Pin|MAIN_SUPPLY_Pin|FRIDGE_OUTPUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : W25Q_CS_Pin LCD_LED_Pin */
  GPIO_InitStruct.Pin = W25Q_CS_Pin|LCD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin LCD_RESET_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LCD_RESET_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DC_Pin SENS_SUPPLY_Pin BUZZER_Pin CONV_OUTPUT_Pin
                           PUMP_OUTPUT_Pin MAIN_SUPPLY_Pin FRIDGE_OUTPUT_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin|SENS_SUPPLY_Pin|BUZZER_Pin|CONV_OUTPUT_Pin
                          |PUMP_OUTPUT_Pin|MAIN_SUPPLY_Pin|FRIDGE_OUTPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DOOR_INPUT_Pin PRX_SENS_INPUT_Pin BUTTON_ENTER_Pin BUTTON_ESC_Pin
                           BUTTON_UP_Pin */
  GPIO_InitStruct.Pin = DOOR_INPUT_Pin|PRX_SENS_INPUT_Pin|BUTTON_ENTER_Pin|BUTTON_ESC_Pin
                          |BUTTON_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IGN_INPUT_Pin */
  GPIO_InitStruct.Pin = IGN_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(IGN_INPUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_DOWN_Pin */
  GPIO_InitStruct.Pin = BUTTON_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_DOWN_GPIO_Port, &GPIO_InitStruct);

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
    if(main_data.res_sensor_resistance <= WATER_SENS_MAX_MAX)snprintf(buffer,sizeof(buffer),"%d",main_data.res_sensor_resistance);
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
  uint16_t float_m, float_n; // переменные для разбития числа на целую и дробную часть

  u8g2_ClearBuffer(&u8g2);					//
  u8g2_SetFont(&u8g2,u8g2_font_5x7_tr);

  u8g2_DrawBox(&u8g2,98,1,31,8);
  u8g2_DrawBox(&u8g2,98,11,31,8);
  u8g2_DrawBox(&u8g2,98,21,31,8);
  u8g2_DrawBox(&u8g2,98,31,31,8);

  u8g2_SetDrawColor(&u8g2,0);

  float_m = (uint16_t)(main_data.battery_voltage * 10);
  float_n = float_m%10;
  float_m = float_m/10;
  snprintf(buffer,sizeof(buffer),"%d.%dv",float_m, float_n);
  u8g2_DrawStr(&u8g2,102, 8, buffer);

  snprintf(buffer,sizeof(buffer),"> %dC", (int)main_data.inside_temperature);
  u8g2_DrawStr(&u8g2, 98, 18, buffer);

  snprintf(buffer,sizeof(buffer),"< %dC", (int)main_data.outside_temperature);
  u8g2_DrawStr(&u8g2, 98, 28, buffer);

  snprintf(buffer,sizeof(buffer),"f %dC", (int)main_data.fridge_temperature);
  u8g2_DrawStr(&u8g2, 98, 38, buffer);

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
	u8g2_SetDrawColor(&u8g2,1);
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

  snprintf(buffer,sizeof(buffer),"%d ",main_data.water_level_liter);
  u8g2_SetFont(&u8g2, u8g2_font_fub20_tn);
  u8g2_DrawStr(&u8g2,55, 55, buffer);

  W25qxx_ReadBytes(imageBuff, (IMAGE_WATER_LEVEL), 1024);
  u8g2_DrawXBM(&u8g2,5, 12, 50, 50, imageBuff);

  u8g2_SendBuffer(&u8g2);
}
//***********************************************************************************************************

//Menu -----------------
void fnMenuProcess(void){
	
	enum Messages{NO_MESSAGES,VALUE_SAVED, VALUE_NOT_SAVED};
	static enum Messages message = NO_MESSAGES;

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

        if(main_data.btn_state == BTN_ENTER_LONG_PRESS){
          menu_mode = MENU_SETPOINTS;
          menu_current_item = 0;
         // tone(BUZZER,500,200);
        }

        if(main_data.btn_state == BTN_ENTER){
          menu_mode = MENU_PARAM_VIEW;
          menu_current_item = 0;
        }
        break;

      case MENU_PARAM_VIEW:

        fnPrintMenuParamView();

        if ((main_data.btn_state == BTN_UP) || (main_data.btn_state == BTN_UP_LONG_PRESS)) {         // Если кнопку нажали или удерживают
          menu_current_item = constrain(menu_current_item - display_num_lines , 0, MENU_PARAM_VIEW_NUM_ITEMS - 1); // Двигаем указатель в пределах дисплея
         // if(SetpointsUnion.setpoints_data.debug_key== DEBUG_KEY_1)Serial.println(menu_current_item);
        }

        if ((main_data.btn_state == BTN_DOWN) || (main_data.btn_state == BTN_DOWN_LONG_PRESS)) {
          menu_current_item = constrain(menu_current_item + display_num_lines, 0, MENU_PARAM_VIEW_NUM_ITEMS - 1);
         // if(SetpointsUnion.setpoints_data.debug_key== DEBUG_KEY_1)Serial.println(menu_current_item);
        }

        if(main_data.btn_state == BTN_ENTER){
          menu_mode = MENU_LOGO_VIEW;
          menu_current_item = 0;
        }

        break;

      case MENU_SETPOINTS:

        printMenuSetpoints();

        if ((main_data.btn_state == BTN_DOWN) || (main_data.btn_state == BTN_DOWN_LONG_PRESS)) {         // Если кнопку нажали или удерживают
          menu_current_item = constrain(menu_current_item + 1, 0, MENU_SETPOINTS_NUM_ITEMS - 1); // Двигаем указатель в пределах дисплея
          //Serial.println(menu_current_item);
        }

        if ((main_data.btn_state == BTN_UP) || (main_data.btn_state == BTN_UP_LONG_PRESS)) {
          menu_current_item = constrain(menu_current_item - 1, 0, MENU_SETPOINTS_NUM_ITEMS - 1);
          //if(SetpointsUnion.setpoints_data.debug_key== DEBUG_KEY_1)Serial.println(menu_current_item);
        }

        if(main_data.btn_state == BTN_ENTER)menu_mode = MENU_SETPOINTS_EDIT_MODE;

        if(main_data.btn_state == BTN_ESC){
          menu_mode = MENU_MAIN_VIEW;
          menu_current_item = 0;
          //tone(BUZZER,500,200);
        }

        break;

      case MENU_SETPOINTS_EDIT_MODE:

        printMenuSetpoints();

        if ((main_data.btn_state == BTN_UP) || (main_data.btn_state == BTN_UP_LONG_PRESS)){
          SetpointsUnion.SetpointsArray[menu_current_item] = constrain(SetpointsUnion.SetpointsArray[menu_current_item]+1,param_range_min[menu_current_item],param_range_max[menu_current_item]);
         // Serial.println(SetpointsUnion.SetpointsArray[menu_current_item]);
        }

        if ((main_data.btn_state == BTN_DOWN) || (main_data.btn_state == BTN_DOWN_LONG_PRESS)){
          SetpointsUnion.SetpointsArray[menu_current_item] = constrain(SetpointsUnion.SetpointsArray[menu_current_item]-1,param_range_min[menu_current_item],param_range_max[menu_current_item]);
         // if(SetpointsUnion.setpoints_data.debug_key== DEBUG_KEY_1)Serial.println(SetpointsUnion.SetpointsArray[menu_current_item]);
        }

        if(main_data.btn_state == BTN_ENTER){
          //выход с сохранением в flash
        	SetpointsUnion.setpoints_data.crc = fnCrc16Calc((uint8_t*)&SetpointsUnion.SetpointsArray, sizeof(SetpointsUnion.SetpointsArray) - 2);
        	W25qxx_EraseSector(SETPOINTS_FLASH_SECTOR);
        	bool flag_empty = false;
        	flag_empty = W25qxx_IsEmptySector(SETPOINTS_FLASH_SECTOR, 0,MENU_SETPOINTS_NUM_ITEMS);
        	if(flag_empty){
        		W25qxx_WriteSector(SetpointsUnion.SetpointsArray, SETPOINTS_FLASH_SECTOR, 0, sizeof(SetpointsUnion.SetpointsArray));
        		flag_empty = false;
        	}

			uint8_t saved_item_value = 0;
			W25qxx_ReadByte(&saved_item_value, (FLASH_SECTOR_SIZE * SETPOINTS_FLASH_SECTOR) + menu_current_item);
			if(saved_item_value == SetpointsUnion.SetpointsArray[menu_current_item]){
				message = VALUE_SAVED;
				StartGTimer(TIMER_MESSAGE_VIEW);
			}
			else{
				message = VALUE_NOT_SAVED;
				StartGTimer(TIMER_MESSAGE_VIEW);
			}
          
          menu_mode = MENU_MESSAGE_VIEW;
        }

        if(main_data.btn_state == BTN_ESC){
		  //выход без сохранения
		  menu_mode = MENU_SETPOINTS;
		}

        break;

	case MENU_LOGO_VIEW:

		switch (SetpointsUnion.setpoints_data.logo) {
		case 0:
			u8g2_ClearBuffer(&u8g2);
			W25qxx_ReadBytes(imageBuff, IMAGE_LOGO_FK, 1024);
			u8g2_DrawXBM(&u8g2, 33, 5, 64, 55, imageBuff);
			u8g2_SendBuffer(&u8g2);
			break;

		case 1:
			u8g2_ClearBuffer(&u8g2);
			W25qxx_ReadBytes(imageBuff, IMAGE_LOGO_2, 1024);
			u8g2_DrawXBM(&u8g2, 33, 5, 64, 55, imageBuff);
			u8g2_SendBuffer(&u8g2);
			break;

		case 2:
			u8g2_ClearBuffer(&u8g2);
			W25qxx_ReadBytes(imageBuff, IMAGE_LOGO_3, 1024);
			u8g2_DrawXBM(&u8g2, 33, 5, 64, 55, imageBuff);
			u8g2_SendBuffer(&u8g2);
			break;

		default:
			u8g2_ClearBuffer(&u8g2);
			//пусто
			u8g2_SendBuffer(&u8g2);
			break;
		}


        if(main_data.btn_state == BTN_ENTER){
          menu_mode = MENU_MAIN_VIEW;
          menu_current_item = 0;
        }

        break;
		
	  case MENU_MESSAGE_VIEW:

		switch (message) {

		case VALUE_SAVED:
			u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 40, 30, "SAVED!");
			u8g2_SendBuffer(&u8g2);
			//buzzer
			break;

		case VALUE_NOT_SAVED:
			u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 40, 30, "NOT SAVED!");
			u8g2_SendBuffer(&u8g2);
			//buzzer
			break;

		case NO_MESSAGES:

			break;

		default:
			break;
		}


		if(GetGTimer(TIMER_MESSAGE_VIEW) > MESSAGE_VIEW_TIME){
			StopGTimer(TIMER_MESSAGE_VIEW);
			message = NO_MESSAGES;
			menu_mode = MENU_SETPOINTS;
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

 	static uint8_t key_pressed;

 	//считываем состояние кнопок и заносим в массив количество нажатий, 1-один клик, 255-удержание
 	//(несколько нажатий пока не востребовано)
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

uint8_t Button_B_Read(void)
{
	return HAL_GPIO_ReadPin(BUTTON_DOWN_GPIO_Port, BUTTON_DOWN_Pin) ? 0 : 1;
}

uint8_t Button_C_Read(void)
{
	return HAL_GPIO_ReadPin(BUTTON_ENTER_GPIO_Port, BUTTON_ENTER_Pin) ? 0 : 1;
}

uint8_t Button_D_Read(void)
{
	return HAL_GPIO_ReadPin(BUTTON_ESC_GPIO_Port, BUTTON_ESC_Pin) ? 0 : 1;
}
//***********************************************************************************

//
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		HAL_ADC_Stop(&hadc1);
		flag_adc_complet = true;
	}
}
//************************************************************************************

//фильтр экспоненциальное бегущее среднее для напряжения аккумулятора -----------------------------------
uint16_t fnEmaFilterBatVolt(uint16_t new_value)
{
	static float k = EMA_FILTER_K;
	static float filtered_value = 0;

	filtered_value += ((float)new_value - filtered_value) * k;
	return  filtered_value;
}

//фильтр экспоненциальное бегущее среднее для напряжения питания сенсоров ----
uint16_t fnEmaFilterSensVolt(uint16_t new_value)
{
	static float k = EMA_FILTER_K;
	static float filtered_value = 0;

	filtered_value += ((float)new_value - filtered_value) * k;
	return  filtered_value;
}

//фильтр экспоненциальное бегущее среднее для измерения сопротивления ------
uint16_t fnEmaFilterResSens(uint16_t new_value)
{
	static float k = EMA_FILTER_K;
	static float filtered_value = 0;

	filtered_value += ((float)new_value - filtered_value) * k;
	return  filtered_value;
}
//*************************************************************************************

//
void fnPumpControl(struct MyData *data, struct SetpointsStruct *setpoints){

  bool prx_trigged = false;
  static bool prx_old_state = false;
  enum fsm_state {off, wait_for_prx,wait_for_T_off};
  static enum fsm_state step = off;

  if((data->proximity_sensor_state == true) && (prx_old_state == false) && (GetGTimer(TIMER_PRX_SENS_FEEDBACK) >= PRX_SENSOR_FEEDBACK_DELAY)){


    StartGTimer(TIMER_PRX_SENS_FEEDBACK);
    prx_trigged = true;
  }

  prx_old_state = data->proximity_sensor_state;


  switch (setpoints->pump_out_mode){

    case OFF_MODE:
      data->pump_output_state = false;
      break;

    case ON_MODE:
      if(!data->door_switch_state)step = off;
      else data->pump_output_state = true;
      break;

    case AUTO_MODE:

      switch (step)
      {
      case off:
        data->pump_output_state = false;
        StopGTimer(TIMER_PUMP_OFF);
        if(data->door_switch_state)step = wait_for_prx;
        else if(!data->door_switch_state)step = off;

        break;

      case wait_for_prx:
        if(prx_trigged){
          StartGTimer(TIMER_PUMP_OFF);
          data->pump_output_state = true;
          step = wait_for_T_off;
        }
        if(!data->door_switch_state)step = off;
        break;

      case wait_for_T_off:
        if(!data->door_switch_state || prx_trigged || (GetGTimer(TIMER_PUMP_OFF) >= setpoints->pump_T_off * 1000 ))step = off;
        break;

      default:
      break;
      }

      break;

    default:
    break;
  }

}
//************************************************************************************

//
uint8_t map(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max)
{
	uint8_t devider = (in_max - in_min);

	if(devider>0){
		return (x - in_min) * (out_max - out_min) / devider + out_min;
	}
	else {
		return 0;
	}
}
//*************************************************************************************

//
// fnWaterLevelControl
void fnWaterLevelControl(struct MyData *data, struct SetpointsStruct *setpoints)
{
	data->water_level_liter = map(data->res_sensor_resistance,setpoints->water_sens_min,setpoints->water_sens_max,0,setpoints->water_tank_capacity);
}
//*******************************************************************************************

//convreter control
void fnConverterControl(struct MyData *data, struct SetpointsStruct *setpoints)
{

  uint16_t voltage = (uint16_t)data->battery_voltage * 10;
  static bool flag_convOff_due_voltage;    // флаг что конветер был выключен по напряжению
  static bool flag_convOff_due_ign_switch; // флаг что конветер был выключен по таймеру после выключения зажигания

  static bool state = true; // изначально (после старта) включен

  switch (setpoints->conv_out_mode){

    case OFF_MODE:
      state = false;
      StopGTimer(TIMER_CONV_U_OFF);
      StopGTimer(TIMER_CONV_IGN_OFF);
      break;

    case ON_MODE:
      state = true;
      StopGTimer(TIMER_CONV_U_OFF);
      StopGTimer(TIMER_CONV_IGN_OFF);
      break;

    case AUTO_MODE:

      if (voltage >= setpoints->conv_U_on)
      {
        if (!flag_convOff_due_ign_switch)state = true; // если напряжение в пределах нормы включаем преобразователь
        flag_convOff_due_voltage = false;    // флаг что было отключение по низкому напряжению
        StopGTimer(TIMER_CONV_U_OFF); // останавливваем таймер выключения
      }

      if (voltage > setpoints->conv_U_on)
      {
    	StartGTimer(TIMER_CONV_U_OFF); // заряжаем таймер на выключение
      }

      else
      {
        if (GetGTimer(TIMER_CONV_U_OFF) > (setpoints->conv_T_U_off * SEC))
        {
          state = false;
          flag_convOff_due_voltage = true;   // флаг что было отключение по низкому напряжению
          StopGTimer(TIMER_CONV_U_OFF); // останавливаем таймер выключения
        }
      }

      // отключение по таймеру после выключения зажигания
      if (data->ignition_switch_state)
      {
        flag_convOff_due_ign_switch = false; //сброс флага что было отключение по ignition switch
        if (!flag_convOff_due_voltage)state = true;
        StartGTimer(TIMER_CONV_IGN_OFF);
      }
      else
      {
        if (GetGTimer(TIMER_CONV_IGN_OFF) > (setpoints->conv_T_IGN_off * MIN))
        {
          state = false;
          flag_convOff_due_ign_switch = true; //установка флага что было отключение по ignition switch
        }
      }

      break;

    default:
    break;
  }

  data->converter_output_state = state;
}
//*****************************************************************************************

//FRIDGE RELAY CONTROL
void fnFridgeControl(struct MyData *data, struct SetpointsStruct *setpoints)
{

  uint16_t voltage = (uint16_t)data->battery_voltage * 10;
  static bool flag_fridgeOff_due_voltage;    // флаг что конветер был выключен по напряжению
  static bool flag_fridgeOff_due_ign_switch; // флаг что конветер был выключен по таймеру после выключения зажигания

  static bool state = true; // изначально (после старта) включен

  switch (setpoints->fridge_out_mode){

    case OFF_MODE:
      state = false;
      StopGTimer(TIMER_FRIDGE_U_OFF);
      StopGTimer(TIMER_FRIDGE_IGN_OFF);
      break;

    case ON_MODE:
      state = true;
      StopGTimer(TIMER_FRIDGE_U_OFF);
      StopGTimer(TIMER_FRIDGE_IGN_OFF);
      break;

    case AUTO_MODE:

      if (voltage >= setpoints->fridge_U_on)
      {
        if (!flag_fridgeOff_due_ign_switch)state = true; // если напряжение в пределах нормы включаем преобразователь
        flag_fridgeOff_due_voltage = false;    // флаг что было отключение по низкому напряжению
        StopGTimer(TIMER_FRIDGE_U_OFF); // останавливваем таймер выключения
      }

      if (voltage > setpoints->fridge_U_on)
      {
    	StartGTimer(TIMER_FRIDGE_U_OFF); // заряжаем таймер на выключение
      }

      else
      {
        if (GetGTimer(TIMER_FRIDGE_U_OFF) > (setpoints->fridge_T_U_off * SEC))
        {
          state = false;
          flag_fridgeOff_due_voltage = true;   // флаг что было отключение по низкому напряжению
          StopGTimer(TIMER_FRIDGE_U_OFF); // останавливаем таймер выключения
        }
      }

      // отключение по таймеру после выключения зажигания
      if (data->ignition_switch_state)
      {
        flag_fridgeOff_due_ign_switch = false; //сброс флага что было отключение по ignition switch
        if (!flag_fridgeOff_due_voltage)state = true;
        StartGTimer(TIMER_FRIDGE_IGN_OFF);
      }
      else
      {
        if (GetGTimer(TIMER_FRIDGE_IGN_OFF) > (setpoints->fridge_T_IGN_off * MIN))
        {
          state = false;
          flag_fridgeOff_due_ign_switch = true; //установка флага что было отключение по ignition switch
        }
      }

      break;

    default:
    break;
  }

  data->fridge_output_state = state;
}
//*****************************************************************************************

// Main power control
void fnMainPowerControl(struct MyData *data, struct SetpointsStruct *setpoints)
{
  static bool state = true;

  if(data->ignition_switch_state)
  {
	StartGTimer(TIMER_SHUTDOWN_DELAY);
    state = true;
  }
  else
  {
	if (GetGTimer(TIMER_SHUTDOWN_DELAY) > (setpoints->shutdown_delay * HOUR))state = false;
  }

  data->main_supply_output_state = state;
}
//**********************************************************************

//
void fnInputsUpdate(void){

  static uint8_t inputs_undebounced_sample = 0;
  static uint8_t inputs_debounced_state = 0;

  if (!HAL_GPIO_ReadPin(DOOR_INPUT_GPIO_Port, DOOR_INPUT_Pin))inputs_undebounced_sample |= (1 << 0);
  else  inputs_undebounced_sample &= ~(1 << 0);

  if (!HAL_GPIO_ReadPin(PRX_SENS_INPUT_GPIO_Port, PRX_SENS_INPUT_Pin))inputs_undebounced_sample |= (1 << 1);
  else inputs_undebounced_sample &= ~(1 << 1);

  if (HAL_GPIO_ReadPin(IGN_INPUT_GPIO_Port,IGN_INPUT_Pin))inputs_undebounced_sample |= (1 << 2);
  else inputs_undebounced_sample &= ~(1 << 2);

  inputs_debounced_state = fnDebounce(inputs_undebounced_sample);

  main_data.door_switch_state = (inputs_debounced_state & (1 << 0));
  main_data.proximity_sensor_state = (inputs_debounced_state & (1 << 1));
  main_data.ignition_switch_state = (inputs_debounced_state & (1 << 2));

}
//**********************************************************************************

//Debounce
uint8_t fnDebounce(uint8_t sample) // антидребезг на основе вертикального счетчика
{
      static uint8_t state, cnt0, cnt1;
      uint8_t delta, toggle;

      delta = sample ^ state;
      cnt1 = cnt1 ^ cnt0;
      cnt0 = ~cnt0;

      cnt0 &= delta;
      cnt1 &= delta;

      toggle = cnt0 & cnt1;
      state ^= toggle;
      return state;
}
//*************************************************************************

//Outputs Update
void fnOutputsUpdate(struct MyData *data)
{
	HAL_GPIO_WritePin(PUMP_OUTPUT_GPIO_Port, PUMP_OUTPUT_Pin, data->pump_output_state);
	HAL_GPIO_WritePin(FRIDGE_OUTPUT_GPIO_Port, FRIDGE_OUTPUT_Pin, data->fridge_output_state);
	HAL_GPIO_WritePin(CONV_OUTPUT_GPIO_Port, CONV_OUTPUT_Pin, data->converter_output_state);
	HAL_GPIO_WritePin(MAIN_SUPPLY_GPIO_Port, MAIN_SUPPLY_Pin, data->main_supply_output_state);
	HAL_GPIO_WritePin(SENS_SUPPLY_GPIO_Port, SENS_SUPPLY_Pin, data->sensors_supply_output_state);
	HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, data->lcd_light_output_state);
}
//*******************************************************************************

// lcd light control
void fnDisplayLightControl(struct MyData *data, struct SetpointsStruct *setpoints){

	static bool door_old_state = false;
	static bool state = false;

	if ((data->door_switch_state && (door_old_state != data->door_switch_state)) || data->btn_state != 0){
		StartGTimer(TIMER_LCD_LIGHT_OFF);
		state = true;
	}

	door_old_state = data->door_switch_state;

	if(GetGTimer(TIMER_LCD_LIGHT_OFF) > (setpoints->lcd_light_T_off * SEC)){
		state = false;
		StopGTimer(TIMER_LCD_LIGHT_OFF);
	}

	data->lcd_light_output_state = state;
}
//********************************************************************************

//CRC16
uint16_t fnCrc16Calc(uint8_t *Data, uint8_t count) {

	uint16_t crc = 0xffff;
	uint8_t i = 0;
	uint8_t j = 0;

	for (j = 0; j < count; j++)
	{
		crc = crc ^ (uint16_t) *Data++ << 8;
		i = 8;

		do {
			if (crc & 0x8000)
				crc = crc << 1 ^ 0x1021;
			else
				crc = crc << 1;
		} while (--i);
	}

	return crc;
}
//*************************************************************************

//check saved setpoints (flash)
/*
 * Читаем уставки из флэшки и проверяем контрольную сумму и волшебное слово
 * Если всё корректно то выходим, если нет записываем уставки по умолчаню в флэшку и инкриментируем счетчик записей
 * Затем снова проверяем контрольную сумму и волшебное слово.
 * Если опять не корректно,записываем уставки по умолчаню в флэшку и инкриментируем счетчик записей
 * Снова проверяем контролюную сумму и волшебное слово
 * Если опять не корректно и счётчик записей больше 2 то копируем уставки по умолчанию прямо в структуру и сообщаем об ошибке флэш
 */
void fnCheckSavedSetpoints(void){
	uint8_t write_cnt = 0;
	enum Steps{
	  	CHECK_CRC_MAGIC_KEY,
		CHECK_WRITE_CNT,
		WRITE_DEFAULT_TO_FLASH,
		WRITE_DEFAULT_TO_SETPOINTS,
		MESSAGE_READ_SUCCESS,
		MESSAGE_REWRITE_DEFAULT,
		MESSAGE_FLASH_FAULT,
		EXIT
	  };
	enum Steps step = CHECK_CRC_MAGIC_KEY;

	while(step != EXIT){
		switch (step) {
			case CHECK_CRC_MAGIC_KEY:
				W25qxx_ReadBytes(SetpointsUnion.SetpointsArray, SETPOINTS_FLASH_SECTOR*FLASH_SECTOR_SIZE, sizeof(SetpointsUnion.SetpointsArray));
				uint16_t crc = !fnCrc16Calc(SetpointsUnion.SetpointsArray, sizeof(SetpointsUnion.SetpointsArray));

				if((crc == 0) && (SetpointsUnion.setpoints_data.magic_key == MAGIC_KEY)){
					step = MESSAGE_READ_SUCCESS;
				}
				else {
					step = CHECK_WRITE_CNT;
				}
				break;

			case  CHECK_WRITE_CNT:
				if(write_cnt > 2){
					step = WRITE_DEFAULT_TO_SETPOINTS;
				}
				else {
					step = WRITE_DEFAULT_TO_FLASH;
					write_cnt++;
				}
				break;

			case WRITE_DEFAULT_TO_FLASH:
				default_setpoints_data.crc = fnCrc16Calc((uint8_t*)&default_setpoints_data, sizeof(default_setpoints_data) - 2);
				W25qxx_EraseSector(SETPOINTS_FLASH_SECTOR);
				bool flag_empty = false;
				flag_empty = W25qxx_IsEmptySector(SETPOINTS_FLASH_SECTOR, 0,sizeof(SetpointsUnion.SetpointsArray));
				if(flag_empty){
					W25qxx_WriteSector((uint8_t*)&default_setpoints_data, SETPOINTS_FLASH_SECTOR, 0, sizeof(default_setpoints_data));
					flag_empty = false;
					//tone(BUZZER,500,200);
				}
				HAL_Delay(50);
				step = MESSAGE_REWRITE_DEFAULT;
				break;

			case WRITE_DEFAULT_TO_SETPOINTS:
				SetpointsUnion.setpoints_data = default_setpoints_data;
				step = MESSAGE_FLASH_FAULT;
				break;

			case MESSAGE_READ_SUCCESS:
				step = EXIT;
				main_data.flag_setpoints_read_success = true;
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
				HAL_Delay(200);
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
				break;

			case MESSAGE_REWRITE_DEFAULT:
				step = CHECK_CRC_MAGIC_KEY;
				for(uint8_t i = 0;i<4;i++){
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
					HAL_Delay(200);
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
					HAL_Delay(200);
				}
				break;

			case MESSAGE_FLASH_FAULT:
				step = EXIT;
				main_data.flag_setpoints_read_success = false;
				for(uint8_t i = 0;i<10;i++){
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
					HAL_Delay(200);
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
					HAL_Delay(200);
				}
				break;

			case EXIT :

				break;

			default:
				break;
		}
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

