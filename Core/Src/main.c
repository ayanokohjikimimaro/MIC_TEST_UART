/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // For atoi, atol
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    CMD_NONE,
    CMD_LISTEN_WITH_PARAMS, // LISTEN SINC=N OV=N ...
    CMD_GET_PARAMS,         // GET_PARAMS (現在のパラメータ確認用)
    CMD_UNKNOWN
} UART_Command_t;

typedef enum {
    STATE_READY,      // コマンド待機中
    STATE_CONFIGURING,  // DFSDM設定中 (DeInit/Init中)
    STATE_LISTENING,  // DMAでデータ取得中
    STATE_DATA_READY, // DMA完了、データ処理待ち (今回はprintf)
    STATE_ERROR       // エラー発生
} MCU_State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_BUFFER_DURATION_SECONDS (1)
#define NOMINAL_SAMPLING_FREQUENCY_HZ (16000) // 基準サンプリング周波数
#define AUDIO_BUFFER_SIZE_SAMPLES_MAX (NOMINAL_SAMPLING_FREQUENCY_HZ * AUDIO_BUFFER_DURATION_SECONDS)
#define AUDIO_BUFFER_SIZE_BYTES_MAX   (AUDIO_BUFFER_SIZE_SAMPLES_MAX * sizeof(int32_t))

#define UART_RX_DMA_BUFFER_SIZE 128 // UART DMA受信バッファサイズ
#define UART_TX_BUFFER_SIZE 256 // printf用UART送信バッファサイズ (sprintfで使用)

#define PRINT_DATA_COUNT 30     // printfで表示するサンプル数
#define PRINT_DATA_OFFSET 0     // printf表示開始オフセット
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
volatile MCU_State_t mcu_state = STATE_READY;
int32_t audio_buffer[AUDIO_BUFFER_SIZE_SAMPLES_MAX];

uint8_t uart_rx_dma_buffer[UART_RX_DMA_BUFFER_SIZE]; // UART DMA受信用バッファ
volatile uint8_t uart_cmd_received_flag = 0;
volatile uint16_t uart_received_dma_size = 0; // DMAで実際に受信したサイズ

char parsed_cmd_payload_buffer[UART_RX_DMA_BUFFER_SIZE]; // LISTENコマンドのパラメータ部分を保持

volatile uint8_t dma_transfer_complete_flag = 0; // DFSDM DMA完了フラグ
volatile uint8_t user_button_pressed_flag = 0;
volatile uint8_t hardfault_indicator_flag = 0;   // HardFault発生表示用フラグ

// DFSDM パラメータのグローバル変数
volatile uint32_t g_dfsdm_sinc_order = DFSDM_FILTER_SINC3_ORDER;
volatile uint32_t g_dfsdm_filter_oversampling = 125;
volatile uint32_t g_dfsdm_integrator_oversampling = 1;
volatile uint32_t g_dfsdm_channel_clock_divider = 8;
volatile uint32_t g_dfsdm_right_bit_shift = 0x02;

uint32_t current_dma_samples_to_transfer = AUDIO_BUFFER_SIZE_SAMPLES_MAX;

HAL_StatusTypeDef last_dma_start_status;
HAL_StatusTypeDef last_dma_stop_status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static UART_Command_t UART_Parse_Command(char* cmd_buffer, uint32_t len);
static void UART_Process_Received_DMA_Data(void); // ★ DMA受信データを処理する関数 (名前変更、引数なし)
static void ReInitialize_DFSDM_And_Start_Listen(const char* params_payload);
static uint32_t MapSincOrderNumToDefine(uint32_t order_num);
static uint32_t Calculate_Current_Sampling_Frequency_Hz(void);
static void Print_Audio_Data(void);
static void Start_UART_DMA_Receive(void); // UART DMA受信を開始/再開する関数
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// printfの出力をUARTにリダイレクト
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY); // ★ 使用するUARTハンドル
  return len;
}
#else
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY); // ★ 使用するUARTハンドル
  return ch;
}
#endif

static void Start_UART_DMA_Receive(void) {
    memset(uart_rx_dma_buffer, 0, UART_RX_DMA_BUFFER_SIZE);
    uart_received_dma_size = 0;
    uart_cmd_received_flag = 0;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_rx_dma_buffer, UART_RX_DMA_BUFFER_SIZE) != HAL_OK) {
        printf("Error starting UART DMA RX to Idle!\r\n");
        Error_Handler();
    }
}


static uint32_t Calculate_Current_Sampling_Frequency_Hz(void) {
    uint32_t pclk2_hz = HAL_RCC_GetPCLK2Freq();
    if (g_dfsdm_channel_clock_divider == 0 || g_dfsdm_filter_oversampling == 0) {
        printf("Error: CLKDIV or OV is zero in Fs calculation.\r\n");
        return NOMINAL_SAMPLING_FREQUENCY_HZ;
    }
    return pclk2_hz / g_dfsdm_channel_clock_divider / g_dfsdm_filter_oversampling;
}

static uint32_t MapSincOrderNumToDefine(uint32_t order_num) {
    switch (order_num) {
        case 1: return DFSDM_FILTER_SINC1_ORDER;
        case 2: return DFSDM_FILTER_SINC2_ORDER;
        case 3: return DFSDM_FILTER_SINC3_ORDER;
        case 4: return DFSDM_FILTER_SINC4_ORDER;
        case 5: return DFSDM_FILTER_SINC5_ORDER;
        default:
            printf("Warn: Invalid SINC order %lu, using current.\r\n", (unsigned long)order_num);
            return g_dfsdm_sinc_order;
    }
}

static void ReInitialize_DFSDM_And_Start_Listen(const char* params_payload) {
    HAL_StatusTypeDef status;
    mcu_state = STATE_CONFIGURING;
    printf("DBG: State changed to CONFIGURING.\r\n");

    if (params_payload != NULL && strlen(params_payload) > 0) {
        printf("DBG: Parsing parameters: '%s'\r\n", params_payload);
        char local_payload_copy[UART_RX_DMA_BUFFER_SIZE];
        strncpy(local_payload_copy, params_payload, sizeof(local_payload_copy) - 1);
        local_payload_copy[sizeof(local_payload_copy) - 1] = '\0';

        char* token = strtok(local_payload_copy, " ");
        while(token != NULL) {
            char* value_str = strchr(token, '=');
            if (value_str != NULL && (value_str - token > 0)) {
                *value_str = '\0'; value_str++;
                long val_long = atol(value_str);

                if (strcmp(token, "SINC") == 0) g_dfsdm_sinc_order = MapSincOrderNumToDefine((uint32_t)val_long);
                else if (strcmp(token, "OV") == 0) g_dfsdm_filter_oversampling = (uint32_t)val_long;
                else if (strcmp(token, "IOSR") == 0) g_dfsdm_integrator_oversampling = (uint32_t)val_long;
                else if (strcmp(token, "CLKDIV") == 0) g_dfsdm_channel_clock_divider = (uint32_t)val_long;
                else if (strcmp(token, "RBS") == 0) g_dfsdm_right_bit_shift = (uint32_t)val_long;
                else { printf("Warn: Unknown param token '%s'\r\n", token); }
            }
            token = strtok(NULL, " ");
        }
        printf("DBG: Parameters applied to globals.\r\n");
    } else {
        printf("DBG: No new parameters provided, using current globals for re-init.\r\n");
    }

    printf("DBG: Stopping current DFSDM DMA (if active)...\r\n");
    status = HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
    printf("DBG: Stop_DMA status before DeInit: %d (0=OK)\r\n", status);

    printf("DBG: De-initializing DFSDM Filter...\r\n");
    status = HAL_DFSDM_FilterDeInit(&hdfsdm1_filter0);
    printf("DBG: FilterDeInit status: %d (0=OK)\r\n", status);
    if (status != HAL_OK) { printf("FATAL: FilterDeInit FAILED.\r\n"); mcu_state = STATE_ERROR; Error_Handler(); return; }

    printf("DBG: De-initializing DFSDM Channel...\r\n");
    status = HAL_DFSDM_ChannelDeInit(&hdfsdm1_channel0);
    printf("DBG: ChannelDeInit status: %d (0=OK)\r\n", status);
    if (status != HAL_OK) { printf("FATAL: ChannelDeInit FAILED.\r\n"); mcu_state = STATE_ERROR; Error_Handler(); return; }

    MX_DMA_Init();
    hdma_dfsdm1_flt0.Init.Mode = DMA_NORMAL;

    printf("DBG: Re-initializing DFSDM (calling MX_DFSDM1_Init)...\r\n");
    MX_DFSDM1_Init();

    char current_params_msg[128];
    uint32_t current_sinc_display = 0;
    if (g_dfsdm_sinc_order == DFSDM_FILTER_SINC1_ORDER) current_sinc_display = 1;
    else if (g_dfsdm_sinc_order == DFSDM_FILTER_SINC2_ORDER) current_sinc_display = 2;
    else if (g_dfsdm_sinc_order == DFSDM_FILTER_SINC3_ORDER) current_sinc_display = 3;
    else if (g_dfsdm_sinc_order == DFSDM_FILTER_SINC4_ORDER) current_sinc_display = 4;
    else if (g_dfsdm_sinc_order == DFSDM_FILTER_SINC5_ORDER) current_sinc_display = 5;

    sprintf(current_params_msg, "DFSDM Re-configured: SINC=%lu, OV=%lu, IOSR=%lu, CLKDIV=%lu, RBS=%lu\r\n",
            current_sinc_display, (unsigned long)g_dfsdm_filter_oversampling,
            (unsigned long)g_dfsdm_integrator_oversampling, (unsigned long)g_dfsdm_channel_clock_divider,
            (unsigned long)g_dfsdm_right_bit_shift);
    printf(current_params_msg);

    uint32_t new_fs = Calculate_Current_Sampling_Frequency_Hz();
    sprintf(current_params_msg, "New Calculated Fs: %lu Hz\r\n", (unsigned long)new_fs);
    printf(current_params_msg);

    dma_transfer_complete_flag = 0; // ★ dma_full_transfer_complete_flag から修正
    mcu_state = STATE_LISTENING;
    printf("DBG: State changed to LISTENING.\r\n");
    HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);

    current_dma_samples_to_transfer = new_fs * AUDIO_BUFFER_DURATION_SECONDS;
    if (current_dma_samples_to_transfer > AUDIO_BUFFER_SIZE_SAMPLES_MAX) {
        printf("Warning: Calculated samples (%lu) exceed max buffer (%lu). Clamping.\r\n",
               (unsigned long)current_dma_samples_to_transfer, (unsigned long)AUDIO_BUFFER_SIZE_SAMPLES_MAX);
        current_dma_samples_to_transfer = AUDIO_BUFFER_SIZE_SAMPLES_MAX;
    }
    if (current_dma_samples_to_transfer == 0) {
        printf("FATAL: DMA transfer size is 0 based on Fs calculation!\r\n");
        mcu_state = STATE_ERROR; Error_Handler(); return;
    }

    printf("DBG: Attempting to start DMA transfer for %lu samples...\r\n", (unsigned long)current_dma_samples_to_transfer);
    last_dma_start_status = HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, audio_buffer, current_dma_samples_to_transfer);
    printf("DBG: Start_DMA status: %d (0=OK)\r\n", last_dma_start_status);

    if (last_dma_start_status != HAL_OK) {
         printf("FATAL: DMA Start Failed!\r\n");
         mcu_state = STATE_ERROR; Error_Handler();
    } else {
        printf("DFSDM DMA started. Acquiring audio data...\r\n");
    }
}

static UART_Command_t UART_Parse_Command(char* cmd_buffer, uint32_t len) {
    if (len == 0) return CMD_NONE;

    const char* listen_prefix = "LISTEN";
    if (strncmp(cmd_buffer, listen_prefix, strlen(listen_prefix)) == 0) {
        if (len > strlen(listen_prefix) && cmd_buffer[strlen(listen_prefix)] == ' ') {
            strncpy(parsed_cmd_payload_buffer, cmd_buffer + strlen(listen_prefix) + 1, sizeof(parsed_cmd_payload_buffer) -1);
            parsed_cmd_payload_buffer[sizeof(parsed_cmd_payload_buffer)-1] = '\0';
        } else {
            parsed_cmd_payload_buffer[0] = '\0';
        }
        return CMD_LISTEN_WITH_PARAMS;
    }

    const char* get_params_str = "GET_PARAMS";
    if (strcmp(cmd_buffer, get_params_str) == 0) {
        return CMD_GET_PARAMS;
    }
    return CMD_UNKNOWN;
}

static void UART_Process_Received_DMA_Data(void) { // ★ 関数名を PFP と合わせる
    uart_rx_dma_buffer[uart_received_dma_size] = '\0';
    printf("UART CMD RX (DMA): '%s'\r\n", (char*)uart_rx_dma_buffer);

    UART_Command_t current_cmd = UART_Parse_Command((char*)uart_rx_dma_buffer, uart_received_dma_size);

    switch (current_cmd) {
        case CMD_LISTEN_WITH_PARAMS:
            printf("CMD: LISTEN_WITH_PARAMS. Args: '%s'\r\n", parsed_cmd_payload_buffer);
            if (mcu_state == STATE_READY || mcu_state == STATE_ERROR) {
                ReInitialize_DFSDM_And_Start_Listen(parsed_cmd_payload_buffer);
            } else {
                printf("CMD: LISTEN ignored. MCU not in READY/ERROR state (Current: %d)\r\n", mcu_state);
            }
            break;
        case CMD_GET_PARAMS:
            printf("CMD: GET_PARAMS received.\r\n");
            char msg[128];
            uint32_t sinc_disp = 0;
            if(g_dfsdm_sinc_order == DFSDM_FILTER_SINC1_ORDER) sinc_disp=1;
            else if(g_dfsdm_sinc_order == DFSDM_FILTER_SINC2_ORDER) sinc_disp=2;
            else if(g_dfsdm_sinc_order == DFSDM_FILTER_SINC3_ORDER) sinc_disp=3;
            else if(g_dfsdm_sinc_order == DFSDM_FILTER_SINC4_ORDER) sinc_disp=4;
            else if(g_dfsdm_sinc_order == DFSDM_FILTER_SINC5_ORDER) sinc_disp=5;
            sprintf(msg, "Params:SINC=%lu OV=%lu IOSR=%lu CLKDIV=%lu RBS=%lu Fs=%luHz\r\n",
                    sinc_disp, (unsigned long)g_dfsdm_filter_oversampling,
                    (unsigned long)g_dfsdm_integrator_oversampling, (unsigned long)g_dfsdm_channel_clock_divider,
                    (unsigned long)g_dfsdm_right_bit_shift, (unsigned long)Calculate_Current_Sampling_Frequency_Hz());
            printf(msg);
            break;
        case CMD_UNKNOWN:
        default:
            printf("Error: Unknown UART command: '%s'\r\n", (char*)uart_rx_dma_buffer);
            break;
    }
    Start_UART_DMA_Receive();
}


static void Print_Audio_Data(void) {
    printf("Audio data acquisition complete. Stop DMA status: %d\r\n", last_dma_stop_status);
    printf("First %d samples (out of %lu) from audio_buffer:\r\n", PRINT_DATA_COUNT, (unsigned long)current_dma_samples_to_transfer);
    for (int i = PRINT_DATA_OFFSET; i < PRINT_DATA_OFFSET + PRINT_DATA_COUNT && i < current_dma_samples_to_transfer; i++) {
        printf("Sample %d: %ld\r\n", i, audio_buffer[i]);
    }
    printf("\nData printing complete. Ready for next command.\r\n");
}
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
  MX_DMA_Init();
  MX_DFSDM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("DFSDM UART DMA Command Interface Ready.\r\n");
  printf("Cmd format: LISTEN SINC=N OV=N IOSR=N CLKDIV=N RBS=N (params optional)\r\n");
  printf("Or press USER button to listen with current parameters.\r\n");

  char initial_params_msg[128];
  uint32_t sinc_disp_init = 0;
    if(g_dfsdm_sinc_order == DFSDM_FILTER_SINC1_ORDER) sinc_disp_init=1;
    else if(g_dfsdm_sinc_order == DFSDM_FILTER_SINC2_ORDER) sinc_disp_init=2;
    else if(g_dfsdm_sinc_order == DFSDM_FILTER_SINC3_ORDER) sinc_disp_init=3;
    else if(g_dfsdm_sinc_order == DFSDM_FILTER_SINC4_ORDER) sinc_disp_init=4;
    else if(g_dfsdm_sinc_order == DFSDM_FILTER_SINC5_ORDER) sinc_disp_init=5;
  sprintf(initial_params_msg, "Initial Params: SINC=%lu OV=%lu IOSR=%lu CLKDIV=%lu RBS=%lu Fs=%luHz\r\n",
            sinc_disp_init, (unsigned long)g_dfsdm_filter_oversampling,
            (unsigned long)g_dfsdm_integrator_oversampling, (unsigned long)g_dfsdm_channel_clock_divider,
            (unsigned long)g_dfsdm_right_bit_shift, (unsigned long)Calculate_Current_Sampling_Frequency_Hz());
  printf(initial_params_msg);

  Start_UART_DMA_Receive();
  mcu_state = STATE_READY;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (hardfault_indicator_flag) {
        HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
        for(volatile int i=0; i<500000; i++);
        continue;
    }

    if (uart_cmd_received_flag) {
        uart_cmd_received_flag = 0;
        if (mcu_state == STATE_LISTENING) {
            printf("WARN: New UART CMD RX while LISTENING. Stopping current DFSDM DMA.\r\n");
            HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
            mcu_state = STATE_READY;
        }
        UART_Process_Received_DMA_Data(); // ★ 関数名を PFP と合わせる
    }

    if (user_button_pressed_flag) {
        user_button_pressed_flag = 0;
        printf("DBG: User Button press detected in main loop.\r\n");
        if (mcu_state == STATE_READY || mcu_state == STATE_ERROR) {
            printf("DBG: User button: Re-initializing and starting listen...\r\n");
            ReInitialize_DFSDM_And_Start_Listen(NULL);
        } else {
            printf("DBG: Button press ignored. MCU not in READY/ERROR state (Current: %d)\r\n", mcu_state);
        }
    }

    if (dma_transfer_complete_flag) { // ★ dma_full_transfer_complete_flag から修正
        dma_transfer_complete_flag = 0; // ★ dma_full_transfer_complete_flag から修正
        if (mcu_state == STATE_LISTENING) {
            printf("DBG: Main loop: DFSDM DMA complete flag set AND state is LISTENING.\r\n");
            mcu_state = STATE_DATA_READY;
            HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
        } else {
            printf("DBG: DFSDM DMA flag set, but MCU state not LISTENING (is %d). Ignoring.\r\n", mcu_state);
        }
    }

    switch (mcu_state) {
        case STATE_READY:
            HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
            if (!hardfault_indicator_flag) { HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET); }
            break;
        case STATE_CONFIGURING:
            break;
        case STATE_LISTENING:
            break;
        case STATE_DATA_READY:
            Print_Audio_Data();
            HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
            mcu_state = STATE_READY;
            printf("DBG: MCU State set to READY after data processing.\r\n");
            break;
        case STATE_ERROR:
            break;
        default:
            printf("Error: Unknown MCU state! Resetting to READY.\r\n");
            mcu_state = STATE_READY;
            break;
    }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */
  hdfsdm1_filter0.Init.FilterParam.SincOrder = g_dfsdm_sinc_order;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = g_dfsdm_filter_oversampling;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = g_dfsdm_integrator_oversampling;

  hdfsdm1_channel0.Init.OutputClock.Divider = g_dfsdm_channel_clock_divider;
  hdfsdm1_channel0.Init.RightBitShift = g_dfsdm_right_bit_shift;

  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;

  hdfsdm1_channel0.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_FALLING;
  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel0.Init.Awd.Oversampling = 1;
  hdfsdm1_channel0.Init.Offset = 0;
  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */
  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 125;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  HAL_DFSDM_FilterInit(&hdfsdm1_filter0);
  hdfsdm1_channel0.Instance = DFSDM1_Channel0;
  hdfsdm1_channel0.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel0.Init.OutputClock.Divider = 8;
  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_FALLING;
  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel0.Init.Awd.Oversampling = 1;
  hdfsdm1_channel0.Init.Offset = 0;
  hdfsdm1_channel0.Init.RightBitShift = 0x02;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_0, DFSDM_CONTINUOUS_CONV_ON);
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // UART and DFSDM GPIOs are configured in their respective MspInit functions
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// UART RX Event Callback (for HAL_UARTEx_ReceiveToIdle_DMA)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART3) // ★ Adjust to your chosen USART
  {
    if (Size > 0) {
        uart_received_dma_size = Size;
        uart_cmd_received_flag = 1;
    }
    // Important: Do NOT restart DMA reception here if you want to process the command
    // and then decide to restart. Restart it in UART_Process_Received_DMA_Data.
    // If you restart here, and main loop is slow, you might get another RxEvent before processing.
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    uint32_t error_code = HAL_UART_GetError(huart);
    printf("UART Error Callback! ErrorCode: 0x%lX\r\n", error_code);
    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
    // Attempt to recover by re-starting DMA reception
    // This might clear some error flags implicitly.
    // Consider more specific error handling based on error_code.
    Start_UART_DMA_Receive();
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  static uint32_t last_button_press_time = 0;
  const uint32_t debounce_delay_ms = 300;

  if (GPIO_Pin == USER_Btn_Pin) {
    if (HAL_GetTick() - last_button_press_time > debounce_delay_ms) {
        last_button_press_time = HAL_GetTick();
        if (mcu_state == STATE_READY || mcu_state == STATE_ERROR) {
            user_button_pressed_flag = 1;
        } else {
            printf("DBG_ISR: Button press ignored, MCU not in READY/ERROR state.\r\n");
        }
    }
  }
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  if (hdfsdm_filter == &hdfsdm1_filter0) {
    if (mcu_state == STATE_LISTENING) {
        last_dma_stop_status = HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
        dma_transfer_complete_flag = 1;
    }
  }
}

void HAL_DFSDM_FilterErrorCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter) {
    if (hdfsdm_filter == &hdfsdm1_filter0) {
        printf("!!! DFSDM Filter Error Callback! ErrorCode: 0x%lX !!!\r\n", (unsigned long)hdfsdm_filter->ErrorCode);
        HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
        HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
        mcu_state = STATE_ERROR;
        hardfault_indicator_flag = 1;
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
  if (!hardfault_indicator_flag) {
    printf("\r\n!!! System Error: Entered Error_Handler() !!!\r\n");
  }
  hardfault_indicator_flag = 1;
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
    for(volatile uint32_t i=0; i < 500000; i++);
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
  printf("Assert failed: file %s on line %lu\r\n", (char*)file, line);
  Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
