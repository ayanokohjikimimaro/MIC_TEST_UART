/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f7xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  * and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_dfsdm1_flt0;
extern DMA_HandleTypeDef hdma_usart3_rx; // USART3 RX用DMAハンドルのextern宣言 (main.cで定義されている想定)

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// DFSDM1_common_initialized_flag は、DFSDM関連の共通リソース (GPIOなど) が初期化済みかどうかのフラグとして使用
static uint32_t DFSDM1_common_initialized_flag = 0;
// USART3_common_initialized_flag は、USART3関連の共通リソース (GPIOなど) が初期化済みかどうかのフラグとして使用
static uint32_t USART3_common_initialized_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
  * @brief DFSDM_Filter MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hdfsdm_filter: DFSDM_Filter handle pointer
  * @retval None
  */
void HAL_DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef* hdfsdm_filter)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /* USER CODE BEGIN DFSDM1_MspInit 0 */

  /* USER CODE END DFSDM1_MspInit 0 */

  /* === DFSDMクロックソース設定とペリフェラルクロック有効化を常に実行 === */
  /** Initializes the DFSDM1 peripheral clock */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_DFSDM1;
  PeriphClkInitStruct.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* DFSDM1 Peripheral clock enable */
  __HAL_RCC_DFSDM1_CLK_ENABLE();
  /* === ここまでを常に実行 === */

  if(DFSDM1_common_initialized_flag == 0) // GPIOなど、本当に初回のみで良い初期化をここに記述
  {
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE(); // PDM_DATA_Pin と PDWM_CLK_Pin があるGPIOCを有効化 (ピン定義に合わせてください)

    /**DFSDM1 GPIO Configuration
    PC1     ------> DFSDM1_DATIN0 (PDM_DATA_Pin の想定)
    PC2     ------> DFSDM1_CKOUT  (PDWM_CLK_Pin の想定)
    */
    GPIO_InitStruct.Pin = PDM_DATA_Pin; // main.h などで定義されているピン名
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_DFSDM1; // 代替機能はデータシートで確認
    HAL_GPIO_Init(PDM_DATA_GPIO_Port, &GPIO_InitStruct); // ポートもmain.hなどで定義

    GPIO_InitStruct.Pin = PDWM_CLK_Pin; // main.h などで定義されているピン名
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1; // 代替機能はデータシートで確認
    HAL_GPIO_Init(PDWM_CLK_GPIO_Port, &GPIO_InitStruct); // ポートもmain.hなどで定義

    /* USER CODE BEGIN DFSDM1_MspInit 1 */

    /* USER CODE END DFSDM1_MspInit 1 */
    DFSDM1_common_initialized_flag = 1; // 共通リソース初期化済みフラグを立てる
  }

  /* DFSDM1 DMA Init */
  /* DFSDM1_FLT0 Init */
  if(hdfsdm_filter->Instance == DFSDM1_Filter0){
    hdma_dfsdm1_flt0.Instance = DMA2_Stream0; // CubeMXで設定されたストリーム
    hdma_dfsdm1_flt0.Init.Channel = DMA_CHANNEL_8; // CubeMXで設定されたチャンネル
    hdma_dfsdm1_flt0.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_dfsdm1_flt0.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dfsdm1_flt0.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dfsdm1_flt0.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dfsdm1_flt0.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
//    hdma_dfsdm1_flt0.Init.Mode = DMA_CIRCULAR; // ★★★ main.cでDMA_NORMALを設定するためコメントアウト ★★★
    hdma_dfsdm1_flt0.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_dfsdm1_flt0.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_dfsdm1_flt0) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hdfsdm_filter,hdmaInj,hdma_dfsdm1_flt0); // Injected DMA (未使用なら不要かも)
    __HAL_LINKDMA(hdfsdm_filter,hdmaReg,hdma_dfsdm1_flt0); // Regular DMA
  }
}

/**
  * @brief DFSDM_Channel MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hdfsdm_channel: DFSDM_Channel handle pointer
  * @retval None
  */
void HAL_DFSDM_ChannelMspInit(DFSDM_Channel_HandleTypeDef* hdfsdm_channel)
{
  /* USER CODE BEGIN DFSDM1_ChannelMspInit 0 */
  // DFSDMの共通クロックと共通GPIOは FilterMspInit で初期化されるため、
  // ここではチャンネル固有のMSP初期化があれば記述します。
  // DFSDM1_common_initialized_flag の操作も FilterMspInit/FilterMspDeInit に集約。
  /* USER CODE END DFSDM1_ChannelMspInit 0 */

  /* USER CODE BEGIN DFSDM1_ChannelMspInit 1 */

  /* USER CODE END DFSDM1_ChannelMspInit 1 */
}

/**
  * @brief DFSDM_Filter MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hdfsdm_filter: DFSDM_Filter handle pointer
  * @retval None
  */
void HAL_DFSDM_FilterMspDeInit(DFSDM_Filter_HandleTypeDef* hdfsdm_filter)
{
  /* USER CODE BEGIN DFSDM1_MspDeInit 0 */
  /* USER CODE END DFSDM1_MspDeInit 0 */

  // if (DFSDM1_common_initialized_flag > 0) // 既にDeInit済みでないか確認 (より安全に)
  // {
    /* Peripheral clock disable */
    __HAL_RCC_DFSDM1_CLK_DISABLE();

    /**DFSDM1 GPIO Configuration
    PC1     ------> DFSDM1_DATIN0
    PC2     ------> DFSDM1_CKOUT
    */
    HAL_GPIO_DeInit(PDM_DATA_GPIO_Port, PDM_DATA_Pin); // ピン定義に合わせてください
    HAL_GPIO_DeInit(PDWM_CLK_GPIO_Port, PDWM_CLK_Pin); // ピン定義に合わせてください

    /* DFSDM1 DMA DeInit */
    if(hdfsdm_filter->hdmaReg != NULL)
    {
      HAL_DMA_DeInit(hdfsdm_filter->hdmaReg);
    }
    if(hdfsdm_filter->hdmaInj != NULL) // Injected DMA (未使用なら不要かも)
    {
      HAL_DMA_DeInit(hdfsdm_filter->hdmaInj);
    }
    DFSDM1_common_initialized_flag = 0; // 共通リソース初期化フラグをリセット
  // }
  /* USER CODE BEGIN DFSDM1_MspDeInit 1 */

  /* USER CODE END DFSDM1_MspDeInit 1 */
}

/**
  * @brief DFSDM_Channel MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hdfsdm_channel: DFSDM_Channel handle pointer
  * @retval None
  */
void HAL_DFSDM_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef* hdfsdm_channel)
{
  /* USER CODE BEGIN DFSDM1_ChannelMspDeInit 0 */
  // 共通リソースの解放は FilterMspDeInit に集約するため、
  // ここではチャンネル固有のMSP解放処理があれば記述します。
  // DFSDM1_common_initialized_flag の操作も FilterMspDeInit に集約。
  // もしチャンネルのDeInitがフィルターのDeInitとは独立して共通リソースを操作する必要がある場合は、
  // DFSDM1_common_initialized_flag のような参照カウントのロジックをより精緻にする必要があります。
  // 現状はFilterMspDeInitで共通リソースが解放される想定です。
  /* USER CODE END DFSDM1_ChannelMspDeInit 0 */

  /* USER CODE BEGIN DFSDM1_ChannelMspDeInit 1 */

  /* USER CODE END DFSDM1_ChannelMspDeInit 1 */
}

/* USER CODE BEGIN 1 */
/**
  * @brief UART MSP Initialization
  * This function configures the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  if(huart->Instance==USART3) // ★ 使用するUARTインスタンス (例: USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */

    /* Peripheral clock source configuration */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1; // または PCLK2 (CubeMX設定に合わせる)
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    if(USART3_common_initialized_flag == 0) // GPIOなど初回のみの初期化
    {
        __HAL_RCC_GPIOD_CLK_ENABLE(); // ★ USART3のピンがあるGPIOポート (例: GPIOD for PD8/PD9)
        /**USART3 GPIO Configuration
        PD8     ------> USART3_TX
        PD9     ------> USART3_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9; // ★ ピン番号は実際の接続に合わせる
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP; // または GPIO_NOPULL
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3; // ★ 代替機能を確認
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);      // ★ GPIOポートを確認
        USART3_common_initialized_flag = 1;
    }

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream1; // ★ CubeMXで設定されたDMAストリーム
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4; // ★ CubeMXで設定されたDMAチャンネル
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL; // For ReceiveToIdle_DMA
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    // hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE; // FIFO設定 (必要なら)
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart,hdmarx,hdma_usart3_rx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0); // UART自体の割り込み優先度 (0-15)
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    /* DMA1 stream1 global interrupt configuration (USART3_RX用) */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0); // ★ DMAストリーム割り込み優先度
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);       // ★ DMAストリーム割り込み有効化

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9); // ★ ピン定義に合わせてください

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);

    /* USART3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn); // ★ DMAストリーム割り込みも無効化

    USART3_common_initialized_flag = 0; // フラグをリセット
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}
/* USER CODE END 1 */
