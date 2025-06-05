#include "adc.h"
#include "main.h"

/* --------------------------------------------------------- STM32 ADC & DMA variables ---------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* --------------------------------------------------------- Global variables ---------------------------------------------------*/
uint8_t dma_adc_complete;                               // It is externed in Interrupt function
uint16_t adc_converter_counter;
/* --------------------------------------------------------- Local variables ---------------------------------------------------*/
uint32_t adcValue[NUMBER_OF_ADC_CHANNELS];                                      // The DMA (RAW ADC) value is saved in here
uint32_t totalAdcValue[NUMBER_OF_ADC_CHANNELS][NUMBER_OF_ADC_AVERAGE];          // The defined number of conversion is added and saved in here
uint16_t adcAverageValue[NUMBER_OF_ADC_CHANNELS];                               // The average ADC value is save in here

/* --------------------------------------------------------- Local Functions ---------------------------------------------------*/
uint16_t getCurrentADCValue(uint8_t channel);
uint16_t getAverageADCValue(uint8_t channel);
uint8_t  getCurrentADCValues(uint16_t *channels_data);
uint8_t  getAverageADCValues(uint16_t *channels_data);

uint8_t startConversation()
{
  uint8_t i, j;
  for(i = 0; i < NUMBER_OF_ADC_CHANNELS; i++)
  {
    for(j = 0; j < NUMBER_OF_ADC_AVERAGE; j++)
    {
      totalAdcValue[i][j] = 0;
    }
  }
  return HAL_ADC_Start_DMA(&hadc1, adcValue, NUMBER_OF_ADC_CHANNELS);
}

uint16_t getCurrentADCValue(uint8_t channel)
{
  if(channel >= NUMBER_OF_ADC_CHANNELS)
    return -1;
  
  return (uint16_t)adcValue[channel];
}

uint16_t getAverageADCValue(uint8_t channel)
{
  if(channel >= NUMBER_OF_ADC_CHANNELS)
    return -1;
  
  return (uint16_t)adcAverageValue[channel];
}

uint8_t getCurrentADCValues(uint16_t *channels_data)
{
  uint8_t i;
  for(i = 0; i < NUMBER_OF_ADC_CHANNELS; i++){
    channels_data[i] = adcValue[i];
  }
  return 1;
}

uint8_t getAverageADCValues(uint16_t *channels_data)
{
  uint8_t i;
  for(i = 0; i < NUMBER_OF_ADC_CHANNELS; i++){
    channels_data[i] = adcAverageValue[i];
  }
  return 1;
}

uint8_t updateADC()
{
  uint8_t i;
  if(dma_adc_complete)
  {
    dma_adc_complete = 0;
    for(i = 0; i < NUMBER_OF_ADC_CHANNELS; i++)
    {
      totalAdcValue[i][0] += adcValue[i];
    }
    adc_converter_counter++;
  }
  
  if(adc_converter_counter >= NUMBER_OF_ADC_AVERAGE)
  {
    for(i = 0; i < NUMBER_OF_ADC_CHANNELS; i++)
    {
      adcAverageValue[i] = (uint16_t)((float)totalAdcValue[i][0] / adc_converter_counter);
      totalAdcValue[i][0] = 0;
    }
    adc_converter_counter = 0;
    return 1;
  }
  return 0;
}

uint8_t updateLastAverageADC()
{
  uint32_t temp = 0;
  uint8_t i, j;
  if(dma_adc_complete)
  {
    dma_adc_complete = 0;
    for(i = 0; i < NUMBER_OF_ADC_CHANNELS; i++)
    {
      totalAdcValue[i][adc_converter_counter] = adcValue[i];
    }
    adc_converter_counter++;
    if(adc_converter_counter >= NUMBER_OF_ADC_AVERAGE) {
      adc_converter_counter = 0;
    }
    
    for(i = 0; i < NUMBER_OF_ADC_CHANNELS; i++)
    {
      temp = 0;
      for(j = 0; j < NUMBER_OF_ADC_AVERAGE; j++)
      {
        temp += totalAdcValue[i][j];
      }
      adcAverageValue[i] = (uint16_t)((float)temp / NUMBER_OF_ADC_AVERAGE);
    }
    return 1;
  }  
  return 0;
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void adc_init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;                                               // 4 to 5 on 2024.03.27
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}


void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC1 GPIO Configuration    
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PC2     ------> ADC1_IN12
    PC3     ------> ADC1_IN13 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }

}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PC2     ------> ADC1_IN12
    PC3     ------> ADC1_IN13 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }

}

/**
  * Enable DMA controller clock
  */
void adc_dma_init(void)
{

  /* DMA controller clock enable */
  
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */
  dma_adc_complete = 1;
  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}