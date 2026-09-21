#include "joystick.h"

#include "board.h"

static ADC_HandleTypeDef hadc1;
static volatile uint16_t latest_sample;
static volatile uint32_t sample_sequence;
static bool initialized;

bool joystick_init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfDiscConversion = 0U;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1U;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        return false;
    }

    ADC_ChannelConfTypeDef channel = {0};
    channel.Channel = ADC_CHANNEL_11;
    channel.Rank = 1U;
    channel.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK)
    {
        return false;
    }

    HAL_NVIC_SetPriority(ADC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
    initialized = true;
    return joystick_start_sample();
}

bool joystick_start_sample(void)
{
    if (!initialized)
    {
        return false;
    }
    HAL_StatusTypeDef status = HAL_ADC_Start_IT(&hadc1);
    return status == HAL_OK || status == HAL_BUSY;
}

bool joystick_ready(void)
{
    return initialized;
}

bool joystick_read_latest(uint16_t *sample, uint32_t *sequence)
{
    if (!initialized || sample == 0 || sequence == 0 || sample_sequence == 0U)
    {
        return false;
    }

    uint32_t first;
    uint16_t value;
    do
    {
        first = sample_sequence;
        value = latest_sample;
        __DMB();
    } while (first != sample_sequence);

    *sample = value;
    *sequence = first;
    return true;
}

void ADC_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1)
    {
        return;
    }
    latest_sample = (uint16_t)HAL_ADC_GetValue(&hadc1);
    __DMB();
    sample_sequence++;
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        initialized = false;
    }
}
