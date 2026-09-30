#include "control_timer.h"

#include "board.h"
#include "control.h"

#define CONTROL_TIMER_COUNTER_HZ 1000000U

static TIM_HandleTypeDef htim2;

bool control_timer_init(void)
{
    uint32_t timer_clock_hz = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
    {
        timer_clock_hz *= 2U;
    }

    if (timer_clock_hz < CONTROL_TIMER_COUNTER_HZ ||
        timer_clock_hz % CONTROL_TIMER_COUNTER_HZ != 0U)
    {
        return false;
    }

    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = timer_clock_hz / CONTROL_TIMER_COUNTER_HZ - 1U;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = CONTROL_PERIOD_US - 1U;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        return false;
    }

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    return true;
}

bool control_timer_start(void)
{
    return HAL_TIM_Base_Start_IT(&htim2) == HAL_OK;
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
        control_timer_tick_isr();
    }
}
