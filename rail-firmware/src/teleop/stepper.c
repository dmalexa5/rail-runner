#include "stepper.h"

#include <limits.h>

#include "board.h"

#define STEPPER_TIMER_HZ 1000000U
#define STEP_PULSE_US 10U
#define STEP_MIN_PERIOD_US (STEP_PULSE_US + 2U)

static TIM_HandleTypeDef htim5;
static volatile int32_t position_steps;
static volatile bool pulse_high;
static volatile bool running;
static volatile bool emergency_stopped;
static volatile bool timing_fault;
static volatile bool direction_positive = true;
static volatile uint32_t period_us;
static bool initialized;

static void set_output_mode(uint32_t mode)
{
    MODIFY_REG(TIM5->CCMR1, TIM_CCMR1_OC1M, mode);
}

static void force_step_low(void)
{
    if (initialized)
    {
        __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_CC1);
        set_output_mode(TIM_OCMODE_FORCED_INACTIVE);
    }
    running = false;
    pulse_high = false;
}

bool stepper_init(void)
{
    uint32_t timer_clock_hz = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
    {
        timer_clock_hz *= 2U;
    }
    if (timer_clock_hz < STEPPER_TIMER_HZ ||
        timer_clock_hz % STEPPER_TIMER_HZ != 0U)
    {
        return false;
    }

    __HAL_RCC_TIM5_CLK_ENABLE();
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = timer_clock_hz / STEPPER_TIMER_HZ - 1U;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = UINT32_MAX;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
    {
        return false;
    }

    TIM_OC_InitTypeDef channel = {0};
    channel.OCMode = TIM_OCMODE_FORCED_INACTIVE;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim5, &channel, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_OC_Start(&htim5, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }

    HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    initialized = true;
    stepper_set_direction(true);
    force_step_low();
    return true;
}

bool stepper_ready(void)
{
    return initialized && !timing_fault;
}

void stepper_enable(bool enable)
{
    if (!enable)
    {
        force_step_low();
    }
    HAL_GPIO_WritePin(BOARD_ENABLE_GPIO_PORT, BOARD_ENABLE_PIN,
                      enable && !emergency_stopped ?
                      GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool stepper_set_direction(bool positive)
{
    if (!initialized || running)
    {
        return false;
    }
    direction_positive = positive;
    HAL_GPIO_WritePin(BOARD_DIR_GPIO_PORT, BOARD_DIR_PIN,
                      positive ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}

bool stepper_set_rate(float pulses_per_second)
{
    if (!stepper_ready() || emergency_stopped)
    {
        return false;
    }
    if (pulses_per_second <= 0.0f)
    {
        stepper_stop();
        return true;
    }

    float requested_period = (float)STEPPER_TIMER_HZ / pulses_per_second;
    if (requested_period > (float)INT32_MAX)
    {
        stepper_stop();
        return true;
    }
    uint32_t new_period = (uint32_t)(requested_period + 0.5f);
    if (new_period < STEP_MIN_PERIOD_US)
    {
        new_period = STEP_MIN_PERIOD_US;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t now = TIM5->CNT;
    if (!running)
    {
        period_us = new_period;
        pulse_high = false;
        TIM5->CCR1 = now + new_period;
        __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC1);
        set_output_mode(TIM_OCMODE_TOGGLE);
        running = true;
        __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_CC1);
    }
    else if (!pulse_high && period_us != new_period)
    {
        int32_t remaining = (int32_t)(TIM5->CCR1 - now);
        if (remaining <= 0)
        {
            timing_fault = true;
            force_step_low();
        }
        else
        {
            uint64_t scaled = (uint64_t)(uint32_t)remaining * new_period /
                              period_us;
            if (scaled < 2U)
            {
                scaled = 2U;
            }
            if (scaled > (uint32_t)INT32_MAX)
            {
                scaled = (uint32_t)INT32_MAX;
            }
            TIM5->CCR1 = now + (uint32_t)scaled;
            period_us = new_period;
        }
    }
    else
    {
        period_us = new_period;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return !timing_fault;
}

void stepper_stop(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    force_step_low();
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void stepper_emergency_stop_isr(void)
{
    emergency_stopped = true;
    force_step_low();
    HAL_GPIO_WritePin(BOARD_ENABLE_GPIO_PORT, BOARD_ENABLE_PIN,
                      GPIO_PIN_RESET);
}

void stepper_clear_emergency(void)
{
    emergency_stopped = false;
}

int32_t stepper_position_steps(void)
{
    return position_steps;
}

void TIM5_IRQHandler(void)
{
    if ((TIM5->SR & TIM_SR_CC1IF) == 0U ||
        (TIM5->DIER & TIM_DIER_CC1IE) == 0U)
    {
        return;
    }

    TIM5->SR = ~TIM_SR_CC1IF;
    uint32_t edge = TIM5->CCR1;
    if (!running || emergency_stopped)
    {
        force_step_low();
        return;
    }

    if (!pulse_high)
    {
        uint32_t latency = TIM5->CNT - edge;
        if (latency >= STEP_PULSE_US - 1U)
        {
            timing_fault = true;
            force_step_low();
            return;
        }
        pulse_high = true;
        position_steps += direction_positive ? 1 : -1;
        TIM5->CCR1 = edge + STEP_PULSE_US;
    }
    else
    {
        uint32_t latency = TIM5->CNT - edge;
        if (latency >= period_us - STEP_PULSE_US - 1U)
        {
            timing_fault = true;
            force_step_low();
            return;
        }
        pulse_high = false;
        TIM5->CCR1 = edge + period_us - STEP_PULSE_US;
    }
}
