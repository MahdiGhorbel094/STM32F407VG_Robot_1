#include "stm32f407xx.h"
#include <stdio.h>
#include <string.h>

/* ===================== Function Prototypes ===================== */

void config_EXTI0(void);
void config_TIM6_IT(void);
void config_ADC1_SCAN_DISCONT_IT(void);
void config_USART2_TXRX(void);
void config_TIM3_PWM(void);

void send_char(char c);
void send_string(const char *str);
void prepare_and_send_buffer(void);

void process_AVANCE(void);
void process_ARRIERE(void);
void process_GAUCHE(void);
void process_DROITE(void);
void process_STOP(void);

/* ===================== Global Variables ===================== */

static uint16_t ADC_VALUE[3];
static uint8_t adc_index = 0;

#define RX_BUF_SIZE 32

static char rx_buffer[RX_BUF_SIZE];
static uint8_t rx_index = 0;

/* ===================== PWM Values ===================== */

#define PWM_MAX   999U
#define PWM_HIGH  800U
#define PWM_LOW   300U
#define PWM_OFF     0U

/* ==============================================================
 * MAIN
 * ============================================================== */

int main(void)
{
    config_TIM3_PWM();
    config_USART2_TXRX();

    send_string("ROBOT READY\n");

    config_TIM6_IT();
    config_ADC1_SCAN_DISCONT_IT();
    config_EXTI0();

    while (1)
    {
    }
}

/* ==============================================================
 * EXTI0 CONFIGURATION
 * PA0 button interrupt
 * ============================================================== */

void config_EXTI0(void)
{
    RCC->AHB1ENR |= (1U << 0);

    GPIOA->MODER &= ~(3U << 0);

    RCC->APB2ENR |= (1U << 14);

    SYSCFG->EXTICR[0] &= ~(0xFU << 0);

    EXTI->RTSR |= (1U << 0);
    EXTI->IMR  |= (1U << 0);

    __NVIC_EnableIRQ(EXTI0_IRQn);
}

void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & (1U << 0))
    {
        TIM6->CR1 |= (1U << 0);

        EXTI->PR = (1U << 0);
    }
}

/* ==============================================================
 * TIM6 CONFIGURATION
 * Generates interrupt every 1 second
 * ============================================================== */

void config_TIM6_IT(void)
{
    RCC->APB1ENR |= (1U << 4);

    TIM6->PSC = 15999;
    TIM6->ARR = 999;

    TIM6->SR = 0;

    TIM6->DIER = (1U << 0);

    __NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void TIM6_DAC_IRQHandler(void)
{
    if (TIM6->SR & (1U << 0))
    {
        adc_index = 0;

        /* Start first ADC conversion */
        ADC1->CR2 |= (1U << 30);

        TIM6->SR &= ~(1U << 0);
    }
}

/* ==============================================================
 * ADC1 CONFIGURATION
 * Reads:
 *   CH4  -> PA4
 *   CH14 -> PC4
 *   CH15 -> PC5
 * ============================================================== */

void config_ADC1_SCAN_DISCONT_IT(void)
{
    /* GPIO configuration */

    RCC->AHB1ENR |= (1U << 0);

    GPIOA->MODER |= (3U << 8);

    RCC->AHB1ENR |= (1U << 2);

    GPIOC->MODER |= (3U << 8) | (3U << 10);

    /* ADC clock enable */

    RCC->APB2ENR |= (1U << 8);

    /* Scan mode enabled */

    ADC1->CR1 |= (1U << 8);

    /* Discontinuous mode enabled */

    ADC1->CR1 |= (1U << 11);

    /* One conversion per trigger */

    ADC1->CR1 &= ~(7U << 13);

    /* 12-bit resolution */

    ADC1->CR1 &= ~(3U << 24);

    /* End of conversion interrupt */

    ADC1->CR1 |= (1U << 5);

    /* EOC after every conversion */

    ADC1->CR2 |= (1U << 10);

    /* Sampling time */

    ADC1->SMPR2 &= ~(7U << 12);

    ADC1->SMPR1 &= ~((7U << 12) | (7U << 15));

    /* Conversion sequence */

    ADC1->SQR1 = (2U << 20);

    ADC1->SQR3 =
          (4U  << 0)
        | (14U << 5)
        | (15U << 10);

    __NVIC_EnableIRQ(ADC_IRQn);

    /* ADC enable */

    ADC1->CR2 |= (1U << 0);
}

void ADC_IRQHandler(void)
{
    if (ADC1->SR & ADC_SR_EOC)
    {
        if (adc_index < 3)
        {
            ADC_VALUE[adc_index] = (uint16_t)ADC1->DR;

            adc_index++;
        }
        else
        {
            (void)ADC1->DR;
        }

        if (adc_index == 3)
        {
            prepare_and_send_buffer();
        }
        else
        {
            /* Start next conversion */

            ADC1->CR2 |= (1U << 30);
        }
    }
}

/* ==============================================================
 * USART2 CONFIGURATION
 * TX -> PA2
 * RX -> PA3
 * Baudrate: 9600
 * ============================================================== */

void config_USART2_TXRX(void)
{
    RCC->AHB1ENR |= (1U << 0);

    RCC->APB1ENR |= (1U << 17);

    GPIOA->MODER &= ~((3U << 4) | (3U << 6));

    GPIOA->MODER |= ((2U << 4) | (2U << 6));

    GPIOA->OTYPER &= ~((1U << 2) | (1U << 3));

    GPIOA->OSPEEDR |= ((2U << 4) | (2U << 6));

    GPIOA->AFR[0] &= ~((0xFU << 8) | (0xFU << 12));

    GPIOA->AFR[0] |= ((7U << 8) | (7U << 12));

    USART2->BRR = 0x0683;

    USART2->CR1 &= ~(1U << 12);

    USART2->CR2 &= ~(3U << 12);

    USART2->CR1 &= ~(1U << 10);

    USART2->CR1 |= (1U << 3);

    USART2->CR1 |= (1U << 2);

    USART2->CR1 |= (1U << 5);

    USART2->CR1 |= (1U << 4);

    USART2->CR1 |= (1U << 13);

    __NVIC_EnableIRQ(USART2_IRQn);
}

/* ==============================================================
 * USART SEND FUNCTIONS
 * ============================================================== */

void send_char(char c)
{
    while ((USART2->SR & (1U << 7)) == 0);

    USART2->DR = (uint8_t)c;
}

void send_string(const char *str)
{
    while (*str)
    {
        send_char(*str++);
    }
}

void prepare_and_send_buffer(void)
{
    char buf[64];

    sprintf(buf,
            "CHx=%d CHy=%d CHz=%d\n",
            ADC_VALUE[0],
            ADC_VALUE[1],
            ADC_VALUE[2]);

    send_string(buf);
}

/* ==============================================================
 * USART2 INTERRUPT
 * Receives Bluetooth commands
 * ============================================================== */

void USART2_IRQHandler(void)
{
    uint32_t sr = USART2->SR;

    /* Character received */

    if (sr & (1U << 5))
    {
        char c = (char)USART2->DR;

        if (rx_index < (RX_BUF_SIZE - 1))
        {
            rx_buffer[rx_index++] = c;

            rx_buffer[rx_index] = '\0';
        }
        else
        {
            rx_index = 0;

            rx_buffer[0] = '\0';
        }
    }

    /* End of message */

    if (sr & (1U << 4))
    {
        (void)USART2->SR;
        (void)USART2->DR;

        if (strstr(rx_buffer, "F"))
        {
            process_AVANCE();
        }
        else if (strstr(rx_buffer, "B"))
        {
            process_ARRIERE();
        }
        else if (strstr(rx_buffer, "L"))
        {
            process_GAUCHE();
        }
        else if (strstr(rx_buffer, "R"))
        {
            process_DROITE();
        }
        else if (strstr(rx_buffer, "STOP"))
        {
            process_STOP();
        }

        rx_index = 0;

        rx_buffer[0] = '\0';
    }
}

/* ==============================================================
 * TIM3 PWM CONFIGURATION
 *
 * PA6 -> TIM3_CH1 -> Left motor forward
 * PA7 -> TIM3_CH2 -> Left motor backward
 * PB0 -> TIM3_CH3 -> Right motor forward
 * PB1 -> TIM3_CH4 -> Right motor backward
 * ============================================================== */

void config_TIM3_PWM(void)
{
    /* GPIO clock enable */

    RCC->AHB1ENR |= (1U << 0);

    RCC->AHB1ENR |= (1U << 1);

    /* PA6 and PA7 as alternate function */

    GPIOA->MODER &= ~((3U << 12) | (3U << 14));

    GPIOA->MODER |= ((2U << 12) | (2U << 14));

    GPIOA->OTYPER &= ~((1U << 6) | (1U << 7));

    GPIOA->OSPEEDR |= ((2U << 12) | (2U << 14));

    GPIOA->AFR[0] &= ~((0xFU << 24) | (0xFU << 28));

    GPIOA->AFR[0] |= ((2U << 24) | (2U << 28));

    /* PB0 and PB1 as alternate function */

    GPIOB->MODER &= ~((3U << 0) | (3U << 2));

    GPIOB->MODER |= ((2U << 0) | (2U << 2));

    GPIOB->OTYPER &= ~((1U << 0) | (1U << 1));

    GPIOB->OSPEEDR |= ((2U << 0) | (2U << 2));

    GPIOB->AFR[0] &= ~((0xFU << 0) | (0xFU << 4));

    GPIOB->AFR[0] |= ((2U << 0) | (2U << 4));

    /* TIM3 clock enable */

    RCC->APB1ENR |= (1U << 1);

    /* PWM frequency setup */

    TIM3->PSC = 0;

    TIM3->ARR = PWM_MAX;

    /* PWM mode for CH1 and CH2 */

    TIM3->CCMR1 =
          (6U << 4)
        | (1U << 3)
        | (6U << 12)
        | (1U << 11);

    /* PWM mode for CH3 and CH4 */

    TIM3->CCMR2 =
          (6U << 4)
        | (1U << 3)
        | (6U << 12)
        | (1U << 11);

    /* Enable all channels */

    TIM3->CCER =
          (1U << 0)
        | (1U << 4)
        | (1U << 8)
        | (1U << 12);

    /* Motors stopped at startup */

    TIM3->CCR1 = PWM_OFF;
    TIM3->CCR2 = PWM_OFF;
    TIM3->CCR3 = PWM_OFF;
    TIM3->CCR4 = PWM_OFF;

    /* Enable auto reload preload */

    TIM3->CR1 |= (1U << 7);

    /* Load registers immediately */

    TIM3->EGR = (1U << 0);

    TIM3->SR = 0;

    /* Start timer */

    TIM3->CR1 |= (1U << 0);
}

/* ==============================================================
 * MOTOR CONTROL FUNCTIONS
 * ============================================================== */

/* Move forward */

void process_AVANCE(void)
{
    TIM3->CCR1 = PWM_HIGH;
    TIM3->CCR2 = PWM_OFF;

    TIM3->CCR3 = PWM_HIGH;
    TIM3->CCR4 = PWM_OFF;
}

/* Move backward */

void process_ARRIERE(void)
{
    TIM3->CCR1 = PWM_OFF;
    TIM3->CCR2 = PWM_HIGH;

    TIM3->CCR3 = PWM_OFF;
    TIM3->CCR4 = PWM_HIGH;
}

/* Turn left */

void process_GAUCHE(void)
{
    TIM3->CCR1 = PWM_HIGH;
    TIM3->CCR2 = PWM_OFF;

    TIM3->CCR3 = PWM_OFF;
    TIM3->CCR4 = PWM_HIGH;
}

/* Turn right */

void process_DROITE(void)
{
    TIM3->CCR1 = PWM_OFF;
    TIM3->CCR2 = PWM_HIGH;

    TIM3->CCR3 = PWM_HIGH;
    TIM3->CCR4 = PWM_OFF;
}

/* Stop motors */

void process_STOP(void)
{
    TIM3->CCR1 = PWM_OFF;
    TIM3->CCR2 = PWM_OFF;

    TIM3->CCR3 = PWM_OFF;
    TIM3->CCR4 = PWM_OFF;
}