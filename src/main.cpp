#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>

#include <TinyRC522.h>
#include <Printf.h>
#include <Errors.h>

#include <FreeRTOS.h>
#include <task.h>

#define LED GPIO13

TinyRC522 rc522;

void vApplicationStackOverflowHook(
    TaskHandle_t xTask __attribute__((unused)),
    char *pcTaskName __attribute__((unused)))
{

    while (1)
        ;
}

static void usuart_setup()
{
    gpio_set_mode(
        GPIOA,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO_USART1_TX);

    gpio_set_mode(

        GPIOA,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_INPUT_FLOAT,
        GPIO_USART1_RX);

    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    usart_enable(USART1);
}

static void Nfc_tester(void *args)
{

    while (1)
    {   
        gpio_toggle(GPIOC, LED);
        printf("RC522 Version: 0x%X\n", rc522.get_version());
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void clock_setup()
{
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_SPI1);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
}

int main()
{
    clock_setup();
    usuart_setup();
    
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, LED);

    rc522.init();

    xTaskCreate(Nfc_tester, "NfcTester", 100, NULL, 2, NULL);
    vTaskStartScheduler();

    while (1)
        ;
    return 0;
}