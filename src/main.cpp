#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>

#include <Printf.h>
#include <Errors.h>

#include <FreeRTOS.h>
#include <task.h>

#define SP1_SELECTED
#include "TinyRC522.h"


#define LED GPIO13

TinyRC522 rc522;

void vApplicationStackOverflowHook(
    TaskHandle_t xTask __attribute__((unused)),
    char *pcTaskName __attribute__((unused)))
{

    while (1)
        ;
}

void usuart_setup()
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

    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    usart_enable(USART1);
}

void nfc_tester(void *args)
{   
    status_type status;
    uint8_t str[16];

    rc522.init();
    printf("version 0x%x\n", rc522.get_version());

    while (1)
    {
        gpio_toggle(GPIOC, LED);

        status = rc522.request_card(PICC_REQIDL, str); 

        if(status == OK)
        {
            printf("Cartão detectado! Tipo do cartão: %d\n",str[0]);

            status = rc522.anticoll(str);
            if(status == OK)
            {
                printf("Numero serial do cartão: %d:%d:%d:%d\n", str[0], str[1], str[2], str[3]);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void clock_setup()
{
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_GPIOC);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_SPI1);
    rcc_periph_clock_enable(RCC_AFIO);
}

int main()
{
    clock_setup();
    usuart_setup();

    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, LED);

    printf("TinyRc522 - Library\n");

    xTaskCreate(nfc_tester, "NFC_TESTER", 4096, NULL, 2, NULL);
    vTaskStartScheduler();

    while (1)
        ;
    return 0;
}