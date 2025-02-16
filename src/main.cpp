#include <TinyRC522.h>
#include <libopencm3/stm32/usart.h>
#include <Printf.h>

#include <FreeRTOS.h>
#include <task.h>

#define TAG_SIZE 64

typedef struct
{

    TinyRC522 g_rc522;
    uint32_t usuart_i;
    uint32_t value;
} nfc;

nfc nfc_rc522;
nfc *ptr_nfc_rc522 = &nfc_rc522;

void vApplicationStackOverflowHook(
    TaskHandle_t xTask __attribute__((unused)),
    char *pcTaskName __attribute__((unused)))
{

    while (1)
        ;
}

void usuart_setup()
{

    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_GPIOA);

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

void test_nfc(void *args)
{

    nfc *rc522;
    rc522 = (nfc *)args;
    while (1)
    {

        printf("hello mundo");
    }
    vTaskDelay(pdMS_TO_TICKS(500));
}

int main()
{

    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    usuart_setup();

    ptr_usart = USART1;

    ptr_nfc_rc522->g_rc522.init();
    ptr_nfc_rc522->usuart_i = USART1;

    xTaskCreate(test_nfc, "NfcTestHandle", 100, (void *)ptr_nfc_rc522, 2, NULL);
    vTaskStartScheduler();

    while (1)
        ;
    return 0;
}