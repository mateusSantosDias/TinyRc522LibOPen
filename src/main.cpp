#include<TinyRC522.h>
#include<libopencm3/stm32/usart.h>

#include<FreeRTOS.h>
#include<task.h>

#define TAG_SIZE 64



typedef struct {

    RC522 g_rc522;
    uint32_t usuart_i;
}nfc;

nfc nfc_rc522;
nfc *ptr_nfc_rc522 = &nfc_rc522;

RC522 g_rc522;

void vApplicationStackOverflowHook(
	TaskHandle_t xTask __attribute__((unused)),
    char *pcTaskName __attribute__((unused))) {

	while(1);
}

void dump_tag_data(uint8_t* buffer, uint16_t buffer_size)
{
}

static void my_usart_print_int(uint32_t usart, uint32_t value)
{
	int8_t i;
	uint8_t nr_digits = 0;
	char buffer[25];

	if (value < 0) {
		usart_send(usart, '-');
		value = value * -1;
	}

	while (value > 0) {
		buffer[nr_digits++] = "0123456789"[value % 10];
		value /= 10;
	}

	for (i = nr_digits; i >= 0; i--){
        usart_send(usart, buffer[i]);
  
        //vTaskDelay(pdMS_TO_TICKS(10));
         for (int i = 0; i < 800000; i++) __asm__("nop");
    }	
}

void usuart_setup(){
    
    
    rcc_periph_clock_enable(RCC_USART1);
    
    rcc_periph_clock_enable(RCC_GPIOA);
    
    gpio_set_mode(
         GPIOA, 
         GPIO_MODE_OUTPUT_50_MHZ,
         GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
         GPIO_USART1_TX  
    );

    gpio_set_mode(
         
         GPIOA,
         GPIO_MODE_OUTPUT_50_MHZ,
         GPIO_CNF_INPUT_FLOAT,
         GPIO_USART1_RX
    );

    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    usart_enable(USART1);

}

void test_nfc(void *args){
     
    nfc *rc522;
    rc522 = (nfc*)args;

    usart_send(rc522->usuart_i, 'S');
    vTaskDelay(pdMS_TO_TICKS(10));
    usart_send(rc522->usuart_i, 'T');
    vTaskDelay(pdMS_TO_TICKS(10));
    usart_send(rc522->usuart_i, 'M');
    vTaskDelay(pdMS_TO_TICKS(10));
    usart_send(rc522->usuart_i, ' ');
    vTaskDelay(pdMS_TO_TICKS(10));
    usart_send(rc522->usuart_i, 'N');
    vTaskDelay(pdMS_TO_TICKS(10));
    usart_send(rc522->usuart_i, 'F');
    vTaskDelay(pdMS_TO_TICKS(10));
    usart_send(rc522->usuart_i, 'C');
    vTaskDelay(pdMS_TO_TICKS(10));
    usart_send(rc522->usuart_i, ' ');
    vTaskDelay(pdMS_TO_TICKS(10));

    while (1){
      
      Tag* p_tag = rc522->g_rc522.get_tag();
      my_usart_print_int(rc522->usuart_i, spi_read(SPI1));
      
      if(p_tag){

        my_usart_print_int(rc522->usuart_i, p_tag->get_uid());
        uint8_t data_buffer[TAG_SIZE];
        
        p_tag->release();
      }
    }    

    vTaskDelay(pdMS_TO_TICKS(100));
}

int main(){

    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    gpio_set(GPIOC, GPIO13);
    
    usuart_setup();
    
    g_rc522.initialize();
    g_rc522.set_antenna_gain(ANTENNA_GAIN_18_DB);
    

    while(1){
      
         Tag* p_tag=g_rc522.get_tag();
         if(p_tag){

            my_usart_print_int(USART1, p_tag->get_uid());

         }
    }
    return 0;
}