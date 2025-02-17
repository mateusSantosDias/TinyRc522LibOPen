#include<TinyRC522.h>

TinyRC522::TinyRC522(){

    
}

void TinyRC522::init(){
    
    setup_spi();

    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO7);

    gpio_clear(GPIOB, GPIO7);
    gpio_set(GPIOB, GPIO7);//Reset

    write_register(TModeReg, 0x80);
    write_register(TPrescalerReg, 0xA9);
    write_register(TReloadRegH, 0x03);
    write_register(TReloadRegL, 0xE8);

    write_register(TxASKReg, 0x40);
    write_register(ModeReg, 0x3D);
    antenna_on();
    

}

void TinyRC522::setup_spi(){

      rcc_periph_clock_enable(RCC_SPI2);
	  rcc_periph_clock_enable(RCC_GPIOB);

      gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO12 |
            								GPIO13 |
                                            GPIO15 );
      
      gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO14);
      
      rcc_periph_reset_pulse(RST_SPI2);

      spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_32, 
                      SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, 
                      SPI_CR1_CPHA_CLK_TRANSITION_1, 
                      SPI_CR1_DFF_8BIT, 
                      SPI_CR1_MSBFIRST);
      
      spi_enable_software_slave_management(SPI2);
	  spi_set_nss_high(SPI2);

      spi_enable(SPI2);
      
}

void TinyRC522::write_register(uint8_t reg, uint8_t value){
      
      reg &= 0x7E;
      
      spi_xfer(SPI2, reg);
      spi_xfer(SPI2, value);
}

uint8_t TinyRC522::read_register(uint8_t reg){
      
      reg |= 0x80;
      
      spi_xfer(SPI2, reg);

      uint8_t result = spi_xfer(SPI2, 0);
      return result;
}

uint8_t TinyRC522::get_version(){
     
     return read_register(VersionReg);
}

void TinyRC522::antenna_on(){

    uint8_t value = read_register(TxControlReg);
    if((value & 0x03) != 0x03){

        write_register(TxControlReg, value | 0x03);
    }
}