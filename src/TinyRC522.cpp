#include <TinyRC522.h>

TinyRC522::TinyRC522()
{
}

void TinyRC522::init()
{
    setup_spi();

    reset_rc522();

    write_register(TModeReg, 0x80);
    write_register(TPrescalerReg, 0xA9);
    write_register(TReloadRegH, 0x03);
    write_register(TReloadRegL, 0xE8);
    write_register(TxAutoReg, 0x40);
    write_register(ModeReg, 0x3D);

    antenna_on();
}

void TinyRC522::setup_spi()
{

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7 | GPIO5 | GPIO6);

    gpio_set_mode(NSS_RST_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, NSS_PIN);

    gpio_set_mode(NSS_RST_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, NSS_PIN);

    rcc_periph_reset_pulse(RST_SPI);

    spi_init_master(SPI_SELECTED, SPI_CR1_BAUDRATE_FPCLK_DIV_64,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_DFF_8BIT,
                    SPI_CR1_MSBFIRST);

    spi_disable_crc(SPI_SELECTED);
    spi_set_full_duplex_mode(SPI_SELECTED);

    spi_enable_software_slave_management(SPI_SELECTED);
    spi_set_nss_high(SPI_SELECTED);

    spi_enable(SPI_SELECTED);
}


uint8_t TinyRC522::spi_transfer(uint8_t byte_s)
{
    uint16_t rx;
    rx = spi_xfer(SPI_SELECTED, (uint8_t)byte_s);
    return rx;
}

void TinyRC522::write_register(uint8_t reg, uint8_t value)
{

    reg = ((reg << 1) & 0x7E);

    CLEAR_NSS;
    spi_transfer(reg);
    spi_transfer(value);
    SET_NSS;
}

uint8_t TinyRC522::read_register(uint8_t reg)
{
    uint16_t result;
    reg = (((reg << 1) & 0x7E) | 0x80);

    CLEAR_NSS;
    spi_transfer(reg);
    result = spi_transfer(0x00);
    SET_NSS;

    return result;
}

uint8_t TinyRC522::get_version()
{
    return read_register(VersionReg);
}

void TinyRC522::reset_rc522()
{
    gpio_clear(NSS_RST_PORT, RST_PIN);
    gpio_set(NSS_RST_PORT, RST_PIN);
    SET_NSS;

    write_register(CommandReg, PCD_RESETPHASE);
}

void TinyRC522::antenna_on()
{
    set_bit_mask(TxControlReg, 0x03);
}

void TinyRC522::set_bit_mask(uint8_t reg, uint8_t mask)
{
    uint8_t tmp;
    tmp = read_register(reg);
    write_register(reg, tmp | mask);
}

void TinyRC522::clear_bit_mask(uint8_t reg, uint8_t mask)
{
    uint8_t tmp;
    tmp = read_register(reg);
    write_register(reg, tmp & (~mask));
}