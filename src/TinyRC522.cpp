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

    rcc_periph_clock_enable(RCC_SPI);
    rcc_periph_clock_enable(RCC_PORT);
    rcc_periph_clock_enable(NSS_RST_RCC_PORT);

    gpio_set_mode(SPI_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, MOSI | MISO | SCK);

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
    uint8_t rx;
    rx = spi_xfer(SPI_SELECTED, byte_s);
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

status_type TinyRC522::to_card(uint8_t command, uint8_t *send_data, uint8_t send_len, uint8_t *back_data, uint16_t *back_len)
{
    uint8_t irq_en = 0x00;
    uint8_t wait_irq = 0x00;
    uint8_t last_bits;
    uint8_t n;

    uint8_t i;

    status_type status;

    switch (command)
    {
    case PCD_AUTHENT:
        irq_en = 0x12;
        wait_irq = 0x10;
        break;

    case PCD_TRANSCEIVE:
        irq_en = 0x77;
        wait_irq = 0x10;
        break;

    default:
        break;
    }

    write_register(CommIEnReg, irq_en | 0x80);
    clear_bit_mask(CommIrqReg, 0x80);
    set_bit_mask(FIFOLevelReg, 0x80);

    write_register(CommandReg, PCD_IDLE);

    for (i = 0; i < send_len; i++)
    {
        write_register(FIFODataReg, send_data[i]);
    }

    write_register(CommandReg, command);
    if (command == PCD_TRANSCEIVE)
    {
        set_bit_mask(BitFramingReg, 0x80);
        vTaskDelay(pdMS_TO_TICKS(5)); //Place a five milleseconds delay, the rc522 no have suficient speed for verification irq.
    }

    i = 2000;
    do
    {
        n = read_register(CommIrqReg);
        i--;
    } while ((i != 0) && (n & 0x01) && !(n & wait_irq));

    clear_bit_mask(BitFramingReg, 0x80);
    if (i != 0)
    {
        if (!(read_register(ErrorReg) & 0x1B))
        {
            status = OK;

            if (n & irq_en & 0x01)
            {
                status = NONE;
            }
            if (command == PCD_TRANSCEIVE)
            {   
                
                n = read_register(FIFOLevelReg);
                last_bits = read_register(ControlReg) & 0x07;

                if (last_bits)
                {
                    *back_len = (n - 1) * 8 + last_bits;
                }
                else
                {
                    *back_len = n * 8;
                }
                if (n == 0)
                {
                    n = 1;
                }
                else if (n > 16)
                {
                    n = 16;
                }
                for (i = 0; i < n; i++)
                {
                    back_data[i] = read_register(FIFODataReg);
                    
                }
            }
        }
        else
        {
            status = ERR;
        }
    }
    else
    {
        printf("~~~request timed out\r\n");
    }

    return status;
}

status_type TinyRC522::request_card(uint8_t req_mode, uint8_t *tag_type)
{
    status_type status;
    uint16_t backbits;

    write_register(BitFramingReg, 0x07);

    tag_type[0] = req_mode;

    status = to_card(PCD_TRANSCEIVE, tag_type, 1, tag_type, &backbits);
    if ((status != OK) || (backbits != 0x10))
    {
        status = ERR;
    }
    return status;
}

status_type TinyRC522::anticoll(uint8_t *ser_num)
{
    status_type status;
    uint8_t i;
    uint8_t ser_num_check = 0;
    uint8_t un_len = 0x00;

    write_register(BitFramingReg, 0x00);

    ser_num[0] = PICC_ANTICOLL;
    ser_num[1] = 0x20;
    status = to_card(PCD_TRANSCEIVE, ser_num, 2, ser_num,(uint16_t*) &un_len);

    if (status == OK)
    {
        for (i = 0; i < 4; i++)
        {
            ser_num_check ^= ser_num[i];
        }
        if (ser_num_check != ser_num[i])
        {
            status = ERR;
        }
    }

    return status;
}