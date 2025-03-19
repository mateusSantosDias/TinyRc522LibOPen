/*
 * Projeto: Sistema de Captação de impacto.
 * Desenvolvedor: Mateus Santos Dias
 * Versão: V.01
 *
 */
#ifndef TAG_DRIVER_H
#define TAG_DRIVER_H
/*Defines do Módulo, Inclusões e typedefs*/

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>

#include <RC522.h>
#include <Errors.h>

#include "FreeRTOS.h"
#include "task.h"

/************************Default Values**************************
 * For New values redefine this macros for your, reset and css custom
 * pins.
*/
#define NSS_RST_PORT GPIOB
#define NSS_PIN GPIO0
#define RST_PIN GPIO1
//**************************************************************//

//************************SPI-Configure**************************//
#ifndef SP1_SELECTED
#define SPI_SELECTED SPI1
#define RST_SPI RST_SPI1

#elif defined(SP2_SELECTED)
#define SPI_SELECTED SPI2
#define RST_SPI RST_SPI2

#elif defined(SP3_SELECTED)
#define SPI_SELECTED SPI3
#define RST_SPI RST_SPI2
//**************************************************************//
#endif

#define SET_NSS gpio_set(NSS_RST_PORT, NSS_PIN)
#define CLEAR_NSS gpio_clear(NSS_RST_PORT, NSS_PIN)

#endif
/*Protóripos de Função*/
class TinyRC522
{

private:
	void reset_rc522();
	void set_nss();
	void clear_nss();
	void setup_spi();
	void set_bit_mask(uint8_t reg, uint8_t mask);
	void clear_bit_mask(uint8_t reg, uint8_t mask);
	void write_register(uint8_t reg, uint8_t value);

	uint8_t read_register(uint8_t reg);
	uint8_t spi_transfer(uint8_t byte_s);

public:
	TinyRC522();

	void antenna_on();
	void init();

	uint8_t get_version();
};

#ifdef DEF_TAG_DVR
#define TAG_DVR extern
#else
#define TAG_DVR
#endif /* TAG_DRIVER_H */
/*Variáveis Globais*/
