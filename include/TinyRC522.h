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

#define NSS_PIN GPIO0
#define RST_PIN GPIO14

#define SET_NSS gpio_set(GPIOB, NSS_PIN)
#define CLEAR_NSS gpio_clear(GPIOB, NSS_PIN)

#endif
/*Protóripos de Função*/
class TinyRC522
{

private:
	void setup_spi();
	void reset_rc522();
	void set_bit_mask(uint8_t reg, uint8_t mask);
	void clear_bit_mask(uint8_t reg, uint8_t mask);

	uint8_t spi_transfer(uint8_t byte_s);
public:
	TinyRC522();

	void init();
	void antenna_on();
	void write_register(uint8_t reg, uint8_t value);

	uint8_t get_version();
	uint8_t read_register(uint8_t reg);
};

#ifdef DEF_TAG_DVR
#define TAG_DVR extern
#else
#define TAG_DVR
#endif /* TAG_DRIVER_H */
	   /*Variáveis Globais*/
