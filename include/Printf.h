#ifndef PRINTF_H
#define PRINTF_H
/*Defines do Módulo, Inclusões e typedefs*/
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <libopencm3/stm32/usart.h>

#endif
/*Protóripos de Função*/

#ifdef DEF_PRINTF
#define PRINTF extern
#else
#define PRINTF
#endif /* TAG_DRIVER_H */
       /*Variáveis Globais*/

uint32_t ptr_usart;