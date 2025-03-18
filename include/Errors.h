#ifndef ERRORS_H
#define ERRORS_H
/*Defines do Módulo, Inclusões e typedefs*/
#include "Printf.h"

typedef enum {
    OK,
    ERR,
    NONE
}status_type;
#endif
/*Protóripos de Função*/
void __Error_Handle();

#ifdef DEF_ERRORS
#define ERRORS extern
#else
#define ERRORS
#endif /* TAG_DRIVER_H */
       /*Variáveis Globais*/

