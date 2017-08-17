//Módulo: Header dos dados compartilhados
//Descrição: Header dos dados compartilhados entre todas as tarefas

//includes
#include "stm32f0xx_hal.h"

//defines gerais
#ifndef DADOSCOMPARTILHADOS_H
#define DADOSCOMPARTILHADOS_H

//typedefs
typedef struct 
{
	  long LeituraADC;	
}TDadosCompartilhados;


#ifdef  DEF_DADOSCOMPARTILHADOS
#define DADOSCOMP   /**/
#else
#define DADOSCOMP   extern
#endif

//variaveis globais		
DADOSCOMP TDadosCompartilhados ConjuntoDadosCompartilhados;
		
#endif

//prototypes globais
void InicializaDadosCompartilhados(void);
void SetLeituraADC(long ValorLeitura);
long GetLeituraADC(long ValorLeitura);

