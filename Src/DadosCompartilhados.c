//Módulo: Dados compartilhados
//Descrição: Módulo de gerenciamento de dados compartilhados entre as tarefas

//include guard
#define DEF_DADOSCOMPARTILHADOS

//includes
#include <string.h>
#include "stm32f0xx_hal.h"
#include "DadosCompartilhados.h"

//defines

//typedefs

//variáveis locais

//variaveis externas

//prototypes locais


//implementações:

//Função: inicialização dos dados compartilhados
//Descrição: inicializa todos os dados compartilhados
//Parâmetros: nenhum
//Retorno: nenhum
void InicializaDadosCompartilhados(void)
{
		memset((unsigned char *)&ConjuntoDadosCompartilhados,0,sizeof(TDadosCompartilhados));
}

/*
*  Gets e Sets
*/

//Função: Escreve valor de leitura de ADC
//Parâmetros: valor da leitura
//Retorno: nenhum
void SetLeituraADC(long ValorLeitura)
{
		ConjuntoDadosCompartilhados.LeituraADC = ValorLeitura;
}

//Função: Le valor da ultima leitura de ADC
//Parâmetros: nenhum
//Retorno: valor da ultima leitura de ADC
long GetLeituraADC(long ValorLeitura)
{
		return ConjuntoDadosCompartilhados.LeituraADC;
}

