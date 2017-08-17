//M�dulo: Dados compartilhados
//Descri��o: M�dulo de gerenciamento de dados compartilhados entre as tarefas

//include guard
#define DEF_DADOSCOMPARTILHADOS

//includes
#include <string.h>
#include "stm32f0xx_hal.h"
#include "DadosCompartilhados.h"

//defines

//typedefs

//vari�veis locais

//variaveis externas

//prototypes locais


//implementa��es:

//Fun��o: inicializa��o dos dados compartilhados
//Descri��o: inicializa todos os dados compartilhados
//Par�metros: nenhum
//Retorno: nenhum
void InicializaDadosCompartilhados(void)
{
		memset((unsigned char *)&ConjuntoDadosCompartilhados,0,sizeof(TDadosCompartilhados));
}

/*
*  Gets e Sets
*/

//Fun��o: Escreve valor de leitura de ADC
//Par�metros: valor da leitura
//Retorno: nenhum
void SetLeituraADC(long ValorLeitura)
{
		ConjuntoDadosCompartilhados.LeituraADC = ValorLeitura;
}

//Fun��o: Le valor da ultima leitura de ADC
//Par�metros: nenhum
//Retorno: valor da ultima leitura de ADC
long GetLeituraADC(long ValorLeitura)
{
		return ConjuntoDadosCompartilhados.LeituraADC;
}

