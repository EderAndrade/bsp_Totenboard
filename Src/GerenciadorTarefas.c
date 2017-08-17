//M�dulo: Gerenciador de tarefas
//Descri��o: M�dulo respons�vel por fazer o gerenciamento de todas as tarefas do firmware. 
//           A execu��o das tarefas � orientada � tempoziza��o, compondo um kernel cooperativo.

//include guard
#define DEF_GERENCIADORTAREFAS

//includes
#include "GerenciadorTarefas.h"
#include "stm32f0xx_hal.h"
#include "DadosCompartilhados.h"

//defines
//typedefs
//vari�veis locais
//variaveis externas

//prototypes locais
void Tarefa1(void);
void Tarefa2(void);
void Tarefa3(void);
void Tarefa4(void);
void Tarefa5(void);
unsigned long ObtemTick(void);
unsigned long TempoGastoTarefa(unsigned long TickInicialTarefa, unsigned long TickFinalTarefa);

//implementa��es:

//Fun��o		: inicializa��o do gerenciador de tarefas
//Descri��o	: faz a inicializa��o do gerenciador de tarefas: escaloma as tarefas
//           	e suas respectivas temporiza��es. Al�m disso, configura o timeout 
//           	da execu��o de tarefas tamb�m.
//           	IMPORTANTE: todas as tarefas devem ter par�metros e retorno do tipo void 
//Par�metros	: nenhum
//Retorno		: nenhum
void IniciaGerenciadorTarefas(void)
{
	//Inicializa��o dos ponteiros de fun��es (tarefas)
	SetupGerenciadorDeTarefas.TarefasAgendadas[INDEX_TAREFA_1] = Tarefa1;
	SetupGerenciadorDeTarefas.TarefasAgendadas[INDEX_TAREFA_2] = Tarefa2;
	SetupGerenciadorDeTarefas.TarefasAgendadas[INDEX_TAREFA_3] = Tarefa3;
	SetupGerenciadorDeTarefas.TarefasAgendadas[INDEX_TAREFA_4] = Tarefa4;
	
	//Inicializa��o dos tempos de execu��o de cada tarefa
	SetupGerenciadorDeTarefas.TempoDeExecucaoTarefas[INDEX_TAREFA_1] = TEMPO_PARA_EXECUTAR_TAREFA1;
	SetupGerenciadorDeTarefas.TempoDeExecucaoTarefas[INDEX_TAREFA_2] = TEMPO_PARA_EXECUTAR_TAREFA2;
	SetupGerenciadorDeTarefas.TempoDeExecucaoTarefas[INDEX_TAREFA_3] = TEMPO_PARA_EXECUTAR_TAREFA3;
	SetupGerenciadorDeTarefas.TempoDeExecucaoTarefas[INDEX_TAREFA_4] = TEMPO_PARA_EXECUTAR_TAREFA4;
	
	
	//Inicializa tempo restante para cada tarefa executar
	SetupGerenciadorDeTarefas.TempoParaExecutarTarefas[INDEX_TAREFA_1] = TEMPO_PARA_EXECUTAR_TAREFA1;
	SetupGerenciadorDeTarefas.TempoParaExecutarTarefas[INDEX_TAREFA_2] = TEMPO_PARA_EXECUTAR_TAREFA2;
	SetupGerenciadorDeTarefas.TempoParaExecutarTarefas[INDEX_TAREFA_3] = TEMPO_PARA_EXECUTAR_TAREFA3;	
     SetupGerenciadorDeTarefas.TempoParaExecutarTarefas[INDEX_TAREFA_4] = TEMPO_PARA_EXECUTAR_TAREFA4;	
  
	//nenhuma tarefa est� em execu��o no momento
     SetupGerenciadorDeTarefas.TimeoutDaTarefa = NAO;
}



//Function	: Execu��o de tarefa
//Descri��o	: verifica se deve executar alguma tarefa. Em caso positivo, a execu��o � feita
//Par�metros	: nenhum
//Retorno		: nenhum
void ExecutaTarefa(void)
{
	
	long i;
	unsigned long TickInicialTarefa;
	unsigned long TickFinalTarefa;
	
	for(i = 0; i < NUMERO_DE_TAREFAS; i++)
	{
		//verifica se est� na hora de executar alguma tarefa
		if((SetupGerenciadorDeTarefas.TarefasAgendadas[i] != 0) && (SetupGerenciadorDeTarefas.TempoParaExecutarTarefas[i] == 0))
		{
			//obtem o valor do tick   
			TickInicialTarefa = ObtemTick();
				 
			//executa a tarefa
			SetupGerenciadorDeTarefas.HaTarefaEmExecucao = SIM;
			SetupGerenciadorDeTarefas.TimeoutDaTarefa 	= TIMEOUT_DE_TAREFA;
			SetupGerenciadorDeTarefas.TarefasAgendadas[i]();  //executa a tarefa agendada
			SetupGerenciadorDeTarefas.HaTarefaEmExecucao = NAO;
			SetupGerenciadorDeTarefas.TempoParaExecutarTarefas[i] = SetupGerenciadorDeTarefas.TempoDeExecucaoTarefas[i];  //reagendamento da tarefa
				 
			//contabiliza tempo de execu��o da tarefa (em ms)
			TickFinalTarefa = ObtemTick();
			SetupGerenciadorDeTarefas.TempoGastoPorTarefa[i] = TempoGastoTarefa(TickInicialTarefa,TickFinalTarefa);
		}
	}
}



//Fun��o		: tarefa 1
//Descri��o	: cont�m rotinas da tarefa 1
//Par�metros	: nenhum
//Retorno		: nenhum
void Tarefa1(void)
{
	
}



//Fun��o		: tarefa 2
//Descri��o	: cont�m rotinas da tarefa 2
//Par�metros	: nenhum
//Retorno		: nenhum
void Tarefa2(void)
{
		
}



//Fun��o		: tarefa 3
//Descri��o	: cont�m rotinas da tarefa 3
//Par�metros	: nenhum
//Retorno		: nenhum
void Tarefa3(void)
{
		
}



//Fun��o		: tarefa 4
//Descri��o	: cont�m rotinas da tarefa 4
//Par�metros	: nenhum
//Retorno		: nenhum
void Tarefa4(void)
{
		
}
 


//Fun��o		: obtem valor atual do tick
//Par�metros	: nenhum
//Retorno		: valor atual do tick
unsigned long ObtemTick(void)
{
	return HAL_GetTick();
}



//Fun��o		: calcula tempo gasto numa tarefa
//Par�metros	: tempos inicial e final (em ms) de uma tarefa
//Retorno		: tempo gasto numa tarefa (em ms)
unsigned long TempoGastoTarefa(unsigned long TickInicialTarefa, unsigned long TickFinalTarefa)
{
	if(TickFinalTarefa > TickInicialTarefa)
	{
		return(TickFinalTarefa-TickInicialTarefa);
	}
	else
	{
		return(0xFFFFFFFF-TickInicialTarefa) + TickFinalTarefa + 1;  
	}
}


