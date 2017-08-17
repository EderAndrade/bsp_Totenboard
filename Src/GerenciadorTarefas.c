//Módulo: Gerenciador de tarefas
//Descrição: Módulo responsável por fazer o gerenciamento de todas as tarefas do firmware. 
//           A execução das tarefas é orientada à tempozização, compondo um kernel cooperativo.

//include guard
#define DEF_GERENCIADORTAREFAS

//includes
#include "GerenciadorTarefas.h"
#include "stm32f0xx_hal.h"
#include "DadosCompartilhados.h"

//defines
//typedefs
//variáveis locais
//variaveis externas

//prototypes locais
void Tarefa1(void);
void Tarefa2(void);
void Tarefa3(void);
void Tarefa4(void);
void Tarefa5(void);
unsigned long ObtemTick(void);
unsigned long TempoGastoTarefa(unsigned long TickInicialTarefa, unsigned long TickFinalTarefa);

//implementações:

//Função		: inicialização do gerenciador de tarefas
//Descrição	: faz a inicialização do gerenciador de tarefas: escaloma as tarefas
//           	e suas respectivas temporizações. Além disso, configura o timeout 
//           	da execução de tarefas também.
//           	IMPORTANTE: todas as tarefas devem ter parÂmetros e retorno do tipo void 
//Parâmetros	: nenhum
//Retorno		: nenhum
void IniciaGerenciadorTarefas(void)
{
	//Inicialização dos ponteiros de funções (tarefas)
	SetupGerenciadorDeTarefas.TarefasAgendadas[INDEX_TAREFA_1] = Tarefa1;
	SetupGerenciadorDeTarefas.TarefasAgendadas[INDEX_TAREFA_2] = Tarefa2;
	SetupGerenciadorDeTarefas.TarefasAgendadas[INDEX_TAREFA_3] = Tarefa3;
	SetupGerenciadorDeTarefas.TarefasAgendadas[INDEX_TAREFA_4] = Tarefa4;
	
	//Inicialização dos tempos de execução de cada tarefa
	SetupGerenciadorDeTarefas.TempoDeExecucaoTarefas[INDEX_TAREFA_1] = TEMPO_PARA_EXECUTAR_TAREFA1;
	SetupGerenciadorDeTarefas.TempoDeExecucaoTarefas[INDEX_TAREFA_2] = TEMPO_PARA_EXECUTAR_TAREFA2;
	SetupGerenciadorDeTarefas.TempoDeExecucaoTarefas[INDEX_TAREFA_3] = TEMPO_PARA_EXECUTAR_TAREFA3;
	SetupGerenciadorDeTarefas.TempoDeExecucaoTarefas[INDEX_TAREFA_4] = TEMPO_PARA_EXECUTAR_TAREFA4;
	
	
	//Inicializa tempo restante para cada tarefa executar
	SetupGerenciadorDeTarefas.TempoParaExecutarTarefas[INDEX_TAREFA_1] = TEMPO_PARA_EXECUTAR_TAREFA1;
	SetupGerenciadorDeTarefas.TempoParaExecutarTarefas[INDEX_TAREFA_2] = TEMPO_PARA_EXECUTAR_TAREFA2;
	SetupGerenciadorDeTarefas.TempoParaExecutarTarefas[INDEX_TAREFA_3] = TEMPO_PARA_EXECUTAR_TAREFA3;	
     SetupGerenciadorDeTarefas.TempoParaExecutarTarefas[INDEX_TAREFA_4] = TEMPO_PARA_EXECUTAR_TAREFA4;	
  
	//nenhuma tarefa está em execução no momento
     SetupGerenciadorDeTarefas.TimeoutDaTarefa = NAO;
}



//Function	: Execução de tarefa
//Descrição	: verifica se deve executar alguma tarefa. Em caso positivo, a execução é feita
//Parâmetros	: nenhum
//Retorno		: nenhum
void ExecutaTarefa(void)
{
	
	long i;
	unsigned long TickInicialTarefa;
	unsigned long TickFinalTarefa;
	
	for(i = 0; i < NUMERO_DE_TAREFAS; i++)
	{
		//verifica se está na hora de executar alguma tarefa
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
				 
			//contabiliza tempo de execução da tarefa (em ms)
			TickFinalTarefa = ObtemTick();
			SetupGerenciadorDeTarefas.TempoGastoPorTarefa[i] = TempoGastoTarefa(TickInicialTarefa,TickFinalTarefa);
		}
	}
}



//Função		: tarefa 1
//Descrição	: contém rotinas da tarefa 1
//Parâmetros	: nenhum
//Retorno		: nenhum
void Tarefa1(void)
{
	
}



//Função		: tarefa 2
//Descrição	: contém rotinas da tarefa 2
//Parâmetros	: nenhum
//Retorno		: nenhum
void Tarefa2(void)
{
		
}



//Função		: tarefa 3
//Descrição	: contém rotinas da tarefa 3
//Parâmetros	: nenhum
//Retorno		: nenhum
void Tarefa3(void)
{
		
}



//Função		: tarefa 4
//Descrição	: contém rotinas da tarefa 4
//Parâmetros	: nenhum
//Retorno		: nenhum
void Tarefa4(void)
{
		
}
 


//Função		: obtem valor atual do tick
//Parâmetros	: nenhum
//Retorno		: valor atual do tick
unsigned long ObtemTick(void)
{
	return HAL_GetTick();
}



//Função		: calcula tempo gasto numa tarefa
//Parâmetros	: tempos inicial e final (em ms) de uma tarefa
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


