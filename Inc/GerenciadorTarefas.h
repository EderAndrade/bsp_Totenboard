//M�dulo: Header do gerenciador de tarefas
//Descri��o: Header do m�dulo respons�vel por fazer o gerenciamento de todas as tarefas do firmware. 
//           A execu��o das tarefas � orientada � tempoziza��o, compondo um kernel cooperativo.

//includes
#include "stm32f0xx_hal.h"


#ifndef GERENCIADORTAREFAS_H
#define GERENCIADORTAREFAS_H

//defines gerais
#define SIM                            1
#define NAO                            0

#define INDEX_TAREFA_1                 0
#define INDEX_TAREFA_2                 1
#define INDEX_TAREFA_3                 2
#define INDEX_TAREFA_4                 3
#define INDEX_TAREFA_5                 4
#define NUMERO_DE_TAREFAS              5
#define TEMPO_PARA_EXECUTAR_TAREFA1    500    //ms
#define TEMPO_PARA_EXECUTAR_TAREFA2    1000   //ms
#define TEMPO_PARA_EXECUTAR_TAREFA3    1500   //ms
#define TEMPO_PARA_EXECUTAR_TAREFA4    2000   //ms
#define TEMPO_PARA_EXECUTAR_TAREFA5    50     //ms
#define TIMEOUT_DE_TAREFA              1000   //ms

//defines da tensao de alimenta��o 
#define LIMIAR_TENSAO_ALIMENTACAO  3.0

//typedefs
//ponteiro de fun��o (utilizado para executar uma tarefa)
typedef void (*pFunc)(void); 

typedef struct
{
	unsigned int 		    TempoExecutarTask;
	unsigned int 		    TempoDeExecucaoTarefas[NUMERO_DE_TAREFAS];    	//Tempos em que cada tarefa deve ser executada
	unsigned int 		    TempoParaExecutarTarefas[NUMERO_DE_TAREFAS];	  //Tempos atuais de execu��o das tarefas (periodicidade)
	unsigned long 		  TempoGastoPorTarefa[NUMERO_DE_TAREFAS];     	  //Tempo (em ms) gasto por cada tarefa
	unsigned long int 	HaTarefaEmExecucao;						                  //indica se h� alguma tarefa em execu��o
	unsigned long int 	TimeoutDaTarefa; 						                    //contabiliza o tempo de execu��o da tarefa corrente (para gerenciar timeout)
	pFunc 		    	    TarefasAgendadas[NUMERO_DE_TAREFAS];		        //Ponteiros de fun��es (um para cada tarefa)
}TSetupGerenciadorDeTarefas;

#ifdef  DEF_GERENCIADORTAREFAS
#define GERTAR   /**/
#else
#define GERTAR   extern
#endif

//vari�veis globais
GERTAR TSetupGerenciadorDeTarefas SetupGerenciadorDeTarefas;   //"objeto" de setup do gerenciador de tarefas

#endif
		
//prototypes globais
void IniciaGerenciadorTarefas(void);
void ExecutaTarefa(void);	
