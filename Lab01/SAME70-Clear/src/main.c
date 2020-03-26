/**
 * 5 semestre - Eng. da Computação - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Projeto 0 para a placa SAME70-XPLD
 *
 * Objetivo :
 *  - Introduzir ASF e HAL
 *  - Configuracao de clock
 *  - Configuracao pino In/Out
 *
 * Material :
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 */

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

#define LED_PIO           PIOC						// periferico que controla o LED
#define LED_PIO_ID        12					    // ID do periférico PIOC (controla LED)
#define LED_PIO_IDX       8							// ID do LED no PIO
#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)		// Mascara para CONTROLARMOS o LED

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

// Função de inicialização do uC
void init(void)
{
	// initialize the board clock
	sysclk_init();
	
	// Desativa Watch Dog Timer
	WDT->WDT_MR = WDT_MR_WDDIS; 
	
	// Ativa o PIO na qual o LED foi conectado
	// para que possamos controlar o LED.
	pmc_enable_periph_clk(LED_PIO_ID);
	
	//Inicializa PC8 como saída
	pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);

}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void)
{
  init();

  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
  while (1)
  {
	  pio_set(PIOC, LED_PIO_IDX_MASK);      // Coloca 1 no pino LED
	  delay_ms(200);                        // Delay por software de 200 ms
	  pio_clear(PIOC, LED_PIO_IDX_MASK);    // Coloca 0 no pino do LED
	  delay_ms(200);                        // Delay por software de 200 ms
  }
  return 0;
}
