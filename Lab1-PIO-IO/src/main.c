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

#define LED_1_PIO           PIOA
#define LED_1_PIO_ID        ID_PIOA
#define LED_1_PIO_IDX       0
#define LED_1_PIO_IDX_MASK  (1 << LED_1_PIO_IDX)

#define LED_2_PIO           PIOC
#define LED_2_PIO_ID        ID_PIOC
#define LED_2_PIO_IDX       30
#define LED_2_PIO_IDX_MASK  (1 << LED_2_PIO_IDX)

#define LED_3_PIO           PIOB
#define LED_3_PIO_ID        ID_PIOB
#define LED_3_PIO_IDX       2
#define LED_3_PIO_IDX_MASK  (1 << LED_3_PIO_IDX)

#define BUT_1_PIO           PIOD
#define BUT_1_PIO_ID        ID_PIOD
#define BUT_1_PIO_IDX       28
#define BUT_1_PIO_IDX_MASK (1u << BUT_1_PIO_IDX)

#define BUT_2_PIO           PIOC
#define BUT_2_PIO_ID        ID_PIOC
#define BUT_2_PIO_IDX       31
#define BUT_2_PIO_IDX_MASK (1u << BUT_2_PIO_IDX)

#define BUT_3_PIO           PIOA
#define BUT_3_PIO_ID        ID_PIOA
#define BUT_3_PIO_IDX       19
#define BUT_3_PIO_IDX_MASK (1u << BUT_3_PIO_IDX)


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

void piscaLed(Pio *pio, uint32_t pio_idx_mask) {
	pio_clear(pio, pio_idx_mask);
	delay_ms(128);
	pio_set(pio, pio_idx_mask);
	delay_ms(128);
	pio_clear(pio, pio_idx_mask);
	delay_ms(128);
	pio_set(pio, pio_idx_mask);
	delay_ms(128);
	pio_clear(pio, pio_idx_mask);
	delay_ms(128);
	pio_set(pio, pio_idx_mask);
	delay_ms(128);
	pio_clear(pio, pio_idx_mask);
	delay_ms(128);
	pio_set(pio, pio_idx_mask);
	delay_ms(128);
	pio_clear(pio, pio_idx_mask);
	delay_ms(128);
	pio_set(pio, pio_idx_mask);
	delay_ms(128);
}

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

// Função de inicialização do uC
void init(void)
{
  // Initialize the board clock
  sysclk_init();

  // Desativa WatchDog Timer
  WDT->WDT_MR = WDT_MR_WDDIS;

  // Ativa o PIO na qual o LED foi conectado
  // para que possamos controlar o LED.
  pmc_enable_periph_clk(LED_1_PIO);
  pmc_enable_periph_clk(LED_2_PIO);
  pmc_enable_periph_clk(LED_3_PIO);


  // Ativa o PIO na qual o BOTÃO foi conectado
  // para que possamos controlar o BOTÃO.
  pmc_enable_periph_clk(BUT_1_PIO_ID);
  pmc_enable_periph_clk(BUT_2_PIO_ID);
  pmc_enable_periph_clk(BUT_3_PIO_ID);

  //Inicializa PC8 como saída
  pio_set_output(LED_1_PIO, LED_1_PIO_IDX_MASK, 0, 0, 0);
  pio_set_output(LED_2_PIO, LED_2_PIO_IDX_MASK, 0, 0, 0);
  pio_set_output(LED_3_PIO, LED_3_PIO_IDX_MASK, 0, 0, 0);
  
  //Inicializa PA11 como entrada
  pio_set_input(BUT_1_PIO, BUT_1_PIO_IDX_MASK, PIO_DEFAULT);
  pio_set_input(BUT_2_PIO, BUT_2_PIO_IDX_MASK, PIO_DEFAULT);
  pio_set_input(BUT_3_PIO, BUT_3_PIO_IDX_MASK, PIO_DEFAULT);
  
  // Inicializa resitor de pul-up no botão
  pio_pull_up(BUT_1_PIO, BUT_1_PIO_IDX_MASK, pio_pull_up);
  pio_pull_up(BUT_2_PIO, BUT_2_PIO_IDX_MASK, pio_pull_up);
  pio_pull_up(BUT_3_PIO, BUT_3_PIO_IDX_MASK, pio_pull_up);
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void) {
  // inicializa sistema e IOs
  init();

  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
  while (1)
  {
	if(pio_get(BUT_1_PIO, PIO_INPUT, BUT_1_PIO_IDX_MASK) == 0) {
		piscaLed(LED_1_PIO, LED_1_PIO_IDX_MASK);

		while(1) {
			if(pio_get(BUT_1_PIO, PIO_INPUT, BUT_1_PIO_IDX_MASK)) {
				break;
			}
		}
	} else {
		pio_set(LED_1_PIO, LED_1_PIO_IDX_MASK);
	}

	if(pio_get(BUT_2_PIO, PIO_INPUT, BUT_2_PIO_IDX_MASK) == 0) {
		piscaLed(LED_2_PIO, LED_2_PIO_IDX_MASK);

		while(1) {
			if(pio_get(BUT_2_PIO, PIO_INPUT, BUT_2_PIO_IDX_MASK)) {
				break;
			}
		}
	} else {
		pio_set(LED_2_PIO, LED_2_PIO_IDX_MASK);
	}

	if(pio_get(BUT_3_PIO, PIO_INPUT, BUT_3_PIO_IDX_MASK) == 0) {
		piscaLed(LED_3_PIO, LED_3_PIO_IDX_MASK);

		while(1) {
			if(pio_get(BUT_3_PIO, PIO_INPUT, BUT_3_PIO_IDX_MASK)) {
				break;
			}
		}
		} else {
		pio_set(LED_3_PIO, LED_3_PIO_IDX_MASK);
	}
  }
  return 0;
}
