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

/*  Default pin configuration (no attribute). */
#define _PIO_DEFAULT (0u << 0)
/*  The internal pin pull-up is active. */
#define _PIO_PULLUP (1u << 0)
/*  The internal glitch filter is active. */
#define _PIO_DEGLITCH (1u << 1)
/*  The internal debouncing filter is active. */
#define _PIO_DEBOUNCE (1u << 3)

#define LED_1_PIO PIOA
#define LED_1_PIO_ID ID_PIOA
#define LED_1_PIO_IDX 0
#define LED_1_PIO_IDX_MASK (1u << LED_1_PIO_IDX)

#define LED_2_PIO PIOC
#define LED_2_PIO_ID ID_PIOC
#define LED_2_PIO_IDX 30
#define LED_2_PIO_IDX_MASK (1u << LED_2_PIO_IDX)

#define LED_3_PIO PIOB
#define LED_3_PIO_ID ID_PIOB
#define LED_3_PIO_IDX 2
#define LED_3_PIO_IDX_MASK (1u << LED_3_PIO_IDX)

#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_PIO_IDX 28
#define BUT_1_PIO_IDX_MASK (1u << BUT_1_PIO_IDX)

#define BUT_2_PIO PIOC
#define BUT_2_PIO_ID ID_PIOC
#define BUT_2_PIO_IDX 31
#define BUT_2_PIO_IDX_MASK (1u << BUT_2_PIO_IDX)

#define BUT_3_PIO PIOA
#define BUT_3_PIO_ID ID_PIOA
#define BUT_3_PIO_IDX 19
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

void _pio_set(Pio *p_pio, const uint32_t ul_mask);

void _pio_clear(Pio *p_pio, const uint32_t ul_mask);

void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_up_enable);

void _pio_set_input(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_attribute);

void _pio_set_output(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_default_level, const uint32_t ul_multidrive_enable, const uint32_t ul_pull_up_enable);

uint32_t _pio_get(Pio *p_pio, const pio_type_t ul_type, const uint32_t ul_mask);

void _delay_ms(int time);

void init(void);

void piscaLed(Pio *pio, uint32_t pio_idx_mask);

void botaoPiscaLed(Pio *Pio_led, const uint32_t pio_led_idx_mask, Pio *Pio_but, const uint32_t pio_but_idx_mask);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void _pio_set(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_SODR = ul_mask;
}

void _pio_clear(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_CODR = ul_mask;
}

void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_up_enable)
{
	p_pio->PIO_PUER = ul_mask;
	p_pio->PIO_PUDR = !ul_mask;
}

void _pio_set_input(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_attribute)
{
	p_pio->PIO_IFSCER = ul_attribute;

	_pio_pull_up(p_pio, ul_mask, ul_attribute);
}

void _pio_set_output(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_default_level, const uint32_t ul_multidrive_enable, const uint32_t ul_pull_up_enable)
{
	p_pio->PIO_PER = ul_mask;
	p_pio->PIO_PDR = !ul_mask;

	p_pio->PIO_OER = ul_mask;
	p_pio->PIO_ODR = !ul_mask;

	if (ul_default_level) {
		_pio_set(p_pio, ul_mask);
	}
	else {
		_pio_clear(p_pio, ul_mask);
	}

	if (ul_pull_up_enable) {
		_pio_pull_up(p_pio, ul_mask, _PIO_PULLUP);
	}
}

uint32_t _pio_get(Pio *p_pio, const pio_type_t ul_type, const uint32_t ul_mask)
{
	int status;
	int output = 0;

	if (ul_type == PIO_INPUT) {
		status = p_pio->PIO_PDSR & ul_mask;
	}
	else if (ul_type == PIO_OUTPUT_0) {
		status = p_pio->PIO_ODSR & ul_mask;
	}

	for (int i = 0; i < 32; i++) {
		output |= (status >> i) & 1;
	}

	return output;
}

void _delay_ms(const int time)
{	
	const int totalClicks = time * 8000;

	for (int click = 0; click < totalClicks; click++)
	{
		asm("nop");
	}
}

// Função de inicialização do uC
void init(void)
{
	// Initialize the board clock
	sysclk_init();

	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;

	pmc_enable_periph_clk(LED_1_PIO);
	pmc_enable_periph_clk(LED_2_PIO);
	pmc_enable_periph_clk(LED_3_PIO);
	pmc_enable_periph_clk(BUT_1_PIO_ID);
	pmc_enable_periph_clk(BUT_2_PIO_ID);
	pmc_enable_periph_clk(BUT_3_PIO_ID);

	_pio_set_output(LED_1_PIO, LED_1_PIO_IDX_MASK, 0, 0, 0);
	_pio_set_output(LED_2_PIO, LED_2_PIO_IDX_MASK, 0, 0, 0);
	_pio_set_output(LED_3_PIO, LED_3_PIO_IDX_MASK, 0, 0, 0);

	_pio_set_input(BUT_1_PIO, BUT_1_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_set_input(BUT_2_PIO, BUT_2_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_set_input(BUT_3_PIO, BUT_3_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
}

void piscaLed(Pio *pio, uint32_t pio_idx_mask)
{
	for (uint i = 0; i < 5; i++)
	{
		_pio_clear(pio, pio_idx_mask);
		_delay_ms(32);
		_pio_set(pio, pio_idx_mask);
		_delay_ms(32);
	}
}

void botaoPiscaLed(Pio *Pio_led, const uint32_t pio_led_idx_mask, Pio *Pio_but, const uint32_t pio_but_idx_mask)
{
	if (_pio_get(Pio_but, PIO_INPUT, pio_but_idx_mask) == 0)
	{
		piscaLed(Pio_led, pio_led_idx_mask);

		while (1)
		{
			if (_pio_get(Pio_but, PIO_INPUT, pio_but_idx_mask))
			{
				break;
			}
		}
	}
	else
	{
		_pio_set(Pio_led, pio_led_idx_mask);
	}
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

int main(void)
{
	init();

	while (1)
	{
		botaoPiscaLed(LED_1_PIO, LED_1_PIO_IDX_MASK, BUT_1_PIO, BUT_1_PIO_IDX_MASK);
		botaoPiscaLed(LED_2_PIO, LED_2_PIO_IDX_MASK, BUT_2_PIO, BUT_2_PIO_IDX_MASK);
		botaoPiscaLed(LED_3_PIO, LED_3_PIO_IDX_MASK, BUT_3_PIO, BUT_3_PIO_IDX_MASK);
	}

	return 0;
}
