#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/** IO */
#define LED1_PIO			PIOA
#define LED1_PIO_ID			ID_PIOA
#define LED1_PIO_PIN		0
#define LED1_PIO_PIN_MASK	( 1 << LED1_PIO_PIN )
#define LED1_TC				TC0
#define LED1_TC_ID			ID_TC1
#define LED1_TC_CHANNEL		1

#define LED2_PIO			PIOC
#define LED2_PIO_ID			ID_PIOC
#define LED2_PIO_PIN		30
#define LED2_PIO_PIN_MASK	( 1 << LED2_PIO_PIN )

#define LED3_PIO			PIOB
#define LED3_PIO_ID			ID_PIOB
#define LED3_PIO_PIN		2
#define LED3_PIO_PIN_MASK	( 1 << LED3_PIO_PIN )
#define LED3_TC				TC1
#define LED3_TC_ID			ID_TC4
#define LED3_TC_CHANNEL		1

#define BUT1_PIO			PIOD
#define BUT1_PIO_ID			ID_PIOD
#define BUT1_PIO_PIN		28
#define BUT1_PIO_PIN_MASK	( 1u << BUT1_PIO_PIN )

#define BUT2_PIO			PIOC
#define BUT2_PIO_ID			ID_PIOC
#define BUT2_PIO_PIN		31
#define BUT2_PIO_PIN_MASK	( 1u << BUT2_PIO_PIN )

#define BUT3_PIO			PIOA
#define BUT3_PIO_ID			ID_PIOA
#define BUT3_PIO_PIN		19
#define BUT3_PIO_PIN_MASK	( 1u << BUT3_PIO_PIN )

/** RTOS  */
#define TASK_TC_STACK_SIZE				( 1024*6/sizeof(portSTACK_TYPE) )
#define TASK_TC_STACK_PRIORITY			( tskIDLE_PRIORITY )
#define TASK_RTT_STACK_SIZE				( 1024*6/sizeof(portSTACK_TYPE) )
#define TASK_RTT_STACK_PRIORITY			( tskIDLE_PRIORITY )
#define TASK_RTC_STACK_SIZE				( 1024*6/sizeof(portSTACK_TYPE) )
#define TASK_RTC_STACK_PRIORITY			( tskIDLE_PRIORITY )

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

static SemaphoreHandle_t xSemaphoreTC1;
static SemaphoreHandle_t xSemaphoreRTT;
static SemaphoreHandle_t xSemaphoreRTC;
static SemaphoreHandle_t xSemaphoreTC2;
static SemaphoreHandle_t xSemaphoreBut1;

/** Prototypes */
/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */
typedef struct {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} datetime_t;

static void but1_callback(void);

static void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
static void RTC_init(Rtc *rtc, uint32_t id_rtc, datetime_t t, uint32_t irq_type);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

static void but1_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreBut1, &xHigherPriorityTaskWoken);
}

void TC1_Handler(void) {
	volatile uint32_t status = tc_get_status(LED1_TC, LED1_TC_CHANNEL);

	xSemaphoreGiveFromISR(xSemaphoreTC1, pdFALSE);
}

void RTT_Handler(void) {
	uint32_t ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {

		xSemaphoreGiveFromISR(xSemaphoreRTT, pdFALSE);
	}
}

void RTC_Handler(void) {
	uint32_t ul_status = rtc_get_status(RTC);
	
	// 	/* seccond tick */
	// 	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
	// 	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		xSemaphoreGiveFromISR(xSemaphoreRTC, pdFALSE);
	}

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

void TC4_Handler(void) {
	volatile uint32_t status = tc_get_status(LED3_TC, LED3_TC_CHANNEL);

	xSemaphoreGiveFromISR(xSemaphoreTC2, pdFALSE);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_tc(void *pvParameters) {
	int led_state = LOW;

	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIO_PIN_MASK, HIGH, DISABLE, ENABLE);
	
	TC_init(LED1_TC, LED1_TC_ID, LED1_TC_CHANNEL, 4);
	tc_start(LED1_TC, LED1_TC_CHANNEL);

	for (;;) {
		if (xSemaphoreTake(xSemaphoreTC1, 0)) {
			if (led_state)
				pio_clear(LED1_PIO, LED1_PIO_PIN_MASK);
			else
				pio_set(LED1_PIO, LED1_PIO_PIN_MASK);

			led_state = !led_state;
		}
	}
}

static void task_rtt(void *pvParameters) {
	int led_state = LOW;

	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_PIO_PIN_MASK, HIGH, DISABLE, ENABLE);

	RTT_init(4, 16, RTT_MR_ALMIEN);

	for (;;) {
		if (xSemaphoreTake(xSemaphoreRTT, 0)) {
			if (led_state)
				pio_clear(LED2_PIO, LED2_PIO_PIN_MASK);
			else
				pio_set(LED2_PIO, LED2_PIO_PIN_MASK);

			led_state = !led_state;

			RTT_init(4, 16, RTT_MR_ALMIEN);
		}
	}
}

static void task_rtc(void *pvParameters) {
	int blink_state = LOW;
	int led_state = LOW;
	datetime_t now = {
		.year = 2022,
		.month = 11,
		.day = 20,
		.week = 0,
		.hour = 12,
		.minute = 0,
		.second = 0,
	};

	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_PIO_PIN_MASK, HIGH, DISABLE, ENABLE);

	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_PIN_MASK);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but1_callback);

	gfx_mono_ssd1306_init();

	RTC_init(RTC, ID_RTC, now, RTC_IER_ALREN);
	TC_init(LED3_TC, LED3_TC_ID, LED3_TC_CHANNEL, 4);

	for (;;) {
		if (xSemaphoreTake(xSemaphoreBut1, 50)) {
			gfx_mono_draw_string("Botao 1 pressionado", 8, 14, &sysfont);

			rtc_get_date(RTC, &now.year, &now.month, &now.day, &now.week);
			rtc_get_time(RTC, &now.hour, &now.minute, &now.second);
			rtc_set_date_alarm(RTC, 1, now.month, 1, now.day);
			rtc_set_time_alarm(RTC, 1, now.hour, 1, now.minute, 1, now.second+2);
		}

		if (xSemaphoreTake(xSemaphoreRTC, 50)) {
			gfx_mono_draw_string("                   ", 8, 14, &sysfont);

			if (blink_state)
				tc_stop(LED3_TC, LED3_TC_CHANNEL);
			else
				tc_start(LED3_TC, LED3_TC_CHANNEL);

			blink_state = !blink_state;
		}

		if (xSemaphoreTake(xSemaphoreTC2, 50)) {
			if (led_state)
				pio_clear(LED3_PIO, LED3_PIO_PIN_MASK);
			else
				pio_set(LED3_PIO, LED3_PIO_PIN_MASK);

			led_state = !led_state;
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/**
* Configura TimerCounter (TC) para gerar uma interrupcao no canal (ID_TC e TC_CHANNEL)
* na taxa de especificada em freq.
* O TimerCounter é meio confuso
* o uC possui 3 TCs, cada TC possui 3 canais
*	TC0 : ID_TC0, ID_TC1, ID_TC2
*	TC1 : ID_TC3, ID_TC4, ID_TC5
*	TC2 : ID_TC6, ID_TC7, ID_TC8
*
**/
static void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

/** 
 * Configura RTT
 *
 * arg0 pllPreScale  : Frequência na qual o contador irá incrementar
 * arg1 IrqNPulses   : Valor do alarme 
 * arg2 rttIRQSource : Pode ser uma 
 *     - 0: 
 *     - RTT_MR_RTTINCIEN: Interrupção por incremento (pllPreScale)
 *     - RTT_MR_ALMIEN : Interrupção por alarme
 */
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {
  uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
  rtt_sel_source(RTT, false);
  rtt_init(RTT, pllPreScale);
  
  if (rttIRQSource & RTT_MR_ALMIEN) {
	uint32_t ul_previous_time;
  	ul_previous_time = rtt_read_timer_value(RTT);
  	while (ul_previous_time == rtt_read_timer_value(RTT));
  	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
  }

  /* config NVIC */
  NVIC_DisableIRQ(RTT_IRQn);
  NVIC_ClearPendingIRQ(RTT_IRQn);
  NVIC_SetPriority(RTT_IRQn, 4);
  NVIC_EnableIRQ(RTT_IRQn);

  /* Enable RTT interrupt */
  if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
  else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
		  
}

static void RTC_init(Rtc *rtc, uint32_t id_rtc, datetime_t t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	xSemaphoreTC1 = xSemaphoreCreateBinary();
	if (xSemaphoreTC1 == NULL) {
		printf("Failed to create TC1 semaphore\n");
	}

	xSemaphoreRTT = xSemaphoreCreateBinary();
	if (xSemaphoreRTT == NULL) {
		printf("Failed to create RTT semaphore\n");
	}

	xSemaphoreRTC = xSemaphoreCreateBinary();
	if (xSemaphoreRTC == NULL) {
		printf("Failed to create RTC semaphore\n");
	}

	xSemaphoreTC2 = xSemaphoreCreateBinary();
	if (xSemaphoreTC2 == NULL) {
		printf("Failed to create TC2 semaphore\n");
	}

	xSemaphoreBut1 = xSemaphoreCreateBinary();
	if (xSemaphoreBut1 == NULL) {
		printf("Failed to create But1 semaphore\n");
	}

	if (xTaskCreate(task_tc, "TC", TASK_TC_STACK_SIZE, NULL, TASK_TC_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create TC task\n");
	}

	if (xTaskCreate(task_rtt, "RTT", TASK_RTT_STACK_SIZE, NULL, TASK_RTT_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create RTT task\n");
	}

	if (xTaskCreate(task_rtc, "RTC", TASK_RTC_STACK_SIZE, NULL, TASK_RTC_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create RTC task\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS shouldn't get here! */
	for (;;);

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
