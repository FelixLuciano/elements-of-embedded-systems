#include "conf_board.h"
#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/** HC-SR04*/
#define TRIG_PIO     PIOC
#define TRIG_PIO_ID  ID_PIOC
#define TRIG_PIO_PIN 19
#define TRIG_PIO_PIN_MASK (1 << TRIG_PIO_PIN)

#define ECHO_PIO     PIOA
#define ECHO_PIO_ID  ID_PIOA
#define ECHO_PIO_PIN 3
#define ECHO_PIO_PIN_MASK (1 << ECHO_PIO_PIN)

/** RTOS  */
#define TASK_HCSR04_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_HCSR04_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_OLED_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** Semáforos do nível do ECHO */
SemaphoreHandle_t xSemaphoreEchoHight;
SemaphoreHandle_t xSemaphoreEchoLow;

/** Fila de leituras do sensor */
QueueHandle_t xQueueReadings;

/** prototypes */
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void echo_callback(void);
static void set_trig();
static void clear_trig();
static char get_echo();

typedef struct {
	int value;
} ref;

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {}
}

extern void vApplicationIdleHook(void) {}

extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void) {
	configASSERT((volatile void *)NULL);
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void echo_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(get_echo()) {
		xSemaphoreGiveFromISR(xSemaphoreEchoHight, &xHigherPriorityTaskWoken);
	}
	else {
		xSemaphoreGiveFromISR(xSemaphoreEchoLow, &xHigherPriorityTaskWoken);
	}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_hcsr04(void *pvParameters) {
	// Initialize TRIGGER pin as OUTPUT
	pmc_enable_periph_clk(TRIG_PIO_ID);
	pio_set_output(TRIG_PIO, TRIG_PIO_PIN_MASK, PIO_IT_LOW_LEVEL, 0, 0);

	// Initialize ECHO pin as OUTPUT
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK, 0);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(ECHO_PIO, ECHO_PIO_ID, ECHO_PIO_PIN_MASK, PIO_IT_EDGE, echo_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_PIN_MASK);
	pio_get_interrupt_status(ECHO_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4); // Prioridade 4

	for (;;) {
		vTaskDelay(60);
		set_trig();
		delay_us(10);
		clear_trig();

		if (xSemaphoreTake(xSemaphoreEchoHight, 0)) {
			RTT_init(10000, 0, 0);
		}
		else if (xSemaphoreTake(xSemaphoreEchoLow, 0)) {
			ref distance = {
				.value = 321,
			};

			// uint32_t time_read = rtt_read_timer_value(RTT);
			// int dist= 1.0/10000*time_read*170*100;
			// distance.value = time_read;

			xQueueSendToBack(xQueueReadings, &distance, 0);
		}
	}
}

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();

	gfx_mono_draw_string("Modo:", 0, 0, &sysfont);

	for (;;) {
		ref distance;
		if (xQueueReceive(xQueueReadings, &distance, 0)) {
			printf("Distancia: %d m\n", distance.value);
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

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

static void set_trig() {
	pio_set(TRIG_PIO, TRIG_PIO_PIN_MASK);
}

static void clear_trig() {
	pio_clear(TRIG_PIO, TRIG_PIO_PIN_MASK);
}

static char get_echo() {
	return pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK);
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

	xQueueReadings = xQueueCreate(128, sizeof(ref));
	if (xQueueReadings == NULL) {
		printf("Failed to create readings queue \n");
	}

	xSemaphoreEchoHight = xSemaphoreCreateBinary();
	if (xQueueReadings == NULL) {
		printf("Failed to create ECHO hight semaphore \n");
	}

	xSemaphoreEchoLow = xSemaphoreCreateBinary();
	if (xSemaphoreEchoLow == NULL) {
		printf("Failed to create ECHO hight semaphore \n");
	}

	/* Create task to control HC-SR04 */
	if (xTaskCreate(task_hcsr04, "hcsr04", TASK_HCSR04_STACK_SIZE, NULL, TASK_HCSR04_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create hcsr04 task\r\n");
	}

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while (1) {}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
