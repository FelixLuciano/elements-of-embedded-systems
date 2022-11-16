/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include <asf.h>
#include <string.h>
#include "ili9341.h"
#include "lvgl.h"
#include "touch/touch.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

#define AFEC_POT			AFEC0
#define AFEC_POT_ID			ID_AFEC0
#define AFEC_POT_CHANNEL	0  // Canal do pino PD30

LV_FONT_DECLARE(dseg70);
LV_FONT_DECLARE(dseg40);
LV_FONT_DECLARE(dseg20);

typedef struct {
	uint32_t value;
} uint32Ref_t;

typedef struct {
	float value;
} floatRef_t;

/**
*  Informacoes para o RTC
*  poderia ser extraida do __DATE__ e __TIME__
*  ou ser atualizado pelo PC.
*/
typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_AFEC_STACK_SIZE				(1024*6 / sizeof(portSTACK_TYPE))
#define TASK_AFEC_STACK_PRIORITY			(tskIDLE_PRIORITY)
#define TASK_LCD_STACK_SIZE					(1024*6/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY				(tskIDLE_PRIORITY)
#define TASK_RTC_STACK_SIZE					(1024*6/sizeof(portSTACK_TYPE))
#define TASK_RTC_STACK_PRIORITY				(tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* recursos RTOS                                                        */
/************************************************************************/

QueueHandle_t xQueueAFEC;
QueueHandle_t xQueueTemperature;
SemaphoreHandle_t xSemaphoreRTC;

/************************************************************************/
/* LCD / LVGL                                                           */
/************************************************************************/

#define LV_HOR_RES_MAX          (320)
#define LV_VER_RES_MAX          (240)

/*A static or global variable to store the buffers*/
static lv_disp_draw_buf_t disp_buf;

/*Static or global buffer(s). The second buffer is optional*/
static lv_color_t buf_1[LV_HOR_RES_MAX * LV_VER_RES_MAX];
static lv_disp_drv_t disp_drv;          /*A variable to hold the drivers. Must be static or global.*/
static lv_indev_drv_t indev_drv;
static lv_obj_t * labelSetValue;
static lv_obj_t * labelClock;

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

void afec_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	
	uint32Ref_t afec = {
		.value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL),
	};

	xQueueSendFromISR(xQueueAFEC, &afec, &xHigherPriorityTaskWoken);
}

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
		afec_start_software_conversion(AFEC_POT);
	}
}

/**
* \brief Interrupt handler for the RTC. Refresh the display.
*/
void RTC_Handler(void) {
	uint32_t ul_status = rtc_get_status(RTC);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	/* seccond tick */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		// o código para irq de segundo vem aqui
		xSemaphoreGiveFromISR(xSemaphoreRTC, 0);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		// o código para irq de alame vem aqui
	}

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

/************************************************************************/
/* lvgl                                                                 */
/************************************************************************/

static void btn_power_event_handler(lv_event_t * e) {
	LV_LOG_USER("Clicked power");
	printf("Clicked power\n");
}

static void btn_menu_event_handler(lv_event_t * e) {
	LV_LOG_USER("Clicked menu");
	printf("Clicked menu\n");
}

static void btn_clock_event_handler(lv_event_t * e) {
	LV_LOG_USER("Clicked clock");
	printf("Clicked clock\n");
}

static void btn_up_event_handler(lv_event_t * e) {
	char *c = lv_label_get_text(labelSetValue);
	int temp = atoi(c);

	lv_label_set_text_fmt(labelSetValue, "%02d", temp + 1);
	LV_LOG_USER("Clicked up");
	printf("Clicked up\n");
}

static void btn_down_event_handler(lv_event_t * e) {
	char *c = lv_label_get_text(labelSetValue);
	int temp = atoi(c);

	lv_label_set_text_fmt(labelSetValue, "%02d", temp - 1);
	LV_LOG_USER("Clicked down");
	printf("Clicked down\n");
}

void lv_termostato(void) {
	// floor floor_temperature = 23.4;

	static lv_style_t style;
	lv_style_init(&style);
	lv_style_set_bg_color(&style, lv_color_black());

	lv_obj_t * btn_power = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn_power, btn_power_event_handler, LV_EVENT_CLICKED, NULL);
	lv_obj_align(btn_power, LV_ALIGN_BOTTOM_LEFT, 0, 0);
	lv_obj_add_style(btn_power, &style, 0);
	lv_obj_t * label_power = lv_label_create(btn_power);
	lv_label_set_text(label_power, "[ " LV_SYMBOL_POWER);
	lv_obj_center(label_power);

	lv_obj_t * btn_menu = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn_menu, btn_menu_event_handler, LV_EVENT_CLICKED, NULL);
	lv_obj_align_to(btn_menu, btn_power, LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
	lv_obj_add_style(btn_menu, &style, 0);
	lv_obj_t * label_menu = lv_label_create(btn_menu);
	lv_label_set_text(label_menu, "|  M");
	lv_obj_center(label_menu);

	lv_obj_t * btn_clock = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn_clock, btn_clock_event_handler, LV_EVENT_CLICKED, NULL);
	lv_obj_align_to(btn_clock, btn_menu, LV_ALIGN_OUT_RIGHT_TOP, 0, 0);
	lv_obj_add_style(btn_clock, &style, 0);
	lv_obj_t * label_clock = lv_label_create(btn_clock);
	lv_label_set_text(label_clock, "| " LV_SYMBOL_SETTINGS " ]");
	lv_obj_center(label_clock);

	lv_obj_t * btn_down = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn_down, btn_down_event_handler, LV_EVENT_CLICKED, NULL);
	lv_obj_align(btn_down, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
	lv_obj_add_style(btn_down, &style, 0);
	lv_obj_t * label_down = lv_label_create(btn_down);
	lv_label_set_text(label_down, LV_SYMBOL_DOWN " ]");
	lv_obj_center(label_down);

	lv_obj_t * btn_up = lv_btn_create(lv_scr_act());
	lv_obj_add_event_cb(btn_up, btn_up_event_handler, LV_EVENT_CLICKED, NULL);
	lv_obj_align_to(btn_up, btn_down, LV_ALIGN_OUT_LEFT_TOP, -28, 0);
	lv_obj_add_style(btn_up, &style, 0);
	lv_obj_t * label_up = lv_label_create(btn_up);
	lv_label_set_text(label_up, "[ " LV_SYMBOL_UP);
	lv_obj_center(label_up);
	
	lv_obj_t * labelFloor = lv_label_create(lv_scr_act());
	lv_obj_align(labelFloor, LV_ALIGN_TOP_LEFT, 35 , 45);
	lv_obj_set_style_text_font(labelFloor, &dseg70, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelFloor, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelFloor, "25");
	// lv_label_set_text_fmt(labelFloor, "%.1f", floor_temperature);
	
	lv_obj_t * labelFloorDecimal = lv_label_create(lv_scr_act());
	lv_obj_align_to(labelFloorDecimal, labelFloor, LV_ALIGN_OUT_RIGHT_BOTTOM, 4, -15);
	lv_obj_set_style_text_font(labelFloorDecimal, &dseg40, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelFloorDecimal, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelFloorDecimal, ".0");
	// lv_label_set_text_fmt(labelFloorDecimal, ".%.1f", floor_temperature * 10.0);
	
	labelSetValue = lv_label_create(lv_scr_act());
	lv_obj_align(labelSetValue, LV_ALIGN_TOP_RIGHT, -24 , 45);
	lv_obj_set_style_text_font(labelSetValue, &dseg40, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelSetValue, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelSetValue, "25");
	
	labelClock = lv_label_create(lv_scr_act());
	lv_obj_align(labelClock, LV_ALIGN_TOP_RIGHT, 0, 0);
	lv_obj_set_style_text_font(labelClock, &dseg20, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelClock, lv_color_white(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelClock, "00:00");
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_afec(void *pvParameters) {
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, afec_callback);
	printf("task afec");

	uint32Ref_t afec;
	floatRef_t temperature;

	RTT_init(100, 10, RTT_MR_ALMIEN);

	for (;;) {
		if (xQueueReceive(xQueueAFEC, &afec, 0)) {
			temperature.value = (float)(afec.value * 20) / 4096.0 + 15.0;
			xQueueSendToBack(xQueueTemperature, &temperature, 0);
			RTT_init(100, 10, RTT_MR_ALMIEN);
		}

		vTaskDelay(10);
	}
}

static void task_lcd(void *pvParameters) {
	floatRef_t temperature;

	lv_termostato();

	for (;;) {
		if (xQueueReceive(xQueueTemperature, &temperature, 0)) {
			lv_label_set_text_fmt(labelSetValue, "%02.0f", temperature.value);
		}

		lv_tick_inc(50);
		lv_task_handler();
		vTaskDelay(50);
	}
}

static void task_rtc(void *pvParameters) {
	calendar now = {
		.year = 2022,
		.month = 11,
		.day = 9,
		.week = 3,
		.hour = 19,
		.minute = 0,
		.second = 1,
	};

	RTC_init(RTC, ID_RTC, now, RTC_IER_SECEN);

	for(;;) {
		if(xSemaphoreTake(xSemaphoreRTC, 0)) {
			/* Leitura do valor atual do RTC */
		 	rtc_get_time(RTC, &now.hour, &now.minute, &now.second);
		 	rtc_get_date(RTC, &now.year, &now.month, &now.day, &now.week);

			/* Atualização do valor do clock */
			if (now.second % 2 == 0) {
				lv_label_set_text_fmt(labelClock, "%02d:%02d", now.hour, now.minute);
			}
			else {
				lv_label_set_text_fmt(labelClock, "%02d %02d", now.hour, now.minute);
			}
		}
		vTaskDelay(700);
	}
}

/************************************************************************/
/* configs                                                              */
/************************************************************************/

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback) {
	/*************************************
	* Ativa e configura AFEC
	*************************************/
	/* Ativa AFEC - 0 */
	afec_enable(afec);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(afec, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(afec, AFEC_TRIG_SW);

	/*** Configuracao espec?fica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	down to 0.
	*/
	afec_channel_set_analog_offset(afec, afec_channel, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

	/* configura IRQ */
	afec_set_callback(afec, afec_channel, callback, 1);
	NVIC_SetPriority(afec_id, 4);
	NVIC_EnableIRQ(afec_id);
}

/**
* Configura o RTC para funcionar com interrupcao de alarme
*/
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
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

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int)(((float)32768) / freqPrescale);

	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);

	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT))
		;
		rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);
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

static void configure_lcd(void) {
	/**LCD pin configure on SPI*/
	pio_configure_pin(LCD_SPI_MISO_PIO, LCD_SPI_MISO_FLAGS);  //
	pio_configure_pin(LCD_SPI_MOSI_PIO, LCD_SPI_MOSI_FLAGS);
	pio_configure_pin(LCD_SPI_SPCK_PIO, LCD_SPI_SPCK_FLAGS);
	pio_configure_pin(LCD_SPI_NPCS_PIO, LCD_SPI_NPCS_FLAGS);
	pio_configure_pin(LCD_SPI_RESET_PIO, LCD_SPI_RESET_FLAGS);
	pio_configure_pin(LCD_SPI_CDS_PIO, LCD_SPI_CDS_FLAGS);
	
	ili9341_init();
	ili9341_backlight_on();
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT,
	};

	/* Configure console UART. */
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/************************************************************************/
/* port lvgl                                                            */
/************************************************************************/

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
	ili9341_set_top_left_limit(area->x1, area->y1);   ili9341_set_bottom_right_limit(area->x2, area->y2);
	ili9341_copy_pixels_to_screen(color_p,  (area->x2 + 1 - area->x1) * (area->y2 + 1 - area->y1));
	
	/* IMPORTANT!!!
	* Inform the graphics library that you are ready with the flushing*/
	lv_disp_flush_ready(disp_drv);
}

void my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data) {
	int px, py, pressed;
	
	if (readPoint(&px, &py))
	data->state = LV_INDEV_STATE_PRESSED;
	else
	data->state = LV_INDEV_STATE_RELEASED;
	
	data->point.x = px;
	data->point.y = py;
}

void configure_lvgl(void) {
	lv_init();
	lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX);
	
	lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
	disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
	disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
	disp_drv.hor_res = LV_HOR_RES_MAX;      /*Set the horizontal resolution in pixels*/
	disp_drv.ver_res = LV_VER_RES_MAX;      /*Set the vertical resolution in pixels*/

	lv_disp_t * disp;
	disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/
	
	/* Init input on LVGL */
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = my_input_read;
	lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* board and sys init */
	board_init();
	sysclk_init();
	configure_console();

	/* LCd, touch and lvgl init*/
	configure_lcd();
	configure_touch();
	configure_lvgl();

	/* Attempt to create a semaphore. */
	xSemaphoreRTC = xSemaphoreCreateBinary();
	if (xSemaphoreRTC == NULL) {
		printf("Failed to create RTC semaphore\n");
	}

	xQueueAFEC = xQueueCreate(4, sizeof(uint32Ref_t));
	if (xQueueAFEC  == NULL) {
		printf("Failed to create AFEC queue \n");
	}

	xQueueTemperature = xQueueCreate(4, sizeof(floatRef_t));
	if (xQueueTemperature == NULL) {
		printf("Failed to create temperature queue \n");
	}

 	if (xTaskCreate(task_afec, "AFEC", TASK_AFEC_STACK_SIZE, NULL, TASK_AFEC_STACK_PRIORITY, NULL) != pdPASS) {
 		printf("Failed to create AFEC task\r\n");
 	}

	/* Create task to control oled */
	if (xTaskCreate(task_lcd, "LCD", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create LCD task\r\n");
	}

	/* Create task to read RTC */
 	if (xTaskCreate(task_rtc, "RTC", TASK_RTC_STACK_SIZE, NULL, TASK_RTC_STACK_PRIORITY, NULL) != pdPASS) {
 		printf("Failed to create RTC task\r\n");
 	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){ }
}
