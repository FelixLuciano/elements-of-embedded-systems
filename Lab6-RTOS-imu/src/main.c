#include <math.h>
#include "conf_board.h"
#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#include "mcu6050.h"

#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

#define LED_PIO     PIOC
#define LED_PIO_ID  ID_PIOC
#define LED_PIO_PIN 8
#define LED_PIO_PIN_MASK (1 << LED_PIO_PIN)

#define MPU6050_RA_WHO_AM_I         0x75

/** RTOS  */
#define TASK_IMU_STACK_SIZE                 (1024*6/sizeof(portSTACK_TYPE))
#define TASK_IMU_STACK_PRIORITY             (tskIDLE_PRIORITY)
#define TASK_THD_STACK_SIZE                 (1024*6/sizeof(portSTACK_TYPE))
#define TASK_THD_STACK_PRIORITY             (tskIDLE_PRIORITY)
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

QueueHandle_t xQueueAcc;
SemaphoreHandle_t xSemaphoreHouseDown;

/** prototypes */

typedef struct {
	float x;
	float y;
	float z;
} vec3_t;

void but_callback(void);
static void BUT_init(void);
static void mcu6050_i2c_bus_init(void);
int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
float get_norm(vec3_t vec);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

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
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_imu(void *pvParameters) {
	/* buffer para recebimento de dados */
	uint8_t bufferRX[10];
	uint8_t bufferTX[10];
	uint8_t response;
	
	mcu6050_i2c_bus_init();
	
	response = twihs_probe(TWIHS2, MPU6050_DEFAULT_ADDRESS);
	if(response == TWIHS_SUCCESS){
		printf("[DADO] [i2c] probe OK\n" );
	} else {
		printf("[ERRO] [i2c] [probe] \n");
	}
	
	response = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, bufferRX, 1);
	if(response!= TWIHS_SUCCESS || bufferRX[0] != MPU6050_DEFAULT_ADDRESS){
		printf("[ERRO] [i2c] [read] \n");
	} else {
		printf("[DADO] [i2c] %x:%x\n", MPU6050_RA_WHO_AM_I, bufferRX[0]);
	}

	// Set Clock source
	bufferTX[0] = MPU6050_CLOCK_PLL_XGYRO;

	response = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, bufferTX, 1);
	if(response != TWIHS_SUCCESS) {
		printf("[ERRO] [i2c] [write] \n");
	}

	// Aceletromtro em 2G
	bufferTX[0] = MPU6050_ACCEL_FS_2 << MPU6050_ACONFIG_AFS_SEL_BIT;

	response = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, bufferTX, 1);
	if(response != TWIHS_SUCCESS) {
		printf("[ERRO] [i2c] [write] \n");
	}

	// Configura range giroscopio para operar com 250 °/s
	bufferTX[0] = 0x00;

	response = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, bufferTX, 1);
	if(response != TWIHS_SUCCESS) {
		printf("[ERRO] [i2c] [write] \n");
	}
	
	int16_t  raw_acc_x, raw_acc_y, raw_acc_z;
	volatile uint8_t  raw_acc_xHigh, raw_acc_yHigh, raw_acc_zHigh;
	volatile uint8_t  raw_acc_xLow,  raw_acc_yLow,  raw_acc_zLow;
	vec3_t proc_acc;

	int16_t  raw_gyr_x, raw_gyr_y, raw_gyr_z;
	volatile uint8_t  raw_gyr_xHigh, raw_gyr_yHigh, raw_gyr_zHigh;
	volatile uint8_t  raw_gyr_xLow,  raw_gyr_yLow,  raw_gyr_zLow;
	vec3_t proc_gyr;

	for (;;)  {
		// Le valor do acc X High e Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, &raw_acc_xHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_L, &raw_acc_xLow,  1);
		// Le valor do acc y High e  Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, &raw_acc_yHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &raw_acc_yLow,  1);
		// Le valor do acc z HIGH e Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, &raw_acc_zHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &raw_acc_zLow,  1);

		// Dados são do tipo complemento de dois
		raw_acc_x = (raw_acc_xHigh << 8) | (raw_acc_xLow << 0);
		raw_acc_y = (raw_acc_yHigh << 8) | (raw_acc_yLow << 0);
		raw_acc_z = (raw_acc_zHigh << 8) | (raw_acc_zLow << 0);
		// Dados em escala real
		proc_acc.x = (float)raw_acc_x/16384;
		proc_acc.y = (float)raw_acc_y/16384;
		proc_acc.z = (float)raw_acc_z/16384;

		// Le valor do gyr X High e Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_H, &raw_gyr_xHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_L, &raw_gyr_xLow,  1);
		// Le valor do gyr y High e  Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_YOUT_H, &raw_gyr_yHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &raw_gyr_yLow,  1);
		// Le valor do gyr z HIGH e Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, &raw_gyr_zHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &raw_gyr_zLow,  1);

		// Dados são do tipo complemento de dois
		raw_gyr_x = (raw_gyr_xHigh << 8) | (raw_gyr_xLow << 0);
		raw_gyr_y = (raw_gyr_yHigh << 8) | (raw_gyr_yLow << 0);
		raw_gyr_z = (raw_gyr_zHigh << 8) | (raw_gyr_zLow << 0);
		// Dados em escala real
		proc_gyr.x = (float)raw_gyr_x/131;
		proc_gyr.y = (float)raw_gyr_y/131;
		proc_gyr.z = (float)raw_gyr_z/131;
		
		printf("%.4f ", get_norm(proc_acc));
		printf("%.4f\n", get_norm(proc_gyr));
		
		xQueueSendToBack(xQueueAcc, &proc_acc, 1);

		// uma amostra a cada 1ms
		vTaskDelay(1);
	}
}

static void task_house_down(void *pvParameters) {
	for(;;) {
		vec3_t acc_vec;
		if(xQueueReceive(xQueueAcc, &acc_vec, (TickType_t)0)) {
			float acc = get_norm(acc_vec);
			
			if (acc <= 0.03) {
				xSemaphoreGive(xSemaphoreHouseDown);
			}
		}
	}
}

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("IMU", 0, 0, &sysfont);

	for (;;)  {
		if(xSemaphoreTake(xSemaphoreHouseDown, 0)) {
			gfx_mono_draw_string("House is Down", 0, 20, &sysfont);

			for (int i = 0; i < 10; i++) {
				pio_clear(LED_PIO, LED_PIO_PIN_MASK);
				vTaskDelay(128 / portTICK_PERIOD_MS);	
			}
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

static void BUT_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf botão como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
}

static void mcu6050_i2c_bus_init(void) {
	twihs_options_t mcu6050_option;
	pmc_enable_periph_clk(ID_TWIHS2);

	/* Configure the options of TWI driver */
	mcu6050_option.master_clk = sysclk_get_cpu_hz();
	mcu6050_option.speed      = 40000;
	twihs_master_init(TWIHS2, &mcu6050_option);
	
	/** Enable TWIHS port to control PIO pins */
	pmc_enable_periph_clk(ID_PIOD);
	pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 28);
	pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 27);
}

int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt) {
	int32_t ierror = 0x00;

	twihs_packet_t p_packet;
	p_packet.chip         = dev_addr;
	p_packet.addr[0]      = reg_addr;
	p_packet.addr_length  = 1;
	p_packet.buffer       = reg_data;
	p_packet.length       = cnt;

	ierror = twihs_master_write(TWIHS2, &p_packet);

	return (int8_t)ierror;
}

int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt) {
	int32_t ierror = 0x00;

	twihs_packet_t p_packet;
	p_packet.chip         = dev_addr;
	p_packet.addr[0]      = reg_addr;
	p_packet.addr_length  = 1;
	p_packet.buffer       = reg_data;
	p_packet.length       = cnt;

	// TODO: Algum problema no SPI faz com que devemos ler duas vezes o registrador para
	//       conseguirmos pegar o valor correto.
	ierror = twihs_master_read(TWIHS2, &p_packet);

	return (int8_t)ierror;
}

float get_norm(vec3_t vec) {
	return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
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

	xQueueAcc = xQueueCreate(16, sizeof(vec3_t));
	if (xQueueAcc == NULL) {
		printf("falha em criar a queue\n");
	}

	xSemaphoreHouseDown = xSemaphoreCreateBinary();
	if (xSemaphoreHouseDown == NULL) {
		printf("falha em criar o semaforo\n");
	}

	if (xTaskCreate(task_imu, "imu", TASK_IMU_STACK_SIZE, NULL, TASK_IMU_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create imu task\r\n");
	}

	if (xTaskCreate(task_house_down, "THD", TASK_THD_STACK_SIZE, NULL, TASK_THD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create THD task\r\n");
	}

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "OLED", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
