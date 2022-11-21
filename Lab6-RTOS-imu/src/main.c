#include <math.h>
#include "conf_board.h"
#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#include "mcu6050.h"
#include "Fusion/Fusion/Fusion.h"

#define LED_0_PIO     PIOC
#define LED_0_PIO_ID  ID_PIOC
#define LED_0_PIO_PIN 8
#define LED_0_PIO_PIN_MASK (1 << LED_0_PIO_PIN)

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

#define MPU6050_RA_WHO_AM_I         0x75

/** RTOS  */
#define TASK_IMU_STACK_SIZE                 (1024*6/sizeof(portSTACK_TYPE))
#define TASK_IMU_STACK_PRIORITY             (tskIDLE_PRIORITY)
#define TASK_THD_STACK_SIZE                 (1024*6/sizeof(portSTACK_TYPE))
#define TASK_THD_STACK_PRIORITY             (tskIDLE_PRIORITY)
#define TASK_ORI_STACK_SIZE                 (1024*6/sizeof(portSTACK_TYPE))
#define TASK_ORI_STACK_PRIORITY             (tskIDLE_PRIORITY)
#define TASK_IO_STACK_SIZE                  (1024*6/sizeof(portSTACK_TYPE))
#define TASK_IO_STACK_PRIORITY              (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

QueueHandle_t xQueueAcc;
QueueHandle_t xQueueGyr;
SemaphoreHandle_t xSemaphoreHouseDown;

/** prototypes */
enum orientacao {
	ESQUERDA,
	FRENTE,
	DIREITA,
};

void but_callback(void);
static void IO_init(void);
static void mcu6050_i2c_bus_init(void);
int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);

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

	int16_t  raw_gyr_x, raw_gyr_y, raw_gyr_z;
	volatile uint8_t  raw_gyr_xHigh, raw_gyr_yHigh, raw_gyr_zHigh;
	volatile uint8_t  raw_gyr_xLow,  raw_gyr_yLow,  raw_gyr_zLow;

	FusionAhrs ahrs;
	FusionAhrsInitialise(&ahrs);

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
		const FusionVector accelerometer = {
			(float)raw_acc_x/16384,
			(float)raw_acc_y/16384,
			(float)raw_acc_z/16384,
		};

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
		const FusionVector gyroscope = {
			(float)raw_gyr_x/131,
			(float)raw_gyr_y/131,
			(float)raw_gyr_z/131,
		};

		xQueueSendToBack(xQueueAcc, &accelerometer, 1);

		// Tempo entre amostras
		float dT = 0.1;
		// aplica o algoritmo
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dT);
		// dados em pitch roll e yaw
		const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

		printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
		
		enum orientacao direcao;

		if (euler.angle.pitch >= -90 && euler.angle.pitch <= -60) {
			if (euler.angle.roll >= -20 && euler.angle.roll <= 20) {
				direcao = FRENTE;
				xQueueSendToBack(xQueueGyr, &direcao, 1);
			}
			else if (euler.angle.roll >= -100 && euler.angle.roll <= -80) {
				direcao = DIREITA;
				xQueueSendToBack(xQueueGyr, &direcao, 1);
			}
			else if (euler.angle.roll >= 80 && euler.angle.roll <= 100) {
				direcao = ESQUERDA;
				xQueueSendToBack(xQueueGyr, &direcao, 1);
			}
		}

		// uma amostra a cada 1ms
		vTaskDelay(1);
	}
}

static void task_house_down(void *pvParameters) {
	FusionVector acceleration;

	for(;;) {
		if(xQueueReceive(xQueueAcc, &acceleration, (TickType_t)0)) {
			float acc = FusionVectorMagnitude(acceleration);

			if (acc <= 0.35) {
				xSemaphoreGive(xSemaphoreHouseDown);
			}
		}
	}
}

static void task_orientacao(void *pvParameters) {
	enum orientacao direcao;

	for(;;) {
		if(xQueueReceive(xQueueGyr, &direcao, (TickType_t) 256)) {
			if (direcao == ESQUERDA) {
				pio_clear(LED_1_PIO, LED_1_PIO_IDX_MASK);
				pio_set(LED_2_PIO, LED_2_PIO_IDX_MASK);
				pio_set(LED_3_PIO, LED_3_PIO_IDX_MASK);
				printf("Esquerda\n");
			}
			else if (direcao == FRENTE) {
				pio_set(LED_1_PIO, LED_1_PIO_IDX_MASK);
				pio_clear(LED_2_PIO, LED_2_PIO_IDX_MASK);
				pio_set(LED_3_PIO, LED_3_PIO_IDX_MASK);
				printf("Frente\n");
			}
			else if (direcao == DIREITA) {
				pio_set(LED_1_PIO, LED_1_PIO_IDX_MASK);
				pio_set(LED_2_PIO, LED_2_PIO_IDX_MASK);
				pio_clear(LED_3_PIO, LED_3_PIO_IDX_MASK);
				printf("Direita\n");
			}
		}
		else {
			pio_set(LED_1_PIO, LED_1_PIO_IDX_MASK);
			pio_set(LED_2_PIO, LED_2_PIO_IDX_MASK);
			pio_set(LED_3_PIO, LED_3_PIO_IDX_MASK);
		}
	}
}

static void task_io(void *pvParameters) {
	IO_init();
	
	IO_init();gfx_mono_ssd1306_init();
	gfx_mono_draw_string("IMU", 0, 0, &sysfont);

	for (;;)  {
		if(xSemaphoreTake(xSemaphoreHouseDown, 0)) {
			gfx_mono_draw_string("House is Down", 0, 20, &sysfont);

			for (int i = 0; i < 5; i++) {
				pio_clear(LED_0_PIO, LED_0_PIO_PIN_MASK);
				vTaskDelay(128 / portTICK_PERIOD_MS);
				pio_set(LED_0_PIO, LED_0_PIO_PIN_MASK);
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

static void IO_init(void) {
	pmc_enable_periph_clk(LED_1_PIO);
	pmc_enable_periph_clk(LED_2_PIO);
	pmc_enable_periph_clk(LED_3_PIO);

	pio_set_output(LED_1_PIO, LED_1_PIO_IDX_MASK, 1, 0, 0);
	pio_set_output(LED_2_PIO, LED_2_PIO_IDX_MASK, 1, 0, 0);
	pio_set_output(LED_3_PIO, LED_3_PIO_IDX_MASK, 1, 0, 0);
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

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	xQueueAcc = xQueueCreate(16, sizeof(FusionVector));
	if (xQueueAcc == NULL) {
		printf("falha em criar a queue acc\n");
	}

	xQueueGyr = xQueueCreate(16, sizeof(enum orientacao));
	if (xQueueGyr == NULL) {
		printf("falha em criar a queue gyr\n");
	}

	xSemaphoreHouseDown = xSemaphoreCreateBinary();
	if (xSemaphoreHouseDown == NULL) {
		printf("falha em criar o semaforo HouseDown\n");
	}

	if (xTaskCreate(task_imu, "imu", TASK_IMU_STACK_SIZE, NULL, TASK_IMU_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create imu task\r\n");
	}

	if (xTaskCreate(task_house_down, "THD", TASK_THD_STACK_SIZE, NULL, TASK_THD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create THD task\r\n");
	}

	if (xTaskCreate(task_orientacao, "ORIENTACAO", TASK_ORI_STACK_SIZE, NULL, TASK_ORI_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Orientacao task\r\n");
	}

	/* Create task to control oled */
	if (xTaskCreate(task_io, "IO", TASK_IO_STACK_SIZE, NULL, TASK_IO_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create io task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
