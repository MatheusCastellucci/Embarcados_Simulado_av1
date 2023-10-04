#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

#define IN1_PIO     PIOD
#define IN1_PIO_ID  ID_PIOD
#define IN1_PIO_PIN 30
#define IN1_PIO_PIN_MASK (1 << IN1_PIO_PIN)

#define IN2_PIO     PIOA
#define IN2_PIO_ID  ID_PIOA
#define IN2_PIO_PIN 6
#define IN2_PIO_PIN_MASK (1 << IN2_PIO_PIN)

#define IN3_PIO     PIOC
#define IN3_PIO_ID  ID_PIOC
#define IN3_PIO_PIN 19
#define IN3_PIO_PIN_MASK (1 << IN3_PIO_PIN)

#define IN4_PIO     PIOA
#define IN4_PIO_ID  ID_PIOA
#define IN4_PIO_PIN 2
#define IN4_PIO_PIN_MASK (1 << IN4_PIO_PIN)

#define BUT1_PIO     PIOD
#define BUT1_PIO_ID  ID_PIOD
#define BUT1_PIO_PIN 28
#define BUT1_PIO_PIN_MASK (1 << BUT1_PIO_PIN)

#define BUT2_PIO     PIOC
#define BUT2_PIO_ID  ID_PIOC
#define BUT2_PIO_PIN 31
#define BUT2_PIO_PIN_MASK (1 << BUT2_PIO_PIN)

#define BUT3_PIO     PIOA
#define BUT3_PIO_ID  ID_PIOA
#define BUT3_PIO_PIN 19
#define BUT3_PIO_PIN_MASK (1 << BUT3_PIO_PIN)


/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

double graus_por_passo = 0,17578125;
int passos_para_completar360 = 2048;
const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

QueueHandle_t xQueueModo;
QueueHandle_t xQueueSteps;

SemaphoreHandle_t xSemaphoreRTT;

/** prototypes */
void but_callback(void);
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);
static void BUT_init(void);
static void task_modo(void *pvParameters);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
static void fio_0(int E);
static void fio_1(int E);
static void fio_2(int E);
static void fio_3(int E);


/** Interrupção**/

void RTT_Handler(void) {
  uint32_t ul_status;
  ul_status = rtt_get_status(RTT);

  /* IRQ due to Alarm */
  if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreRTT, &xHigherPriorityTaskWoken);
   }  
}

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

void but1_callback(void) {
	int grau = 180;
	xQueueSendFromISR(xQueueModo, &id, 0);
}

void but2_callback(void) {
	int grau = 90;
	xQueueSendFromISR(xQueueModo, &id, 0);
}

void but3_callback(void) {
	int grau = 45;
	xQueueSendFromISR(xQueueModo, &id, 0);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
  gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
  gfx_mono_draw_string("oii", 0, 20, &sysfont);

	for (;;)  {
    

	}
}

static void task_modo(){
	char msg[20];
	for(;;){
		if( xQueueReceive( xQueueModo, &grau, ( TickType_t ) 500 )){
			int passos = (int)grau / graus_por_passo;
			xQueueSendToBack(xQueueSteps, &passos, 0);

			sprintf(msg, "Modo: %d", grau);
			gfx_mono_draw_string(msg, 0, 10, &sysfont);
      	}
	  	vTaskDelay(xDelay);
	}
}

static void task_motor(void *pvParameters) {
	for (;;) {
		if( xQueueReceive( xQueueSteps, &passos, ( TickType_t ) 500 )){
			int counter = 1;
			for(int i = 0; i < (passos/4); i++){
				fio_0(counter &1);
				fio_1(counter &2);
				fio_2(counter &4);
				fio_3(counter &8);
				//verificar se o semaforo do rtt esta livre
				if(RTT_Handler(RTT)){
					if (counter == 8){
						counter = 1;
					}
					else{
						counter *= 2;
					}
				}
		}
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

static void BUT_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf bot�o como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);

	/* configura prioridae */
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4);

	/* conf bot�o como entrada */
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_PIN_MASK);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but1_callback);

	/* configura prioridae */
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4);

	/* conf bot�o como entrada */
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_PIN_MASK);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but2_callback);

	/* configura prioridae */
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4);

	/* conf bot�o como entrada */
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_PIN_MASK);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but3_callback);
}

static void fio_0(int E){ 
	if(E){
		pio_set(IN1_PIO, IN1_PIO_IDX_MASK);
	}
	else{
		pio_clear(IN1_PIO, IN1_PIO_IDX_MASK);
	}
}

static void fio_1(int E){
	if(E){
		pio_set(IN2_PIO, IN2_PIO_IDX_MASK);
	}
	else{
		pio_clear(IN2_PIO, IN2_PIO_IDX_MASK);
	}
}

static void fio_2(int E){
	if(E){
		pio_set(IN3_PIO, IN3_PIO_IDX_MASK);
	}
	else{
		pio_clear(IN3_PIO, IN3_PIO_IDX_MASK);
	}
}

static void fio_3(int E){
	if(E){
		pio_set(IN4_PIO, IN4_PIO_IDX_MASK);
	}
	else{
		pio_clear(IN4_PIO, IN4_PIO_IDX_MASK);
	}
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

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}

	if (xTaskCreate(task_modo, "modo", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create modo task\r\n");
	}

	xQueueModo = xQueueCreate(32, sizeof(int) );
	xQueueSteps = xQueueCreate(32, sizeof(int) );

	// verifica se fila foi criada corretamente
	if (xQueueSteps == NULL)
		printf("falha em criar a fila \n");
	// verifica se fila foi criada corretamente
	if (xQueueModo == NULL)
		printf("falha em criar a fila \n");

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
