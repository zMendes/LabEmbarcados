/*
* Example RTOS Atmel Studio
*/

#include <asf.h>
#include "conf_board.h"
#define TASK_LED1_STACK_SIZE (1024/sizeof(portSTACK_TYPE))
#define TASK_LED1_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define LED1_PIO			PIOA
#define LED1_PIO_ID			ID_PIOA
#define LED1_PIO_IDX		0
#define LED1_PIO_IDX_MASK	(1 << LED1_PIO_IDX)

#define BUT1_PIO            PIOD
#define BUT1_PIO_ID         16
#define BUT1_PIO_IDX        28
#define BUT1_PIO_IDX_MASK   (1u << BUT1_PIO_IDX)
/** Semaforo a ser usado pela task led 
    tem que ser var global! */
SemaphoreHandle_t xSemaphore;
QueueHandle_t xQueueChar;
QueueHandle_t xQueueCommand;


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

void LED_init(int estado);

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}
void but1_callback(void){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    printf("but_callback \n");
    xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
    printf("semafaro tx \n");
}
/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}
static void task_led(void *pvParameters) {
  /* We are using the semaphore for synchronisation so we create a binary
  semaphore rather than a mutex.  We must make sure that the interrupt
  does not attempt to use the semaphore before it is created! */
  xSemaphore = xSemaphoreCreateBinary();

  /* devemos iniciar a interrupcao no pino somente apos termos alocado
  os recursos (no caso semaforo), nessa funcao inicializamos 
  o botao e seu callback*/
  /* init botão */
  pmc_enable_periph_clk(BUT1_PIO_ID);
  pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP);
  pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but1_callback);
  pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
  NVIC_EnableIRQ(BUT1_PIO_ID);
  NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4

  if (xSemaphore == NULL)
    printf("falha em criar o semaforo \n");

  for (;;) {
    if( xSemaphoreTake(xSemaphore, ( TickType_t ) 500) == pdTRUE ){
      LED_Toggle(LED0);
    }
  }
}
/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
    static portCHAR szList[256];
    UNUSED(pvParameters);

    for (;;) {
        printf("--- task ## %u", (unsigned int)uxTaskGetNumberOfTasks());
        vTaskList((signed portCHAR *)szList);
        printf(szList);
        vTaskDelay(3000);
    }
}
void pisca_led(int n, int t,int led){
	for (int i=0;i<n;i++){
	if(led==1){
		pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
		vTaskDelay(t);
		pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
		vTaskDelay(t);
		}
			
		}
	}
void LED_init(int estado){
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, estado, 0, 0 );
	
}

/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */

static void task_led1(void *pvParameters)
{
	LED_init(1);
	UNUSED(pvParameters);
	for (;;) {
		pisca_led(3, 200,1);
		vTaskDelay(1500);
	}
}

static void USART1_init(void){
  /* Configura USART1 Pinos */
  sysclk_enable_peripheral_clock(ID_PIOB);
  sysclk_enable_peripheral_clock(ID_PIOA);
  pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4); // RX
  pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
  MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;

  /* Configura opcoes USART */
  const sam_usart_opt_t usart_settings = {
      .baudrate       = 115200,
      .char_length    = US_MR_CHRL_8_BIT,
      .parity_type    = US_MR_PAR_NO,
      .stop_bits    = US_MR_NBSTOP_1_BIT    ,
      .channel_mode   = US_MR_CHMODE_NORMAL
  };

  /* Ativa Clock periferico USART0 */
  sysclk_enable_peripheral_clock(ID_USART1);

  stdio_serial_init(CONF_UART, &usart_settings);

  /* Enable the receiver and transmitter. */
  usart_enable_tx(USART1);
  usart_enable_rx(USART1);

  /* map printf to usart */
  ptr_put = (int (*)(void volatile*,char))&usart_serial_putchar;
  ptr_get = (void (*)(void volatile*,char*))&usart_serial_getchar;

  /* ativando interrupcao */
  usart_enable_interrupt(USART1, US_IER_RXRDY);
  NVIC_SetPriority(ID_USART1, 4);
  NVIC_EnableIRQ(ID_USART1);
}

void USART1_Handler(void){
  uint32_t ret = usart_get_status(USART1);

  BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  char c;

  // Verifica por qual motivo entrou na interrupçcao?
  // RXRDY ou TXRDY

  //  Dados disponível para leitura
  if(ret & US_IER_RXRDY){
      usart_serial_getchar(USART1, &c);
      printf("%c", c);
	  xQueueSend(xQueueChar, &c, 0);

  // -  Transmissoa finalizada
  } else if(ret & US_IER_TXRDY){

  }
}

uint32_t usart1_puts(uint8_t *pstring){
    uint32_t i ;

    while(*(pstring + i))
        if(uart_is_tx_empty(USART1))
            usart_serial_putchar(USART1, *(pstring+i++));
}

static void task_uartRX(){
	xQueueChar = xQueueCreate(1,sizeof(char));

	char command[1];

	for(int i;;i++){
		if (xQueueReceive(xQueueChar, %(command[i]), ( TickType_t )  100 / portTICK_PERIOD_MS)){
			if (command[i]==0) xQueueCommand.send(command);
		}
	}

}

static void task_execute(){

	xQueueCommand = xQueueCreate(10, sizeof(char));
	char a;
	if (xQueueReceive(xQueueCommand, %(a), ( TickType_t )  100 / portTICK_PERIOD_MS)){

		if (a =='1') LED_Toggle(LED1);

		if (a =='2') LED_Toggle(LED2);
		
		if (a =='3') LED_Toggle(LED3);



	}


}

/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	/* Initialize the console uart */
	USART1_init();

	/* Output demo information. */
	printf("-- Freertos Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

	xTaskCreate(task_led1, "Led1", TASK_LED1_STACK_SIZE, NULL, TASK_LED1_STACK_PRIORITY, NULL);
	/* Create task to monitor processor activity */
	if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL,
			TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Monitor task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
			TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
