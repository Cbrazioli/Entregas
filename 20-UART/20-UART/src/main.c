/**
 *	20-UART 
 * Prof. Rafael Corsi
 *
 *    (e.g., HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 */

#include "asf.h"
#include "conf_board.h"
#include "conf_clock.h"

/************************************************************************/
/* Configurações                                                        */
/************************************************************************/

#define STRING_EOL    "\r"
#define STRING_VERSAO "-- "BOARD_NAME" --\r\n" \
					  "-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

#define CONF_UART_BAUDRATE 115200		
#define CONF_UART          CONSOLE_UART

#define RXRDY 0	

/** 
 * LEDs
 */ 
#define PIN_LED_BLUE	19
#define PORT_LED_BLUE	PIOA
#define ID_LED_BLUE		ID_PIOA
#define MASK_LED_BLUE	(1u << PIN_LED_BLUE)

#define ID_LED_GREEN	ID_PIOA
#define PORT_LED_GREEN	PIOA
#define PIN_LED_GREEN	20
#define MASK_LED_GREEN	(1u << PIN_LED_GREEN)

#define ID_LED_RED		ID_PIOC
#define PORT_LED_RED	PIOC
#define PIN_LED_RED		20
#define MASK_LED_RED	(1u << PIN_LED_RED)

/************************************************************************/
/* Variaveis                                                            */
/************************************************************************/
char g_str[11];
uint32_t g_i = 0;

uint32_t g_Period_red = 0;
uint32_t g_Period_green = 0;
uint32_t g_Period_blue = 0;

uint32_t g_acumulador_red = 0;
uint32_t g_acumulador_green = 0;
uint32_t g_acumulador_blue = 0;

/************************************************************************/
/* Flags                                                                */
/************************************************************************/
uint32_t g_flag = 0;
uint32_t g_flag_red = 0;
uint32_t g_flag_blue = 0;
uint32_t g_flag_green = 0;

/************************************************************************/
/* Handler                                                               */
/************************************************************************/
void UART0_Handler(void){
	
	if(g_i < 10){
		g_str[g_i] = (UART0->UART_RHR) & (0xFF);
		if(g_str[g_i] == '\n'){
			g_str[g_i] = NULL;
			g_i=0;
			g_flag = 1;
		}
		else{
			g_i++;
		}
	}
	else{
		g_str[10] = NULL;
		g_i=0;
		g_flag = -1;
	}
	
}


void TC0_Handler(void)
{
	volatile uint32_t ul_dummy;

    /****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	if(g_flag_red == 1){
		g_acumulador_red++;
		if(g_acumulador_red >= g_Period_red){
			/** Muda o estado do LED */
			if(PIOC->PIO_ODSR & MASK_LED_RED)
				pio_clear(PORT_LED_RED, (1 << PIN_LED_RED));
			else
				pio_set(PORT_LED_RED, (1 << PIN_LED_RED));
			g_acumulador_red = 0;
		}
	}
	
	if(g_flag_green == 1){
		g_acumulador_green++;
		if(g_acumulador_green >= g_Period_green){
			/** Muda o estado do LED */
			if(PIOA->PIO_ODSR & MASK_LED_GREEN)
				pio_clear(PORT_LED_GREEN, (1 << PIN_LED_GREEN));
			else
				pio_set(PORT_LED_GREEN, (1 << PIN_LED_GREEN));
			g_acumulador_green = 0;
		}
	}
	
	if(g_flag_blue == 1){
		g_acumulador_blue++;
		if(g_acumulador_blue >= g_Period_blue){
			/** Muda o estado do LED */
			if(PIOA->PIO_ODSR & MASK_LED_BLUE)
				pio_clear(PORT_LED_BLUE, (1 << PIN_LED_BLUE));
			else
				pio_set(PORT_LED_BLUE, (1 << PIN_LED_BLUE));
			g_acumulador_blue = 0;
		}
	}

}


/************************************************************************/
/* Configura Periferico                                                     */
/************************************************************************/
void config_uart(void){
	
	/* configura pinos */
	gpio_configure_group(PINS_UART0_PIO, PINS_UART0, PINS_UART0_FLAGS);
	
	/* ativa clock */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	
	/* Configuração UART */
	const usart_serial_options_t uart_serial_options = {
		.baudrate   = CONF_UART_BAUDRATE,
		.paritytype = UART_MR_PAR_NO,
		.stopbits   = 0
	};
	
	stdio_serial_init((Usart *)CONF_UART, &uart_serial_options);
	
	/*Ativa interrupção no port B*/
	NVIC_EnableIRQ(ID_UART0);
	
	/*Ativa UART0 p/ gerar interrupção*/
	UART0->UART_IER = (1u << RXRDY);
}

static void configure_tc(void)
{
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	pmc_enable_periph_clk(ID_TC0);
	tc_init(TC0, 0, TC_CMR_CPCTRG | TC_CMR_TCCLKS_TIMER_CLOCK5);
    tc_write_rc(TC0, 0, 33);
	tc_enable_interrupt(TC0, 0, TC_IER_CPCS);
	NVIC_EnableIRQ(ID_TC0);
    tc_start(TC0, 0);
}

/**
 *  \brief Configure the LEDs
 *
 */
static void configure_leds(void)
{
	pmc_enable_periph_clk(ID_LED_GREEN);
	pio_set_output(PORT_LED_GREEN , MASK_LED_GREEN  ,1,0,0);
	pmc_enable_periph_clk(ID_LED_BLUE);
	pio_set_output(PORT_LED_BLUE , MASK_LED_BLUE  ,1,0,0);
	pmc_enable_periph_clk(ID_LED_RED);
	pio_set_output(PORT_LED_RED , MASK_LED_RED ,1,0,0);
}

/************************************************************************/
/* Funcoes                                                      */
/************************************************************************/
static void display_menu(void)
{
	puts(" 1 : exibe novamente esse menu \n\r");
}

void ToGetPeriod(char str[], uint32_t *p)
{
	uint32_t i;
	
	for(i=0; i<=8; i++)
		str[i] = str[i+2];
		
	*p = (uint32_t)(1000/atoi(str));
	printf("uC freuqencia lida: %i \n", *p);
	printf("frequencia digitada: %i \n", atoi(str));
}


/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{
	uint32_t str[11];
	
	/* Initialize the global variables */
	g_i = 0;
	g_flag = 0;
	
	/* Initialize the system */
	sysclk_init();
	board_init();

	/* Configure LED 1 */
	configure_leds();

	/** Configura o timer */
	configure_tc();
	
	/* Initialize debug console */
	config_uart();
		
	/* frase de boas vindas */
	puts(" ---------------------------- \n\r"
	 	 " Bem vindo terraquio !		\n\r"
		 " ---------------------------- \n\r");
		 
	/* display main menu */
	display_menu();

	while (1) {
		
		if(g_flag == 1){
			printf("uC recebeu: %s \n", g_str);
			g_flag = 0;
			
			switch(g_str[1]){
				case 'r':{
					if(g_str[0] == '0'){
						g_flag_red = 0;
						pio_clear(PORT_LED_RED, MASK_LED_RED);
					}
					else if(g_str[0] == '1'){
						g_flag_red = 1;
						ToGetPeriod(g_str, &g_Period_red);
						printf("Periodo: %i\n", g_Period_red);
					}
					else{
						printf("Operacao invalida\n");
					}			
					break;
				}
				case 'g':{
					if(g_str[0] == '0'){
						g_flag_green = 0;
						pio_set(PORT_LED_GREEN, MASK_LED_GREEN);

					}
					else if(g_str[0] == '1'){
						g_flag_green = 1;
						ToGetPeriod(g_str, &g_Period_green);
					}
					else{
						printf("Operacao invalida\n");
					}
					break;
				}
				case 'b':{
					if(g_str[0] == '0'){
						g_flag_blue = 0;
						pio_set(PORT_LED_BLUE, MASK_LED_BLUE);
					}
					else if(g_str[0] == '1'){
						g_flag_blue = 1;
						ToGetPeriod(g_str, &g_Period_blue);
					}
					else{
						printf("Operacao invalida\n");
					}
					break;
				}
				default:{
					printf("Opcao de led invalido\n");
					break;
				}
			}
		}
		else if(g_flag == -1){
			printf("uC recebeu dados invalidos:  %s \n", g_str);
			g_flag = 0;	
		}
		else{
			
		}
	}
}






