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

/************************************************************************/
/* Variaveis                                                            */
/************************************************************************/
uint8_t g_str[11];
uint8_t g_i = 0;

/************************************************************************/
/* Flags                                                                */
/************************************************************************/

/************************************************************************/
/* Handler                                                               */
/************************************************************************/
void UART0_Handler(void){
	
	g_str[g_i] = (UART0->UART_RHR) & (0xFF);
	if(g_i>=10){
		g_i=0;
		g_str[10] = '\n';
	}
	else{
		g_i++;
		g_str[g_i] = '\n';
	}
}

/************************************************************************/
/* Configura UART                                                       */
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

/************************************************************************/
/* Display Menu                                                         */
/************************************************************************/
static void display_menu(void)
{
	puts(" 1 : exibe novamente esse menu \n\r"
		 " 2 : Ativa o LED  \n\r"
		 " 3 : Desliga o LED \n\r ");
}


/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{
	uint8_t str[11];

	/* Initialize the system */
	sysclk_init();
	board_init();

	/* Configure LED 1 */
	pmc_enable_periph_clk(ID_LED_BLUE);
	pio_set_output(PORT_LED_BLUE  , MASK_LED_BLUE	,1,0,0);

	/* Initialize debug console */
	config_uart();
		
	/* frase de boas vindas */
	puts(" ---------------------------- \n\r"
	 	 " Bem vindo terraquio !		\n\r"
		 " ---------------------------- \n\r");
		 
	/* display main menu */
	display_menu();

	while (1) {

		//printf("Opcao nao definida: %s \n\r", str);
		/*switch (uc_key) {
			case '1':
				display_menu();
				break;
			case '2':
				pio_clear(PORT_LED_BLUE, MASK_LED_BLUE);
				puts("Led ON \n\r");
				break;
			case '3':
				pio_set(PORT_LED_BLUE, MASK_LED_BLUE);
				puts("Led OFF \n\r");
				break;
			default:
				printf("Opcao nao definida: %d \n\r", *uc_key);
		}*/
	}
}






