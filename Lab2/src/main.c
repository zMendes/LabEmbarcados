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

#define LED_PIO           PIOC                 // periferico que controla o LED
#define LED_PIO_ID        12                  // ID do periférico PIOC (controla LED)
#define LED_PIO_IDX       8                    // ID do LED no PIO
#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED 

// Configuracoes do botao
#define BUT_PIO			  PIOA
#define BUT_PIO_ID		  10
#define BUT_PIO_IDX		  11
#define BUT_PIO_IDX_MASK (1u << BUT_PIO_IDX)

/*  Default pin configuration (no attribute). */
#define _PIO_DEFAULT             (0u << 0)
/*  The internal pin pull-up is active. */
#define _PIO_PULLUP              (1u << 0)
/*  The internal glitch filter is active. */
#define _PIO_DEGLITCH            (1u << 1)
/*  The internal debouncing filter is active. */
#define _PIO_DEBOUNCE            (1u << 3)

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);
void _pio_set(Pio *p_pio, const uint32_t ul_mask);
void _pio_clear(Pio *p_pio, const uint32_t ul_mask);
void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_up_enable);
void _pio_set_input(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_attribute);
void _pio_set_output(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_default_level, const uint32_t ul_multidrive_enable, const uint32_t ul_pull_up_enable);

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
void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_up_enable){
	if(ul_pull_up_enable){
		p_pio->PIO_PUER = ul_mask;} else {
			p_pio->PIO_PUDR;
		}
}
void _pio_set_input(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_attribute) {
		_pio_pull_up(p_pio, ul_mask, ul_attribute & _PIO_PULLUP);
		if (ul_attribute & (_PIO_DEGLITCH | _PIO_DEBOUNCE)){ 
			p_pio->PIO_IFER = ul_mask; } else { p_pio->PIO_IFDR = ul_mask;
	}
}
void _pio_set_output(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_default_level, const uint32_t ul_multidrive_enable, const uint32_t ul_pull_up_enable){
	
}

// Função de inicialização do uC
void init(void)
{
  // Initialize the board clock
  sysclk_init();

  // Desativa WatchDog Timer
  WDT->WDT_MR = WDT_MR_WDDIS;
  
  // Ativa o PIO na qual o LED foi conectado
  // para que possamos controlar o LED.
  pmc_enable_periph_clk(LED_PIO_ID);
  
  // Inicializa PIO do botao
  pmc_enable_periph_clk(BUT_PIO_ID);
  
  //Inicializa PC8 como saída
  pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);
  _pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);

}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void)
{
  init();
  char status;
  
  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
  while (1)
  {
	  status = pio_get(BUT_PIO,PIO_INPUT,BUT_PIO_IDX_MASK);
	  if (!status){
		  _pio_set(PIOC, LED_PIO_IDX_MASK);      // Coloca 1 no pino LED
		  delay_ms(500);                        // Delay por software de 200 
		  _pio_clear(PIOC, LED_PIO_IDX_MASK);    // Coloca 0 no pino do LED
		  delay_ms(200);}                        // Delay por software de 200 ms
	  else {
		  _pio_set(LED_PIO,LED_PIO_IDX_MASK);
		  delay_ms(600);
		  
	  }

  }
  return 0;
}
