/*
===============================================================================
 Name        : 28_Day61_16112021_UART003.c

 Description : Program to demonstrate the UART0 Reception using interrupt method

 TESTCASE0   : UART0 Reception using interrupt method and Transmission using polling method
 TESTCASE1	 : UART0 Reception using interrupt method and Transmission using interrupt method


 Layered Architecture used for this project
 ************************************
 Application layer-28_Day61_16112021_UART003.c
 ************************************
 Board layer -  configboard.h, led.c/.h
 ************************************
 Low level drivers or chip level - pinmux.c/.h,uart.c/.h, gpio.c/.h
 ************************************
 Hardware
 ************************************
===============================================================================
*/

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Private includes ----------------------------------------------------------*/
#include "pinmux.h"
#include "uart.h"
#include "led.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define TESTCASE0 0
#define TESTCASE1 1
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
void vUARTCallbackHandler(void)
{
	volatile uint32_t UART0IntIdentifcationReg=0;
	volatile uint32_t UART0Intsrc=0;
	volatile char recevd_data=0x00;
	UART0IntIdentifcationReg = UART0->IIR;
	UART0Intsrc = (UART0IntIdentifcationReg >> 1) & 0x07;
	switch(UART0Intsrc)
	{

	case THRE_INT:
#if TESTCASE1
		/* For demonstration purpose */
		LPC_UART0->THR = 'K';
		vUARTInterruptDisable(UART0,THRE_INTERRUPT);
#endif
		break;
	case RDA:
		/* Receiving the data from the UART Terminal */
		recevd_data = UART0->RBR;
		/* Echoing it back to UART Terminal for viewing purpose of received data */
		UART0->THR = recevd_data;
#if TESTCASE1
		vUARTInterruptEnable(UART0,THRE_INTERRUPT);
#endif
		break;
	default:
		/* DO NOTHING */
		break;
	}
}

/**
  * @brief  Initialize all the hardware connected
  * @retval none
  */
void vAppHardwareInit(void)
{
	vPinmuxInitialize();
	vLedInitialize();
	vUARTInitialize(UART0,UART_0,BAUDRATE_9600);
	/* Attaching the vUARTCallbackHandler function when interrupt occurs in UART0 */
	vUARTIntAttachCallback(UART_0,vUARTCallbackHandler);
}

/**
  * @brief  Crude Delay
  * @retval none
  */
void vAppDelay(uint32_t count)
{
	int i,j;
	for(i=0;i<count;i++)
		for(j=0;j<0xA00;j++);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Initialize all configured peripherals */
   vAppHardwareInit();

#if	TESTCASE0
	vUARTInterruptDisable(UART0,THRE_INTERRUPT);
	vUARTPutCharBlocking(UART0,'T');
#endif

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)  // for(;;)
  {

		vLedOn(LED_0);
		vAppDelay(0x200);
		vLedOff(LED_0);
		vAppDelay(0x200);

  }
  /* End of Application entry point */
}




