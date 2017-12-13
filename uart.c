/***********************************************************************
 * $Id:: uart.c 1604 2012-04-24 11:34:47Z nxp31103     $
 *
 * Project: CANopen Application Example for LPC11Cxx
 *
 * Description:
 *   UART driver source file
 *
 * Copyright(C) 2012, NXP Semiconductor
 * All rights reserved.
 *
 ***********************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 **********************************************************************/

#include "uart.h"

/**
  * @brief  Initializes the UART0.
  *
  * @param  baudrate: Specifies the baud rate in Hz.
  * @retval None
  */
void UART_Init(uint32_t baudrate)
{
    uint32_t  Fdiv;

	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<16) ;

	LPC_IOCON->PIO1_6 &= ~0x07;     /* UART I/O config */
	LPC_IOCON->PIO1_6 |=  0x01;     /* UART RXD */
	LPC_IOCON->PIO1_7 &= ~0x07;
	LPC_IOCON->PIO1_7 |=  0x01;     /* UART TXD */

	/* Enable UART clock */
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);
	LPC_SYSCON->UARTCLKDIV     = 0x1;     /* divided by 1 */

    LPC_UART->LCR = 0x83;		/* 8 bits, no Parity, 1 Stop bit */
    Fdiv = ((SystemCoreClock/LPC_SYSCON->UARTCLKDIV)/16)/baudrate ;	/*baud rate */
    LPC_UART->DLM = Fdiv / 256;
    LPC_UART->DLL = Fdiv % 256;
    LPC_UART->LCR = 0x03;		/* DLAB = 0 */
    //LPC_UART->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */
}

/**
  * @brief  Transmit a single character through UART0.
  *
  * @param  ch: character to be transmitted.
  * @retval None
  */
void UART_PutChar (uint8_t ch)
{
   while (!(LPC_UART->LSR & 0x20));
   LPC_UART->THR  = ch;
}

/**
  * @brief  Receive a single character through UART0.
  *
  * @param  None.
  * @retval Received character
  */
uint8_t UART_GetChar (void)
{
  while (!(LPC_UART->LSR & 0x01));
  return (LPC_UART->RBR);
}

/**
  * @brief  Transmit a string through UART0.
  *
  * @param  str: Pointer to the string terminated with '\0'.
  * @retval None
  */
void UART_PutString (const uint8_t * str)
{
    while ((*str) != 0)
    {
        if (*str == '\n') {
            UART_PutChar(*str++);
            UART_PutChar('\r');
        } else {
            UART_PutChar(*str++);
        }
    }
}
