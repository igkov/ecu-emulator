/***********************************************************************
 * $Id:: uart.h 1604 2012-04-24 11:34:47Z nxp31103     $
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

#ifndef UART_H
#define UART_H

#include  "LPC11xx.h"
#include <stdio.h>

volatile extern uint8_t StopReadingUART;

void    UART_Init (uint32_t  baudrate);
void    UART_PutChar (uint8_t ch);
uint8_t UART_GetChar (void);
void    UART_PutString (const uint8_t  *str);

#endif
