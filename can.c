/*----------------------------------------------------------------------------
 * Name:    can.c
 * Purpose: CAN interface for for MCB100 populated with LPC11C14
 * Note(s): 
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2010 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include "LPC11xx.h"                             /* LPC11xx definitions        */
#include "can.h"                                 /* LPC11xx CAN adaption layer */
#include "dbg.h"

/*------- LPC11Cxx  CAN Register Definitions ---------------------------------*/

/*- CAN CMDREQ register ------------------------------------------------------*/
#define _CMDREQ_BUSY       (1UL << 15)

/*- CAN CTRL register --------------------------------------------------------*/
#define _CNTL_INIT         (1UL <<  0)
#define _CNTL_IE           (1UL <<  1) 
#define _CNTL_SIE          (1UL <<  2)
#define _CNTL_EIE          (1UL <<  3) 
#define _CNTL_DAR          (1UL <<  5)
#define _CNTL_CCE          (1UL <<  6) 
#define _CNTL_TEST         (1UL <<  7)
	
/*- CAN TEST register --------------------------------------------------------*/
#define _TEST_BASIC        (1UL <<  2)
#define _TEST_SILENT       (1UL <<  3)
#define _TEST_LBACK        (1UL <<  4)

/*- CAN STAT register --------------------------------------------------------*/
#define _STAT_LEC          (7UL <<  0)
#define _STAT_TXOK         (1UL <<  3) 
#define _STAT_RXOK         (1UL <<  4)
#define _STAT_EPASS        (1UL <<  5) 
#define _STAT_EWARN        (1UL <<  6)
#define _STAT_BOFF         (1UL <<  7)

/*- CAN CMDMASK register -----------------------------------------------------*/
#define	_CMDMASK_DATAB     (1UL <<  0)
#define	_CMDMASK_DATAA     (1UL <<  1)
#define	_CMDMASK_TREQ      (1UL <<  2)
#define	_CMDMASK_INTPND    (1UL <<  3)
#define	_CMDMASK_CTRL      (1UL <<  4)
#define	_CMDMASK_ARB       (1UL <<  5)
#define	_CMDMASK_MASK      (1UL <<  6)
#define	_CMDMASK_WR        (1UL <<  7)
#define _CMDMASK_RD        (0UL <<  7)

/*- CAN MSK1 register --------------------------------------------------------*/
#define _MSK1_MSK          (0xFFFF)

/*- CAN MSK2 register --------------------------------------------------------*/
#define _MSK2_MSK          (0x1FFF)
#define	_MSK2_MXTD         (1UL << 15)
#define	_MSK2_MDIR         (1UL << 14)

/*- CAN ARB1 register --------------------------------------------------------*/
#define _ARB1_ID           (0xFFFF)

/*- CAN ARB2 register --------------------------------------------------------*/
#define _ARB2_ID           (0x1FFF)
#define	_ARB2_DIR          (1UL << 13)
#define	_ARB2_XTD          (1UL << 14)
#define	_ARB2_MSGVAL       (1UL << 15)

/*- CAN MCTRL register -------------------------------------------------------*/
#define	_MCTRL_DLC         (0x0F)   
#define	_MCTRL_EOB         (1UL <<  7)
#define _MCTRL_TXRQST      (1UL <<  8)
#define	_MCTRL_RMTEN       (1UL <<  9)
#define	_MCTRL_RXIE        (1UL << 10)
#define	_MCTRL_TXIE        (1UL << 11)
#define _MCTRL_UMASK       (1UL << 12)
#define	_MCTRL_INTPND      (1UL << 13)
#define	_MCTRL_MSGLST      (1UL << 14)
#define	_MCTRL_NEWDAT      (1UL << 15)


#define _MSG_OBJ_MAX        0x0020         /* 1..32 */

#define _STD_FORMAT         0x000007FF     /* 11 bit standard format */	
#define _EXT_FORMAT         0x1FFFFFFF     /* 29 bit extended format */
/* -- end LPC11Cxx  CAN Register Definitions -------------------------------- */


CAN_msg       CAN_TxMsg;                         /* CAN message for sending */
CAN_msg       CAN_RxMsg;                         /* CAN message for receiving */                                
CAN_msg       CAN_RtMsg;                         /* CAN message for sending */

unsigned int  CAN_TxRdy = 0;                     /* CAN HW ready to transmit a message */
unsigned int  CAN_RxRdy = 0;                     /* CAN HW received a message */


/*----------------------------------------------------------------------------
  configure the requested baudrate
 *----------------------------------------------------------------------------*/
static void CAN_cfgBaudrate (uint32_t baudrate)  {
  LPC_CAN->BT   = 0x2301;        /* 500kBit/s @ 8MHz CAN clk */
  LPC_CAN->BRPE = 0x0000;
}


/*----------------------------------------------------------------------------
  set the testmode
 *----------------------------------------------------------------------------*/
void CAN_testmode (void) {
  LPC_CAN->CNTL |=  _CNTL_TEST;                 /* enable Test mode     */ // important to set this register first!
  LPC_CAN->TEST  = _TEST_LBACK;                 /* enable loopback mode */
}

/*----------------------------------------------------------------------------
  setup CAN interface.
 *----------------------------------------------------------------------------*/
void CAN_setup (void)  {
  uint32_t i;

  /* CAN_RX, CAN_TX are dedicated Pins so no GPIO configuration is necessary */ 

  /* configure CAN */
  LPC_SYSCON->PRESETCTRL    |=  (1UL <<  3);    /* de-asserts the reset signal */
  LPC_SYSCON->SYSAHBCLKCTRL |=  (1UL << 17);    /* enable power to CAN  block */
    
  LPC_CAN->CLKDIV = (SystemCoreClock / 8000000UL) - 1; /* set CAN clk to 8MHz */

  LPC_CAN->CNTL  =  _CNTL_INIT;                 /* set initialization mode by default */

  LPC_CAN->CNTL |=  _CNTL_CCE;                  /* Sart configuring bit timing */
  CAN_cfgBaudrate(500000);                      /* Set bit timing */
  LPC_CAN->CNTL &= ~_CNTL_CCE;                  /* Stop configuring bit timing */

  NVIC_EnableIRQ(CAN_IRQn);                     /* enable CAN interrupt */

  /* pre-initialize CAN message objects */ 
  for ( i = 0; i < _MSG_OBJ_MAX; i++ )
  {
	LPC_CAN->IF1_CMDMSK = _CMDMASK_WR    | _CMDMASK_MASK | _CMDMASK_ARB | _CMDMASK_CTRL |
                          _CMDMASK_DATAA | _CMDMASK_DATAB;
    LPC_CAN->IF1_MCTRL  = 0;

    LPC_CAN->IF1_MSK1   = 0;
    LPC_CAN->IF1_MSK2   = 0;

    LPC_CAN->IF1_ARB1   = 0;
    LPC_CAN->IF1_ARB2   = 0;

	LPC_CAN->IF1_DA1    = 0;
	LPC_CAN->IF1_DA2    = 0;
	LPC_CAN->IF1_DB1    = 0;
	LPC_CAN->IF1_DB2    = 0;

	LPC_CAN->IF1_CMDREQ = i+1;                 /* Transfer message object data to message RAM */
	while (LPC_CAN->IF1_CMDREQ & _CMDREQ_BUSY);
  }

  LPC_CAN->STAT = 0;                            /* reset CAN status register */
  LPC_CAN->CNTL |= (//_CNTL_DAR |                 /* disable automatic retransmision */
                    _CNTL_IE  |                 /* enable CAN module interrupts */
                    //_CNTL_EIE |                 /* enable CAN error interrupts */
                    _CNTL_SIE );                /* enable CAN status change interrupts */

}


/*----------------------------------------------------------------------------
  leave initialisation mode.
 *----------------------------------------------------------------------------*/
void CAN_start (void)  {
  LPC_CAN->CNTL &= ~_CNTL_INIT;                     /* Enter normal operating mode */
}

/*----------------------------------------------------------------------------
  check if transmit mailbox is empty
 *----------------------------------------------------------------------------*/
void CAN_waitReady (void)  {
  CAN_TxRdy = 1;
}


int CAN_isboff(void)  {
  return (LPC_CAN->STAT & _STAT_BOFF);
}

/*----------------------------------------------------------------------------
  set a message to CAN peripheral which is send upon a remote transmission request.
 *----------------------------------------------------------------------------*/
void CAN_stMsg (CAN_msg *msg)  {
  int32_t i;
  uint32_t can_msgObj;
  uint32_t can_msgv;

  can_msgv = (LPC_CAN->MSGV2 << 16) | LPC_CAN->MSGV1;

  for (i = _MSG_OBJ_MAX - 1; i > -1; i--) {
    if ((can_msgv & (1 << i)) == 0)
	  break;
  }
  if (i > -1)
    can_msgObj = i + 1;			           /*   valid message object number */
  else
    can_msgObj = 0;			               /* invalid message object number */


  LPC_CAN->IF1_CMDMSK = _CMDMASK_WR   | _CMDMASK_MASK  | _CMDMASK_ARB | _CMDMASK_CTRL |
                                        _CMDMASK_DATAA | _CMDMASK_DATAB;
  LPC_CAN->IF1_MCTRL  = _MCTRL_UMASK  /*| _MCTRL_TXRQST */ | _MCTRL_RMTEN | _MCTRL_TXIE  | _MCTRL_EOB    | (msg->len & _MCTRL_DLC);

  if (msg->format == STANDARD_FORMAT) {   /* handle standard format */
    LPC_CAN->IF1_MSK1  = 0;
    LPC_CAN->IF1_MSK2  = ((msg->id & _STD_FORMAT) << 2) | _MSK2_MDIR;

    LPC_CAN->IF1_ARB1  = 0;
    LPC_CAN->IF1_ARB2  = ((msg->id & _STD_FORMAT) << 2)           | _ARB2_DIR | _ARB2_MSGVAL;
  }
  else {                                  /* handle extended format */
    LPC_CAN->IF1_MSK1 = ( msg->id        & _MSK1_MSK);
    LPC_CAN->IF1_MSK2 = ((msg->id >> 16) & _MSK2_MSK) | _MSK2_MXTD;

    LPC_CAN->IF1_ARB1 = ( msg->id        & _ARB1_ID);
    LPC_CAN->IF1_ARB2 = ((msg->id >> 16) & _ARB2_ID)  | _ARB2_XTD | _ARB2_DIR | _ARB2_MSGVAL;
  }

  LPC_CAN->IF1_DA1 = *(uint16_t *)&msg->data[0];
  LPC_CAN->IF1_DA2 = *(uint16_t *)&msg->data[2];
  LPC_CAN->IF1_DB1 = *(uint16_t *)&msg->data[4];
  LPC_CAN->IF1_DB2 = *(uint16_t *)&msg->data[6];

  LPC_CAN->IF1_CMDREQ = can_msgObj;               /* Transfer message object data to message RAM */
  while (LPC_CAN->IF1_CMDREQ & _CMDREQ_BUSY);

}

/*----------------------------------------------------------------------------
  write a message to CAN peripheral and transmit it.
 *----------------------------------------------------------------------------*/
void CAN_wrMsg (CAN_msg *msg)  {
   int32_t i;
  uint32_t can_msgObj;
  uint32_t can_msgv;

  can_msgv = (LPC_CAN->MSGV2 << 16) | LPC_CAN->MSGV1;

  for (i = _MSG_OBJ_MAX - 1; i > -1; i--) {
    if ((can_msgv & (1 << i)) == 0)
	  break;
  }
  if (i > -1)
    can_msgObj = i + 1;			           /*   valid message object number */
  else
    can_msgObj = 0;			               /* invalid message object number */


  LPC_CAN->IF1_CMDMSK = _CMDMASK_WR   | _CMDMASK_ARB   | _CMDMASK_CTRL |
                        _CMDMASK_TREQ | _CMDMASK_DATAA | _CMDMASK_DATAB;
  LPC_CAN->IF1_MCTRL  =                 _MCTRL_TXRQST  | _MCTRL_TXIE  | _MCTRL_EOB    | (msg->len & _MCTRL_DLC);

  if (msg->format == STANDARD_FORMAT) {   /* handle standard format */
    LPC_CAN->IF1_ARB1  = 0;
    LPC_CAN->IF1_ARB2  = ((msg->id & _STD_FORMAT) << 2)           | _ARB2_DIR | _ARB2_MSGVAL;
  }
  else {                                  /* handle extended format */
    LPC_CAN->IF1_ARB1 = ( msg->id        & _ARB1_ID);
    LPC_CAN->IF1_ARB2 = ((msg->id >> 16) & _ARB2_ID)  | _ARB2_XTD | _ARB2_DIR | _ARB2_MSGVAL;
  }

  LPC_CAN->IF1_DA1 = *(uint16_t *)&msg->data[0];
  LPC_CAN->IF1_DA2 = *(uint16_t *)&msg->data[2];
  LPC_CAN->IF1_DB1 = *(uint16_t *)&msg->data[4];
  LPC_CAN->IF1_DB2 = *(uint16_t *)&msg->data[6];

  if (msg->type == REMOTE_FRAME) {
    LPC_CAN->IF1_CMDMSK &= ~(_CMDMASK_DATAA | _CMDMASK_DATAB);
    LPC_CAN->IF1_ARB2   &= ~(_ARB2_DIR);                 /* trasnmit a Remote Frame Request */
  }

  LPC_CAN->IF1_CMDREQ = can_msgObj;               /* Transfer message object data to message RAM */
  while (LPC_CAN->IF1_CMDREQ & _CMDREQ_BUSY);

}

/*----------------------------------------------------------------------------
  read a message from CAN peripheral and release it.
 *----------------------------------------------------------------------------*/
void CAN_rdMsg (uint32_t can_msgObj, CAN_msg *msg)  {

  while (LPC_CAN->IF2_CMDREQ & _CMDREQ_BUSY);
  LPC_CAN->IF2_CMDMSK = _CMDMASK_RD     | _CMDMASK_MASK | _CMDMASK_ARB   | _CMDMASK_CTRL |
                        _CMDMASK_INTPND | _CMDMASK_TREQ | _CMDMASK_DATAA | _CMDMASK_DATAB;	
  LPC_CAN->IF2_CMDREQ = can_msgObj;        /* Transfer message object data from message RAM */
  while (LPC_CAN->IF2_CMDREQ & _CMDREQ_BUSY);

  /* check the message object whether it is receive or transmit*/
  if (LPC_CAN->IF2_MCTRL & _MCTRL_TXIE) {
    CAN_TxRdy = 1;                          /*  set transmit flag   */

    /* release message obect */
	LPC_CAN->IF2_CMDMSK = _CMDMASK_WR    | _CMDMASK_MASK | _CMDMASK_ARB | _CMDMASK_CTRL |
                          _CMDMASK_DATAA | _CMDMASK_DATAB;
    LPC_CAN->IF2_MCTRL  = 0x0000;

    LPC_CAN->IF2_MSK1   = 0x0000;
    LPC_CAN->IF2_MSK2   = 0x0000;

    LPC_CAN->IF2_ARB1   = 0x0000;
    LPC_CAN->IF2_ARB2   = 0x0000;

	LPC_CAN->IF2_DA1    = 0x0000;
	LPC_CAN->IF2_DA2    = 0x0000;
	LPC_CAN->IF2_DB1    = 0x0000;
	LPC_CAN->IF2_DB2    = 0x0000;

	LPC_CAN->IF2_CMDREQ = can_msgObj;	   	/* Transfer message object data to message RAM */
	while( LPC_CAN->IF2_CMDREQ & _CMDREQ_BUSY );
  }
  else {
    if (LPC_CAN->IF2_ARB2 & (1UL << 14)) {  /* check XTD bit (extended format) */
      msg->id = ((LPC_CAN->IF2_ARB2 & _ARB2_ID) << 16) | LPC_CAN->IF2_ARB1;
      msg->format = EXTENDED_FORMAT;
    }
    else {
      msg->id = (LPC_CAN->IF2_ARB2 >> 2) & _STD_FORMAT; /* bit 28-18 is 11-bit standard frame */
      msg->format = STANDARD_FORMAT;
    }

/* add code to check for remote / data frame */

    msg->len = LPC_CAN->IF2_MCTRL & _MCTRL_DLC;	         /* get message object data length */
    *(uint16_t *)&msg->data[0] = LPC_CAN->IF2_DA1;
    *(uint16_t *)&msg->data[2] = LPC_CAN->IF2_DA2;
    *(uint16_t *)&msg->data[4] = LPC_CAN->IF2_DB1;
    *(uint16_t *)&msg->data[6] = LPC_CAN->IF2_DB2;

    CAN_RxRdy = 1;                          /*  set receive flag   */
	CAN_LOOPBACK(msg);
  }
}


/*----------------------------------------------------------------------------
  setup acceptance filter.
 *----------------------------------------------------------------------------*/
void CAN_wrFilter (uint32_t id, uint8_t format)  {
  int32_t i;
  uint32_t can_msgObj;
  uint32_t can_msgv;

  can_msgv = (LPC_CAN->MSGV2 << 16) | LPC_CAN->MSGV1;

  for (i = 0; i < _MSG_OBJ_MAX; i++) {
    if ((can_msgv & (1 << i)) == 0)
	  break;
  }
  if (i < _MSG_OBJ_MAX)
    can_msgObj = i + 1;			           /*   valid message object number */
  else
    can_msgObj = 0;			               /* invalid message object number */

	LPC_CAN->IF1_CMDMSK = _CMDMASK_WR  | _CMDMASK_MASK | _CMDMASK_ARB | _CMDMASK_CTRL;
    LPC_CAN->IF1_MCTRL  = _MCTRL_UMASK | _MCTRL_RXIE   | _MCTRL_EOB   | _MCTRL_DLC;

    if (format == STANDARD_FORMAT) {       /* handle standard format */
      id = id & _STD_FORMAT;               

      LPC_CAN->IF1_MSK1  =  0;
      LPC_CAN->IF1_MSK2  = (id << 2);

      LPC_CAN->IF1_ARB1  =  0;
      LPC_CAN->IF1_ARB2  = (id << 2)            | _ARB2_MSGVAL;      /* id is stored left-aligned */
    }
    else {                                  /* handle extended format */
      id = id & _EXT_FORMAT;               

	  LPC_CAN->IF1_MSK1 = (id        & _MSK1_MSK);
	  LPC_CAN->IF1_MSK2 = (id >> 16)             | _MSK2_MXTD;

	  LPC_CAN->IF1_ARB1 = (id        & _ARB1_ID);
	  LPC_CAN->IF1_ARB2 = (id >> 16)             | _ARB2_XTD | _ARB2_MSGVAL;
    }

  LPC_CAN->IF1_CMDREQ = can_msgObj;         /* Transfer message object data to message RAM */
  while (LPC_CAN->IF1_CMDREQ & _CMDREQ_BUSY);
}

void CAN_noFilter (uint8_t format)  {
  int32_t i;
  uint32_t can_msgObj;
  uint32_t can_msgv;

  can_msgv = (LPC_CAN->MSGV2 << 16) | LPC_CAN->MSGV1;

  for (i = 0; i < _MSG_OBJ_MAX; i++) {
    if ((can_msgv & (1 << i)) == 0)
	  break;
  }
  if (i < _MSG_OBJ_MAX)
    can_msgObj = i + 1;			           /*   valid message object number */
  else
    can_msgObj = 0;			               /* invalid message object number */

	LPC_CAN->IF1_CMDMSK = _CMDMASK_WR  | _CMDMASK_MASK | _CMDMASK_ARB | _CMDMASK_CTRL;
    LPC_CAN->IF1_MCTRL  = _MCTRL_UMASK | _MCTRL_RXIE   | _MCTRL_EOB   | _MCTRL_DLC;

    if (format == STANDARD_FORMAT) {       /* handle standard format */
      //id = id & _STD_FORMAT;               

      LPC_CAN->IF1_MSK1  =  0;
      LPC_CAN->IF1_MSK2  =  0;

      LPC_CAN->IF1_ARB1  =  0;
      LPC_CAN->IF1_ARB2  =  0| _ARB2_MSGVAL;      /* id is stored left-aligned */
    }
    else {                                  /* handle extended format */
      //id = id & _EXT_FORMAT;               

	  LPC_CAN->IF1_MSK1 = 0;
	  LPC_CAN->IF1_MSK2 = 0 | _MSK2_MXTD;

	  LPC_CAN->IF1_ARB1 = 0 & _ARB1_ID;
	  LPC_CAN->IF1_ARB2 = 0 | _ARB2_XTD | _ARB2_MSGVAL;
    }

  LPC_CAN->IF1_CMDREQ = can_msgObj;         /* Transfer message object data to message RAM */
  while (LPC_CAN->IF1_CMDREQ & _CMDREQ_BUSY);
}


/*----------------------------------------------------------------------------
  CAN interrupt handler
 *----------------------------------------------------------------------------*/
void CAN_IRQHandler (void)  {
  volatile uint32_t can_int, can_stat;
  uint32_t can_msgObj;

  can_int = LPC_CAN->INT;                          /* read interrupt status */

  switch (can_int) {
    case 0x0000:                                   /* no interrupt */
      ;
      break;

    case 0x8000:                                   /* status interrupt */
      can_stat = LPC_CAN->STAT;

      if (can_stat & _STAT_TXOK) {                   /* TXOK       */
        LPC_CAN->STAT &= ~_STAT_TXOK;                /* reset TXOK */
	  }	  
      if (can_stat & _STAT_RXOK) {                   /* RXOK       */	 // got this never to work!
        LPC_CAN->STAT &= ~_STAT_RXOK;                /* reset RXOK */
	  }
	  if (can_stat & 0x0007) {
		DBG("STAT = %08x, INT = %08x!\r\n", can_stat, can_int);
		//LPC_CAN->STAT &= ~0x0007;
		LPC_CAN->STAT = 0;
	  }
	  break;

    default:                                      /* message object interrupt */
      can_msgObj = can_int & 0x7FFF;
      if ((can_msgObj >=  1) &&                 
          (can_msgObj <= 32)   ) {                /*   valid msgObj number */
	    CAN_rdMsg (can_msgObj, &CAN_RxMsg);      /*  read the message  */
      }
      else {                                      /* invalid msgObj number */
        ;
      } 
      break;
  }

}
