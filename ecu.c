/*
	ECU11 firmware

	Простой вариант эмулятора ЭБУ.
	Отвечает на некоторые запросы параметров.

	Events:
	0 - buttons
	1 - engine 608
	2 - engine 215
	3 - engine 308
	4 - transmission 218
	5 - second update
	6 - wheel sensor 236
	other:
	7 - send id 101
	8 - send id 212
	9 - send id 312
	10 - send id 325
	11 - send id 415
	12 - send id 424
	13 - send id 445
	14 - send id 447

	igorkov / fsp@igorkov.org / 2016
	
	Site: igorkov.org/bcomp11
 */
#include <stdio.h>
#include <lpc11xx.h>
#include <math.h>
#include <string.h>

#include "leds.h"
#include "event.h"
#include "can.h"
#include "uart.h"
#include "dbg.h"
#include "buttons.h"

void ProtectDelay(void) {
	int n;
	for (n = 0; n < 100000; n++) { __NOP(); } 
}

void BeepOff(void) {
	LPC_SYSCON->SYSAHBCLKCTRL |= (1UL<<6)|(1UL<<16);
	// pio init:
	LPC_IOCON->R_PIO1_1 &= ~0x07; /*  As  */
	LPC_IOCON->R_PIO1_1 |=  0x01; /* GPIO */
  	// set 0:
	LPC_GPIO1->DIR  |=  (1UL<<1);
	LPC_GPIO1->DATA &= ~(1UL<<1);
}

volatile uint8_t new_data = 0;
volatile uint8_t more_data = 0;
uint16_t ans_addr = 0xFFFF;
uint16_t ans_pid = 0xFFFF;
uint8_t ans_data[128];
int ans_size = 0;
CAN_msg frame;

void can_dispatcher(CAN_msg *p) {
#if 0
	DBG("CAN RECV: %03xh [%02x %02x %02x %02x %02x %02x %02x %02x]\r\n",
		p->id, 
		p->data[0], p->data[1], p->data[2], p->data[3], 
		p->data[4], p->data[5], p->data[6], p->data[7]);
#endif	
	if (p->len != 8) {
		DBG("ERROR: incorrect DLC in CAN frame, DLC = %d!\r\n", p->len);
		return;
	}
	if (p->id != 0x7DF &&
		p->id != 0x7E0 &&
		p->id != 0x7E1) {

		int i;

#if !defined( RELEASE )
		led_red(1);
		for (i=0;i<10000;i++) __NOP();
		led_red(0);
#endif

		// ignore this data...
		DBG("WARNING: data to address 0x%03x! Ignoring...\r\n", p->id);
		return;
	}
	if (p->data[0] == 0x30) {
		//frame = *p;
		//new_data = 1;
		//return;
		if (ans_size) {
			more_data = 1;
		}
	} else 
	if (p->data[0] == 0x02) {
		//DBG("can_dispatcher(): Request receive!\r\n");
		ans_size = 0;
		more_data = 0;
		frame = *p;
		new_data = 1;
		return;
	} else {
		DBG("ERROR: incorrect lenght in answer!\r\n");
		return;
	}
}

struct {
	float rpm;
	float speed;
	float t_at;
	float t_engine;
	float t_air;
	float m_air;
	float p_air;
	float p_intake;
	float p_rail;
	uint8_t trottle;
	uint8_t res1[3];
	int time;
	float volt;
} par;

void update(int n) {
	if (n == 0) {
#if 0
		par.rpm      = 700.0f + (rand()%1000);
		par.volt     = 13.5f + (rand()%20)-10.0f;
		par.t_at     = 80.0f + (rand()%30);
		par.t_engine = 70.0f + (rand()%40);
		par.t_air    = 30.0f + (rand()%40);
		par.m_air    = 3.0f + (float)(rand()%100)/100;
		par.p_air    = 100000.0f + (rand()%10000) - 5000.0f;
		par.p_intake = 120000.0f + (rand()%10000) - 5000.0f;
		par.p_rail   = 100.0e6f + (rand()%1000)*1e6f;
		par.trottle  = 25 + (rand()%100); // %
		par.speed    = 50+(rand()%20);
#else
		par.rpm      = 1320.0f;
		par.volt     = 13.5f;
		par.t_at     = 80.0f;
		par.t_engine = 70.0f;
		par.t_air    = 30.0f;
		par.m_air    = 3.33f;
		par.p_air    = 102000.0f;
		par.p_intake = 123000.0f;
		par.p_rail   = 170e6;
		par.trottle  = 25; // %
		par.speed    = 60.0f;
#endif
	} else if (n == 1) {
		par.rpm      = 2570.0f;
		par.volt     = 10.5f;
		par.t_at     = 80.0f;
		par.t_engine = 70.0f;
		par.t_air    = 30.0f;
		par.m_air    = 3.33f;
		par.p_air    = 102000.0f;
		par.p_intake = 123000.0f;
		par.p_rail   = 170e6;
		par.trottle  = 27; // %
		par.speed    = 60.0f;
	} else if (n == 2) {
		par.rpm      = 0.0f;
		par.volt     = 13.5f;
		par.t_at     = 115.0f;
		par.t_engine = 70.0f;
		par.t_air    = 30.0f;
		par.m_air    = 3.33f;
		par.p_air    = 102000.0f;
		par.p_intake = 123000.0f;
		par.p_rail   = 170e6;
		par.trottle  = 28; // %
		par.speed    = 60.0f;
	} else if (n == 3) {
		par.rpm      = 1000.0f;
		par.volt     = 13.5f;
		par.t_at     = 95.0f;
		par.t_engine = 105.0f;
		par.t_air    = 30.0f;
		par.m_air    = 3.33f;
		par.p_air    = 102000.0f;
		par.p_intake = 123000.0f;
		par.p_rail   = 170e6;
		par.trottle  = 29; // %
		par.speed    = 60.0f;
	}
}

volatile int async_id = 0;

//
// 0x100 - 4b - (start) - (-)
// 0x101 - 1b - 100ms   - (*)
// 0x110 - 1b - (start) - (-)
// 0x111 - 3b - (start) - (-)
// 0x212 - 8b - 20ms    - (*)
// 0x215 - 8b - 20ms    - (+)
// 0x218 - 8b - 20ms    - (+)
// 0x308 - 8b - 20ms    - (+)
// 0x312 - 8b - 20ms    - (*)
// 0x325 - 2b - 20ms    - (*)
// 0x415 - 7b - 100ms   - (*)
// 0x424 - 8b - 40ms    - (*)
// 0x445 - 7b - 40ms    - (*)
// 0x447 - 3b - 40ms    - (*)
// 0x608 - 8b - 100ms   - (+)
//

void engine_send608(void) {
	async_id = 0x608;
	// 100ms
	event_set(1, engine_send608, 100);
}

void engine_send215(void) {
	async_id = 0x215;
	// 20ms
	event_set(2, engine_send215, 20);
}

void engine_send308(void) {
	async_id = 0x308;
	// 20ms
	event_set(3, engine_send308, 20);
}

void engine_send218(void) {
	async_id = 0x218;
	// 20ms
	event_set(4, engine_send218, 20);
}

void engine_send236(void) {
	async_id = 0x236;
	// 10ms
	event_set(6, engine_send236, 10);
}

void engine_send101(void) {
	async_id = 0x101;
	// 100ms
	event_set(7, engine_send101, 100);
}

void engine_send212(void) {
	async_id = 0x212;
	// 20ms
	event_set(8, engine_send212, 20);
}

void engine_send312(void) {
	async_id = 0x312;
	// 20ms
	event_set(9, engine_send312, 20);
}

void engine_send325(void) {
	async_id = 0x325;
	// 20ms
	event_set(10, engine_send325, 20);
}

void engine_send415(void) {
	async_id = 0x415;
	// 100ms
	event_set(11, engine_send415, 100);
}

void engine_send424(void) {
	async_id = 0x424;
	// 40ms
	event_set(12, engine_send424, 40);
}

void engine_send445(void) {
	async_id = 0x445;
	// 40ms
	event_set(13, engine_send445, 40);
}

void engine_send447(void) {
	async_id = 0x447;
	// 40ms
	event_set(14, engine_send447, 50);
}

void engine_sendA(void) {
	//async_id = 0x410; // зажигает подсветку, т=-2
	async_id = 0xFFF; // перебор
	// 40ms						
	event_set(16, engine_sendA, 40);
}

int g_id = 0x400;
int g_len = 1;

void async_send(void) {
	uint8_t data[8];
	CAN_msg frame_a;

	frame_a.id = async_id;
	async_id = 0;

	switch (frame_a.id) {
	case 0x608:
		data[0] = (int)par.t_engine + 40; // t двиг
		data[1] = 0x00;
		data[2] = 0xFF;
		data[3] = 0xBF;
		data[4] = 0xFF;
		data[5] = 0x07;
		data[6] = 0xE3;
		data[7] = 0x00;
		frame_a.len = 8;
		break;
	case 0x308:
		data[0] = 0x00;
		data[1] = 0x05; // rpm
		data[2] = 0x14; // rpm
		data[3] = 0x40;
		data[4] = 0x00;
		data[5] = 0x00;
		data[6] = 0x00;
		data[7] = 0x03;
		frame_a.len = 8;
		break;
	case 0x215:
		data[0] = 0x1E;	// speed
		data[1] = 0x00; // speed
		data[2] = 0x00;
		data[3] = 0x8C;
		data[4] = 0x00;
		data[5] = 0x8C;
		data[6] = 0x00;
		data[7] = 0x00;
		frame_a.len = 8;
		break;
	case 0x218:
		data[0] = 0x09;
		data[1] = 0x73;
		data[2] = 0x33;	// drive
		data[3] = 0x40;
		data[4] = 0x00;
		data[5] = 0x80;
		data[6] = 0x09;
		data[7] = 0x00;
		frame_a.len = 8;
		break;
	case 0x236:
		data[0] = 0x3F;
		data[1] = 0xFF;
		data[2] = 0x3F;
		data[3] = 0xFF;
		data[4] = 0x9F;
		data[5] = 0x00;
		data[6] = 0x00;
		data[7] = 0x15;
		frame_a.len = 8;
		break;
	// other
	case 0x101:
		data[0] = 0x04;
		frame_a.len = 1;
		break;
	case 0x212:
		data[0] = 0x00;
		data[1] = 0xFF;
		data[2] = 0x00;
		data[3] = 0x00;
		data[4] = 0x26;
		data[5] = 0x68;
		data[6] = 0x29;
		data[7] = 0x2B;
		frame_a.len = 8;
		break;
	case 0x312:
		data[0] = 0x09;
		data[1] = 0x1c;
		data[2] = 0x09;
		data[3] = 0x1c;
		data[4] = 0x1f;
		data[5] = 0xff;
		data[6] = 0x1f;
		data[7] = 0xff;
		frame_a.len = 8;
		break;
	case 0x325:
		data[0] = 0x01;
		data[1] = 0x02;
		frame_a.len = 2;
		break;
	case 0x415:
		data[0] = 0x00;
		data[1] = 0xFF;
		data[2] = 0x00;
		data[3] = 0x00;
		data[4] = 0xFF;
		data[5] = 0x02;
		data[6] = 0xFF;
		frame_a.len = 7;
		break;
	case 0x424:
		data[0] = 0x03;
		data[1] = 0x00;
		data[2] = 0x0f;
		data[3] = 0x00;
		data[4] = 0x29;
		data[5] = 0x76;
		data[6] = 0x00;
		data[7] = 0xf0;
		frame_a.len = 8;
		break;
	case 0x445:
		data[0] = 0x40;
		data[1] = 0x00;
		data[2] = 0x04;
		data[3] = 0x84;
		data[4] = 0x10;
		data[5] = 0x2c;
		data[6] = 0x00;
		frame_a.len = 7;
		break;
	case 0x447:
		data[0] = 0x80;
		data[1] = 0x00;
		data[2] = 0x05;
		frame_a.len = 3;
		break;
	case 0x410:
		// Зажигает подсветку...
		data[0] = 0x00;
		frame_a.len = 1;
		break;
	case 0x584:
		// RV meter
		data[0] = 0x80; 
		data[1] = 0x00; 
		data[2] = 0x00; 
		data[3] = 0x05; 
		data[4] = 0xa0; 
		data[5] = 0x39; 
		data[6] = 0x00; 
		frame_a.len = 7;
		break;
	case 0xFFF:
rep_case:
		if (g_id == 0x800) {
			g_id = 0x001;
		}
		switch (g_id) {
		case 0x308:
		case 0x215:
		case 0x218:
		case 0x236:
		case 0x101:
		case 0x212:
		case 0x312:
		case 0x325:
		case 0x415:
		case 0x424:
		case 0x445:
		case 0x447:

		case 0x410: // исключаем
			g_id++;
			goto rep_case;
		default:
			frame_a.id  = g_id;
			frame_a.len = 8;
			memset(frame_a.data, 0xAA, 8);
			g_id++;
			break;
		}
	default:
		DBG("WARN(%04x)\r\n", frame_a.id);
		return;
	}

	frame_a.format = STANDARD_FORMAT;
	frame_a.type = DATA_FRAME;
	memcpy(frame_a.data, data, 8);
	CAN_wrMsg(&frame_a);
}

void second_update(void) {
	par.time++;
	event_set(5, second_update, 1000);
}


/* MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN */

int main (void) {
	int n = 0;

#if defined( _DBGOUT )
	UART_Init(115200);
	DBG("UART_Init() ok!\r\n");
#endif
	event_init();
	leds_init();
	button_init();

	CAN_setup();                                   /* setup CAN Controller   */
	//CAN_wrFilter(0x21, STANDARD_FORMAT);           /* Enable reception of messages */
	CAN_noFilter(STANDARD_FORMAT);                 /* All receive */
	CAN_start();                                   /* start CAN Controller    */
	CAN_waitReady();                               /* wait til tx mbx is empty */

	DBG("Pripheral initialize ok!\r\n");

	event_set(5, second_update, 1000); delay_ms(7);
	event_set(1, engine_send608, 10); delay_ms(7);
	event_set(2, engine_send215, 10); delay_ms(7);
	event_set(3, engine_send308, 10); delay_ms(7);
	event_set(4, engine_send218, 10); delay_ms(7);
	event_set(6, engine_send236, 10); delay_ms(7);
	// other
	event_set(7, engine_send101, 10); delay_ms(7);
	event_set(8, engine_send212, 10); delay_ms(7);
	event_set(9, engine_send312, 10); delay_ms(7);
	event_set(10, engine_send325, 10); delay_ms(7);
	event_set(11, engine_send415, 10); delay_ms(7);
	event_set(12, engine_send424, 10); delay_ms(7);
	event_set(13, engine_send445, 10); delay_ms(7);
	event_set(14, engine_send447, 10); delay_ms(7);
	
	//event_set(15, engine_send584, 10); delay_ms(7);
	// 40ms: 80 00 00 05 a0 39 00 from RV-meter

	//event_set(16, engine_sendA, 10); delay_ms(7);
	DBG("Events setup!\r\n");

#if 0
	// FOR TEMPERATURE TEST:
	while (1) {
		for (n=1; n<9; n++) {
			for (pid=0x400; pid<0x500;pid++) {
				CAN_msg frame_a;
				frame_a.id = pid;
				frame_a.len = n;
				frame_a.format = STANDARD_FORMAT;
				frame_a.type = DATA_FRAME;
				memset(frame_a.data, 0x20, n);
				// 0x10 - -8*C
				// 0x20 - 10*C
				// 0x21 - 11*C
				// 0x22 - ??*C
				// 0x30	- -18*c
				// 
				CAN_wrMsg(&frame_a);
				
				delay_ms(5);		
			}
		}
		led_red(0);
		delay_ms(200);		
		led_red(1);
		delay_ms(100);		
	}

#endif

	// Задаем параметры ЭБУ:
	update(0);
	par.time = 0;

	while (1) {
		uint16_t pid;
		uint8_t ecu_engine = 0;
		uint8_t ecu_at = 0;
		uint8_t data[8];

		while (!new_data) {
			int state;
			__WFI();
			
			// Состояние кнопок:
			state = button_read();
			if (state & BUTT_SW1) {
				DBG("BUTT_SW1 (n = %d)\r\n", n);
				if (n < 4) n++;
				update(n);
			}
			if (state & BUTT_SW2) {
				DBG("BUTT_SW2 (n = %d)\r\n", n);
				if (n > 0) n--;
				update(n);
			}

			// Обнволение счетчика секунд:
			// Перенесено в функцию second_update().

			// Отправка асинхронных данных, имитирующих шину NMPS.
			// Асинхронный обработчик:
			async_send();
			
			if (more_data) {
#if 0				
				at_addr(ans_addr);
				at_send(ans_data, 8);
				DBG("CAN SEND (M): %03xh [%02x %02x %02x %02x %02x %02x %02x %02x]\r\n",
					ans_addr, 
					ans_data[0], ans_data[1], ans_data[2], ans_data[3], 
					ans_data[4], ans_data[5], ans_data[6], ans_data[7]);
#else
				frame.id = ans_addr;
				frame.len = 8;
				frame.format = STANDARD_FORMAT;
				frame.type = DATA_FRAME;
				memcpy(frame.data, ans_data, 8);
				CAN_wrMsg(&frame);
#endif				
				
				ans_size -= 8;
				if (ans_size <= 0) {
					more_data = 0;
				} else {
					memmove(ans_data, &ans_data[8], ans_size);
				}
			}
		}
		new_data = 0;

		led_green(1);
		delay_ms(2);
		led_green(0);
		
		// 0x7DF - широковещательный адрес
		// 0x7E0 - адрес Engine ECU
		// 0x7E1 - адрес Automatic Transmission ECU

		if (frame.id == 0x7DF) {
			ecu_engine = 1;
			ecu_at = 1;
		}
		if (frame.id == 0x7E0) {
			ecu_engine = 1;
			ecu_at = 0;
		}
		if (frame.id == 0x7E1) {
			ecu_engine = 0;
			ecu_at = 1;
		}
		
		pid = (frame.data[1]<<8)|frame.data[2];
		
		if (ecu_engine) {
			DBG("Engine ECU process (pid = 0x%04x)...\r\n", pid);
			memset(data, 0, 8);
			switch (pid) {
			case 0x0101: // Статус
				data[0] = 0x05;
				data[1] = 0x41;
				data[2] = 0x01;
				data[3] = 0x81; // mil and 1 error
				data[4] = 0x04; // diesel
				data[5] = 0; // nop
				break;
			case 0x0102: // Ошибка
				data[0] = 0x04;
				data[1] = 0x41;
				data[2] = 0x02;
				data[3] = 0x12; // P1234
				data[4] = 0x34;
				break;
			case 0x0105: // Температура двигателя.
				data[0] = 0x03;
				data[1] = 0x41;
				data[2] = 0x05;
				data[3] = (uint8_t)(par.t_engine + 40.0f);
				break;
			case 0x010B: // Давление впускного коллектора.
				data[0] = 0x03;
				data[1] = 0x41;
				data[2] = 0x0B;
				data[3] = (uint8_t)(par.p_intake/1000);
				break;
			case 0x010C: // Частота вращения коленчатого вала.
				data[0] = 0x04;
				data[1] = 0x41;
				data[2] = 0x0C;
				data[3] = ((uint16_t)par.rpm*4)>>8;
				data[4] = ((uint16_t)par.rpm*4)&0xFF;
				break;
			case 0x010D: // Скорость автомобиля.
				data[0] = 0x03;
				data[1] = 0x41;
				data[2] = 0x0D;
				#if 0
					data[3] = 0x12; // Скорость
				#else
					data[3] = (uint8_t)par.speed; // Скорость (1)
				#endif
				break;
			case 0x010F: // Температура воздуха на впуске.
				data[0] = 0x03;
				data[1] = 0x41;
				data[2] = 0x0F;
				data[3] = (uint8_t)(par.t_air + 40.0f);
				break;
			case 0x0110: // Поток воздуха:
				data[0] = 0x03;
				data[1] = 0x41;
				data[2] = 0x10;
				data[3] = (uint8_t)((uint16_t)(par.m_air * 100)>>8);
				data[4] = (uint8_t)((int)(par.m_air * 100));
				break;
			case 0x0111: // Положение дроссельной заслонки:
				data[0] = 0x03;
				data[1] = 0x41;
				data[2] = 0x11;
				data[3] = (uint8_t)(par.trottle);
				break;
			case 0x011F: // Время работы двигателя
				data[0] = 0x04;
				data[1] = 0x41;
				data[2] = 0x1F;
				data[3] = ((uint16_t)par.time)>>8;
				data[4] = ((uint16_t)par.time)&0xFF;
				break;
			case 0x0123: // Давление топлива в рейке
				data[0] = 0x04;
				data[1] = 0x41;
				data[2] = 0x23;
				data[3] = ((uint16_t)(par.p_rail/10000))>>8;
				data[4] = ((uint16_t)(par.p_rail/10000))&0xFF;
				break;
			case 0x0133: // Наружное давление
				data[0] = 0x03;
				data[1] = 0x41;
				data[2] = 0x33;
				#if 0
					data[3] = 101; // 101kPa
				#else
					data[3] = (uint8_t)(par.p_air/1000);
				#endif
				break;
			case 0x0142: // Напряжение из ЭБУ
				data[0] = 0x04;
				data[1] = 0x41;
				data[2] = 0x42;
				#if 0
					data[3] = 0x34; // 
					data[4] = 0xE6; // 13.542V
				#else
					data[3] = ((uint16_t)(par.volt*1000))>>8;
					data[4] = ((uint16_t)(par.volt*1000))&0xFF;
				#endif
				break;
			case 0x0902: // Получение VIN.
				data[0] = 0x10; // Стартовый байт/класс команды ???
				data[1] = 0x14; // 20 (полный размер данных)
				data[2] = 0x49; // 0x40 + 0x09
				data[3] = 0x02; //        0x02
				data[4] = 0x01; // A
				data[5] = 0x4d; // B
				data[6] = 0x4d; // C
				data[7] = 0x43; // D
				// Дополнительные данные:
				ans_data[0]  = 0x21; //
				ans_data[1]  = 0x47; // E
				ans_data[2]  = 0x52; // F
				ans_data[3]  = 0x4b; // G
				ans_data[4]  = 0x48; // H
				ans_data[5]  = 0x38; // I
				ans_data[6]  = 0x30; // J
				ans_data[7]  = 0x39; // K
				// Дополнительные данные 2:
				ans_data[8]  = 0x22; // 
				ans_data[9]  = 0x46; // L
				ans_data[10] = 0x5a; // M
				ans_data[11] = 0x30; // N
				ans_data[12] = 0x30; // O
				ans_data[13] = 0x30; // P
				ans_data[14] = 0x30; // Q
				ans_data[15] = 0x31; // R
				
				ans_size = 16;
				ans_addr = 0x7E8;
				ans_pid  = pid;
				break;
			case 0x2103:
				n = 123456; //km
				data[0] = 0x06; // Стартовый байт/класс команды
				data[1] = 0x61;
				data[2] = 0x03;
				data[3] = (n>>24)&0xFF;
				data[4] = (n>>16)&0xFF;
				data[5] = (n>> 8)&0xFF;
				data[6] = (n>> 0)&0xFF;
				break;
			default:
				DBG("ERROR: Unknown Engine ECU request!\r\n");
				continue;  // todo fix
			}
#if 0
			at_addr(0x7E8);
			at_send(data, 8);
			DBG("CAN SEND (E): %03xh [%02x %02x %02x %02x %02x %02x %02x %02x]\r\n",
				0x7E8, 
				data[0], data[1], data[2], data[3], 
				data[4], data[5], data[6], data[7]);
#else
			frame.id = 0x7E8;
			frame.len = 8;
			frame.format = STANDARD_FORMAT;
			frame.type = DATA_FRAME;
			memcpy(frame.data, data, 8);
			CAN_wrMsg(&frame);
#endif
		}

//
// Alphabet catalog:
//
//  1 - [A] - data[3]        27 - [AA] - data[29]
//  2 - [B] - data[4]        28 - [AB] - data[30]
//  3 - [C] - data[5]        29 - [AC] - data[31]
//  4 - [D] - data[6]        30 - [AD] - data[32]
//  5 - [E] - data[7]        31 - [AE] - data[33]
//  6 - [F] - data[8]        32 - [AF] - data[34]
//  7 - [G] - data[9]        33 - [AG] - data[35]
//  8 - [H] - data[10]       34 - [AH] - data[36]
//  9 - [I] - data[11]       35 - [AI] - data[37]
// 10 - [J] - data[12]       36 - [AJ] - data[38]
// 11 - [K] - data[13]       37 - [AK] - data[39]
// 12 - [L] - data[14]       38 - [AL] - data[40]
// 13 - [M] - data[15]       39 - [AM] - data[41]
// 14 - [N] - data[16]       40 - [AN] - data[42]
// 15 - [O] - data[17]       41 - [AO] - data[43]
// 16 - [P] - data[18]       42 - [AP] - data[44]
// 17 - [Q] - data[19]       43 - [AQ] - data[45]
// 18 - [R] - data[20]       44 - [AR] - data[46]
// 19 - [S] - data[21]       45 - [AS] - data[47]
// 20 - [T] - data[22]       46 - [AT] - data[48]
// 21 - [U] - data[23]       47 - [AU] - data[49]
// 22 - [V] - data[24]       48 - [AV] - data[50]
// 23 - [W] - data[25]       49 - [AW] - data[51]
// 24 - [X] - data[26]       50 - [AX] - data[52]
// 25 - [Y] - data[27]       51 - [AY] - data[53]
// 26 - [Z] - data[28]       52 - [AZ] - data[54]

		if (ecu_at) {
			DBG("AT ECU process (pid = 0x%04x)...\r\n", pid);
			memset(data, 0, 8);
			switch (pid) {
			case 0x2102: // Блок данных от АКПП.
				data[0] = 0x10; // Стартовый байт/класс команды
				data[1] = 0x0c; // 12 (полный размер)
				data[2] = 0x61; // 0x40 + 0x21
				data[3] = 0x02; //        0x02
				data[4] = 0x00; // A
				data[5] = 0x06; // B
				data[6] = 0xb0; // C
				data[7] = 0x00; // D
				
				ans_data[0] = 0x21; // 
				ans_data[1] = 0x00; // E
				#if 0
					ans_data[2] = 0x2c; // F
				#else
					ans_data[2] = (uint8_t)(par.t_at+40); // F
				#endif
				ans_data[3] = 0x70; // G
				ans_data[4] = 0x00; // H
				ans_data[5] = 0x60; // I
				ans_data[6] = 0x01; // J
				ans_data[7] = 0x00; // K
				
				ans_size = 8;
				ans_addr = 0x7E9;
				ans_pid  = pid;
				break;

			case 0x2101: // Блок данных от АКПП (NISSAN)
				data[0] = 0x10; // Стартовый байт/класс команды
				data[1] = 0x22; // 6 + 7 * 4 = 34 (полный размер)
				data[2] = 0x61; // 0x40 + 0x21
				data[3] = 0x01; //        0x01
				data[4] = 0x01; // A data[3]
				data[5] = 0x02; // B 4
				data[6] = 0x03; // C 5
				data[7] = 0x04; // D 6
				
				ans_data[0] = 0x21; // PACK1
				ans_data[1] = 0x06; // E data[7]
				ans_data[2] = 0x07; // F
				ans_data[3] = 0x08; // G
				ans_data[4] = 0x09; // H
				ans_data[5] = 0x0A; // I
				ans_data[6] = 0x0B; // J
				ans_data[7] = 0x0C; // K
				
				ans_data[8] = 0x22; // PACK2
				ans_data[9] = 0x0D; // L data[14]
				ans_data[10] = 0x0E; // M
				ans_data[11] = 0x0F; // N
				ans_data[12] = 0x10; // O
				ans_data[13] = 0x11; // P
				ans_data[14] = 0x12; // Q
				ans_data[15] = 0x13; // R
				
				ans_data[16] = 0x23; // PACK3
				ans_data[17] = 0x14; // S data[21]
				ans_data[18] = 0x15; // T
				ans_data[19] = 0x16; // U
				ans_data[20] = 0x17; // V
				ans_data[21] = 0x18; // W
				ans_data[22] = 0x19; // X
				ans_data[23] = 0x1A; // Y
				
				ans_data[24] = 0x24; // PACK4
				ans_data[25] = 0x1B; // Z data[28]
				ans_data[26] = 0x1C; // AA
				ans_data[27] = 0x1D; // AB
				ans_data[28] = 0x1F; // AC
				//ans_data[29] = 0x20; // AD data[32]
				ans_data[29] = 130; // calculation 63°C 
				ans_data[30] = 0x21; // AE data[33]
				ans_data[31] = 0x22; // AF data[34]
				
				ans_size = 32;
				ans_addr = 0x7E9;
				ans_pid  = pid;
				break;
			default:
				DBG("ERROR: Unknown AT ECU request!\r\n");
				continue;
			}
#if 0
			at_addr(0x7E9);
			at_send(data, 8);
			DBG("CAN SEND (A): %03xh [%02x %02x %02x %02x %02x %02x %02x %02x]\r\n",
				0x7E9, 
				data[0], data[1], data[2], data[3], 
				data[4], data[5], data[6], data[7]);
#else
			frame.id = 0x7E9;
			frame.len = 8;
			frame.format = STANDARD_FORMAT;
			frame.type = DATA_FRAME;
			memcpy(frame.data, data, 8);
			CAN_wrMsg(&frame);
#endif
		}
	}
}
