#include <LPC11xx.h>
#include <string.h>
#include "event.h"
#include "buttons.h"

#define MAX_BUTT 2

static int state;
static int last_state[MAX_BUTT];

int button_state(int n) {
	switch (n) {
	case BUTT_SW1:
		return (LPC_GPIO0->DATA & (1UL<<1))?0:1;
	case BUTT_SW2:
		return (LPC_GPIO0->DATA & (1UL<<3))?0:1;
	default:
		return 0;
	}
}

static void button_proc(void) {
	int n;
	for (n=0; n<MAX_BUTT; n++) {
		if (button_state(1<<n)) {
			if (last_state[n] < -4) // защита от повторного нажания (не быстрее чем через 50мс)
				last_state[n] = 0;
			last_state[n]++;
			if (last_state[n] == 3) { // только удержание в течении 30мс  более.
				state |= (1<<n);
			} else 
			if (last_state[n] > 100) { // больше секунды начинаются повторы
				if (last_state[n]%15 == 0) // замедление, повтор кнопки не чаще 150мс
					state |= (1<<n);
			}
		} else {
			if (last_state[n] > 0)
				last_state[n] = 0;
			last_state[n]--;
		}
	}
	event_set(0, button_proc, 10);
}

void button_init(void) {
	state = 0;
	memset(last_state, 0x00, sizeof(last_state));
	event_set(0, button_proc, 10);
}

int button_read(void) {
	int ret;
	ret = state; 
	state = 0; // С минимальной вероятностью, существует возможность пропуска нажатия.
	return ret;
}
