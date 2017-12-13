#ifndef __BUTTONS_H__
#define __BUTTONS_H__

#define BUTT_SW1  (1<<0)
#define BUTT_SW2  (1<<1)

void button_init(void);
int button_read(void);
int button_state(int n); // низкоуровневая функция

#endif // __BUTTONS_H__
