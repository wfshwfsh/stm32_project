#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "usart.h"


#define IBUS_FRAME_SIZE 32

enum {
	eCH1 = 0, 
	eCH2, 
	eCH3_THROTTLE, 
	eCH4,
};


extern uint8_t ibus_buffer[IBUS_FRAME_SIZE];
extern uint16_t ibus_channels[10];
extern uint16_t cur_channels[10];
extern int idx, ibus_idx;

void IBUS_Init();
void IBUS_update_ch();
void IBUS_show_ch();

