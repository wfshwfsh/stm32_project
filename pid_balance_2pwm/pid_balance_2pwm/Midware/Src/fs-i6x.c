#include "fs-i6x.h"


uint8_t ibus_buffer[IBUS_FRAME_SIZE]={};
uint16_t ibus_channels[10]={};
uint16_t cur_channels[10]={};
int idx=0, ibus_idx =0;

void IBUS_Init()
{
	HAL_UARTEx_ReceiveToIdle_IT(&huart6, ibus_buffer, IBUS_FRAME_SIZE);
}

void IBUS_update_ch()
{
	for(int i=0;i<10;i++){
		cur_channels[i] = ibus_channels[i];
	}
}

void IBUS_show_ch()
{ 
	printf("CH1:%d  CH2:%d  CH3:%d  CH4:%d  CH5:%d  CH6:%d\r\n",
					cur_channels[0], cur_channels[1], cur_channels[2],
					cur_channels[3], cur_channels[4], cur_channels[5]);
}

