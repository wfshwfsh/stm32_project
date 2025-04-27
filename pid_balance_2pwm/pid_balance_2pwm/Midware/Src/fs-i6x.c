#include "fs-i6x.h"


uint8_t ibus_buffer[IBUS_FRAME_SIZE]={};
uint16_t ibus_channels[10]={};
uint16_t cur_channels[10]={};


void IBUS_show_ch()
{ 
	printf("CH1:%d  CH2:%d  CH3:%d  CH4:%d  CH5:%d  CH6:%d\r\n",
					cur_channels[0], cur_channels[1], cur_channels[2],
					cur_channels[3], cur_channels[4], cur_channels[5]);
}

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

void IBUS_Parse() {
    if (ibus_buffer[0] != 0x20 || ibus_buffer[1] != 0x40) {
        return;
    }
		
    uint8_t checksum = 0;
    for (int i = 0; i < 31; i++) {
        checksum += ibus_buffer[i];
			  //printf("%d: %02x ", i, ibus_buffer[i]);
    }
		//printf("\n");
		
		//printf("CRC = %d, b[31] = %d \n", checksum, ibus_buffer[31]);
    //if (checksum == ibus_buffer[31]) {
        for (int i = 0; i < 10; i++) {  // ??? 10 ???
            ibus_channels[i] = ibus_buffer[4 + (i * 2)] | (ibus_buffer[3 + (i * 2)] << 8);
        }

        //printf("CH1:%d  CH2:%d  CH3:%d  CH4:%d  CH5:%d  CH6:%d\r\n",
        //    ibus_channels[0], ibus_channels[1], ibus_channels[2],
        //    ibus_channels[3], ibus_channels[4], ibus_channels[5]);
    //}
}

