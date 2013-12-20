////////////THis part is for tile switching and sounding beeps.



#include "mbed.h"
#include "m3pi.h"
#include "MRF24J40.h"
#include <string.h>

DigitalOut tile0(p15);
DigitalOut tile1(p16);
DigitalOut led1(p25);
DigitalOut led2(p26);
DigitalOut led3(p27);
DigitalOut led4(p29);
DigitalOut led5(p30);

AnalogOut buzzerC(p18);
PwmOut buzzerB(p22);

MRF24J40 mrf(p11, p12, p13, p14, p21);

char txBuffer[128];
char rxBuffer[128];
int rxLen;
int active;
int status;

void ledMusic(void);
void tileSwitch(void);
int rf_receive(char *data, uint8_t maxLength);
void rf_send(char *data, uint8_t len);

void ledMusic(void)
{
	static int j;
	//sound the "bee[" once the base has hitten by the rubber band. 
	buzzerC = 1.0f;
	buzzerB = 1.0f;
	while(1)
	{
		led1 = 1;
		led2 = 0;
		led3 = 0;
		led4 = 0;
		led5 = 0;
		wait(1);
	
		led1 = 0;
		led2 = 1;
		led3 = 0;
		led4 = 0;
		led5 = 0;
		wait(1);
	
		led1 = 0;
		led2 = 0;
		led3 = 1;
		led4 = 0;
		led5 = 0;
		wait(1);
	
		led1 = 0;
		led2 = 0;
		led3 = 0;
		led4 = 1;
		led5 = 0;
		wait(1);
	
		led1 = 0;
		led2 = 0;
		led3 = 0;
		led4 = 0;
		led5 = 1;
		wait(1);
		
		for (j=0; j<10; j++)
		{
			sprintf(txBuffer, "rotate: %d\r\n", 3);
			rf_send(txBuffer, strlen(txBuffer) + 1);
			printf("Sent:%s", txBuffer);
		}
	}
}

void tileSwitch(int num)
{
	num = num % 2;
	
	switch (num)	   //switching tiles.
	{
		case 0:				   //tile 1
			tile0 = 1;
			tile1 = 0;
		  break;
		
		case 1:				  // tile 2
			tile0 = 0;
			tile1 = 1;
			break;
		
		default:
			tile0 = 1;
			tile1 = 0;
			break;
	}
}

int rf_receive(char *data, uint8_t maxLength)				//receive xbee data 
{
	uint8_t len = mrf.Receive((uint8_t *)data, maxLength);
	uint8_t header[8] = {1, 8, 0, 0xA1, 0xB2, 0xC3, 0xD4, 0x00};
	uint8_t i;

	if (len > 10)
	{
		for (i=0; i<len-2; i++)
		{
			if (i < 8)
			{
				if (data[i] != header[i])
				{
					return 0;
				}
			}
			else
			{
				data[i-8] = data[i];
			}
		}
	}
	
	return ((int)len) - 10;
}

void rf_send(char *data, uint8_t len)		  //send data
{
	uint8_t header[8] = {1, 8, 0, 0xA1, 0xB2, 0xC3, 0xD4, 0x00};
	uint8_t *send_buf = (uint8_t *) malloc(sizeof(uint8_t) * (len+8));
	uint8_t i;

	for (i=0; i<len+8; i++)
	{
		send_buf[i] = (i<8)? header[i] : data[i-8];
	}
	
	mrf.Send(send_buf, len+8);
	free(send_buf);
}

int main(void)
{
	mrf.SetChannel(15);
	active = 0;
	tileSwitch(active);
	static int i;
	
	while(1)
	{
		rxLen = rf_receive(rxBuffer, 128);
		printf("Message Length:%d\r\n", rxLen);
		if (rxLen > 0)
		{
			if ((rxBuffer[0] == 'S') && (status == 0))
			{
				status = 1;
				active = active + 1;
				tileSwitch(active);
				
				for (i=0; i<10; i++)
				{
					sprintf(txBuffer, "active: %d\r\n", active);
					rf_send(txBuffer, strlen(txBuffer) + 1);
					printf("Sent:%s", txBuffer);
				}
			}
			else if (rxBuffer[0] == 'F')
			{
				ledMusic();
			}
		}
	}
}
			