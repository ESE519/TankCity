/////////////This part is used to determine when does the game ends.
//////////// When the rubber band hits the base, and uses the sound sensor to sense the sound and anounces game lost.

#include "mbed.h"
#include "MRF24J40.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

AnalogIn mic(p19);


DigitalOut myled(LED1);

MRF24J40 mrf(p11, p12, p13, p14, p21);

char txBuffer[128];
char rxBuffer[128];
//extern const int com_channel;
int rxLen;
int rotate = 0;
int active = 0;
int status = 0;

int rf_receive(char *data, uint8_t maxLength)
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

void rf_send(char *data, uint8_t len)
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

void ledMusic(void)
{
	mrf.SetChannel(15);
	static int i;
	
	while(1)
	{
		for (i=0; i<3; i++)
		{
			sprintf(txBuffer, "Finished\r\n");
			rf_send(txBuffer, strlen(txBuffer) + 1);
			printf("Sent:%s\r\n", txBuffer);
			wait_ms(30);
		}
		
		rxLen = rf_receive(rxBuffer, 128);
		printf("Received Length:%d\r\n", rxLen);
		while (rxLen <= 0)
		{
			printf("Listening\r\n");
			rxLen = rf_receive(rxBuffer, 128);
		}
		rotate = rxBuffer[8] - 48;
		
		if (rotate == 3)
		{
			myled = 1;
			wait(0.5);
			myled = 0;
			wait(0.5);
			myled = 1;
			break;
		}
	}
}

int main(void)
{
	float mic_value;
	while(1)
	{
		mic_value = mic.read();
		printf("Mic Value:%2f\r\n", mic_value);
		
		if ((mic_value >= 0.60) || (mic_value <= 0.3))		   /////////if the sound is within this range, then announces the end of the game with "beeeeeeeeeep!"
		{
			printf("On target!\r\n");
			ledMusic();
			//break;
		}
	}
}