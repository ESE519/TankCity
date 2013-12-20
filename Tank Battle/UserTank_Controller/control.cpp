#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <hal.h>
#include <nrk_error.h>
#include <nrk_events.h>
#include <nrk_timer.h>
#include <nrk_time.h>
#include <nrk_stack_check.h>
#include <nrk_stats.h>
#include <nrk_driver.h>
#include <nrk_driver_list.h>
#include <string.h>
#include <ff_basic_sensor.h>
#include <math.h>
#include "mbed.h"
#include "basic_rf.h"
#include "bmac.h"

Serial pc(USBTX, USBRX);
//Assign pins
//Assign input and output pins for Fire!
AnalogIn AinF(p18);
DigitalOut Fout(p10);
//Assign input and output pins for joystick control.
AnalogIn AinV(p19);
AnalogIn AinH(p20);
DigitalOut AoutF(p15);
DigitalOut AoutB(p16);
DigitalOut AoutL(p17);
DigitalOut AoutR(p14);

float ADCdataIV;
float ADCdataIH;


float ADCdataF;
float ADCdataB;
float ADCdataL;
float ADCdataR;
float Fire;
		
int main() {
	pc.printf("ADC Data values...\n\r");
	
	while(1){
		ADCdataIV=AinV;
		ADCdataIH=AinH;

		ADCdataF=AoutF;
		ADCdataB=AoutB;
		ADCdataL=AoutL;
		ADCdataR=AoutR;
		Fire=AinF;
		
//		printf("%f \r\n",ADCdataIV);
		// Fire command
		if(Fire>0.65 && Fire<7)
		{
			Fout=1;
			wait(0.5);
		}
		else{
			Fout=0;
		}

		 //joystick command.
		if(ADCdataIV>0.7)	//if joystick point front, send a forward signal.
		{
			AoutF=1;
			wait(0.1);
		}
		else
		{
			AoutF=0;
		}
		
		if(ADCdataIV<0.4) 	//if joystick point back, send a backward signal.
		{
			AoutB=1;
			wait(0.1);
		}
		else
		{
			AoutB=0;
		}
	
		if(ADCdataIH>0.7)	 //if joystick point left, send a left signal.
		{
			AoutL=1;
			wait(0.1);
			}
		else
		{
			AoutL=0;
		}
		
	
		if(ADCdataIH<0.4)	 //if joystick point right, send a right signal.
		{
			AoutR=1;
			wait(0.1);
		}
		else
		{
			AoutR=0;
		}
		
		
		
		
		}
	}