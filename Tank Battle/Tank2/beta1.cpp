#include "mbed.h"
#include "m3pi.h"
#include "MRF24J40.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
///fromelf --bin -o E:\@L.bin !L
// take reference from kako's source code: http://www.kako.com/neta/2008-009/2008-009.html
// i2c protocol details from - http://blog.makezine.com/archive/2008/11/hacking_the_wiimote_ir_ca.html
// wiring from - http://translate.google.com/translate?u=http://www.kako.com/neta/2007-001/2007-001.html&hl=en&ie=UTF-8&sl=ja&tl=en
// obviously mbed is 3.3v so no level translation is needed
// using built in i2c on pins 9/10
//
// PC GUI client here: http://code.google.com/p/wii-cam-blobtrack/
//
// Interfacing details here: http://www.bot-thoughts.com/2010/12/connecting-mbed-to-wiimote-ir-camera.html
//
DigitalOut myled(LED1);
AnalogIn AinLight1(p16);
AnalogIn AinLight2(p17);
AnalogIn AinLight3(p18);
AnalogIn AinLight4(p19);
DigitalOut Life1(p8);
DigitalOut Life2(p7);
float ADCLight1;
float ADCLight2;
float ADCLight3;
float ADCLight4;
int life=20;
int cnt=0;

MRF24J40 mrf(p11, p12, p13, p14, p21);

char txBuffer[128];
char rxBuffer[128];

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

void calibrate(void)		///////sending the XBEE signal
{
	mrf.SetChannel(15);
	int i;
	
	while (!status)
	{
		sprintf(txBuffer, "Switch\r\n");
		rf_send(txBuffer, strlen(txBuffer) + 1);
		printf("Sent:%s\r\n", txBuffer);
		wait_ms(30);
		
		for (i=0; i<1; i++)
		{
			rxLen = rf_receive(rxBuffer, 128);
			printf("Received Length:%d\r\n", rxLen);
			while (rxLen <= 0)
			{
				printf("Listening\r\n");
				rxLen = rf_receive(rxBuffer, 128);
			}
			active = rxBuffer[8] - 48;
		}
		
		if (active == 1)
		{
			printf("Switch Done!\r\n");
			status = 1;
			//break;
		}
	}
	
	//mrf.SetChannel(com_channel);
}

m3pi tank(p23,p9,p10);
DigitalOut Fire(p15);

PwmOut servo(p24);
Serial pc(USBTX, USBRX); // tx, rx
I2C i2c(p28, p27);        // sda, scl
const int addr = 0xB0;   // define the I2C Address of camera

// for entering the loops in the main ( for different position of the map)
int okx0=0;
int okt0=0;
int oky0=0;
int okx1=0;
int okt1=0;
int oky1=0;
int okt2=0;
int okx2=0;
int okx3=0;
int okt3=0;
int oky2=0;
int okt4=0;
int ok5=0;
////////////////////////to get xy coordinate from Wii IR camera//////////////
void i2c_write2(int addr, char a, char b)
{
    char cmd[2];

    cmd[0] = a;
    cmd[1] = b;
    i2c.write(addr, cmd, 2);
    wait(0.07); // delay 70ms
}

void clock_init()
{
    // set up ~20-25MHz clock on p21
    LPC_PWM1->TCR = (1 << 1);               // Reset counter, disable PWM
    LPC_SC->PCLKSEL0 &= ~(0x3 << 12);
    LPC_SC->PCLKSEL0 |= (1 << 12);          // Set peripheral clock divider to /1, i.e. system clock
    LPC_PWM1->MR0 = 4;                     // Match Register 0 is shared period counter for all PWM1
    LPC_PWM1->MR3 = 2;                      // Pin 21 is PWM output 6, so Match Register 6
    LPC_PWM1->LER |= 1;                     // Start updating at next period start
    LPC_PWM1->TCR = (1 << 0) || (1 << 3);   // Enable counter and PWM
}

void cam_init()
{
    // Init IR Camera sensor
    i2c_write2(addr, 0x30, 0x01);
    i2c_write2(addr, 0x30, 0x08);
    i2c_write2(addr, 0x06, 0x90);
    i2c_write2(addr, 0x08, 0xC0);
    i2c_write2(addr, 0x1A, 0x40);
    i2c_write2(addr, 0x33, 0x33);
    wait(0.1);
}

void read_data(int res[]) {
				char cmd[8];
				char buf[36];
		//		int Ix1,Iy1,Ix2,Iy2;
		//		int Ix3,Iy3,Ix4,Iy4;
				int s;
				cmd[0] = 0x36;
        i2c.write(addr, cmd, 1);
        i2c.read(addr, buf, 16); // read the 16-byte result

         //for this project, the mode used is the Normal Mode, there are 12 bytes of data in total
    s = buf[3];
    res[0] = buf[1] + ((s & 0x30) <<4);  //x1
    res[1] = buf[2] + ((s & 0xC0) <<2);  //y1
    res[2] = s & 0x0F;  //size of point1
    s = buf[6];
    res[3] = buf[4] + ((s & 0x30) <<4);  //x2
    res[4] = buf[5] + ((s & 0xC0) <<2);  //y2
    res[5] = s & 0x0F;  //size of point2
    s = buf[9];
    res[6] = buf[7] + ((s & 0x30) <<4);  //x3
    res[7] = buf[8] + ((s & 0xC0) <<2);  //y3
    res[8] = s & 0x0F;  //size of point3
    s = buf[12];
    res[9]  = buf[10] + ((s & 0x30) <<4);  //x4
    res[10] = buf[11] + ((s & 0xC0) <<2);  //y4
    res[11] = s & 0x0F;  //size of point4
    wait(0.050);

}

/////done getting xy coordinate from Wii IR camera/////////////////////////////


////////////////////////////location///////////////////////



int distance (int x1, int y1, int x2, int y2){
    return (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);  //calculate the square of the distance
}


int min (int data[6]){  //to find the max and min in an array of 6 integers (for the distances)
    int i;
    int result = data[0];
    for (i=0; i<=5; i++){
        if (data[i]<result){
            result = data[i];
        }
    }
    return result;
}


int max (int data[6]){  //to find the max and min in an array of 6 integers (for the distances)
    int i;
    int result = data[0];
    for (i=0; i<=5; i++){
        if (data[i]>result){
            result = data[i];
        }
    }
    return result;
}
void wait_switch(void)		/////Waiting for changing the tile.
{
	mrf.SetChannel(15);
	static int done_sw = 0;
	
	while(1)
	{
		rxLen = rf_receive(rxBuffer, 128);
		printf("Received Length:%d\r\n", rxLen);
		while (rxLen <= 0)
		{
			printf("Listening\r\n");
			rxLen = rf_receive(rxBuffer, 128);
		}
		
		done_sw  = rxBuffer[8] - 48;
		
		if (done_sw == 1)
		{
			break;
		}
	}
}

void load(int data[12], int position[8]){
// initialize variables
	 int x1,y1,x2,y2,x3,y3,x4,y4,dis[6],dismin,dismax,pos1x,pos1y,pos2x,pos2y,pos3x,pos3y,pos4x,pos4y; 
    
    int score1 = 0;
    int score2 = 0;
    int score3 = 0;
    int score4 = 0;

	int maxscore;
	int minscore;

  ////start loding the data from the camera sensor////////
    x1 = data[0];      y1 = data[1];	       x2 = data[3];	    y2 = data[4];
    x3 = data[6];      y3 = data[7];	       x4 = data[9];        y4 = data[10];
   
   
   
   //calculate the distance between detected points
    dis[0] = distance (x1,y1,x2,y2);      dis[1] = distance (x1,y1,x3,y3);  //12,13
    dis[2] = distance (x1,y1,x4,y4);      dis[3] = distance (x2,y2,x3,y3);  //14,23
    dis[4] = distance (x4,y4,x2,y2);      dis[5] = distance (x4,y4,x3,y3);  //42,43
    dismin = min(dis);     dismax = max(dis);
   
   
   
 ///////////////start assign scores///////////  
 //find the which one is the min and which one is max.  
  
   if (dismin==dis[0]){
        minscore=0;
     }
    if (dismax==dis[0]){
		maxscore=0;
     }
    if (dismin==dis[1]){
		minscore=1;
     }
    if (dismax==dis[1]){
		maxscore=1;
        
    }
    if (dismin==dis[2]){
		minscore=2;
        
    }
    if (dismax==dis[2]){
		maxscore=2;
        
    }
    if (dismin==dis[3]){
		minscore=3;
        
    }
    if (dismax==dis[3]){
        
    }
    if (dismin==dis[4]){
		minscore=4;
        
    }
    if (dismax==dis[4]){
		maxscore=4;
        
    }
    if (dismin==dis[5]){
		minscore=5;
        
    }
    if (dismax==dis[5]){
		maxscore=5;
        
    }
//assign scores
	 switch(maxscore){
	 case 0:
	    score1 += 2;
        score2 += 2;
		break;
	 case 1:
	 	score1 += 2;
        score3 += 2;
		break;
	 case 2:
	 	score1 += 2;
        score4 += 2;
		break;
	 case 3:
	 	score2 += 2;
        score3 += 2;
		break;
	 case 4:
	 	score4 += 2;
        score2 += 2;
		break;
	 case 5:
	 	score4 += 2;
        score3 += 2;
		break;
	 }

	 switch(minscore){
	 case 0:
	    score1 += 1;
        score2 += 1;
		break;
	 case 1:
	 	score1 += 1;
        score3 += 1;
		break;
	 case 2:
	 	score1 += 1;
        score4 += 1;
		break;
	 case 3:
	 	score2 += 1;
        score3 += 1;
		break;
	 case 4:
	 	score4 += 1;
        score2 += 1;
		break;
	 case 5:
	 	score4 += 1;
        score3 += 1;
		break;
	 }

//assign the position according to the score.
//assign value for x1 and y1
	if (score1==0)
	{
		pos1x = x1;
        pos1y = y1;	
	}else if  (score1==1)
	{
		pos2x = x1;
        pos2y = y1;
	}else if  (score1==2)
	{
		pos3x = x1;
        pos3y = y1;
	}else if  (score1==3)
	{
		pos4x = x1;
        pos4y = y1;
	}
//assign value for x2 and y2
		if (score2==0)
	{
		pos1x = x2;
        pos1y = y2;	
	}else if  (score2==1)
	{
		pos2x = x2;
        pos2y = y2;
	}else if  (score2==2)
	{
		pos3x = x2;
        pos3y = y2;
	}else if  (score2==3)
	{
		pos4x = x2;
        pos4y = y2;
	}
//assign value for x3 and y3
		if (score3==0)
	{
		pos1x = x3;
        pos1y = y3;	
	}else if  (score3==1)
	{
		pos2x = x3;
        pos2y = y3;
	}else if  (score3==2)
	{
		pos3x = x3;
        pos3y = y3;
	}else if  (score3==3)
	{
		pos4x = x3;
        pos4y = y3;
	}	
//assign value for x4 and y4	
		if (score4==0)
	{
		pos1x = x4;
        pos1y = y4;	
	}else if  (score4==1)
	{
		pos2x = x4;
        pos2y = y4;
	}else if  (score4==2)
	{
		pos3x = x4;
        pos3y = y4;
	}else if  (score4==3)
	{
		pos4x = x4;
        pos4y = y4;
	}		
	
//assign the position according to the score.

    position[0] = pos1x;      position[1] = pos1y;		      position[2] = pos2x;		    position[3] = pos2y;
    position[4] = pos3x;	  position[5] = pos3y;	          position[6] = pos4x;		    position[7] = pos4y;
}
																						   

void Locate(double pos[]){
    /*This functino returns the position of the robot in a world system*/
   int data[12],posi[8];
   	double d=0,	 dx=0 ,dy=0, angle1=0, angle2=0,angle3=0;
    double pos1x = double(posi[0]);
    double pos1y = double(posi[1]);
    double pos2x = double(posi[2]);
    double pos2y = double(posi[3]);
    double pos3x = double(posi[4]);
    double pos3y = double(posi[5]);
    double pos4x = double(posi[6]);
    double pos4y = double(posi[7]);

	   //find the cross point of 
    double r12 = (pos2y - pos1y)/(pos2x - pos1x);
    double r34 = (pos4y - pos3y)/(pos4x - pos3x);
    double xc= (pos3y - pos1y - r34*pos3x + r12*pos1x)/(r12 - r34);
    double yc= r12*(xc - pos1x) + pos1y;



  ////////start calculate
	read_data(data);
    load(data, posi);
    dx = xc-510;   //the camera resolution is 1024*768. so do this to get the origin.
    dy = yc-384;
    d = dx*dx + dy*dy;
    d = sqrt(d);
    angle2  = atan2(dx, dy);   // find the angle base on system 3 and 4
	double betax = double(pos3x-pos4x);
    double betay = double(pos3y-pos4y);
    angle3 = atan2(betax, betay);
    angle3 += 154*(3.14159265)/180;
    angle1 = angle2 - angle3 + 3.14159265;
    pos[0] = d*cos(angle1);					   ///x of the ir system
		pos[1] = d*sin(angle1);				  ////y of the ir system
    pos[2] = (-angle3+3.14159265);			  /////angle of the ir system

}
//////////////////location end////////////////


int main() {

   		ADCLight1=AinLight1;
		ADCLight2=AinLight2;
		ADCLight3=AinLight3;
		ADCLight4=AinLight4;
//if constanly sense the light, reduct the hp value		
		if(ADCLight1>0.57 | ADCLight2>0.57 | ADCLight3>0.57 | ADCLight4>0.57)
		{
		cnt++;
		}else{
		cnt=0;}

		if(cnt>3){
		  life=life-cnt;
		  cnt=0;
		  }
		
//checking Hp of the tank
		if(life>10){
		Life1=1;
		Life2=1;
		}
		else if(life>0 && life<=10){
		Life1=1;
		Life2=0;
		}
		else{
		Life1=0;
		Life2=0;
		tank.stop();

		system("pasuse");

		}



   clock_init();

     //PC serial output
    pc.baud(115200);
    pc.printf("Initializing camera...");

    cam_init();
	wait_switch(); //////waiting for chaning the tile, otherwise stand still.

    while(1) {
	int data[12];
    int po[8];
	double posoh[3];
	int state; 

	read_data(data);
	load(data, po);
	
	Locate(posoh);
/////start moving the tank////////	

                   if(okx0==0)	 ///////first move to x 200 with degree 1.38~1.52
                   {

                        if(1.52>= posoh[2] && posoh[2] >= 1.38)
                        {
                                state =1;
                        }
                        if (posoh[2] >1.52)
                        {
                                state =2;
                        }
                        if(posoh[2]<1.38)
                        {
                                state =3;
                        }

                        switch(state)
                        {
                                case 1:
                                        if(posoh[1]<=200)
                                        {
                                        tank.forward(0.1);
                                         }
                                         else{
                                         tank.stop();
                                         okx0=1;
                                         okt0=1;
                                         }

                                        break;
                                case 2:
                                        tank.right(0.068);
                                break;
                                case 3:
                                        tank.left(0.068);
                                break;

                                default:
                                break;
                        }


                    }


////////////////////////////////done x0//////////////////
                        if(okt0==1)			////move to x-140 with degree 2.95~3.1
                        {
	                        if(3.1>= posoh[2] && posoh[2] >= 2.95)
	                        {
	                                state =1;
	                        }
	                        if (posoh[2] >3.1)
	                        {
	                                state =2;
	                        }
	                        if(posoh[2]<2.95)
	                        {
	                                state =3;
	                        }
	
	                        switch(state)
	                        {
	                                case 1:
	                                        if(posoh[0]>=-140)
	                                        {
	                                        tank.forward(0.1);
	                                         }
	                                         else{
	                                         tank.stop();
	                                         okt0=0;
	                                         okt1=1;
	                                         }
	
	                                        break;
	                                case 2:
	                                        tank.right(0.068);
	                                break;
	                                case 3:
	                                        tank.left(0.068);
	                                break;
	
	                                default:
	                                break;
	                        }
                        }
//////////////done x and t 0////////
                        if(okt1==1)	 //////move to y=-200 with angel -1.72~-1.58
                        {
                            if(-1.58>= posoh[2] && posoh[2] >= -1.72)
                            {
                                    state =1;
                            }
                            if (posoh[2] >-1.58)
                            {
                                    state =2;
                            }
                            if(posoh[2]<-1.72 || posoh[2]>2.9)
                            {
                                    state =3;
                            }

                            switch(state)
                            {
                                    case 1:
                                            if(posoh[1]>=-200)
                                            {
                                            tank.forward(0.1);
                                             }
                                             else{
                                             tank.stop();
                                             okt1=0;
                                             okt2=1;
                                             }

                                            break;
                                    case 2:
                                            tank.right(0.068);
                                    break;
                                    case 3:
                                            tank.left(0.068);
                                    break;

                                    default:
                                    break;
                            }
                        }


// ////////////////////////turn ////////////////////
                        if(okt2==1)	   //////////////trun to angel -2.4
                        {
                         if(posoh[2]>-2.4)
                         {
                                tank.right(0.075);
                         }


                         else
                         {
                         tank.stop();
                                okt2=0;
                                oky0=1;
                         }
                        }
 /////////////////////Fire the rubber band
                      if(oky0==1)
                      {
							Fire=0;
							wait(0.5);
							Fire=1;
							wait(0.5);
							Fire=0;
                      }

}
}