#include "mbed.h"
#include "m3pi.h"

m3pi m3pi;
Serial pc(USBTX, USBRX);
// assign digital pin inputs
DigitalIn AoutF(p20);
DigitalIn AoutB(p19);
DigitalIn AoutL(p17);
DigitalIn AoutR(p18);



int main() {

    m3pi.locate(0,1);
    
    wait (2.0);

		
	while(1)
	{

		if(AoutF==1)  ///if pin20 is high then move forward.
		{
	    m3pi.forward(0.2); 
	    }      
	    if(AoutL==1)  //if pin17 is high then move left.
		{
		m3pi.left(0.2);     
		}	
   	   if(AoutB==1)	   //if pin19 is high, then move backward
		{
		m3pi.backward(0.2);
 	    }    
		if(AoutR==1)   //if pin18 is high then move righ.
		{
		m3pi.right(0.2); 
		}
    	m3pi.stop();       
	}
}