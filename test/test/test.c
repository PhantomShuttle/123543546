/*
 * test.c
 *
 * Created: 2016/2/8 13:16:16
 *  Author: x
 */ 
# define F_CPU 16000000UL//cpu时钟定义要放在延时函数前面，，，
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Oled.h"
//#include "I2C_drive.h"
//#include "SPI_drive.h"
#include "mcu_9250_spi.h"
#include "Quaternions.h"
#include <math.h>
#include <avr/io.h>

#define TimerRst()  TCNT1=0
#define TimerTo(X)	while(TCNT1<X)

//#include 
float xh,yh,tempAngle,panStart=0;
float accG[3],cos0,cos1,sin0,sin1;               // G-force in each direction
float accAngle[3],GY[3];           // Measured angle from accelerometer
float R;                     // Unit vector - total G.

int acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,axis_x,axis_y,axis_z,
        xg_offset,yg_offset,zg_offset,sss=0,sss1,zzz=0;
int original_data[10],max_m[6]={0}; 
int guolv_data[10];	
char s1=0,s2=0,s3=0,s4=0,bz=0,jpanStart=0;
//控制变量
int mx,my,mz,ax,ay,az,vv=0,vvv=0;
void port_init(void)
{
	DDRB = 0xbf;        //pb4 0 miso 输入11110111
	PORTB= 0x0f;
	DDRC = 0xff; 		   //PC输入
	PORTC= 0xff; 		   //无电阻 减少干扰
	DDRD = 0xF7;              //端口3舵机信号输入
	PORTD=0x00;
}

int oread(unsigned char DDR)
{ int read1,read2,read3;
	read1=SPI_read_9250(DDR);
	read2=SPI_read_9250(DDR+1);
	read3=(read1<<8)+read2;
	return read3;
}

int oread_M(unsigned char DDR)
{ int read1,read2,read3;
	read1=MPU9250_Mag_Read(DDR);
	read2=MPU9250_Mag_Read(DDR+1);
	read3=(read1<<8)+read2;
	return read3;
}

void offset(void)
{ int gx=0,gy=0,gz=0;
	MPU9250_getData( original_data );
	for (uchar i=0;i<100;i++)
	{ _delay_us(50);
		MPU9250_getData( original_data );
	 gx+=original_data[4];
	 gy+=original_data[5];
	 gz+=original_data[6];
	}
	xg_offset=-gx/100;
	yg_offset=-gy/100;
	zg_offset=-gz/100;
	 SPI_write_9250(0x13,(xg_offset>>8));//角速度偏移高位
	SPI_write_9250(0x14,((xg_offset<<8)>>8));
	 
	SPI_write_9250(0x15,(yg_offset>>8));//角速度偏移高位
	 SPI_write_9250(0x16,((yg_offset<<8)>>8));
	SPI_write_9250(0x17,(zg_offset>>8));//角速度偏移高位
	 SPI_write_9250(0x18,((zg_offset<<8)>>8));
	}



void max_min(void)
{
	if (original_data[7]>max_m[0]){max_m[0]=original_data[7];}
     else if  (original_data[7]<max_m[1]){max_m[1]=original_data[7];}
		 
	if (original_data[8]>max_m[2]){max_m[2]=original_data[8];}
	else if  (original_data[8]<max_m[3]){max_m[3]=original_data[8];}
		
	if (original_data[9]>max_m[4]){max_m[4]=original_data[9];}
	else if  (original_data[9]<max_m[5]){max_m[5]=original_data[9];}		 

	}

void Share_filter(int *indata,uchar lens,float Share,int *outdata )
{  for (uchar i=0;i<lens;i++)
{outdata[i]=(1-Share)*outdata[i]+Share*indata[i];
}
	
}

int main(void)
{   port_init();
	OledInit ();
    SPI_MasterInit();
	MPU9250_Init();
	offset();
	
 //	SREG = 0x80;        //使能全局中断
// 	TCCR2=0x05;
//	TCCR1A=0x00;
//	TCCR1B=0x02 ;
    while(1)
    {TimerRst() ;
	MPU9250_getData( original_data );
	
	Share_filter(original_data,10,0.06,guolv_data);
	GY[0]=guolv_data[4]* MPU9250G_2000dps;
	GY[1]=guolv_data[5]* MPU9250G_2000dps;
	GY[2]=guolv_data[6]* MPU9250G_2000dps;
//	max_min();


Quaternions_Count	( //0,0,1,
                     original_data[1],original_data[2],original_data[3],
                   GY[0],GY[1],GY[2],
	                         //        0,0,0,
					  original_data[8],original_data[7],-original_data[9]
		                //    1,0,0

);
Quaternions_to_EulerAngles();
		
		Cache_MDigit5_int( yaw,0,0,1); 	Cache_MDigit5_int(original_data[7],0,50,1);
					Cache_MDigit5_int(pitch,2,0,1);	    Cache_MDigit5_int(original_data[8],2,50,1);
					Cache_MDigit5_int(roll,4,0,1);     Cache_MDigit5_int(original_data[9],4,50,1);
// 					
// 						Cache_MDigit5_int( max_m[0],0,0,1); 	Cache_MDigit5_int(max_m[1],0,50,1);
// 						Cache_MDigit5_int(max_m[2],2,0,1);	    Cache_MDigit5_int(max_m[3],2,50,1);
// 						Cache_MDigit5_int(max_m[4],4,0,1);     Cache_MDigit5_int(max_m[5],4,50,1);
// 		
	//	Cache_MDigit5_int(axis_x,0,0,1); 	
	//	Cache_MDigit5_int(axis_y,2,0,1);	    
	//	Cache_MDigit5_int(axis_z,4,0,1);     
	//	sss=TCNT1;
	//	halfT=sss/2000000;
		Cache_MDigit5_int(sss,6,0,1);Cache_MDigit5_int(sss1,6,50,1);
		
        //TODO:: Please write your application code 
    }
}