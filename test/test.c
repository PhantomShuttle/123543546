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
float p_mid[10]={0,0,0,0,0 ,0,0,0,0,0},sss=0;
int acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,axis_x,axis_y,axis_z,
        xg_offset,yg_offset,zg_offset,sss1,zzz=0;
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


void offset(void)
{ int gx=0,gy=0,gz=0;
	MPU9250_getData( original_data );
	for (uchar i=0;i<200;i++)
	{ _delay_us(50);
		MPU9250_getData( original_data );
	 gx+=original_data[4];
	 gy+=original_data[5];
	 gz+=original_data[6];
	}
	xg_offset=-gx/200;
	yg_offset=-gy/200;
	zg_offset=-gz/200;
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
{outdata[i]=(1-Share)*outdata[i]+Share*indata[i];}

	
}

/*        
        Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
        R:测量噪声，R增大，动态响应变慢，收敛稳定性变好        
*/

void	kalmah(int *indata,uchar lens,float *p_mid,float Q,float R,int *outdata)
{// wk=indata[i]-outdata[i];
	float kg;
	for (uchar i=0;i<lens;i++)
	{p_mid[i]=p_mid[i]+Q;  //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
		kg=p_mid[i]/(p_mid[i]+R); //kg为kalman filter，R为噪声
		outdata[i]=outdata[i]+kg*(indata[i]-outdata[i]);
		p_mid[i]=(1-kg)*p_mid[i];//最优值对应的covariance
	}
	
}
int main(void)
{   port_init();
	OledInit ();
    SPI_MasterInit();
	MPU9250_Init();
	offset();
	
 //	SREG = 0x80;        //使能全局中断
 	TCCR2=0x05;
	TCCR1A=0x00;
	TCCR1B=0x02 ;
    while(1)
    {TimerRst() ;
	MPU9250_getData( original_data );
//	kalmah(original_data,10,p_mid,0.018,1,guolv_data);
//	Share_filter(original_data,10,0.1,guolv_data);
	guolv_data[1]=guolv_data[1]*0.6+original_data[1]*0.4;
	guolv_data[2]=guolv_data[2]*0.6+original_data[2]*0.4;
	guolv_data[3]=guolv_data[3]*0.6+original_data[3]*0.4;
	
	guolv_data[4]=guolv_data[4]*0.95+original_data[4]*0.05;
	guolv_data[5]=guolv_data[5]*0.95+original_data[5]*0.05;
	guolv_data[6]=guolv_data[6]*0.95+original_data[6]*0.05;
	
		guolv_data[7]=guolv_data[7]*0.6+original_data[7]*0.4;
		guolv_data[8]=guolv_data[8]*0.6+original_data[8]*0.4;
		guolv_data[9]=guolv_data[9]*0.6+original_data[9]*0.4;
		
	GY[0]=guolv_data[4]* MPU9250G_1000dps;
	GY[1]=guolv_data[5]* MPU9250G_1000dps;
	GY[2]=guolv_data[6]* MPU9250G_1000dps;
//	max_min();


Quaternions_Count	( //  0,0,1,
                   guolv_data[1],guolv_data[2],guolv_data[3],
                 GY[0],GY[1],GY[2],
				// 0,0,0,
					    guolv_data[8],guolv_data[7],-guolv_data[9]
		            //      1,0,0

);
Quaternions_to_EulerAngles();
		
		Cache_MDigit5_int( yaw,0,0,1); 	Cache_MDigit5_int(original_data[4],0,50,1);
					Cache_MDigit5_int(pitch,2,0,1);	    Cache_MDigit5_int(original_data[5],2,50,1);
					Cache_MDigit5_int(roll,4,0,1);     Cache_MDigit5_int(original_data[6],4,50,1);
// 					
// 						Cache_MDigit5_int( max_m[0],0,0,1); 	Cache_MDigit5_int(max_m[1],0,50,1);
// 						Cache_MDigit5_int(max_m[2],2,0,1);	    Cache_MDigit5_int(max_m[3],2,50,1);
// 						Cache_MDigit5_int(max_m[4],4,0,1);     Cache_MDigit5_int(max_m[5],4,50,1);
// 		
	//	Cache_MDigit5_int(axis_x,0,0,1); 	
	//	Cache_MDigit5_int(axis_y,2,0,1);	    
	//	Cache_MDigit5_int(axis_z,4,0,1);     
		sss=TCNT1;
		halfT=sss/2000000;
		Cache_MDigit5_int(sss,6,0,1);Cache_MDigit5_int(sss1,6,50,1);
		
        //TODO:: Please write your application code 
    }
}