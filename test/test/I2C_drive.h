#include <math.h>
#include <avr/io.h>
# define F_CPU 16000000UL//cpu时钟定义要放在延时函数前面，，，
#include <util/delay.h>
#define uint unsigned int
#define uchar unsigned char
#define ack 0x01  ////ack=1,发送正常
//#define SLA_W       0xa0  	//从机地址,主机写操作
//#define SLA_R        0xa1  	//从机地址,主机读操作
#define SCL_CLR	PORTC &= ~(1 << PC0)/*电平置低*/
#define SCL_SET	PORTC |= (1 << PC0)/*电平置高*/
#define SCL_IN	DDRC &= ~(1 << PC0)/*方向输入*/
#define SCL_OUT	DDRC |= (1 << PC0)/*方向输出*/

#define SDA_CLR	PORTC &= ~(1 << PC1)/*电平置低*/
#define SDA_SET	PORTC |= (1 << PC1)/*电平置高*/
#define SDA_R	PINC & (1 << PC1)/*电平读取*/
#define SDA_IN	DDRC &= ~(1 << PC1)/*方向输入*/
#define SDA_OUT	DDRC |= (1 << PC1)/*方向输出*/

uchar flag;



void I2C_start(void){ 
     SDA_SET; //1
	 
     SCL_SET;  //1
    _delay_us(1);
     SDA_CLR;//0
  _delay_us(1);
     SCL_CLR;	
}

void I2C_stop(void){
     SDA_CLR; //0
   _delay_us(1);
     SCL_SET;   //1
   _delay_us(1);
     SDA_SET;  //1
}

void Sendbyte(uchar write_data)
{
    uchar i=8;
    while (i--){
        if (write_data&0x80){SDA_SET;}else{SDA_CLR;}
        write_data<<=1; 
    _delay_us(5);
        SCL_SET; //1
   _delay_us(5);
        SCL_CLR;//0 
    }
    SDA_SET;  //1
  _delay_us(1);
    SCL_SET;   //1
    SDA_IN;
_delay_us(1);
    if (SDA_R){flag|=ack;}else{flag&=~ack;}
    SCL_CLR;  //0
   	SDA_OUT;
}

uchar Recbyte(void)
{
     uchar i=8,temp=0;
     SDA_IN;
  _delay_us(1);
     while (i--){
         SCL_SET;  //1
         temp<<=1;
         if (SDA_R){temp++;} 
         SCL_CLR; //0
     _delay_us(1);
     }
   	 SDA_OUT;
     return temp;
}

void I2C_ack(uchar flagtemp){
     if (flagtemp){SDA_SET;}else{SDA_CLR;}
     SCL_SET; //1
   _delay_us(1);
     SCL_CLR;//0
}
void IIC_write(uchar nameaddr,uint addr,uchar value)
{
	uchar i=2;
	SDA_OUT;
	SCL_OUT;
	while (i--){
		I2C_start();
		Sendbyte(nameaddr);
		if (flag&ack){
			I2C_stop();
			continue;
		}
		Sendbyte(addr);
		if (flag&ack){
			I2C_stop();
			continue;
		}
		Sendbyte(value);
		if (flag&ack){
			I2C_stop();
			continue;
		}
		I2C_stop();
		if(!(flag&ack))break;
	}
}
void write_A(uchar value,uint addr)
{
     uchar i=2;
     SDA_OUT;
	 SCL_OUT;
     while (i--){
         I2C_start();
         Sendbyte(0xD0); 
         if (flag&ack){
            I2C_stop();
            continue;
         }
         Sendbyte(addr);
         if (flag&ack){
            I2C_stop(); 
            continue;
         } 
         Sendbyte(value);
         if (flag&ack){
            I2C_stop(); 
            continue; 
         }
         I2C_stop();
         if(!(flag&ack))break;
     }   
}
void write_M(uchar value,uint addr)
{
     uchar i=2;
     SDA_OUT;
	 SCL_OUT;
     while (i--){
         I2C_start();
         Sendbyte(0x0c); 
         if (flag&ack){
            I2C_stop();
            continue;
         }
         Sendbyte(addr);
         if (flag&ack){
            I2C_stop(); 
            continue;
         } 
         Sendbyte(value);
         if (flag&ack){
            I2C_stop(); 
            continue; 
         }
         I2C_stop();
         if(!(flag&ack))break;
     }   
}
int IIC_read(uchar nameaddr,uint addr)
{
	uchar  value;
	SDA_OUT;
	SCL_OUT;
	I2C_start();
	Sendbyte(nameaddr);
	Sendbyte(addr);
	I2C_start();
	Sendbyte(nameaddr+1);
	value=Recbyte();
	I2C_ack(1);
	I2C_stop();
	return value;
}
int read_A(uint addr){
     uchar  value;
     SDA_OUT;
	 SCL_OUT;
	 I2C_start();
     Sendbyte(0xD0);
     Sendbyte(addr); 
     I2C_start();
     Sendbyte(0xD1);
     value=Recbyte();
     I2C_ack(1); 
     I2C_stop(); 
     return value;
}


int read_M(uint addr){
     uchar  value;
     SDA_OUT;
	 SCL_OUT;
	 I2C_start();
     Sendbyte(0x0c);
     Sendbyte(addr); 
     I2C_start();
     Sendbyte(0x0d);
     value=Recbyte();
     I2C_ack(1); 
     I2C_stop(); 
     return value;
}



