
#define SS_L	PORTB &= ~(1 << PB0)/*电平置低*/
#define SS_H	PORTB |= (1 << PB0)/*电平置高*/
void SPI_write_9250(unsigned char addr, unsigned char data)
{
	SS_L;
	/* 启动数据传输 */
	SPDR = addr;
	/* 等待传输结束 */
	while(!(SPSR & (1<<SPIF)));
	SPDR = data;
	/* 等待传输结束 */
	while(!(SPSR & (1<<SPIF)));
	SS_H;
}

uchar SPI_read_9250(unsigned char addr)
{  uchar readData=0;
	addr|=0x80;
	SS_L;
	/* 启动数据传输 */
	SPDR = addr;
	/* 等待传输结束 */
	while(!(SPSR & (1<<SPIF)));
	
	SPDR = 0x00;
	while(!(SPSR & (1<<SPIF)));
	/* 读返回数据 */
	readData=SPDR;
	SS_H;
	
	return readData;//返回数据一定要写在最后
	
}
void SPI_MasterInit(void)
{
	
	//DDRB = 0xb0;
	//PORTB=0xff;
	/* 使能SPI 主机模式，设置时钟速率为fck/8 */
	
	SPCR = 0X51;
	SPSR=0X01;
}