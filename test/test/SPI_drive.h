
#define SS_L	PORTB &= ~(1 << PB0)/*��ƽ�õ�*/
#define SS_H	PORTB |= (1 << PB0)/*��ƽ�ø�*/
void SPI_write_9250(unsigned char addr, unsigned char data)
{
	SS_L;
	/* �������ݴ��� */
	SPDR = addr;
	/* �ȴ�������� */
	while(!(SPSR & (1<<SPIF)));
	SPDR = data;
	/* �ȴ�������� */
	while(!(SPSR & (1<<SPIF)));
	SS_H;
}

uchar SPI_read_9250(unsigned char addr)
{  uchar readData=0;
	addr|=0x80;
	SS_L;
	/* �������ݴ��� */
	SPDR = addr;
	/* �ȴ�������� */
	while(!(SPSR & (1<<SPIF)));
	
	SPDR = 0x00;
	while(!(SPSR & (1<<SPIF)));
	/* ���������� */
	readData=SPDR;
	SS_H;
	
	return readData;//��������һ��Ҫд�����
	
}
void SPI_MasterInit(void)
{
	
	//DDRB = 0xb0;
	//PORTB=0xff;
	/* ʹ��SPI ����ģʽ������ʱ������Ϊfck/8 */
	
	SPCR = 0X51;
	SPSR=0X01;
}