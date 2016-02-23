#define uint unsigned int
#define uchar unsigned char

#define MPU9250_I2C_ADDR            0xD0

 
#define MPU9250_I2C_SLV4_ADDR       0x31
#define MPU9250_I2C_SLV4_REG        0x32
#define MPU9250_I2C_SLV4_DO         0x33
#define MPU9250_I2C_SLV4_CTRL       0x34
#define MPU9250_I2C_SLV4_DI         0x35
#define MPU9250_I2C_MST_STATUS      0x36

#define MPU9250_I2C_SLVx_EN         0x80
#define MPU9250_I2C_SLV4_DONE       0x40
#define MPU9250_I2C_SLV4_NACK       0x10

#define AK8963_I2C_ADDR             0x0C
#define AK8963_Device_ID            0x48



void MPU9250_Write(uchar writeAddr, uchar writeData);
uchar MPU9250_Read(uchar readAddr);


void MPU9250_Init(void)
{write_A(0x80,0x6B);//Reset Device数据，地址
	write_A(0x01,0x6B); // Clock Source
	write_A(0x00,0x6C); // Enable Acc & Gyro
	write_A(0x07,0x1A);
	
	//write_A(0x20,0x1B);// +-1000dps 有低通滤波器
	write_A(0x20,0x18);// +-1000dps 无低通滤波器
	write_A(0x08,0x1C);// +-4G
	write_A(0x00,0x1D);// Set Acc Data Rates
	write_A(0x30,0x37);
	write_A(0x40,0x24);// I2C Speed 348 kHz
	write_A(0x20,0x6A);// Enable AUX



	MPU9250_Mag_Write(0x0b,0x01);
	MPU9250_Mag_Write(0x0A,0x12);
	
}
void MPU9250_Write(uchar writeAddr, uchar writeData)
{IIC_write(MPU9250_I2C_ADDR,writeAddr,writeData);}
	
uchar MPU9250_Read(uchar readAddr)
{ return IIC_read( MPU9250_I2C_ADDR,readAddr);}

void MPU9250_Mag_Write( uchar writeAddr, uchar writeData )
{
	uchar  status = 0;
	

	MPU9250_Write(MPU9250_I2C_SLV4_ADDR, AK8963_I2C_ADDR);
	//_delay_ms(1);
	MPU9250_Write(MPU9250_I2C_SLV4_REG, writeAddr);
	//_delay_ms(1);
	MPU9250_Write(MPU9250_I2C_SLV4_DO, writeData);
	//_delay_ms(1);
	MPU9250_Write(MPU9250_I2C_SLV4_CTRL, MPU9250_I2C_SLVx_EN);
//	_delay_ms(1);

	do {
		status = MPU9250_Read(MPU9250_I2C_MST_STATUS);
	//	_delay_ms(1);
	} while(((status & MPU9250_I2C_SLV4_DONE) == 0) );
}

uchar MPU9250_Mag_Read( uchar readAddr )
{
	uchar  status = 0;
	uchar readDate=0;
	

	IIC_write(MPU9250_I2C_ADDR,MPU9250_I2C_SLV4_ADDR,AK8963_I2C_ADDR | 0x80);
	//_delay_ms(1);
	IIC_write(MPU9250_I2C_ADDR,MPU9250_I2C_SLV4_REG,readAddr);
	//_delay_ms(1);
	IIC_write(MPU9250_I2C_ADDR,MPU9250_I2C_SLV4_CTRL,MPU9250_I2C_SLVx_EN);
	//_delay_ms(1);

	do {
		status = MPU9250_Read(MPU9250_I2C_MST_STATUS);
		
	} while(((status & MPU9250_I2C_SLV4_DONE) == 0) );
	readDate=IIC_read(MPU9250_I2C_ADDR,MPU9250_I2C_SLV4_DI);
	return readDate;
}