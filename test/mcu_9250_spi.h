//#include "SPI_drive.h"

#define uint unsigned int
#define uchar unsigned char

#define MPU9250A_2g       ((float)0.000061035156f)  // 0.000061035156 g/LSB
#define MPU9250A_4g       ((float)0.000122070312f)  // 0.000122070312 g/LSB
#define MPU9250A_8g       ((float)0.000244140625f)  // 0.000244140625 g/LSB
#define MPU9250A_16g      ((float)0.000488281250f)  // 0.000488281250 g/LSB

#define MPU9250G_250dps   ((float)0.007633587786f)  // 0.007633587786 dps/LSB
#define MPU9250G_500dps   ((float)0.015267175572f)  // 0.015267175572 dps/LSB
#define MPU9250G_1000dps  ((float)0.030487804878f)  // 0.030487804878 dps/LSB
#define MPU9250G_2000dps  ((float)0.060975609756f)  // 0.060975609756 dps/LSB

#define MPU9250_I2C_ADDR            0xD0

#define MPU9250_SMPLRT_DIV          0x19
#define MPU9250_CONFIG              0x1A
#define MPU9250_GYRO_CONFIG         0x1B
#define MPU9250_ACCEL_CONFIG        0x1C
#define MPU9250_ACCEL_CONFIG_2      0x1D

#define MPU9250_I2C_MST_CTRL        0x24
#define MPU9250_I2C_SLV0_ADDR       0x25
#define MPU9250_I2C_SLV0_REG        0x26
#define MPU9250_I2C_SLV0_CTRL       0x27

#define MPU9250_I2C_SLV4_ADDR       0x31
#define MPU9250_I2C_SLV4_REG        0x32
#define MPU9250_I2C_SLV4_DO         0x33
#define MPU9250_I2C_SLV4_CTRL       0x34
#define MPU9250_I2C_SLV4_DI         0x35
#define MPU9250_I2C_MST_STATUS      0x36
#define MPU9250_INT_PIN_CFG         0x37
#define MPU9250_INT_ENABLE          0x38
#define MPU9250_INT_STATUS          0x3A
#define MPU9250_ACCEL_XOUT_H        0x3B
#define MPU9250_ACCEL_XOUT_L        0x3C
#define MPU9250_ACCEL_YOUT_H        0x3D
#define MPU9250_ACCEL_YOUT_L        0x3E
#define MPU9250_ACCEL_ZOUT_H        0x3F
#define MPU9250_ACCEL_ZOUT_L        0x40
#define MPU9250_TEMP_OUT_H          0x41
#define MPU9250_TEMP_OUT_L          0x42
#define MPU9250_GYRO_XOUT_H         0x43
#define MPU9250_GYRO_XOUT_L         0x44
#define MPU9250_GYRO_YOUT_H         0x45
#define MPU9250_GYRO_YOUT_L         0x46
#define MPU9250_GYRO_ZOUT_H         0x47
#define MPU9250_GYRO_ZOUT_L         0x48

#define MPU9250_I2C_MST_DELAY_CTRL  0x67

#define MPU9250_USER_CTRL       0x6A
#define MPU9250_PWR_MGMT_1      0x6B
#define MPU9250_PWR_MGMT_2      0x6C

#define MPU9250_I2C_SLVx_EN         0x80
#define MPU9250_I2C_SLV4_DONE       0x40
#define MPU9250_I2C_SLV4_NACK       0x10


#define AK8963_I2C_ADDR             0x0C
#define AK8963_Device_ID            0x48

// Read-only Reg
#define AK8963_WIA                  0x00
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02
#define AK8963_HXL                  0x03
#define AK8963_HXH                  0x04
#define AK8963_HYL                  0x05
#define AK8963_HYH                  0x06
#define AK8963_HZL                  0x07
#define AK8963_HZH                  0x08
#define AK8963_ST2                  0x09
// Write/Read Reg
#define AK8963_CNTL1                0x0A
#define AK8963_CNTL2                0x0B
// Status
#define AK8963_STATUS_DRDY          0x01
#define AK8963_STATUS_DOR           0x02
#define AK8963_STATUS_HOFL          0x08

void  MPU9250_Mag_Write( uchar writeAddr, uchar writeData );
uchar MPU9250_Mag_Read( uchar readAddr );
void MPU9250_Init(void);

#define SS_L	PORTB &= ~(1 << PB0)/*电平置低*/
#define SS_H	PORTB |= (1 << PB0)/*电平置高*/

float AK8963_ASA[3];
int AK8963_offset[3]={30,-220,105};
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
	SPSR=0X00;
}

void MPU9250_Init(void)
{  
//      SPI_write_9250(MPU9250_PWR_MGMT_1     ,0x80);//复位
// 	 SPI_write_9250(MPU9250_PWR_MGMT_1     ,0x01);
// 	 SPI_write_9250(MPU9250_PWR_MGMT_2     ,0x00);//使能 加速度计 角速度计
// 	 SPI_write_9250(MPU9250_CONFIG         ,0x07);// 低通滤波
//      // SPI_write_9250(MPU9250_GYRO_CONFIG  ,0x13);//角速度计 +-1000dps量程 ，低通滤波 
// 	 SPI_write_9250(MPU9250_GYRO_CONFIG    ,0x18);//角速度计 +-2000dps量程 ，低通滤波  
//    //SPI_write_9250(MPU9250_ACCEL_CONFIG ,0x00);//加速度计+-2g
// 	 SPI_write_9250(MPU9250_ACCEL_CONFIG   ,0x08);//加速度计+-4g
// 	 SPI_write_9250(MPU9250_ACCEL_CONFIG_2 ,0x00);//加速度计低通滤波关闭 
// 	 SPI_write_9250(MPU9250_INT_PIN_CFG    ,0x30);//开启中断 高电平 推举式 一直保持 读取清零
// 	 SPI_write_9250(MPU9250_I2C_MST_CTRL  ,0x40);// I2C Speed 
// 	 SPI_write_9250(MPU9250_USER_CTRL      ,0x20);// Enable AUX

 SPI_write_9250(MPU9250_PWR_MGMT_1     ,0x80);// [0]  Reset Device
 SPI_write_9250(MPU9250_PWR_MGMT_1     ,0x04);// [1]  Clock Source
 SPI_write_9250(MPU9250_INT_PIN_CFG    ,0x10);// [2]  Set INT_ANYRD_2CLEAR
 SPI_write_9250(MPU9250_INT_ENABLE     ,0x01);// [3]  Set RAW_RDY_EN
 SPI_write_9250(MPU9250_PWR_MGMT_2     ,0x00);// [4]  Enable Acc & Gyro
 SPI_write_9250(MPU9250_SMPLRT_DIV     ,0x00);// [5]  Sample Rate Divider
 SPI_write_9250(MPU9250_GYRO_CONFIG    ,0x18);// [6]  default : +-2000dps
 SPI_write_9250(MPU9250_ACCEL_CONFIG   ,0x08);// [7]  default : +-4G
 SPI_write_9250(MPU9250_CONFIG         ,0x01);// [8]  default : LPS_184Hz
 SPI_write_9250(MPU9250_ACCEL_CONFIG_2 ,0x03);// [9]  default : LPS_41Hz
 //SPI_write_9250(MPU9250_I2C_MST_CTRL ,0x40);// I2C Speed
 SPI_write_9250(MPU9250_USER_CTRL      ,0x30);// [10] Set I2C_MST_EN, I2C_IF_DIS

   MPU9250_Mag_Write(AK8963_CNTL2,0x01);//磁力计复位 
   MPU9250_Mag_Write(AK8963_CNTL1, 0x10);       // Power-down mode
   
   MPU9250_Mag_Write(AK8963_CNTL1, 0x1F);   // Fuse ROM access mode
   
   AK8963_ASA[0]= MPU9250_Mag_Read(0x10);  // 读取磁传感器灵敏度的调整值
   AK8963_ASA[1]= MPU9250_Mag_Read(0x11);
   AK8963_ASA[2]= MPU9250_Mag_Read(0x12);
   
   AK8963_ASA[0]=((AK8963_ASA[0]-128)/256)+1; //计算灵敏度调整值
   AK8963_ASA[1]=((AK8963_ASA[1]-128)/256)+1;
   AK8963_ASA[2]=((AK8963_ASA[2]-128)/256)+1; 
   
   
   MPU9250_Mag_Write(AK8963_CNTL1, 0x10);       // Power-down mode 
   
   SPI_write_9250(MPU9250_I2C_MST_CTRL, 0x5D);
   SPI_write_9250(MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80); //设置辅助iic读取 器件地址
   SPI_write_9250(MPU9250_I2C_SLV0_REG, AK8963_ST1);        //开始读的地址  
   SPI_write_9250(MPU9250_I2C_SLV0_CTRL, MPU9250_I2C_SLVx_EN | 8); 
  
   MPU9250_Mag_Write(AK8963_CNTL1, 0x16);       // Continuous measurement mode 2
   
   SPI_write_9250(MPU9250_I2C_SLV4_CTRL, 0x09);  //往后连续读的位数 一共读 9+1
   
   SPI_write_9250(MPU9250_I2C_MST_DELAY_CTRL, 0x81);
   
	
}

void MPU9250_Mag_Write( uchar writeAddr, uchar writeData )
{
	uchar  status = 0;
	

	SPI_write_9250(MPU9250_I2C_SLV4_ADDR, AK8963_I2C_ADDR);
	SPI_write_9250(MPU9250_I2C_SLV4_REG, writeAddr);
	SPI_write_9250(MPU9250_I2C_SLV4_DO, writeData);
	SPI_write_9250(MPU9250_I2C_SLV4_CTRL, MPU9250_I2C_SLVx_EN);

	do {
		status = SPI_read_9250(MPU9250_I2C_MST_STATUS);
	   } while(((status & MPU9250_I2C_SLV4_DONE) == 0) );
}

uchar MPU9250_Mag_Read( uchar readAddr )
{
	uchar  status = 0;
	uchar readDate=0;
	
	SPI_write_9250(MPU9250_I2C_SLV4_ADDR,AK8963_I2C_ADDR | 0x80);
	SPI_write_9250(MPU9250_I2C_SLV4_REG,readAddr);
	SPI_write_9250(MPU9250_I2C_SLV4_CTRL,MPU9250_I2C_SLVx_EN);

	do {
		status = SPI_read_9250(MPU9250_I2C_MST_STATUS);	
	   } while(((status & MPU9250_I2C_SLV4_DONE) == 0) );
	
	readDate= SPI_read_9250(MPU9250_I2C_SLV4_DI);
	return readDate;
}
void MPU9250_Read( uchar readAddr, int *readData, uchar lens )
{
			
		readAddr|=0x80;
		
		SS_L;
		/* 启动数据传输 */
		SPDR =readAddr;
		/* 等待传输结束 */
		while(!(SPSR & (1<<SPIF)));
		
		for(uchar i = 0; i < lens; i++)
		{
		SPDR = 0x00;
		while(!(SPSR & (1<<SPIF)));	
		
		readData[i] = SPDR;
		
		}
		
		SS_H;
}

void MPU9250_getData( int16_t *dataIMU )
{
	int tmpRead[22] = {0};

	
	MPU9250_Read(MPU9250_ACCEL_XOUT_H, tmpRead, 22);


	dataIMU[0] =  (tmpRead[6]<<8)+tmpRead[7];     // Temp
	dataIMU[1] =  (tmpRead[0]<<8)+tmpRead[1];     // Acc.X
	dataIMU[2] =  (tmpRead[2]<<8)+tmpRead[3];     // Acc.Y
	dataIMU[3] =  (tmpRead[4]<<8)+tmpRead[5];     // Acc.Z
	dataIMU[4] =  (tmpRead[8]<<8)+tmpRead[9];     // Gyr.X
	dataIMU[5] = (tmpRead[10]<<8)+tmpRead[11];    // Gyr.Y
	dataIMU[6] = (tmpRead[12]<<8)+tmpRead[13];    // Gyr.Z
	
//if(!(tmpRead[14] & AK8963_STATUS_DRDY) || (tmpRead[14] & AK8963_STATUS_DOR) || (tmpRead[21] & AK8963_STATUS_HOFL))
	//return;

// 	dataIMU[7] = (tmpRead[16]<<8)+tmpRead[15];    // Mag.X
// 	dataIMU[8] = (tmpRead[18]<<8)+tmpRead[17];    // Mag.Y
//     dataIMU[9] = (tmpRead[20]<<8)+tmpRead[19];    // Mag.Z


	dataIMU[7] = ((tmpRead[16]<<8)+tmpRead[15])*AK8963_ASA[0]+ AK8963_offset[0];    // Mag.X
	dataIMU[8] = ((tmpRead[18]<<8)+tmpRead[17])*AK8963_ASA[1]+ AK8963_offset[1];    // Mag.Y
    dataIMU[9] = ((tmpRead[20]<<8)+tmpRead[19])*AK8963_ASA[2]+ AK8963_offset[2];    // Mag.Z


}