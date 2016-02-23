/*

四元数实现了“DCM过滤器”的[Mayhony等]。采用了磁性失真
//从我的过滤器[Madgwick]它省去了一个参考
// 补偿算法 通量（BX BZ）的方向被预先定义和限制磁性失真的效果偏航轴
//
//
// 用户必须定义'halfT“为（采样周期/ 2），以及滤波器增益'Kp' 和'Ki'.
//
// 全局变量'q0', 'q1', 'q2', 'q3' 是代表估计的四元要素
//方向。看到我的报告，在此应用程序中使用四元数的概述。
//
// 用户必须调用'AHRSupdate()'每个采样周期和解析校准陀螺仪('gx', 'gy', 'gz'),
// 加速度计('ax', 'ay', 'ay') 和磁强计('mx', 'my', 'mz') 的数据. 陀螺单位弧度/秒
//加速度计和磁强计的单位无关的矢量进行归一化。


*/

#include <math.h>

#define Kp 50.0 //比例增益 收敛速度，以加速度计/磁强计
#define Ki 0.001  // 积分增益 陀螺仪偏差的收敛速度
float halfT=0.005;//一半的采样周期

double  q0 = 1, q1 = 0, q2 = 0, q3 = 0; // 四元数元素方位估计
float exInt = 0, eyInt = 0, ezInt = 0; // 缩放积分误差
float yaw,pitch,roll;
void Quaternions_Count( float ax, float ay, float az,float gx, float gy, float gz, float mx, float my, float mz)
 {
	double norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	///辅助变量，以减少重复操作次数
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	// 归一化测量
	norm = sqrt(ax*ax + ay*ay + az*az);
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;
	norm = sqrt(mx*mx + my*my + mz*mz);
	mx = mx / norm;
	my = my / norm;
	mz = mz / norm;
	//磁力计计算参考方向
	hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
	hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
	hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz;
	// 重力和磁链的估计方向(v and w)
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3); 
	wz =2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
	// 误差计算
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
	//积分误差比例积分增益
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
	//调整陀螺仪的测量
  	gx = gx + Kp*ex + exInt;
   	gy = gy + Kp*ey + eyInt;
   	gz = gz + Kp*ez + ezInt;
 	
	
	
	
	
	// 四元数整合和规范化
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
	//ffff=q3;
	// 四元数规范化
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
}

void Quaternions_to_EulerAngles()
{// x_EulerAngles=atan2(2*q0*q3,1-2*q2*q2-2*q3*q3);
	yaw=(atan2(2*q1*q2+2*q0*q3,(2*q0*q0+2*q1*q1)-1))*57.3;
	pitch=(-asin(2 * q1 * q3 - 2 * q0 * q2))*57.3;     
	 roll=(atan2(2 * q2 * q3 + 2 * q0 * q1,(2 * q0 * q0 + 2 * q3 * q3 )-1))*57.3; 
	};