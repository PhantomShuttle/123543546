/*

��Ԫ��ʵ���ˡ�DCM����������[Mayhony��]�������˴���ʧ��
//���ҵĹ�����[Madgwick]��ʡȥ��һ���ο�
// �����㷨 ͨ����BX BZ���ķ���Ԥ�ȶ�������ƴ���ʧ���Ч��ƫ����
//
//
// �û����붨��'halfT��Ϊ����������/ 2�����Լ��˲�������'Kp' ��'Ki'.
//
// ȫ�ֱ���'q0', 'q1', 'q2', 'q3' �Ǵ�����Ƶ���ԪҪ��
//���򡣿����ҵı��棬�ڴ�Ӧ�ó�����ʹ����Ԫ���ĸ�����
//
// �û��������'AHRSupdate()'ÿ���������ںͽ���У׼������('gx', 'gy', 'gz'),
// ���ٶȼ�('ax', 'ay', 'ay') �ʹ�ǿ��('mx', 'my', 'mz') ������. ���ݵ�λ����/��
//���ٶȼƺʹ�ǿ�Ƶĵ�λ�޹ص�ʸ�����й�һ����


*/

#include <math.h>

#define Kp 50.0 //�������� �����ٶȣ��Լ��ٶȼ�/��ǿ��
#define Ki 0.001  // �������� ������ƫ��������ٶ�
float halfT=0.005;//һ��Ĳ�������

double  q0 = 1, q1 = 0, q2 = 0, q3 = 0; // ��Ԫ��Ԫ�ط�λ����
float exInt = 0, eyInt = 0, ezInt = 0; // ���Ż������
float yaw,pitch,roll;
void Quaternions_Count( float ax, float ay, float az,float gx, float gy, float gz, float mx, float my, float mz)
 {
	double norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	///�����������Լ����ظ���������
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
	// ��һ������
	norm = sqrt(ax*ax + ay*ay + az*az);
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;
	norm = sqrt(mx*mx + my*my + mz*mz);
	mx = mx / norm;
	my = my / norm;
	mz = mz / norm;
	//�����Ƽ���ο�����
	hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
	hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
	hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz;
	// �����ʹ����Ĺ��Ʒ���(v and w)
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3); 
	wz =2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
	// ������
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
	//������������������
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
	//���������ǵĲ���
  	gx = gx + Kp*ex + exInt;
   	gy = gy + Kp*ey + eyInt;
   	gz = gz + Kp*ez + ezInt;
 	
	
	
	
	
	// ��Ԫ�����Ϻ͹淶��
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
	//ffff=q3;
	// ��Ԫ���淶��
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