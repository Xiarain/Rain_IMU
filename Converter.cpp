#include "Converter.h"

namespace RAIN_IMU
{
/**
*	Hamilton (q_w,q_v) Right hand
*  Q = q_w+q_x*i+q_y*j+q_z*k	i * j = k
*  vector: q = [q_w q_v] = [q_w q_x q_y q_z]
*	x_g = q * x_l * q_*
*	euler£∫z y x÷· 
*		   yaw pitch roll 
*
**/

Converter::Converter()
{
}


Converter::~Converter()
{
}

/**
 * euler[0] euler[1] euler[2]
 * yaw		pitch	 roll 
 *
**/
Eigen::Quaterniond Converter::quat2euler(const Eigen::Vector3d euler)
{
	double cYaw = cos(euler[0]), sYaw = sin(euler[0]);
	double cPit = cos(euler[1]), sPit = sin(euler[1]);
	double cRol = cos(euler[2]), sRol = sin(euler[2]);
	
	Eigen::Quaterniond q;
	q.w = cRol*cPit*cYaw + sRol*sPit*sYaw;
	q.x = sRol*cPit*cYaw + cRol*sPit*sYaw;
	q.y = cRol*sPit*cYaw + sRol*cPit*sYaw;
	q.z = cRol*cPit*sYaw + sRol*sPit*cYaw;

	return q;
}

Eigen::Vector3d Converter::euler2quat(const Eigen::Quaterniond q)
{
	double yaw, pitch, roll;
	double r1 = 2*(q.w*q.x + q.y*q.z);
	double r2 = 1 - 2*(q.x*q.x + q.y*q.y);
	double r3 = 2*(q.w*q.y - q.z*q.x);
	double r4 = 2*(q.w*q.z + q.x*q.y);
	double r5 = 1 - 2*(q.y*q.y + q.z*q.z);

	roll = atan2(r1,r2);
	pitch = asin(r3);
	yaw = atan2(r4,r5);

	Eigen::Vector3d euler(yaw,pitch,roll);

	return euler;
}
void Converter::quatNormalize(const Eigen::Quaterniond &q)
{
	double norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
	q.w = q.w / norm;
	q.x = q.x / norm;
	q.y = q.y / norm;
	q.z = q.z / norm;
}

Eigen::Quaterniond Converter::quatMultiquat(Eigen::Quaterniond q1, Eigen::Quaterniond q2)
{
	Eigen::Quaterniond q;
	q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
	q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
	q.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
	q.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;

	return q;
}

Eigen::Quaterniond Converter::quatleftproduct(Eigen::Quaterniond q)
{

}

}

