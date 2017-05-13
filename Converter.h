#ifndef CONVERTER_H
#define CONVERTER_H

#include<Eigen/Dense>
#include <Eigen/Core>
#include <math.h>

namespace RAIN_IMU
{

class Converter
{
public:
	Converter();
	~Converter();

	Eigen::Quaterniond Converter::quat2euler(const Eigen::Vector3d euler);
	Eigen::Vector3d Converter::euler2quat(const Eigen::Quaterniond q);
	void Converter::quatNormalize(const Eigen::Quaterniond &q);
	Eigen::Quaterniond Converter::quatMultiquat(Eigen::Quaterniond q1, Eigen::Quaterniond q2);
	Eigen::Quaterniond Converter::quatleftproduct(Eigen::Quaterniond q);
};


}// namespace RAIN_IMU


#endif


