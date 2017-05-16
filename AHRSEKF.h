#ifndef AHRSEKF_H
#define AHRSEKF_H

#include "Eigen/Core"
#include <math.h>
#include <iostream>
#include <vector>
#include "SensorData.h"

namespace RAIN_IMU
{

class AHRSEKF
{
public:
	const double PI = 3.14159265359;
	const double DEG_RAD = PI / 180;
	const double RAD_DEG = 180 / PI;

	AHRSEKF();
	~AHRSEKF();
	Eigen::Vector3d Initialize(const SensorData &sensordata);
	void ReadSensorData();
	Eigen::Matrix<double, 4, 4> DiscreteTime(const Eigen::Matrix<double, 4, 4> &rotM, const double &T);
	Eigen::Matrix<double, 4, 4> KalmanGain(Eigen::Matrix<double, 4, 4> Hk, Eigen::Matrix<double, 4, 4> Pk);
	Eigen::Matrix<double, 4, 4> Gyro2RotationalMatrix(const SensorData &sensordata);
	SensorData AHRSEKF::GetSensordatabyID(long unsigned int nId);
private:
	std::vector<SensorData> vSensorData;
};

}
#endif

