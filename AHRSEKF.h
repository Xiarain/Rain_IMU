#ifndef AHRSEKF_H
#define AHRSEKF_H

#include "Eigen/Core"
#include "Eigen/Dense"
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
	void ReadSensorData();
	Eigen::Vector3d Initialize(const SensorData &sensordata);
	Eigen::Matrix<double, 4, 4> DiscreteTime(const Eigen::Matrix<double, 4, 4> &rotM, const double &T);
	Eigen::Matrix<double, 4, 4> KalmanGain(Eigen::Matrix<double, 4, 4> Hk, Eigen::Matrix<double, 4, 4> Pk);
	Eigen::Matrix<double, 4, 4> Gyro2RotationalMatrix(const SensorData &sensordata);
	Eigen::Matrix<double, 3, 4> JacobianHk1Matrix(const Eigen::Quaterniond &q);
	Eigen::Matrix<double, 3, 1> Calculateh1Matrix(const Eigen::Quaterniond &q);
	void initalizePPrior(Eigen::Matrix<double, 4, 4> &PPrior0);
	SensorData GetSensordatabyID(const long unsigned int &nId);

private:
	std::vector<SensorData> vSensorData;
};

}
#endif

