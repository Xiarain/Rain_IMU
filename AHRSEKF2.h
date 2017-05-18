#ifndef AHRSEKF2_H
#define AHRSEKF2_H

#include <vector>
#include "SensorData.h"

namespace RAIN_IMU
{

class EulerAngle
{


public:
	double Yaw;
	double Pitch;
	double Roll;

	EulerAngle(double yaw, double pitch, double roll)
	{
		Yaw = yaw;
		Pitch = pitch;
		Roll = roll;
	}
	EulerAngle()
	{
		Yaw = 0;
		Pitch = 0;
		Roll = 0;
	}
};
class AHRSEKF2
{
public:

	const double PI = 3.14159265359;
	const double DEG_RAD = PI / 180;
	const double RAD_DEG = 180 / PI;

	AHRSEKF2();
	~AHRSEKF2();
	void ReadSensorData();
	SensorData GetSensordatabyID(const long unsigned int &nId);
	EulerAngle InitializeEuler(const SensorData &sensordata);
	void InitializeVarMatrix(Eigen::Matrix<double, 6, 6> &Q, Eigen::Matrix<double, 7, 7> &R);

private:
	std::vector<SensorData> vSensorData;
};

}
#endif
