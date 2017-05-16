#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <Eigen/Core>
#include <iostream>

namespace RAIN_IMU
{

struct sdXYZData
{
	double X;
	double Y;
	double Z;
};

struct sdEuler
{
	double Roll;
	double Pitch;
	double Yaw;
};

class SensorData
{
public:
	SensorData();
	~SensorData();
	void ReadIMURawData();

	long unsigned int nId;
	sdXYZData Acc;
	sdXYZData Gyro;
	sdXYZData Mag;

	sdEuler EulerGroundTruth;
};

}
#endif

