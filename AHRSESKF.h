#ifndef AHRSESKF_H
#define AHRSESKF_H

#include <Eigen/core>
#include <Eigen/Geometry>
#include <vector>

#include "SensorData.h"

namespace RAIN_IMU
{

struct state
{
	Eigen::Quaterniond q;
	Eigen::Vector3d wb;
};
class AHRSESKF
{
public:
	AHRSESKF();
	~AHRSESKF();
	
	void ReadSensorData();
	SensorData GetSensordatabyID(const long unsigned int &nId, bool flagnorm);
	Eigen::Vector3d Initialize(const SensorData &sensordata);
	void InitializeVarMatrix(Eigen::Matrix<double, 6, 6> &Q, Eigen::Matrix<double, 6, 6> &R, Eigen::Matrix<double, 6, 6> &PPrior);

	void PredictNominalState(const SensorData sensordata, const double T);
	void PredictErrorState(const SensorData sensordata, const double T);

	Eigen::Matrix<double, 6, 6> CalcTransitionMatrix(const SensorData sensordata, const double T);

	state NominalStatesPrior;
	state NominalStates;
	state ErrorStates;

private:
	static const unsigned long int DataLength = 100;
	const double PI = 3.14159265358979323846;
	const double DEG_RAD = PI / 180;
	const double RAD_DEG = 180 / PI;

	double Yaw, Roll, Pitch;

	// just for temporary storage
	std::vector<SensorData> vSensorData;
	std::vector<Eigen::Matrix<double, 1, 3>> EulerAngle; // Yaw Pitch Roll 	

};

}
#endif


