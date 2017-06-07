#ifndef AHRSESKF_H
#define AHRSESKF_H

#include <Eigen/core>
#include <Eigen/Geometry>
#include <vector>

#include "SensorData.h"

namespace RAIN_IMU
{

struct State
{
	Eigen::Quaterniond q;
	Eigen::Vector3d wb;
};

struct ErrorState
{
	Eigen::Vector3d det_theta;
	Eigen::Vector3d det_wb;
};
class AHRSESKF
{
public:
	AHRSESKF();
	~AHRSESKF();

	// just for temporary storage
	std::vector<SensorData> vSensorData;
	static const unsigned long int DataLength = 42000;

	State NominalStatesPrior;
	State NominalStates;
	ErrorState ErrorStates;

	const double PI = 3.14159265358979323846;
	const double DEG_RAD = PI / 180;
	const double RAD_DEG = 180 / PI;

	double Yaw, Roll, Pitch;
	
	void ReadSensorData();
	SensorData GetSensordatabyID(const long unsigned int nId, bool flagnorm);
	Eigen::Vector3d Initialize(const SensorData &sensordata);
	void InitializeVarMatrix(Eigen::Matrix<double, 6, 6> &Q, Eigen::Matrix<double, 6, 6> &R, Eigen::Matrix<double, 6, 6> &PPrior);

	void PredictNominalState(const SensorData sensordata, const SensorData sensordata2, const double T);
	void PredictErrorState(const SensorData sensordata, const double T);

	Eigen::Matrix<double, 6, 6> CalcTransitionMatrix(const SensorData sensordata, const double T);
	void PredictErrorState(const Eigen::Matrix<double, 6, 6> &Fx);
	void EnforcePSD(Eigen::Matrix<double, 6, 6> &P);

	void CalcObservationMatrix(Eigen::Matrix<double, 6, 6> &Hk,Eigen::Matrix<double, 1, 6> &hk, const SensorData sensordata, const double T);
	void ObserveValue(Eigen::Matrix<double, 1, 6> &z, const SensorData sensordatanorm);

	Eigen::Matrix<double, 1, 7> State2Vector(const State &state);
	State Vector2State(const Eigen::Matrix<double, 1, 7> &x);

	Eigen::Quaterniond BuildUpdateQuat(ErrorState errorstate);
private:
	
	std::vector<Eigen::Matrix<double, 1, 3>> EulerAngle; // Yaw Pitch Roll 	

};

}
#endif


