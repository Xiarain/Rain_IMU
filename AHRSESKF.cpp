#include "AHRSESKF.h"

#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "Converter.h"

namespace RAIN_IMU
{
/**
 *	local angular error
 * 
**/

AHRSESKF::AHRSESKF()
{

}
AHRSESKF::~AHRSESKF()
{

}

void AHRSESKF::ReadSensorData()
{
	std::cout << "read the sensor raw data" << std::endl;

	const unsigned long int ROW = 36, VOL = DataLength;
	double d[VOL][ROW];
	std::ifstream in("myfile.txt");
	for (unsigned long int i = 0; i < VOL; i++)
	{
		for (int j = 0; j < ROW; j++)
		{
			in >> d[i][j];
		}
	}
	in.close();

	SensorData sensordata;
	for (unsigned long int i = 0; i < VOL; i++)
	{
		sensordata.nId = i;

		sensordata.Acc.X = d[i][8];
		sensordata.Acc.Y = d[i][9];
		sensordata.Acc.Z = d[i][10];

		sensordata.Gyro.X = d[i][26];
		sensordata.Gyro.Y = d[i][27];
		sensordata.Gyro.Z = d[i][28];

		sensordata.Mag.X = d[i][14];
		sensordata.Mag.Y = d[i][15];
		sensordata.Mag.Z = d[i][16];

		sensordata.EulerGroundTruth.Roll = d[i][29];
		sensordata.EulerGroundTruth.Pitch = d[i][30];
		sensordata.EulerGroundTruth.Yaw = d[i][31];

		vSensorData.push_back(sensordata);
	}

	std::cout << "finish loading the dataset" << std::endl;
}

SensorData AHRSESKF::GetSensordatabyID(const long unsigned int &nId, bool flagnorm)
{
	SensorData sensordata = vSensorData.at(nId);

	if (flagnorm == true)
	{
		double norm = std::sqrt(sensordata.Acc.X*sensordata.Acc.X + sensordata.Acc.Y*sensordata.Acc.Y + sensordata.Acc.Z*sensordata.Acc.Z);
		sensordata.Acc.X /= norm;
		sensordata.Acc.Y /= norm;
		sensordata.Acc.Z /= norm;

		norm = std::sqrt(sensordata.Gyro.X*sensordata.Gyro.X + sensordata.Gyro.Y*sensordata.Gyro.Y + sensordata.Gyro.Z*sensordata.Gyro.Z);
		sensordata.Gyro.X /= norm;
		sensordata.Gyro.Y /= norm;
		sensordata.Gyro.Z /= norm;

		norm = std::sqrt(sensordata.Mag.X*sensordata.Mag.X + sensordata.Mag.Y*sensordata.Mag.Y + sensordata.Mag.Z*sensordata.Mag.Z);
		sensordata.Mag.X /= norm;
		sensordata.Mag.Y /= norm;
		sensordata.Mag.Z /= norm;
	}
	else;

	return sensordata;
}

Eigen::Vector3d AHRSESKF::Initialize(const SensorData &sensordata)
{
	double pitch, roll, yaw;

	pitch = atan2(sensordata.Acc.X, sqrt(sensordata.Acc.Y*sensordata.Acc.Y + sensordata.Acc.Z*sensordata.Acc.Z));
	roll = atan2(-sensordata.Acc.Y, -sensordata.Acc.Z);

	double r1 = -sensordata.Mag.Y*cos(roll) + sensordata.Mag.Z*sin(roll);
	double r2 = sensordata.Mag.X*cos(pitch) + sensordata.Mag.Y*sin(pitch)*sin(roll) + sensordata.Mag.Z*sin(pitch)*cos(roll);

	yaw = atan2(r1, r2) - 8.3 * DEG_RAD;

	return Eigen::Vector3d(yaw, pitch, roll);
}

void AHRSESKF::InitializeVarMatrix(Eigen::Matrix<double, 6, 6> &Q, Eigen::Matrix<double, 6, 6> &R, Eigen::Matrix<double, 6, 6> &PPrior)
{
	const double wn_var = 1e-5;
	const double wbn_var = 1e-9;
	const double an_var = 1e-3;
	const double mn_var = 1e-4;
	const double q_var = 1e-5;
	const double wb_var = 1e-7;

	Q.block<3, 3>(0, 0) = wn_var * Eigen::MatrixXd::Identity(3, 3);
	Q.block<3, 3>(3, 3) = wn_var * Eigen::MatrixXd::Identity(3, 3);

	R.block<3, 3>(0, 0) = an_var * Eigen::MatrixXd::Identity(3, 3);
	R.block<3, 3>(3, 3) = mn_var * Eigen::MatrixXd::Identity(3, 3);
	
	PPrior.block<3, 3>(0, 0) = q_var * Eigen::MatrixXd::Identity(3, 3);
	PPrior.block<3, 3>(3, 3) = wb_var * Eigen::MatrixXd::Identity(3, 3);
}

void AHRSESKF::PredictNominalState(const SensorData sensordata, const double T)
{
	Eigen::Quaterniond qw;

	qw.w() = 0; // this value need to deep consider? TODO
	qw.x() = T*(sensordata.Gyro.X - NominalStates.wb[0]);
	qw.y() = T*(sensordata.Gyro.Y - NominalStates.wb[1]);
	qw.z() = T*(sensordata.Gyro.Z - NominalStates.wb[2]);

	NominalStatesPrior.q = NominalStates.q * qw;

	double norm;

	norm = sqrt(NominalStatesPrior.q.w()*NominalStatesPrior.q.w() + NominalStatesPrior.q.x()*NominalStatesPrior.q.x() +
				NominalStatesPrior.q.y()*NominalStatesPrior.q.y() + NominalStatesPrior.q.z()*NominalStatesPrior.q.z());
	
	NominalStatesPrior.q.w() /= norm;
	NominalStatesPrior.q.x() /= norm;
	NominalStatesPrior.q.y() /= norm;
	NominalStatesPrior.q.z() /= norm;

	NominalStatesPrior.wb = NominalStates.wb;
}

Eigen::Matrix<double, 6, 6> AHRSESKF::CalcTransitionMatrix(const SensorData sensordata, const double T)
{
	Eigen::Matrix<double, 6, 6> Fx = Eigen::MatrixXd::Zero(6, 6);
	Eigen::Vector3d omega(sensordata.Gyro.X - NominalStates.wb[0], sensordata.Gyro.Y - NominalStates.wb[1], sensordata.Gyro.Z - NominalStates.wb[2]);

	omega = omega * T;
	// With the Rodrigues'formula, get the rorations matrix. reference: joan sola 3D algebra for vision system in robotics P14.
	double theta = sqrt(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]);
	
	Eigen::Vector3d u(omega[0]/theta, omega[1]/theta, omega[2]/theta);

	Eigen::Matrix<double, 3, 3> omegaMatrix = Converter::CrossProductMatrix(u);

	Eigen::Matrix<double, 3, 3> R = Eigen::MatrixXd::Identity(3, 3) + sin(theta)*omegaMatrix + (1 - cos(theta))*omegaMatrix*omegaMatrix;

	Fx.block<3, 3>(0, 0) = R;
	Fx.block<3, 3>(0, 3) = -T*Eigen::MatrixXd::Identity(3, 3);
	Fx.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(3, 3);

	return Fx;
}

void AHRSESKF::PredictErrorState(const SensorData sensordata, const double T)
{

}



}