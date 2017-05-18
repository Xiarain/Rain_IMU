#include "AHRSEKF2.h"
#include <iostream>
#include <fstream>
#include <Eigen/core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Sensordata.h"

namespace RAIN_IMU
{

AHRSEKF2::AHRSEKF2()
{
}


AHRSEKF2::~AHRSEKF2()
{
}

void AHRSEKF2::ReadSensorData()
{
	std::cout << "read the sensor raw data" << std::endl;

	const unsigned int ROW = 36, VOL = 1000;
	double d[VOL][ROW];
	std::ifstream in("myfile.txt");
	for (int i = 0; i < VOL; i++)
	{
		for (int j = 0; j < ROW; j++)
		{
			in >> d[i][j];
		}
	}
	in.close();

	SensorData sensordata;
	for (int i = 0; i < VOL; i++)
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

	//std::cout.precision(10);
	//for (int i = 0; i < 100;i++)
	//{
	//	std::cout << vSensorData.at(i).Mag.X << std::endl;
	//}

	std::cout << "finish loading the dataset" << std::endl;
}

SensorData AHRSEKF2::GetSensordatabyID(const long unsigned int &nId)
{
	return vSensorData.at(nId);
}
EulerAngle AHRSEKF2::InitializeEuler(const SensorData &sensordata)
{
	double pitch, roll, yaw;

	pitch = atan2(sensordata.Acc.X, sqrt(sensordata.Acc.Y*sensordata.Acc.Y + sensordata.Acc.Z*sensordata.Acc.Z));
	roll = atan2(-sensordata.Acc.Y, -sensordata.Acc.Z);

	double r1 = -sensordata.Mag.Y*cos(roll) + sensordata.Mag.Z*sin(roll);
	double r2 = sensordata.Mag.X*cos(pitch) + sensordata.Mag.Y*sin(pitch)*sin(roll) + sensordata.Mag.Z*sin(pitch)*cos(roll);

	yaw = atan2(r1, r2) - 8.3 * DEG_RAD;

	return EulerAngle(yaw, pitch, roll);
}

// R 6*6(0.1 单位阵)  Q 7*7(4*4 1e-6 单位阵 3*3 1e-8 单位阵)

void AHRSEKF2::InitializeVarMatrix(Eigen::Matrix<double, 6, 6> &R, Eigen::Matrix<double, 7, 7> &Q)
{
	const double wn_var = 1e-6;  // rot vel var 
	const double wbn_var = 1e-8; // gyro bias change var
	const double an_var = 1e-1;  // acc var 
	const double mn_var = 1e-1;  // mag var 

	Q = Eigen::MatrixXd::Identity(7, 7);
	R = Eigen::MatrixXd::Identity(6, 6);

	Q.block<4, 4>(0, 0) *= wn_var;
	Q.block<3, 3>(4, 4) *= wbn_var;

	R.block<3, 3>(0, 0) *= an_var;
	R.block<3, 3>(3, 3) *= mn_var;

	//std::cout << Q << std::endl;
	//std::cout << R << std::endl;

}

}