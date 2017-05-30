#include "AHRSEKF2.h"

#include <iostream>
#include <fstream>
#include <Eigen/core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>

#include "Sensordata.h"
#include "Converter.h"

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

	const unsigned long int ROW = 36, VOL = 4000;
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

	//std::cout.precision(10);
	//for (int i = 0; i < 10;i++)
	//{
	//	std::cout << vSensorData.at(i).Gyro.X << std::endl;
	//}

	std::cout << "finish loading the dataset" << std::endl;
}

SensorData AHRSEKF2::GetSensordatabyID(const long unsigned int &nId, bool flagnorm)
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

// x q.w q.x q.y q.z bx by bz
//   0   1   2   3   4  5  6
void AHRSEKF2::UpdateState(Eigen::Matrix<double, 1, 7> &x, Eigen::Matrix<double, 1, 7> &x_,const SensorData sensordata, const double T)
{
	double Gyro_Xcorrect, Gyro_Ycorrect, Gyro_Zcorrect;
	
	Gyro_Xcorrect = sensordata.Gyro.X - x[4];
	Gyro_Ycorrect = sensordata.Gyro.Y - x[5];
	Gyro_Zcorrect = sensordata.Gyro.Z - x[6];

	x_[0] = x[0] + (-x[1]*Gyro_Xcorrect - x[2]*Gyro_Ycorrect - x[3]*Gyro_Zcorrect) * T/2;
	x_[1] = x[1] + ( x[0]*Gyro_Xcorrect + x[2]*Gyro_Zcorrect - x[3]*Gyro_Ycorrect) * T/2;
	x_[2] = x[2] + ( x[0]*Gyro_Ycorrect - x[1]*Gyro_Zcorrect + x[3]*Gyro_Xcorrect) * T/2;
	x_[3] = x[3] + ( x[0]*Gyro_Zcorrect + x[1]*Gyro_Ycorrect - x[2]*Gyro_Xcorrect) * T/2;

	x_[4] = x[4];
	x_[5] = x[5];
	x_[6] = x[6];

	double norm;
	norm = sqrt(x_[0]*x_[0] + x_[1]*x_[1] + x_[2]*x_[2] + x_[3]*x_[3]);
	x_[0] /= norm;
	x_[1] /= norm;
	x_[2] /= norm;
	x_[3] /= norm;
}

void AHRSEKF2::FillObserveState(Eigen::Matrix<double, 1, 6> &z, const SensorData sensordata)
{
	z[0] = sensordata.Acc.X;
	z[1] = sensordata.Acc.Y;
	z[2] = sensordata.Acc.Z;

	z[3] = sensordata.Mag.X;
	z[4] = sensordata.Mag.Y;
	z[5] = sensordata.Mag.Z;
}

void AHRSEKF2::FillTransiteMatrix(Eigen::Matrix<double, 7, 7> &Ak, const SensorData sensordata, Eigen::Matrix<double, 1, 7> &x, const double T)
{
	Ak = Eigen::MatrixXd::Zero(7, 7);

	Ak.block<4, 4>(0, 0) << 0, -sensordata.Gyro.X, -sensordata.Gyro.Y, -sensordata.Gyro.Z,
							sensordata.Gyro.X, 0, sensordata.Gyro.Z, -sensordata.Gyro.Y,
							sensordata.Gyro.Y, -sensordata.Gyro.Z, 0, sensordata.Gyro.X,
							sensordata.Gyro.Z, sensordata.Gyro.Y, -sensordata.Gyro.X, 0;
	
	Ak.block<4, 3>(0, 4) <<  x[1],  x[2],  x[3],
							-x[0],  x[3], -x[2],
							-x[3], -x[0],  x[1],
							 x[2], -x[1], -x[0];

	Ak = Eigen::MatrixXd::Identity(7, 7) + 0.5 * T * Ak;
}

void  AHRSEKF2::FillObserveMatrix(const Eigen::Matrix<double, 1, 7> &x_, Eigen::Matrix<double, 1, 6> &hk, Eigen::Matrix<double, 6, 7> &Hk, const SensorData sensordata)
{
	Eigen::Matrix<double, 1, 3> hk1, hk2;
	Eigen::Matrix<double, 1, 4> b;
	Eigen::Quaterniond qh, qx,qmag,qxinv;

	// there is no difference below the two formulas
	hk1 << -2*(x_[1]*x_[3] - x_[0]*x_[2]), -2*(x_[2]*x_[3] + x_[0]*x_[1]), -(x_[0]*x_[0] - x_[1]*x_[1] - x_[2]*x_[2] + x_[3]*x_[3]);
	//hk1 << -2*(x_[1]*x_[3] - x_[0]*x_[2]), -2*(x_[2]*x_[3] + x_[0]*x_[1]), -(1 - 2*x_[1]*x_[1] - 2*x_[2]*x_[2]);

	qx.w() = x_[0];
	qx.x() = x_[1];
	qx.y() = x_[2];
	qx.z() = x_[3];

	qmag.w() = 0;
	qmag.x() = sensordata.Mag.X;
	qmag.y() = sensordata.Mag.Y;
	qmag.z() = sensordata.Mag.Z;
	
	qxinv.w() =  qx.w();
	qxinv.x() = -qx.x();
	qxinv.y() = -qx.y();
	qxinv.z() = -qx.z();

	// it need to delete
	qxinv.w() =  qx.w();
	qxinv.x() = -qx.x();
	qxinv.y() = -qx.y();
	qxinv.z() = -qx.z();

	qh = Converter::quatMultiquat(qx,Converter::quatMultiquat(qmag, qxinv));

	b[0] = 0;
	b[1] = sqrt(qh.x()*qh.x() + qh.y()*qh.y());
	b[2] = 0;
	b[3] = qh.z();

	hk2 <<  b[1]*(x_[0]*x_[0] + x_[1]*x_[1] - x_[2]*x_[2] - x_[3]*x_[3]) + 2*b[3]*(x_[1]*x_[3] - x_[0]*x_[2]),
			2*b[1]*(x_[1]*x_[2] - x_[0]*x_[3]) + 2*b[3]*(x_[0]*x_[1] + x_[2]*x_[3]),
			2*b[1]*(x_[0]*x_[2] + x_[1]*x_[3]) + b[3]*(x_[0]*x_[0] - x_[1]*x_[1] - x_[2]*x_[2] + x_[3]*x_[3]);

	hk.block<1, 3>(0, 0) = hk1;
	hk.block<1, 3>(0, 3) = hk2;

	Eigen::Matrix<double, 3, 7> Hk1 = Eigen::MatrixXd::Zero(3, 7); 
	Eigen::Matrix<double, 3, 7> Hk2 = Eigen::MatrixXd::Zero(3, 7);

	Hk1.block<3, 4>(0, 0) <<  x_[2], -x_[3],  x_[0], -x_[1],
							 -x_[1], -x_[0], -x_[3], -x_[2],
							 -x_[0],  x_[1],  x_[2], -x_[3];

	//Hk1.block<3, 4>(0, 0) <<  x_[2], -x_[3],  x_[0], -x_[1],
	//						 -x_[1], -x_[0], -x_[3], -x_[2],
	//						      0, 2*x_[0],  2*x_[2],    0;

	Hk1 = 2 * Hk1;

	Hk2.block<3, 4>(0, 0) << -2*b[3]*x_[2],				   2*b[3]*x_[3],			     -4*b[1]*x_[2] - 2*b[3]*x_[0], -4*b[1]*x_[3] + 2*b[3]*x_[1],
							 -2*b[1]*x_[3] + 2*b[3]*x_[1], 2*b[1]*x_[2] + 2*b[3]*x_[0],  2*b[1]*x_[1] + 2*b[3]*x_[3], -2*b[1]*x_[0] + 2*b[3]*x_[2],
							  2*b[1]*x_[2],				   2*b[1]*x_[3] - 4*b[3]*x_[1],   2*b[1]*x_[0] - 4*b[3]*x_[2],  2*b[1]*x_[1];
	

	//Hk2.block<3, 4>(0, 0) <<  2*b[1]*x_[0] - 2*b[3]*x_[2], 2*b[1]*x_[1] + 2*b[3]*x_[3],	-2*b[1]*x_[2] - 2*b[3]*x_[0], -2*b[1]*x_[3] + 2*b[3]*x_[1],
	//						 -2*b[1]*x_[3] + 2*b[3]*x_[1], 2*b[1]*x_[2] + 2*b[3]*x_[0],  2*b[1]*x_[1] + 2*b[3]*x_[3], -2*b[1]*x_[0] + 2*b[3]*x_[2],
	//						  2*b[1]*x_[2] + 2*b[3]*x_[0], 2*b[1]*x_[3] - 2*b[3]*x_[1],  2*b[1]*x_[0] - 2*b[3]*x_[2],  2*b[1]*x_[1] + 2*b[3]*x_[3];
	Hk2 = 2 * Hk2;

	Hk.block<3, 7>(0, 0) = Hk1;
	Hk.block<3, 7>(3, 0) = Hk2;
}

// Hk
//void AHRSEKF2::FillStateGain(Eigen::Matrix<double, 7, 7>)
//{
//
//}



}