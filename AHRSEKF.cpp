#include "AHRSEKF.h"
#include <iostream>
#include <fstream>
#include <Eigen/core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Converter.h"

namespace RAIN_IMU
{

AHRSEKF::AHRSEKF()
{
}

AHRSEKF::~AHRSEKF()
{
}

/**
 *  euler[0] euler[1] euler[2]
 *  yaw		pitch	 roll
 *
**/
/**
 * 在数据集中27 28 29 分别是角速度计的x、y、z轴数据
 *			  9 10 11 分别是加速度计的x、y、z轴数据
 *			 15 16 17 分别是磁罗盘的x、y、z轴数据
 *			 30 31 32 分别是解算姿态的ground truth的Roll、Pitch、Yaw

**/
void AHRSEKF::ReadSensorData()
{
	std::cout << "read the sensor raw data" << std::endl;

	const unsigned int ROW = 36, VOL = 10000;
	double d[VOL][ROW];
	std::ifstream in("RawData.txt");
	for (int i = 0; i < VOL; i++)
	{
		for (int j = 0; j < ROW; j++)
		{
			in >> d[i][j];
		}
	}	
	in.close();

	SensorData sensordata;
	for (int i = 0; i <	VOL; i++)
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

SensorData AHRSEKF::GetSensordatabyID(const long unsigned int &nId, bool flagnorm)
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

Eigen::Vector3d AHRSEKF::Initialize(const SensorData &sensordata)
{
	double pitch, roll, yaw;

	pitch = atan2(sensordata.Acc.X, sqrt(sensordata.Acc.Y*sensordata.Acc.Y + sensordata.Acc.Z*sensordata.Acc.Z));
	roll = atan2(-sensordata.Acc.Y, -sensordata.Acc.Z);
	
	double r1 = -sensordata.Mag.Y*cos(roll) + sensordata.Mag.Z*sin(roll);
	double r2 = sensordata.Mag.X*cos(pitch) + sensordata.Mag.Y*sin(pitch)*sin(roll) + sensordata.Mag.Z*sin(pitch)*cos(roll);

	yaw = atan2(r1, r2) - 8.3 * DEG_RAD;

	return Eigen::Vector3d(yaw, pitch, roll);
}

Eigen::Matrix<double, 4, 4> AHRSEKF::DiscreteTime(const Eigen::Matrix<double, 4, 4> &rotM, const double &T)
{
	Eigen::Matrix<double, 4, 4> r1 = 0.5 * T * rotM; // 

	return (Eigen::MatrixXd::Identity(4, 4) + r1);

}

Eigen::Matrix<double, 4, 4> AHRSEKF::Gyro2RotationalMatrix(const SensorData &sensordata)
{
	Eigen::Matrix<double, 4, 4> rotM;

	rotM << 0, -sensordata.Acc.X, -sensordata.Acc.Y, -sensordata.Acc.Z,
			sensordata.Acc.X, 0, sensordata.Acc.Z, -sensordata.Acc.Y,
			sensordata.Acc.Y, -sensordata.Acc.Z, 0, sensordata.Acc.X,
			sensordata.Acc.Z, sensordata.Acc.Y, -sensordata.Acc.X, 0;
	
	return rotM;
}

Eigen::Matrix<double, 3, 4> AHRSEKF::JacobianHk1Matrix(const Eigen::Quaterniond &q)
{
	Eigen::Matrix<double, 3, 4> Hk1;

	Hk1 << -2*q.y(),  2*q.z(), -2*q.w(), 2*q.x(),
			2*q.x(),  2*q.w(),  2*q.z(), 2*q.y(),
			2*q.w(), -2*q.x(), -2*q.y(), 2*q.z();

	return Hk1;
}

void AHRSEKF::initalizevarMatrix(Eigen::Matrix<double, 4, 4> &PPrior0)
{
	PPrior0 << 0.1250, 0.0003, 0.0003, 0.0003,
			   0.0003, 0.1250, 0.0003, 0.0003,
			   0.0003, 0.0003, 0.1250, 0.0003,
			   0.0003, 0.0003, 0.0003, 0.1250;
	//PPrior0 << 1, 0, 0, 0,
	//		   0, 1, 0, 0,
	//		   0, 0, 1, 0,
	//		   0, 0, 0, 1;
}

Eigen::Matrix<double, 3, 1> AHRSEKF::Calculateh1Matrix(const Eigen::Quaterniond &q)
{
	Eigen::Matrix<double, 3, 1> h1;
	h1 << 2*q.x()*q.z() - 2*q.w()*q.y(),
		  2*q.w()*q.x() + 2*q.y()*q.z(),
		  q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z();

	return h1;
}

Eigen::Matrix<double, 3, 4> AHRSEKF::CalculateHk2Matrix(const Eigen::Quaterniond &q)
{
	Eigen::Matrix<double, 3, 4> Hk2;

	Hk2 << q.z(),  q.y(),  q.x(),  q.w(),
		   q.w(), -q.x(), -q.y(), -q.z(),
		  -q.x(), -q.w(),  q.z(),  q.y();
	
	Hk2 = 2*Hk2;

	return Hk2;
}
Eigen::Matrix<double, 3, 1> AHRSEKF::Calculateh2Matrix(const Eigen::Quaterniond &q)
{
	Eigen::Matrix<double, 3, 1> h2;

	h2 << 2*q.x()*q.y() + 2*q.w()*q.z(),
		  q.w()*q.w() - q.x()*q.x() - q.y()*q.y() - q.z()*q.z(),
		  2*q.y()*q.z() - 2*q.w()*q.x();

	return h2;
}
void AHRSEKF::CalcObservationMatrix(const Eigen::Quaterniond &q, Eigen::Matrix<double, 3, 4> &Hk2, Eigen::Matrix<double, 3, 1> &hk2, const SensorData sensordatanorm, const double T)
{

	Eigen::Matrix<double, 1, 4> b;
	Eigen::Quaterniond qh,qmag,qxinv;

	qmag.w() = 0;
	qmag.x() = sensordatanorm.Mag.X;
	qmag.y() = sensordatanorm.Mag.Y;
	qmag.z() = sensordatanorm.Mag.Z;

	qxinv.w() =  q.w();
	qxinv.x() = -q.x();
	qxinv.y() = -q.y();
	qxinv.z() = -q.z();

	qh = Converter::quatMultiquat(q,Converter::quatMultiquat(qmag, qxinv));

	b[0] = 0;
	b[1] = sqrt(qh.x()*qh.x() + qh.y()*qh.y());
	b[2] = 0;
	b[3] = qh.z();

	hk2 <<    b[1]*(q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z()) + 2*b[3]*(q.x()*q.z() - q.w()*q.y()),
			2*b[1]*(q.x()*q.y() - q.w()*q.z()) + 2*b[3]*(q.w()*q.x() + q.y()*q.z()),
			2*b[1]*(q.w()*q.y() + q.x()*q.z()) + b[3]*(q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z());

	Hk2 << -2*b[3]*q.y(),				  2*b[3]*q.z(),			       -4*b[1]*q.y() - 2*b[3]*q.w(), -4*b[1]*q.z() + 2*b[3]*q.x(),
		   -2*b[1]*q.z() + 2*b[3]*q.x(),  2*b[1]*q.y() + 2*b[3]*q.w(),  2*b[1]*q.x() + 2*b[3]*q.z(), -2*b[1]*q.w() + 2*b[3]*q.y(),
			2*b[1]*q.y(),				  2*b[1]*q.z() - 4*b[3]*q.x(),  2*b[1]*q.w() - 4*b[3]*q.y(),  2*b[1]*q.x();
}



}