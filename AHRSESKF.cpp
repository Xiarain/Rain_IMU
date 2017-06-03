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
	// it is very important, the MatrixXd::Identity can not initialize the whole matrix.
	PPrior = Eigen::MatrixXd::Zero(6, 6);
	Q = Eigen::MatrixXd::Zero(6, 6);
	R = Eigen::MatrixXd::Zero(6, 6);

	const double wn_var = 1e-5;
	const double wbn_var = 1e-9;
	const double an_var = 1e-3;
	const double mn_var = 1e-4;
	const double q_var = 1e-5;
	const double wb_var = 1e-7;

	Q.block<3, 3>(0, 0) = wn_var * Eigen::MatrixXd::Identity(3, 3);
	Q.block<3, 3>(3, 3) = wbn_var * Eigen::MatrixXd::Identity(3, 3);

	R.block<3, 3>(0, 0) = an_var * Eigen::MatrixXd::Identity(3, 3);
	R.block<3, 3>(3, 3) = mn_var * Eigen::MatrixXd::Identity(3, 3);
	
	PPrior.block<3, 3>(0, 0) = q_var * Eigen::MatrixXd::Identity(3, 3);
	PPrior.block<3, 3>(3, 3) = wb_var * Eigen::MatrixXd::Identity(3, 3);
}

void AHRSESKF::PredictNominalState(const SensorData sensordata, const double T)
{
	Eigen::Quaterniond qw;

	qw.w() = 1; // this value need to deep consider? TODO
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

	Eigen::Matrix<double, 3, 3> R = Eigen::MatrixXd::Identity(3, 3) + sin(theta)*omegaMatrix + omegaMatrix.transpose()*omegaMatrix*(1 - cos(theta));

	// why need to get R matrix transpose??? TODO
	Fx.block<3, 3>(0, 0) = R.transpose();
	Fx.block<3, 3>(0, 3) = -T*Eigen::MatrixXd::Identity(3, 3);
	Fx.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(3, 3);

	return Fx;
}

void AHRSESKF::PredictErrorState(const Eigen::Matrix<double, 6, 6> &Fx)
{
	Eigen::Matrix<double, 1, 6> det_x;
	det_x.block<1,3>(0, 0) = ErrorStates.det_theta;
	det_x.block<1,3>(0, 3) = ErrorStates.det_wb;
	det_x = Fx * det_x.transpose();
		
	ErrorStates.det_theta = det_x.block<1, 3>(0, 0);
	ErrorStates.det_wb = det_x.block<1, 3>(0, 3);
}

void AHRSESKF::EnforcePSD(Eigen::Matrix<double, 6, 6> &P)
{
	int i, j;

	for (i = 0; i < P.rows(); i++)
	{
		for (j = 0; j < P.cols(); j++)
		{
			if (i == j)
			{
				P(i,j) = abs(P(i,j));
			}
			else 
			{
				double meanvalue = 0.5*(P(i,j) + P(j,i));
				P(i,j) = meanvalue;
				P(j,i) = meanvalue;
			}
		}
	}
}

void AHRSESKF::CalcObservationMatrix(Eigen::Matrix<double, 6, 6> &Hk,Eigen::Matrix<double, 1, 6> &hk, const SensorData sensordata, const double T)
{
	Eigen::Quaterniond mk, hmk,q,qinv;
	Eigen::Vector4d b;

	// this assignment just for the code look more easy.
	q = NominalStatesPrior.q;

	mk.w() = 0;
	mk.x() = sensordata.Mag.X;
	mk.y() = sensordata.Mag.Y;
	mk.z() = sensordata.Mag.Z;

	qinv.w() =  q.w();
	qinv.x() = -q.x();
	qinv.y() = -q.y();
	qinv.z() = -q.z();

	//hmk = NominalStatesPrior.q * mk * qinv;
	hmk = Converter::quatMultiquat(q,Converter::quatMultiquat(mk, qinv));

	b[1] = sqrt(hmk.x()*hmk.x() + hmk.y()*hmk.y());
	b[3] = hmk.z();

	// Hk
	Eigen::Matrix<double, 3, 4> Hk1;
	Eigen::Matrix<double, 3, 4> Hk2;
	Eigen::Matrix<double, 6, 7> Hx;
	Hk1 <<  2*q.y(), -2*q.z(),  2*q.w(), -2*q.x(),
		   -2*q.x(), -2*q.w(), -2*q.z(), -2*q.y(),
			0, 4*q.x(),  4*q.y(),    0;
	
	Hk2 << -2*b[3]*q.y(),				   2*b[3]*q.z(),			     -4*b[1]*q.y() - 2*b[3]*q.w(), -4*b[1]*q.z() + 2*b[3]*q.x(),
		   -2*b[1]*q.z() + 2*b[3]*q.x(),   2*b[1]*q.y() + 2*b[3]*q.w(),   2*b[1]*q.x() + 2*b[3]*q.z(), -2*b[1]*q.w() + 2*b[3]*q.y(),
			2*b[1]*q.y(),				   2*b[1]*q.z() - 4*b[3]*q.x(),   2*b[1]*q.w() - 4*b[3]*q.y(),  2*b[1]*q.x();

	Hx = Eigen::MatrixXd::Zero(6, 7);

	Hx.block<3, 4>(0, 0) = Hk1;
	Hx.block<3, 4>(3, 0) = Hk2;

	Eigen::Matrix<double, 4, 3> Qdettheta;
	Eigen::Matrix<double, 7, 6> Xdetx = Eigen::MatrixXd::Zero(7, 6);

	Qdettheta << -q.x(), -q.y(), -q.z(),
				  q.w(), -q.z(),  q.y(),
				  q.z(),  q.w(), -q.x(),
				 -q.y(),  q.x(),  q.w();
	
	Qdettheta = 0.5 * Qdettheta;

	Xdetx.block<4, 3>(0, 0) = Qdettheta;
	Xdetx.block<3, 3>(4, 3) = Eigen::MatrixXd::Identity(3, 3);

	Hk = Hx*Xdetx;

	// hk
	Eigen::Matrix<double, 1, 3> hk1, hk2;

	
	hk1 << 2*(q.x()*q.z() - q.w()*q.y()), 2*(q.y()*q.z() + q.w()*q.x()), (q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z());

	hk2 <<    b[1]*(q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z()) + 2*b[3]*(q.x()*q.z() - q.w()*q.y()),
			2*b[1]*(q.x()*q.y() - q.w()*q.z()) + 2*b[3]*(q.w()*q.x() + q.y()*q.z()),
			2*b[1]*(q.w()*q.y() + q.x()*q.z()) + b[3]*(q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z());
	
	hk.block<1, 3>(0, 0) = -hk1;
	hk.block<1, 3>(0, 3) = -hk2;
}

void AHRSESKF::ObserveValue(Eigen::Matrix<double, 1, 6> &z, const SensorData sensordatanorm)
{
	z[0] = sensordatanorm.Acc.X;
	z[1] = sensordatanorm.Acc.Y;
	z[2] = sensordatanorm.Acc.Z;

	z[3] = sensordatanorm.Mag.X;
	z[4] = sensordatanorm.Mag.Y;
	z[5] = sensordatanorm.Mag.Z;
}

Eigen::Matrix<double, 1, 7> AHRSESKF::State2Vector(const State &state)
{
	Eigen::Matrix<double, 1, 7> vx;

	vx[0] = state.q.w();
	vx[1] = state.q.x();
	vx[2] = state.q.y();
	vx[3] = state.q.z();

	vx[4]= state.wb[0];
	vx[5]= state.wb[1];
	vx[6]= state.wb[2];

	return vx;
}

State AHRSESKF::Vector2State(const Eigen::Matrix<double, 1, 7> &x)
{
	State state;

	state.q.w() = x[0];
	state.q.x() = x[1];
	state.q.y() = x[2];
	state.q.z() = x[3];

	state.wb[0] = x[4];
	state.wb[1] = x[5];
	state.wb[2] = x[6];
	
	return state;
}

Eigen::Quaterniond AHRSESKF::BuildUpdateQuat(ErrorState errorstate)
{
	Eigen::Vector3d deltaq;
	Eigen::Vector4d vquat;
	Eigen::Quaterniond quat;
	double norm;

	deltaq = 0.5 * errorstate.det_theta;

	norm = deltaq.transpose()*deltaq; 

	if (norm > 1)
	{
		vquat[0] = 1;
		vquat[1] = deltaq[0];
		vquat[2] = deltaq[1];
		vquat[3] = deltaq[2];
		vquat = vquat / (sqrt(1 + norm));
	}
	else
	{
		vquat[0] = sqrt(1 - norm);
		vquat[1] = deltaq[0];
		vquat[2] = deltaq[1];
		vquat[3] = deltaq[2];
	}

	quat.w() = vquat[0];
	quat.x() = vquat[1];
	quat.y() = vquat[2];
	quat.z() = vquat[3];

	return quat;
}




}