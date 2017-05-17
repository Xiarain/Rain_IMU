#include <iostream>

#include "System.h"
#include "AHRSEKF.h"
#include "SensorData.h"
#include "Converter.h"

namespace RAIN_IMU
{

System::System()
{
}


System::~System()
{
}

int System::RunEKF()
{
	AHRSEKF ekf;
	unsigned int index = 0;
	const double T = 0.02;
	
	ekf.ReadSensorData();

	Eigen::Vector3d eulerinit = ekf.Initialize(ekf.GetSensordatabyID(0));

	std::cout << eulerinit << std::endl;
	
	Eigen::Quaterniond quatinit = Converter::euler2quat(eulerinit);

	Eigen::Quaterniond qProri = quatinit;

	Eigen::Matrix<double, 4, 4> PPriork;

	// initialize the P prior matrix, the initialization value is not very clear
	ekf.initalizePPrior(PPriork);
 
	while(1)
	{
		index++;
		Eigen::Matrix<double, 4, 4> OmegaMatrix = Converter::OmegaMatrix(ekf.GetSensordatabyID(index));

		Eigen::Matrix<double, 4, 4> Ak = ekf.DiscreteTime(OmegaMatrix, T);
	
		qProri = Ak * Converter::quat2vector4d(qProri);

		Eigen::Matrix<double, 4, 4> Qk = (double)1e-6 *  Eigen::MatrixXd::Identity(4, 4);

		PPriork = Ak * PPriork * Ak.transpose() + Qk;

		Eigen::Matrix<double, 3, 4> Hk1 = ekf.JacobianHk1Matrix(qProri);

		Eigen::Matrix<double, 3, 3> Vk = Eigen::MatrixXd::Identity(3, 3);
		Eigen::Matrix<double, 3, 3> R1 = 2 * Eigen::MatrixXd::Identity(3, 3);

		Eigen::Matrix<double, 4, 3> Kk1 = PPriork * Hk1.transpose() * (Hk1 * PPriork * Hk1.transpose() + Vk * R1 * Vk.transpose()).inverse();

		Eigen::Matrix<double, 3, 1> h1 = ekf.Calculateh1Matrix(qProri);

		Eigen::Vector4d vq1 = Kk1 * (Converter::Sensordate2zMatrix(ekf.GetSensordatabyID(index)) - h1);

		vq1[3] = 0;

		Eigen::Quaterniond qpost = Converter::quatplusquat(qProri , Converter::vector4d2quat(vq1));

		Converter::quatNormalize(qpost);

		Eigen::Matrix<double, 4, 4> pPost = (Eigen::MatrixXd::Identity(4, 4) - Kk1 * Hk1) *  PPriork;

		Eigen::Vector3d euler = Converter::quat2euler(qpost);

		std::cout << "=====================================" << std::endl;
		std::cout << euler[0] << " " << euler[1] << " " << euler[2] << std::endl;

		if (index == 10)
			return 0;
	}


}

}