#include <iostream>

#include "System.h"
#include "AHRSEKF.h"
#include "AHRSEKF2.h"
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

	//std::cout << eulerinit << std::endl;
	
	Eigen::Quaterniond quatinit = Converter::euler2quat(eulerinit);

	Eigen::Quaterniond qProri;
	Eigen::Quaterniond qPost = quatinit;
	Eigen::Matrix<double, 4, 4> PPriork;
	Eigen::Matrix<double, 4, 4> PPost;

	// initialize the P prior matrix, the initialization value is not very clear
	ekf.initalizevarMatrix(PPost);
 
	while(1)
	{
		index++;
		Eigen::Matrix<double, 4, 4> OmegaMatrix = Converter::OmegaMatrix(ekf.GetSensordatabyID(index));

		Eigen::Matrix<double, 4, 4> Ak = ekf.DiscreteTime(OmegaMatrix, T);

		//std::cout << Ak << std::endl;
	
		qProri = Ak * Converter::quat2vector4d(qPost);

		Eigen::Matrix<double, 4, 4> Qk = (double)1e-6 *  Eigen::MatrixXd::Identity(4, 4);

		PPriork = Ak * PPost * Ak.transpose() + Qk;

		Eigen::Matrix<double, 3, 4> Hk1 = ekf.JacobianHk1Matrix(qProri); // get from the quaternion

		// std::cout << Hk1 << std::endl;

		Eigen::Matrix<double, 3, 3> Vk = Eigen::MatrixXd::Identity(3, 3);
		Eigen::Matrix<double, 3, 3> R1 = 0.1 * Eigen::MatrixXd::Identity(3, 3);

//		Eigen::Matrix<double, 4, 3> Kk1 = PPriork * Hk1.transpose() * (Hk1 * PPriork * Hk1.transpose() + Vk * R1 * Vk.transpose()).inverse();
		Eigen::Matrix<double, 3, 3> temp = Hk1 * PPriork * Hk1.transpose() + Vk * R1 * Vk.transpose();
		Eigen::Matrix<double, 3, 3> temp2 = temp.inverse();
		Eigen::Matrix<double, 4, 3> Kk1 = PPriork * Hk1.transpose() * temp2;

		//std::cout << Kk1 << std::endl;
		//Kk1 = Eigen::MatrixXd::Identity(4, 3);	

		Eigen::Matrix<double, 3, 1> h1 = ekf.Calculateh1Matrix(qProri);
		//std::cout << h1 << std::endl;

		Eigen::Vector4d vq1 = Kk1 * (Converter::Sensordate2zMatrix(ekf.GetSensordatabyID(index)) - h1);

		vq1[3] = 0;
		
		// posteriori quaternion
		qPost = Converter::quatplusquat(qProri, Converter::vector4d2quat(vq1));

		Converter::quatNormalize(qPost);

		PPost = (Eigen::MatrixXd::Identity(4, 4) - Kk1 * Hk1) *  PPriork;

		//std::cout << PPost << std::endl;

		Eigen::Vector3d euler = Converter::quat2euler(qPost);

		std::cout << "=====================================" << std::endl;
		std::cout << euler[0] << " " << euler[1] << " " << euler[2] << std::endl;

		if (index == 30)
			return 0;
	}


}

int System::RunEKF2()
{
	AHRSEKF2 ekf;

	ekf.ReadSensorData();

	Eigen::Matrix<double, 1, 7> x;
	Eigen::Matrix<double, 7, 7> Pk;

	Eigen::Matrix<double, 6, 6> R;
	Eigen::Matrix<double, 7, 7> Q;

	EulerAngle eulerinit;
	eulerinit = ekf.InitializeEuler(ekf.GetSensordatabyID(0));

	Eigen::Quaterniond qinit = Converter::euler2quat(Eigen::Vector3d(eulerinit.Yaw,eulerinit.Pitch,eulerinit.Roll));

	ekf.InitializeVarMatrix(R,Q);

	while(1)
	{
		
		return 0;
	}

	
}


}