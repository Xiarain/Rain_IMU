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

		Eigen::Matrix<double, 3, 1> h1 = ekf.Calculateh1Matrix(qProri);
		//std::cout << h1 << std::endl;

		Eigen::Vector4d vq1 = Kk1 * (Converter::Sensordate2zMatrix(ekf.GetSensordatabyID(index)) - h1);

		vq1[3] = 0;
		
		// posteriori quaternion
		qPost = Converter::quatplusquat(qProri, Converter::vector4d2quat(vq1));

		Converter::quatNormalize(qPost);

		PPost = (Eigen::MatrixXd::Identity(4, 4) - Kk1 * Hk1) *  PPriork;

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
	const double T = 0.02;
	unsigned int index = 0;
	SensorData sensordatanormk;
	SensorData sensordataunnormk;

	ekf.ReadSensorData();

	Eigen::Matrix<double, 1, 7> x = Eigen::MatrixXd::Zero(1, 7);
	Eigen::Matrix<double, 1, 7> x_ = Eigen::MatrixXd::Zero(1, 7);
	Eigen::Matrix<double, 1, 6> z;

	Eigen::Matrix<double, 7, 7> Pk_ = Eigen::MatrixXd::Zero(7, 7);
	Eigen::Matrix<double, 7, 7> Pk = Eigen::MatrixXd::Identity(7, 7);
	Eigen::Matrix<double, 1, 6> hk;
	Eigen::Matrix<double, 6, 7> Hk = Eigen::MatrixXd::Zero(6, 7);
	Eigen::Matrix<double, 7, 6> Kk = Eigen::MatrixXd::Zero(7, 6);

	Eigen::Matrix<double, 6, 6> R;
	Eigen::Matrix<double, 7, 7> Q;
	Eigen::Matrix<double, 7, 7> Ak;

	Eigen::Vector3d euler;
	Eigen::Quaterniond qfilter;

	EulerAngle eulerinit;
	eulerinit = ekf.InitializeEuler(ekf.GetSensordatabyID(0,false));

	Eigen::Quaterniond qinit = Converter::euler2quat(Eigen::Vector3d(eulerinit.Yaw,eulerinit.Pitch,eulerinit.Roll));

	ekf.InitializeVarMatrix(R,Q);

	x[0] = qinit.w(), x[1] = qinit.x(), x[2] = qinit.y(), x[3] = qinit.z();
	while(1)
	{
		//  the sensordatak that have the normalized, if want to use the unnormalized data, set the false flag in the GetSensordata function  
		sensordatanormk = ekf.GetSensordatabyID(index,true);
		sensordataunnormk = ekf.GetSensordatabyID(index,false);

		ekf.FillObserveState(z,sensordatanormk);

		ekf.UpdateState(x,x_,sensordataunnormk,T);

		ekf.FillTransiteMatrix(Ak, sensordataunnormk, x, T);

		Pk_ = Ak * Pk * Ak.transpose() + Q;

		ekf.FillObserveMatrix(x_,hk,Hk,sensordatanormk);

		Kk = Pk_ * Hk.transpose() * (Hk * Pk_ * Hk.transpose() + R).inverse();

		qfilter = Converter::vector4d2quat(x.block<1,4>(0, 0));

		std::cout.precision(10);
		euler = Converter::quat2euler(qfilter);// yaw pitch roll
		euler[0] = euler[0] * ekf.RAD_DEG  - 8.3;
		euler[1] = euler[1] * ekf.RAD_DEG;
		euler[2] = euler[2]*ekf.RAD_DEG;
		std::cout << "euler:" << euler.transpose() << std::endl;
		sensordatanormk.EulerGroundTruth.Yaw *= ekf.RAD_DEG;
		sensordatanormk.EulerGroundTruth.Pitch *= ekf.RAD_DEG;
		sensordatanormk.EulerGroundTruth.Roll *= ekf.RAD_DEG;
		std::cout << "truth"  << sensordatanormk.EulerGroundTruth.Yaw << " " << sensordatanormk.EulerGroundTruth.Pitch << " "  << sensordatanormk.EulerGroundTruth.Roll << std::endl; 

		x = (x_.transpose() + Kk * (z - hk).transpose()).transpose();

		Converter::Normalize(x);

		Pk = (Eigen::MatrixXd::Identity(7, 7) - Kk * Hk) * Pk_;

		index++;
		if (index == 1000)
			return 0;
	}

	
}


}