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

void System::RunEKF()
{
	AHRSEKF ekf;
	unsigned int index = 0;
	unsigned int T;
	
	ekf.ReadSensorData();

	Eigen::Vector3d eulerinit = ekf.Initialize(ekf.GetSensordatabyID(0));

	std::cout << eulerinit << std::endl;
	
	Eigen::Quaterniond quatinit = Converter::euler2quat(eulerinit);

	while(1)
	{
		index++;
		Eigen::Matrix<double, 4, 4> OmegaMatrix = Converter::OmegaMatrix(ekf.GetSensordatabyID(index));

		ekf.DiscreteTime(OmegaMatrix,);
	
	}


}

}