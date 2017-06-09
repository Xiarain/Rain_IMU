#include "System.h"
#include <iostream>
#include "Eigen/core"

using namespace RAIN_IMU;
using namespace std;

int main(void)
{
	System imusystem;
	
	cout << "AHRSEKF1 running " << endl;

	imusystem.RunEKF();
}