#ifndef SYSTEM_H
#define SYSTEM_H

namespace RAIN_IMU
{

class System
{
public:
	System();
	~System();
	void RunEKF();
};

}
#endif
