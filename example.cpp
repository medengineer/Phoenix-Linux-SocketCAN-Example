#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

using namespace ctre::phoenix::sensors;

PigeonIMU pigeon(0);

/** simple wrapper for code cleanup */
void sleepApp(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() {

	std::string interface = "can0";

	ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

	while (true) {

		double ypr[3];
		pigeon.GetYawPitchRoll(ypr);
		std::cout << "Yaw: " << ypr[0] << "Pitch: " << ypr[1] << "Roll: " << ypr[2] << std::endl;
		sleepApp(20);
	}

	return 0;
}
