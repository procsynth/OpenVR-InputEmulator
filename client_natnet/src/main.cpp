#include <iostream>
#include <openvr.h>
#include "client_natnet.h"

#include "NatNetConnection.hpp"
#include <vrinputemulator.h>


int main(int argc, const char* argv[]) {

	std::cout << "DaruTrack 4" << std::endl;


	vr::EVRInitError peError;
	vr::VR_Init(&peError, vr::VRApplication_Overlay);
	if (peError != vr::VRInitError_None) {
		std::cout << "OpenVR Error: " << vr::VR_GetVRInitErrorAsEnglishDescription(peError) << std::endl;
		exit(-2);
	}

	vrinputemulator::VRInputEmulator inputEmulator;
	try {
		inputEmulator.connect();
	}
	catch (std::exception& e) {
		std::cout << "Caught exception: " << e.what() << std::endl;
		exit(-3);
	}

	// Trex: 1ere connection à Motive
	NatNetConnection::Get()->Init(/* server: */"127.0.0.1", /* client: */"127.0.0.1", ConnectionType_Multicast);
	NatNetConnection::Get()->AddRigidBody(new OptitrackRigidBody("left_controller", 2, &inputEmulator));
	NatNetConnection::Get()->AddRigidBody(new OptitrackRigidBody("right_controller", 3, &inputEmulator));

	vr::VREvent_t vrevent;
	bool stopLoop = false;
	while (!stopLoop) {
		if (vr::VRSystem()->PollNextEvent(&vrevent, sizeof(vr::VREvent_t))) {
			if (vrevent.eventType == vr::VREvent_Quit) {
				stopLoop = true;
			}
		}
		else {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}

	vr::VR_Shutdown();

	return 0;
}

