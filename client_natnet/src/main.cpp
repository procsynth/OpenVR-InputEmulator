#include <iostream>
#include <openvr.h>
#include "client_natnet.h"

#include "NatNetConnection.hpp"
#include <vrinputemulator.h>


int main(int argc, const char* argv[]) {

	std::cout << "DaruTrack 9" << std::endl;


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

	// Trex: 1ere connection � Motive
	NatNetConnection::Get()->Init(/* server: */"127.0.0.1", /* client: */"127.0.0.1", ConnectionType_Multicast);

	
	NatNetConnection::Get()->AddRigidBody(new OptitrackRigidBody("2_LEFT", 2, &inputEmulator));
	NatNetConnection::Get()->AddRigidBody(new OptitrackRigidBody("3_RIGHT", 3, &inputEmulator));

	NatNetConnection::Get()->AddRigidBody(new OptitrackRigidBody("4_WAIST", 4, &inputEmulator));

	NatNetConnection::Get()->AddRigidBody(new OptitrackRigidBody("5_FOOT_LEFT", 5, &inputEmulator));
	NatNetConnection::Get()->AddRigidBody(new OptitrackRigidBody("6_FOOT_RIGHT", 6, &inputEmulator));


	vr::VREvent_t vrevent;
	bool stopLoop = false;
	while (!stopLoop) {
		if (vr::VRSystem()->PollNextEvent(&vrevent, sizeof(vr::VREvent_t))) {
			if (vrevent.eventType == vr::VREvent_Quit) {
				stopLoop = true;
			}
		}
		else {
			NatNetConnection::Get()->UpdateRigidBodies();
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}

	vr::VR_Shutdown();

	return 0;
}

