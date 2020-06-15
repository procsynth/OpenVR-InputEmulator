#include <iostream>
#include <openvr.h>
#include "client_natnet.h"

#include "NatNetConnection.hpp"
#include <vrinputemulator.h>


void printHelp(int argc, const char* argv[]) {
	std::cout << "Usage: client_commandline.exe <command> ..." << std::endl << std::endl
		<< "Available commands (enter \"<command> help\" for help):" << std::endl << std::endl
		<< "  listdevices\t\t\tLists all openvr devices" << std::endl
		<< "  buttonevent\t\t\tSends button event" << std::endl
		<< "  axisevent\t\t\tSends axis event" << std::endl
		<< "  proximitysensor\t\tSends proximity sensor event" << std::endl
		<< "  getdeviceproperty\t\tReturns a device property" << std::endl
		<< "  listvirtual\t\t\tLists all virtual devices" << std::endl
		<< "  addcontroller\t\t\tCreates a new virtual controller" << std::endl
		<< "  publishdevice\t\t\tAdds a virtual controller to openvr" << std::endl
		<< "  setdeviceproperty\t\tSets a device property" << std::endl
		<< "  removedeviceproperty\t\tRemoves a device property" << std::endl
		<< "  setdeviceconnection\t\tSets the connection state of a virtual device" << std::endl
		<< "  setdeviceposition\t\tSets the position of a virtual device" << std::endl
		<< "  setdevicerotation\t\tSets the rotation of a virtual device" << std::endl
		<< "  deviceoffsets\t\t\tConfigure the device translation/rotation offsets" << std::endl
		<< "  benchmarkipc\t\t\tipc benchmarks" << std::endl;
}


int main(int argc, const char* argv[]) {

	std::cout << "DaruTrack 2" << std::endl;


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
	NatNetConnection::Get()->AddRigidBody(new OptitrackRigidBody("left_controller", 1, &inputEmulator));

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

