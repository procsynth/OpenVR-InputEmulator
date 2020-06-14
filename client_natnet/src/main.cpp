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

	std::cout << "DaruTrack 1" << std::endl;

	// Trex: 1ere connection à Motive
	NatNetConnection::Get()->Init(/* server: */"127.0.0.1", /* client: */"127.0.0.1", ConnectionType_Multicast);

	int c;
	bool bExit = false;

	while (c = _getch())
	{
		switch (c)
		{
		case 'q':
			bExit = true;
			break;
		default:
			break;
		}
		if (bExit)
			break;
	}


	return 0;
}

