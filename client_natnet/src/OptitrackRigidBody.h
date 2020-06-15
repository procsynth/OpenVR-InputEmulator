#pragma once

#include <vrinputemulator.h>
#include <NatNetTypes.h>

class OptitrackRigidBody
{
protected:
	vrinputemulator::VRInputEmulator* inputEmulator;
	const char* serial;
	int rigidbody_id;
	double last_time;
	int virtualId;

	vr::DriverPose_t pose;
	vr::VRControllerState_t state;
public:
	OptitrackRigidBody(const char* serial, int rigidbody_id, vrinputemulator::VRInputEmulator* inputEmulator);
	void ReceivedData(sFrameOfMocapData* data, void* pUserData);
};
