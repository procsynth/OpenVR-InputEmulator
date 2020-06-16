#include "OptitrackRigidBody.h"
#include "openvr_math.h"
#include <Windows.h>

#include <iostream>

OptitrackRigidBody::OptitrackRigidBody(const char* serial, int rigidbody_id, vrinputemulator::VRInputEmulator* inputEmulator) :
	serial(serial), rigidbody_id(rigidbody_id), inputEmulator(inputEmulator)
{

	memset(&pose, 0, sizeof(vr::DriverPose_t));
	memset(&state, 0, sizeof(vr::VRControllerState_t));
	pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };

	last_time = (double)std::chrono::duration_cast <std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void OptitrackRigidBody::ReceivedData(sFrameOfMocapData* data, void* pUserData)
{
	// pas sur de ce code
	auto now = (double)std::chrono::duration_cast <std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	pose.poseTimeOffset = (last_time - now) / 1000;
	auto framerate = -1 / pose.poseTimeOffset;
	last_time = now;
	//pose.poseTimeOffset = 0;

	pose.willDriftInYaw = false;
	pose.shouldApplyHeadModel = false;
	//pose.qDriverFromHeadRotation.w = pose.qWorldFromDriverRotation.w = pose.qRotation.w = 1.0;
	pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };

	pose.deviceIsConnected = true;


	int i = 0;
	for (i = 0; i < data->nRigidBodies; i++)
	{
		if (data->RigidBodies[i].ID == this->rigidbody_id) {
			break;
		}
	}
	sRigidBodyData r_data = data->RigidBodies[i];


	//std::cout << pose.vecVelocity[0] << "-" << pose.vecVelocity[1] << "-" << pose.vecVelocity[2] << std::endl << std::endl;

	vr::TrackedDevicePose_t hmdPose;
	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0, &hmdPose, 1);

	if (hmdPose.bPoseIsValid) {

		pose.vecVelocity[0] = (((double)r_data.x - hmdPose.mDeviceToAbsoluteTracking.m[0][3]) - (double)pose.vecPosition[0]) * framerate;
		pose.vecVelocity[1] = (((double)r_data.y - hmdPose.mDeviceToAbsoluteTracking.m[1][3]) - (double)pose.vecPosition[1]) * framerate;
		pose.vecVelocity[2] = (((double)r_data.z - hmdPose.mDeviceToAbsoluteTracking.m[2][3]) - (double)pose.vecPosition[2]) * framerate;

		pose.vecPosition[0] = (double)r_data.x - hmdPose.mDeviceToAbsoluteTracking.m[0][3];
		pose.vecPosition[1] = (double)r_data.y - hmdPose.mDeviceToAbsoluteTracking.m[1][3];
		pose.vecPosition[2] = (double)r_data.z - hmdPose.mDeviceToAbsoluteTracking.m[2][3];


		//std::cout << "POS: " << pose.vecPosition[0] << "-" << pose.vecPosition[1] << "-" << pose.vecPosition[2] << std::endl << std::endl;
		//std::cout << "VEL: " << pose.vecVelocity[0] << "-" << pose.vecVelocity[1] << "-" << pose.vecVelocity[2] << std::endl << std::endl;

		//pose.qWorldFromDriverRotation = vrmath::quaternionFromRotationMatrix(hmdPose.mDeviceToAbsoluteTracking);
		pose.vecWorldFromDriverTranslation[0] = hmdPose.mDeviceToAbsoluteTracking.m[0][3];
		pose.vecWorldFromDriverTranslation[1] = hmdPose.mDeviceToAbsoluteTracking.m[1][3];
		pose.vecWorldFromDriverTranslation[2] = hmdPose.mDeviceToAbsoluteTracking.m[2][3];
	}

	pose.qRotation.w = r_data.qw;
	pose.qRotation.x = r_data.qx;
	pose.qRotation.y = r_data.qy;
	pose.qRotation.z = r_data.qz;

	if (!readyFlag) {
		virtualId = inputEmulator->addVirtualDevice(vrinputemulator::VirtualDeviceType::TrackedController, serial, false);
		if (rigidbody_id < 4) {
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_DeviceClass_Int32, (int32_t)vr::TrackedDeviceClass_Controller);
		}
		else {
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_DeviceClass_Int32, (int32_t)vr::TrackedDeviceClass_GenericTracker);
		}
		//inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_SupportedButtons_Uint64, (uint64_t)
		//	vr::ButtonMaskFromId(vr::k_EButton_System) |
		//	vr::ButtonMaskFromId(vr::k_EButton_ApplicationMenu) |
		//	vr::ButtonMaskFromId(vr::k_EButton_Grip) |
		//	vr::ButtonMaskFromId(vr::k_EButton_Axis0) |
		//	vr::ButtonMaskFromId(vr::k_EButton_Axis1)
		//);
		//inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_Axis0Type_Int32, (int32_t)vr::k_eControllerAxis_Joystick);
		//inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_Axis1Type_Int32, (int32_t)vr::k_eControllerAxis_Trigger);
		inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_HardwareRevision_Uint64, (uint64_t)666);
		inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_FirmwareVersion_Uint64, (uint64_t)666);
		inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_RenderModelName_String, std::string("generic_controller"));
		inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_ManufacturerName_String, std::string("Leap Motion"));
		inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_ModelNumber_String, std::string("Leap Motion Controller"));
		inputEmulator->publishVirtualDevice(virtualId);

		readyFlag = true;
	}


	//if (rigidbody_id == 2) {
	//	if (GetAsyncKeyState(32 /* space */) != 0) {
	//		std::cout << "space" << std::endl << std::endl;
	//		state.ulButtonPressed |= vr::ButtonMaskFromId(vr::k_EButton_Axis0);
	//		state.ulButtonPressed |= vr::ButtonMaskFromId(vr::k_EButton_Axis1);
	//		state.ulButtonPressed |= vr::ButtonMaskFromId(vr::k_EButton_Axis2);
	//	}
	//	else {
	//		state.ulButtonPressed &= ~vr::ButtonMaskFromId(vr::k_EButton_Axis0);
	//		state.ulButtonPressed &= ~vr::ButtonMaskFromId(vr::k_EButton_Axis1);
	//		state.ulButtonPressed &= ~vr::ButtonMaskFromId(vr::k_EButton_Axis2);
	//	}
	//}


	//if (rigidbody_id == 2){
	//	if (GetAsyncKeyState(80 /* p */) != 0) {
	//		std::cout << "system 2" << std::endl << std::endl;
	//		state.ulButtonPressed |= vr::ButtonMaskFromId(vr::k_EButton_System);
	//	}
	//	else {
	//		state.ulButtonPressed &= ~vr::ButtonMaskFromId(vr::k_EButton_System);
	//	}
	//}

	//if (rigidbody_id == 2){
	//	if (GetAsyncKeyState(79 /* p */) != 0) {
	//		std::cout << "system 3" << std::endl << std::endl;
	//		state.ulButtonPressed |= vr::ButtonMaskFromId(vr::k_EButton_System);
	//	}
	//	else {
	//		state.ulButtonPressed &= ~vr::ButtonMaskFromId(vr::k_EButton_System);
	//	}
	//}

	//if (0.6 > 0.4) {
	//	state.rAxis[1] = { (float)std::min(1.0, std::max(0.0, (0.6 - 0.4) / 0.6)), 0.0f };
	//	state.ulButtonTouched |= vr::ButtonMaskFromId(vr::k_EButton_Axis1);
	//	if (state.rAxis[1].x >= 0.95) {
	//		state.ulButtonPressed |= vr::ButtonMaskFromId(vr::k_EButton_Axis1);
	//	}
	//	else {
	//		state.ulButtonPressed &= ~vr::ButtonMaskFromId(vr::k_EButton_Axis1);
	//	}
	//}
	//else {
	//	state.rAxis[1] = { 0.0f, 0.0f };
	//	state.ulButtonPressed &= ~vr::ButtonMaskFromId(vr::k_EButton_Axis1);
	//	state.ulButtonTouched &= ~vr::ButtonMaskFromId(vr::k_EButton_Axis1);
	//}

	pose.poseIsValid = true;
	pose.result = vr::ETrackingResult::TrackingResult_Running_OK;

	inputEmulator->setVirtualDevicePose(virtualId, pose);
	inputEmulator->setVirtualControllerState(virtualId, state);
}