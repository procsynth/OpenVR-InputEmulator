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

	bool device_is_ctrl = rigidbody_id < 1; // <4

	int i = 0;
	for (i = 0; i < data->nRigidBodies; i++)
	{
		if (data->RigidBodies[i].ID == this->rigidbody_id) {
			break;
		}
	}
	sRigidBodyData r_data = data->RigidBodies[i];
	sRigidBodyData hmd_data = data->RigidBodies[i];


	//std::cout << pose.vecVelocity[0] << "-" << pose.vecVelocity[1] << "-" << pose.vecVelocity[2] << std::endl << std::endl;

	//vr::TrackedDevicePose_t hmdPose;
	//vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0, &hmdPose, 1);


	pose.vecVelocity[0] = (((double)r_data.x - hmd_data.x) - (double)pose.vecPosition[0]) * framerate;
	pose.vecVelocity[1] = (((double)r_data.y - hmd_data.y) - (double)pose.vecPosition[1]) * framerate;
	pose.vecVelocity[2] = (((double)r_data.z - hmd_data.z) - (double)pose.vecPosition[2]) * framerate;

	pose.vecPosition[0] = (double)r_data.x - hmd_data.x;
	pose.vecPosition[1] = (double)r_data.y - hmd_data.y;
	pose.vecPosition[2] = (double)r_data.z - hmd_data.z;


	//std::cout << "POS: " << pose.vecPosition[0] << "-" << pose.vecPosition[1] << "-" << pose.vecPosition[2] << std::endl << std::endl;
	//std::cout << "VEL: " << pose.vecVelocity[0] << "-" << pose.vecVelocity[1] << "-" << pose.vecVelocity[2] << std::endl << std::endl;

	//pose.qWorldFromDriverRotation = vrmath::quaternionFromRotationMatrix(hmdPose.mDeviceToAbsoluteTracking);
	pose.vecWorldFromDriverTranslation[0] = hmd_data.x;
	pose.vecWorldFromDriverTranslation[1] = hmd_data.y;
	pose.vecWorldFromDriverTranslation[2] = hmd_data.z;
	

	pose.qRotation.w = r_data.qw;
	pose.qRotation.x = r_data.qx;
	pose.qRotation.y = r_data.qy;
	pose.qRotation.z = r_data.qz;

	if (!readyFlag) {
		virtualId = inputEmulator->addVirtualDevice(vrinputemulator::VirtualDeviceType::TrackedController, serial, false);
		if (device_is_ctrl) {
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_DeviceClass_Int32, (int32_t)vr::TrackedDeviceClass_Controller);
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_ControllerRoleHint_Int32, (int32_t)vr::TrackedControllerRole_Invalid);
		}
		else {
			std::cout << serial << std::endl;
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_DeviceClass_Int32, (int32_t)vr::TrackedDeviceClass_GenericTracker);
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_ControllerRoleHint_Int32, (int32_t)vr::TrackedControllerRole_Invalid);
		}

		if (rigidbody_id < 4) {

			
			//inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_ControllerHandSelectionPriority_Int32, (int32_t)15-rigidbody_id);
			
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_SupportedButtons_Uint64, (uint64_t)
				vr::ButtonMaskFromId(vr::k_EButton_System) |
				vr::ButtonMaskFromId(vr::k_EButton_ApplicationMenu) |
				vr::ButtonMaskFromId(vr::k_EButton_Grip) |
				vr::ButtonMaskFromId(vr::k_EButton_Axis0) |
				vr::ButtonMaskFromId(vr::k_EButton_Axis1) |
				vr::ButtonMaskFromId(vr::k_EButton_Axis2) |
				vr::ButtonMaskFromId(vr::k_EButton_A)
			);
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_Axis0Type_Int32, (int32_t)vr::k_eControllerAxis_Joystick);
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_Axis1Type_Int32, (int32_t)vr::k_eControllerAxis_Trigger);
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_Axis2Type_Int32, (int32_t)vr::k_eControllerAxis_Joystick);
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_RenderModelName_String, std::string("vr_controller_vive_1_5"));
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_HardwareRevision_Uint64, (uint64_t)666);
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_FirmwareVersion_Uint64, (uint64_t)666);
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_ManufacturerName_String, std::string("Daruma"));
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_ModelNumber_String, std::string("OptiController"));
			//inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_InputProfileName_String, std::string("{htc}/input/vive_controller_profile.json"));
		}
		else {

			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_RenderModelName_String, std::string("ref_controller"));

			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_HardwareRevision_Uint64, (uint64_t)665);
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_FirmwareVersion_Uint64, (uint64_t)665);
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_ManufacturerName_String, std::string("Daruma"));
			inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_ModelNumber_String, std::string("OptiTracker"));
			//inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_InputProfileName_String, std::string("{htc}/input/vive_tracker_profile.json"));
		}


		inputEmulator->publishVirtualDevice(virtualId);

		readyFlag = true;
	} 

	if (rigidbody_id < 4) {



		if (rigidbody_id == 2) {
			if (GetAsyncKeyState(32 /* space */) != 0) {
				std::cout << "space" << std::endl << std::endl;
				state.rAxis[1] = { 1.0f, 0.0f };
				state.ulButtonPressed |= vr::ButtonMaskFromId(vr::k_EButton_Axis1);
			}
			else {
				state.rAxis[1] = { 0.0f, 0.0f };
				state.ulButtonPressed &= ~vr::ButtonMaskFromId(vr::k_EButton_Axis1);
			}
		}

		if (rigidbody_id == 3) {
			if (GetAsyncKeyState(190 /* space */) != 0) {
				std::cout << "space" << std::endl << std::endl;
				state.rAxis[1] = { 1.0f, 0.0f };
				state.ulButtonPressed |= vr::ButtonMaskFromId(vr::k_EButton_Axis1);
			}
			else {
				state.rAxis[1] = { 0.0f, 0.0f };
				state.ulButtonPressed &= ~vr::ButtonMaskFromId(vr::k_EButton_Axis1);
			}
		}


		if (rigidbody_id == 2) {
			if (GetAsyncKeyState(80 /* p */) != 0) {
				std::cout << "system 2" << std::endl << std::endl;
				state.ulButtonPressed |= vr::ButtonMaskFromId(vr::k_EButton_System);
			}
			else {
				state.ulButtonPressed &= ~vr::ButtonMaskFromId(vr::k_EButton_System);
			}
		}

		if (rigidbody_id == 3) {
			if (GetAsyncKeyState(79 /* p */) != 0) {
				std::cout << "system 3" << std::endl << std::endl;
				state.ulButtonPressed |= vr::ButtonMaskFromId(vr::k_EButton_System);
			}
			else {
				state.ulButtonPressed &= ~vr::ButtonMaskFromId(vr::k_EButton_System);
			}
		}

		if (rigidbody_id == 2) {
			if (GetAsyncKeyState(65 /* p */) != 0) {
				std::cout << "A 2" << std::endl << std::endl;
				state.ulButtonPressed |= vr::ButtonMaskFromId(vr::k_EButton_A);
			}
			else {
				state.ulButtonPressed &= ~vr::ButtonMaskFromId(vr::k_EButton_A);
			}
		}

		if (rigidbody_id == 3) {
			if (GetAsyncKeyState(90 /* p */) != 0) {
				std::cout << "A 3" << std::endl << std::endl;
				state.ulButtonPressed |= vr::ButtonMaskFromId(vr::k_EButton_A);
			}
			else {
				state.ulButtonPressed &= ~vr::ButtonMaskFromId(vr::k_EButton_A);
			}
		}
	}

	//if (0.6 > 0.4) {
	//	state.rAxis[1] = { 1.0f, 0.0f };
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