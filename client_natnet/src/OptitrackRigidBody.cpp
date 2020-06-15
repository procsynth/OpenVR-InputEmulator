#include "OptitrackRigidBody.h"
#include "openvr_math.h"

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
	last_time = now;
	//pose.poseTimeOffset = 0;

	pose.willDriftInYaw = false;
	pose.shouldApplyHeadModel = false;
	//pose.qDriverFromHeadRotation.w = pose.qWorldFromDriverRotation.w = pose.qRotation.w = 1.0;

	pose.deviceIsConnected = true;
	pose.poseIsValid = true;
	pose.result = vr::ETrackingResult::TrackingResult_Running_OK;

	
	int i = 0;
	for (i = 0; i < data->nRigidBodies; i++)
	{
		if (data->RigidBodies[i].ID == this->rigidbody_id) {
			break;
		}
	}
	sRigidBodyData r_data = data->RigidBodies[i];
	
	pose.vecVelocity[0] = (r_data.x - pose.vecPosition[0])/100;
	pose.vecVelocity[1] = (r_data.y - pose.vecPosition[1])/100;
	pose.vecVelocity[2] = (r_data.z - pose.vecPosition[2])/100;

	vr::TrackedDevicePose_t hmdPose;
	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0, &hmdPose, 1);

	if (hmdPose.bPoseIsValid) {
		pose.vecPosition[0] = r_data.x - hmdPose.mDeviceToAbsoluteTracking.m[0][3];
		pose.vecPosition[1] = r_data.y - hmdPose.mDeviceToAbsoluteTracking.m[1][3];
		pose.vecPosition[2] = r_data.z - hmdPose.mDeviceToAbsoluteTracking.m[2][3];

		pose.qWorldFromDriverRotation = vrmath::quaternionFromRotationMatrix(hmdPose.mDeviceToAbsoluteTracking);
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
		inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_DeviceClass_Int32, (int32_t)vr::TrackedDeviceClass_Controller);
		inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_SupportedButtons_Uint64, (uint64_t)
			vr::ButtonMaskFromId(vr::k_EButton_System) |
			vr::ButtonMaskFromId(vr::k_EButton_ApplicationMenu) |
			vr::ButtonMaskFromId(vr::k_EButton_Grip) |
			vr::ButtonMaskFromId(vr::k_EButton_Axis0) |
			vr::ButtonMaskFromId(vr::k_EButton_Axis1)
		);
		inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_Axis0Type_Int32, (int32_t)vr::k_eControllerAxis_Joystick);
		inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_Axis1Type_Int32, (int32_t)vr::k_eControllerAxis_Trigger);
		inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_HardwareRevision_Uint64, (uint64_t)666);
		inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_FirmwareVersion_Uint64, (uint64_t)666);
		inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_RenderModelName_String, std::string("vr_controller_vive_1_5"));
		inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_ManufacturerName_String, std::string("Leap Motion"));
		inputEmulator->setVirtualDeviceProperty(virtualId, vr::Prop_ModelNumber_String, std::string("Leap Motion Controller"));
		inputEmulator->publishVirtualDevice(virtualId);

		readyFlag = true;
	}

	inputEmulator->setVirtualDevicePose(virtualId, pose);
	inputEmulator->setVirtualControllerState(virtualId, state);
}