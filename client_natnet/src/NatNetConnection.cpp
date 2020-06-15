#include "NatNetConnection.hpp"

#include <inttypes.h>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <vector>
#include <mutex>

NatNetConnection* NatNetConnection::Instance = nullptr;

NatNetConnection::NatNetConnection()
{

}

NatNetConnection::~NatNetConnection()
{
	if (g_pClient)
	{
		g_pClient->Disconnect();
		delete g_pClient;
		g_pClient = NULL;
	}
}

void NatNetConnection::Init(const char* serverAddress, const char* localAddress, ConnectionType connection)
{
	// Install logging callback
	NatNet_SetLogCallback(MessageHandler);

	// create NatNet client
	g_pClient = new NatNetClient();

	// set the frame callback handler
	g_pClient->SetFrameReceivedCallback(DataHandler, g_pClient);	// this function will receive data from the server

	g_connectParams.connectionType = connection;
	// server
	g_connectParams.serverAddress = serverAddress;
	// client
	g_connectParams.localAddress = localAddress;

	int iResult = ConnectClient();
}

// MessageHandler receives NatNet error/debug messages
void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg)
{
	// Optional: Filter out debug messages
	if (msgType < Verbosity_Info)
	{
		return;
	}

	std::string prefix = "[NatNetLib]";

	switch (msgType)
	{
	case Verbosity_Debug:
		prefix += " [DEBUG]";
		break;
	case Verbosity_Info:
		prefix += "  [INFO]";
		break;
	case Verbosity_Warning:
		prefix += "  [WARN]";
		break;
	case Verbosity_Error:
		prefix += " [ERROR]";
		break;
	default:
		prefix += " [?????]";
		break;
	}

}

void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData)
{
	printf("received data");
	NatNetConnection::Get()->ReceivedData(data, pUserData);
}

int NatNetConnection::ConnectClient()
{
	MessageHandler(Verbosity_Info, "NatNet trying to connect");
	printf("NatNet trying to connect");
	// Release previous server
	g_pClient->Disconnect();

	// Init Client and connect to NatNet server
	int retCode = g_pClient->Connect(g_connectParams);
	if (retCode != ErrorCode_OK)
	{
		printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
		MessageHandler(Verbosity_Error, "Unable to connect to server");
		return ErrorCode_Internal;
	}

	// connection succeeded
	int nBytes = 0;
	ErrorCode ret = ErrorCode_OK;

	// print server info
	memset(&g_serverDescription, 0, sizeof(g_serverDescription));
	ret = g_pClient->GetServerDescription(&g_serverDescription);
	if (ret != ErrorCode_OK || !g_serverDescription.HostPresent)
	{
		printf("Unable to connect to server. Host not present. Exiting.");
		MessageHandler(Verbosity_Error, "Unable to connect to server. Host not present. Exiting");
		return 1;
	}
	return ErrorCode_OK;
}


sFrameOfMocapData NatNetConnection::Frame()
{
	mtx.lock();
	sFrameOfMocapData copy = _Frame;
	mtx.unlock();
	return copy;
}

void NatNetConnection::AddRigidBody(OptitrackRigidBody* rigidbody)
{
	rigidbodies.push_back(rigidbody);
}

void NatNetConnection::ReceivedData(sFrameOfMocapData* data, void* pUserData)
{
	mtx.lock();
	this->_Frame = (*data);
	mtx.unlock();

	for (auto& body : rigidbodies)
	{
		body->ReceivedData(data, pUserData);
	}
}
