#pragma once

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <conio.h>

#include <vector>

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>
#include <mutex>


void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg);
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);

void _WriteHeader(FILE* fp, sDataDescriptions* pBodyDefs);
void _WriteFrame(FILE* fp, sFrameOfMocapData* data);
void _WriteFooter(FILE* fp);

class NatNetConnection
{
protected:
	mutable std::mutex mtx;
	int ConnectClient();

	NatNetConnection();
	virtual ~NatNetConnection();
	static NatNetConnection* Instance;
	NatNetClient* g_pClient = NULL;
	FILE* g_outputFile;

	std::vector< sNatNetDiscoveredServer > g_discoveredServers;
	sNatNetClientConnectParams g_connectParams;
	char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;
	int g_analogSamplesPerMocapFrame = 0;
	sServerDescription g_serverDescription;
	sFrameOfMocapData _Frame;

public:
	sFrameOfMocapData Frame();

	void ReceivedData(sFrameOfMocapData* data, void* pUserData);
	void Init(const char* serverAddress, const char* localAddress, ConnectionType connection);

	static NatNetConnection* Get()
	{
		if (Instance == NULL)
			Instance = new NatNetConnection();
		return Instance;
	}
};

