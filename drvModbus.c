//======================================================//
// Name: drvModbus.c
// Purpose: Simple modbus driver over TCP
// Authors: Jeremy L.
// Date Created: June 14, 2019
//======================================================//
#include "drvModbus.h"


/* Standard includes */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <memory.h>
#include <string.h>
#include <stdlib.h>

/* EPICS includes */
#include <epicsExport.h>
#include <epicsStdio.h>
#include <osiSock.h>
#include <epicsString.h>
#include <epicsPrint.h>
#include <epicsMutex.h>
#include <epicsTypes.h>
#include <epicsAssert.h>
#include <epicsThread.h>

/* Some util macros */
#if defined(__VERBOSE) || defined(__DEBUG)
#define __LOG_ERROR(str) epicsPrintf("%s:%u %s\n", __FILE__,__LINE__,str)
#define LOG_ERROR(str) __LOG_ERROR(str)
#define LOG_ERROR_FORMATTED(str, ...) {char s[512]; sprintf(s,str, __VA_ARGS__); __LOG_ERROR(s); }
#define PRINT_LOG(str) epicsPrintf("%s:%u %s\n", __FILE__, __LINE__, str)
#define PRINT_LOG_FORMATTED(str, ...) {char s[512]; sprintf(s,str,__VA_ARGS__); PRINT_LOG(s); }
#else
#define LOG_ERROR(str)
#define LOG_ERROR_FORMATTED(str,...)
#define PRINT_LOG(str)
#define PRINT_LOG_FORMATTED)
#endif

#define BIG_TO_LITTLE_ENDIAN(_short) _short = (((_short) >> 8) & 0x00FF) | (((_short) << 8) & 0xFF00)
#define LITTLE_TO_BIG_ENDIAN(_short) _short = (((_short) << 8) & 0xFF00) | (((_short) >> 8) & 0x00FF)

#define CHECK_RESULT(x, str) if(x) { LOG_ERROR(str); }
#define CHECK_RESULT_FORMATTED(x, str, ...) if(x) { LOG_ERROR_FORATTED(str, __VA_ARGS__); } 

#define MALLOC_MUST_SUCCEED(var, size) { var = malloc(size); assert(var != NULL); if(!var) return -1; }
#define CALLOC_MUST_SUCCEED(var, num, size) { var = calloc(num, size); assert(var != NULL); if(!var) return -1; }

/* This is going to be the socket we use to actually do comms on modbus */
SOCKET g_ModbusSocket = -1;
extern epicsMutexId g_Mutex = NULL;

//======================================================//
// Name: modbus_Init
// Purpose: Initialize the modbus driver
//======================================================//
void modbus_Init()
{
	
	SOCKET sock = epicsSocketCreate(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(sock < 0)
	{
		char errbuf[128];
		epicsSocketConvertErrnoToString(errbuf, 127);
		errbuf[127] = '\0'; /* Might need this, idk */
		epicsPrintf("%s:%u Failed to create socket for Modbus driver: %s\n", __FILE__, __LINE__, errbuf);
		return;
	}
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(struct sockaddr_in));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(MODBUS_PORT);
	int result = bind(sock, &addr, sizeof(struct sockaddr_in));
	if(result < 0)
	{
		epicsPrintf("%s:%u Failed to bind socket for Modbus driver.\n", __FILE__, __LINE__);
		return;
	}
	g_ModbusSocket = sock;
	epicsPrintf("Initialized Modbus driver.\n");
}

//======================================================//
// Name: modbus_CreateDevice
// Purpose: Initialize a device
//======================================================//
modbus_device_t* modbus_CreateDevice(const struct sockaddr_in* ip)
{
	modbus_device_t* device = malloc(sizeof(modbus_device_t));
	device->addr = *ip;
	device->addr.sin_port = htons(MODBUS_PORT);
	device->addr.sin_family = AF_INET;
	device->mutex = epicsMutexCreate();
	if(!device)
	{
		char buf[64];
		ipAddrToDottedIP(&ip, buf, 64);
		LOG_ERROR_FORMATTED("Failed to create device at IP %s", buf);
		return NULL;
	}
	return device;
}

//======================================================//
// Name: modbus_Init
// Purpose: Destroy a device
//======================================================//
void modbus_DestroyDevice(modbus_device_t* device)
{
	if(device)
	{
		epicsMutexDestroy(device->mutex);
	}
}

//======================================================//
// Name: modbus_Shutdown
// Purpose: Shutdown the modbus driver. Mainly to terminate
// the sockets
//======================================================//
void modbus_Shutdown()
{
	if(g_ModbusSocket >= 0)
	{
		epicsSocketDestroy(g_ModbusSocket);
		epicsPrintf("%s:%u Unloaded Modbus driver.\n", __FILE__, __LINE__);
		return;
	}
	else
	{
		epicsPrintf("%s:%u Socket was invalid while unloading Modbus driver. This could be a sign of a problem.\n", __FILE__, __LINE__);
		return;
	}
}

//======================================================//
// MODBUS INTERNAL FUNCTIONS. NO DOCUMENTATION WILL BE
// PROVIDED
//======================================================//

void modbus_ConnectDevice(modbus_device_t* device)
{
	if(device == NULL)
	{
		epicsPrintf("%s:%u Device passed to modbus_ConnectDevice is NULL!\n", __FILE__, __LINE__);
		return;
	}
	if(g_ModbusSocket < 0)
	{
		LOG_ERROR("Modbus socket is not open!");
		epicsPrintf("%s:%u Modbus socket is not open!\n", __FILE__, __LINE__);
		return;
	}

	if(epicsMutexLock(device->mutex) != epicsMutexLockOK)
	{
		LOG_ERROR("Error while locking the device mutex!");
		return;
	}
}

void modbus_DisconnectDevice(modbus_device_t* device)
{
#if 0
	if(shutdown(g_ModbusSocket, SHUT_RDWR) < 0)
	{
		/* Report error only if we have some type of error that is unrecoverable */
		if(errno == ENOTCONN)
			/* Not connected anyways, so just whatever */
			goto SHUTDOWN;
		
		char buf[128];
		buf[127] = '\0';
		epicsSocketConvertErrorToString(buf, 127, errno);
		LOG_ERROR_FORMATTED("Unable to shutdown socket connection properly: %s", buf);
		/* NOTE: mutex will remain locked because we have an issue */
		return;
	}
#endif
SHUTDOWN:
	epicsMutexUnlock(device->mutex);

}

/* Assumes device is already connected, and mutex is locked */
/* Returns -1 on error, or the number of bytes on reset */
ssize_t modbus_RecvBlock(modbus_device_t* pDevice, char* pBuf, size_t nLen)
{
	if(!pDevice)
	{
		LOG_ERROR("Invalid device.");
		return -1;
	}
	if(!pBuf)
	{
		LOG_ERROR("Parameter was NULL.");
		return -1;
	}
	struct sockaddr_in addr;
	ssize_t len;
	do
	{
		memset(&addr, 0, sizeof(struct sockaddr_in));
		if((len = recvfrom(g_ModbusSocket, pBuf, nLen, MSG_WAITALL, &addr, sizeof(struct sockaddr_in))) < 0)
		{
			if(errno == ENOTCONN)
				LOG_ERROR("While connecting to device: the socket is not connected!");
			else if(errno == ETIMEDOUT)
				LOG_ERROR("While receiving block from device: timed out while waiting on device!");
			else
			{
				char buf[128];
				buf[127] = '\0';
				epicsSocketConvertErrorToString(buf, 127, errno);
				LOG_ERROR_FORMATTED("While receiving block from device: %s", buf);
			}
			return -1;
		}
	} while(memcmp(&addr, &pDevice->addr, sizeof(struct sockaddr_in)) != 0);

	return (int)len;
}

/* Returns -1 on error, or the num of bytes sent */
ssize_t modbus_SendBlock(char* pBuf, size_t nLen, struct sockaddr_in dest)
{
	if(pBuf == NULL)
	{
		LOG_ERROR("Parameter was NULL.");
		return -1;
	}
	ssize_t len = 0;
	if((len = sendto(g_ModbusSocket, pBuf, nLen, 0, &dest, sizeof(struct sockaddr_in))) < 0)
	{
		if(errno == ENOTCONN)
			LOG_ERROR("While sending data to device: the socket was not connected.");
		else
		{
			char buf[128];
			buf[127] = '\0';
			epicsSocketConvertErrorToString(buf, 127, errno);
			LOG_ERROR_FORMATTED("While receiving block from device: %s", buf);
		}
	}
	return len;
}

/* This should construct a modbus packet, do CRC checks, etc */
/* pOutBuf is the location to copy the data into */
/* pOutLen is the length of the bytes copied */
int modbus_ConstructPacket(void* pBuf, size_t nLen, void* pOutBuf, size_t* pOutLen, uint16_t* transactionID)
{
	//pOutBuf = malloc(sizeof(modbus_mbap_header_t) + nLen);
	if(!pOutBuf || !pBuf)
	{
		LOG_ERROR("Failure when creating packet, memory failed to allocate.");
		return -1;
	}
	if(pOutLen < (sizeof(modbus_mbap_header_t) + nLen))
		return -1;
	
	modbus_mbap_header_t header;
	memset(&header, 0, sizeof(modbus_mbap_header_t));
	header.protocol_id = 0;
	header.trans_id = rand() % 65535; /* we can just generate a random transaction id */
	header.unit_id = 255;
	header.len = nLen + 1; /* +1 because it includes the size of the unit_id field. */
	*transactionID = header.trans_id;
	memcpy(pOutBuf, &header, sizeof(modbus_mbap_header_t));
	memcpy((pOutBuf+sizeof(modbus_mbap_header_t)), pBuf, nLen);
	*pOutLen = sizeof(modbus_mbap_header_t) + nLen;
	return 0;
}

/* Send a modbus packet built around the provided data */
/* when OK, returns transaction ID, when failure, returns -1 */
int modbus_SendPacket(modbus_device_t* device, void* pData, size_t nLen)
{
	if(!device)
	{
		LOG_ERROR("Invalid device.");
		return -1;
	}

	size_t outLen = sizeof(modbus_mbap_header_t) + nLen + 1;
	uint16_t tID = 0;
	void* pBuf = malloc(outLen);
	if(modbus_ConstructPacket(pData, nLen, pBuf, &outLen, &tID) != 0)
		return -1;
	modbus_SendBlock(pBuf, outLen, device->addr);
	free(pBuf);
	return tID;
}

/* Receive a modbus packet, and take the PDU out of it and return it */
/* tID is the transaction ID. This should be received from modbus_SendPacket */
/* TODO: Need to add some type of proper timeout code! */
/* Returns length of recved data or -1 */
int modbus_RecvPacket(modbus_device_t* pDevice, void* pOutData, size_t nLen, uint16_t tID)
{
	modbus_mbap_header_t* header;
	ssize_t len = modbus_RecvBlock(pDevice, pOutData, nLen);
	header = (modbus_mbap_header_t*)pOutData; /* Cast first bytes to the header */
	if(header->trans_id == tID)
	{
		memcpy(pOutData, (pOutData+sizeof(modbus_mbap_header_t)), len-sizeof(modbus_mbap_header_t));
		return len;
	}
	return -1;
}


//======================================================//
// Name: modbus_ReadCoils
// Purpose: Reads coils from the device
// Notes:
// 		- ncoils must be less than 0x7D0
//		- device musn't be null
//======================================================//
/* Request */
struct modbus_ReadCoils_req
{
	uint8_t function_code;
	uint16_t addr;
	uint16_t ncoils;
};

int modbus_ReadCoils(modbus_device_t* device, uint16_t addr, uint16_t ncoils, uint8_t* pOutBuf, uint8_t* nOutCoils)
{
	if(!device)
	{
		LOG_ERROR("Device was invalid.");
		return -1;
	}

	if(!pOutBuf || !nOutCoils)
	{
		LOG_ERROR("Invalid parameter was passed.");
		return -1;
	}

	if(ncoils > 0x7D0)
	{
		LOG_ERROR("Unable to read more than 0x7D0 coils");
		return -1;
	}
	
	struct modbus_ReadCoils_req packet;
	packet.function_code = MB_RD_COILS_CODE;
	packet.ncoils = LITTLE_TO_BIG_ENDIAN(ncoils);
	packet.addr = LITTLE_TO_BIG_ENDIAN(addr);
	
	modbus_ConnectDevice(device);

	int tID = modbus_SendPacket(device, &packet, sizeof(struct modbus_ReadCoils_req));

	/* We know about how large the return packet should be */
	size_t len = ncoils + 2;
	void* pBuf;
	MALLOC_MUST_SUCCEED(pBuf, len);
	if(modbus_RecvPacket(device, pBuf, len, tID) > -1)
	{
		if(*(uint8_t*)pBuf == 0x81)
		{
			LOG_ERROR("Modbus error while reading coils.");
			return *(uint8_t*)pBuf+1;
		}
		nOutCoils = *((char*)(pBuf) + 1); /* Second byte is n coils */
		memcpy(pOutBuf, (pBuf+2), len-2);
	}
	else
	{
		char buf[64];
		ipAddrToDottedIP(&device->addr, buf, 64);
		LOG_ERROR_FORMATTED("Failed to write to device at ip %s", buf);
		return -1;
	}

	modbus_DisconnectDevice(device);
	free(pBuf);
	return 0;
}

//======================================================//
// Name: modbus_WriteSingleRegister
// Purpose: Write to a single holding register
// Notes:
// 		- addr is the address of the register
//		- value is the new value
//======================================================//
struct modbus_WriteSingleRegister_req
{
	uint8_t code;
	uint16_t addr;
	uint16_t value;
};

int modbus_WriteSingleRegister(modbus_device_t* device, uint16_t addr, uint16_t value)
{
	if(device == NULL)
	{
		LOG_ERROR("Invalid parameter passed.");
		return;
	}

	/* Construct the packet */
	struct modbus_WriteSingleRegister_req packet;
	memset(&packet, 0, sizeof(struct modbus_WriteSingleRegister_req));
	/* Make sure this gets converted to big endian representation, as modbus is 100% big endian */
	packet.addr = LITTLE_TO_BIG_ENDIAN(addr);
	packet.value = LITTLE_TO_BIG_ENDIAN(value);
	packet.code = MB_WR_SIN_REG_CODE;

	/* Now connect device */
	modbus_ConnectDevice(device);
	int tID = modbus_SendPacket(device, &packet, sizeof(struct modbus_WriteSingleRegister_req));

	/* Create new packet for response */
	struct modbus_WriteSingleRegister_req res_packet;
	memset(&res_packet, 0, sizeof(struct modbus_WriteSingleRegister_req));
	int bytes = modbus_RecvPacket(device, &res_packet, sizeof(struct modbus_WriteSingleRegister_req), tID);

	/* verify writing */
	/* NOTE: the values we just received are already in big-endian format */
	if(packet.value != res_packet.value || packet.addr != res_packet.addr)
	{
		char buf[64];
		ipAddrToDottedIP(&device->addr, buf, 64);
		LOG_ERROR_FORMATTED("Failed to write register in target device at %s. Different values were returned.", buf);
		return -1;
	}
	modbus_DisconnectDevice(device);
	return 0;
}

//======================================================//
// Name: modbus_ReadDiscreteInputs
// Purpose: Read up to 2000 discrete inputs
// Notes:
// 		- idk
//======================================================//
struct modbus_ReadDiscreteInputs_req
{
	uint8_t code;
	uint16_t addr;
	uint16_t coils;
};

int modbus_ReadDiscreteInputs(modbus_device_t* device, uint16_t addr, uint16_t ncoils, uint8_t* pOutBuf, uint8_t* nOutCoils)
{
	if(!device)
	{
		LOG_ERROR("The device was invalid.");
		return -1;
	}

	if(!pOutBuf || !nOutCoils)
	{
		LOG_ERROR("Invalid parameter was passed.");
		return -1;
	}

	/* Connect device and begin with sending input */
	modbus_ConnectDevice(device);
	struct modbus_ReadDiscreteInputs_req packet;
	packet.code = MB_RD_DISC_INPUTS_CODE;
	packet.addr = LITTLE_TO_BIG_ENDIAN(addr);
	packet.coils = LITTLE_TO_BIG_ENDIAN(ncoils);
	int tID = modbus_SendPacket(device, &packet, sizeof(struct modbus_ReadDiscreteInputs_req));

	if(tID < 0)
		return -1;

	/* 2 bytes describe what we have returned */
	size_t len = ncoils + 2;
	void* pBuf;
	MALLOC_MUST_SUCCEED(pBuf, len);
	len = modbus_RecvPacket(device, pBuf, len, tID);
	if(len < 0)
	{
		LOG_ERROR("Error while recieving data.");
		return -1;
	}
	uint8_t code = (uint8_t)((char*)pBuf); /* This will tell us if we have errors */
	modbus_DisconnectDevice(device);

	/* This means error! */
	if(code == 0x82)
	{
		LOG_ERROR("A modbus error ocurred while processing the request.");
		return (uint8_t)((char*)pBuf + 1);
	}
	nOutCoils = len-2;
	memcpy(pOutBuf, (pBuf+2), len-2);
	free(pBuf);
	return 0;
}

//======================================================//
// Name: modbus_ReadHoldingRegisters
// Purpose: Read up to 16 registers
// Notes:
//		-	items in the output buffer are endian corrected
//======================================================//
struct modbus_ReadHoldingRegisters_req
{
	uint8_t code;
	uint16_t addr;
	uint16_t count;
};

int modbus_ReadHoldingRegisters(modbus_device_t* device, uint16_t addr, uint16_t nregs, uint16_t* pOutBuf, uint16_t* nOutRegs)
{
	if(!device)
	{
		LOG_ERROR("Invalid device.");
		return -1;
	}

	struct modbus_ReadHoldingRegisters_req packet;
	packet.code = MB_RD_HOL_REG_CODE;
	packet.addr = LITTLE_TO_BIG_ENDIAN(addr);
	packet.count = LITTLE_TO_BIG_ENDIAN(nregs);
	modbus_ConnectDevice(device);
	int tID = modbus_SendPacket(device, &packet, sizeof(struct modbus_ReadHoldingRegisters_req));
	if(tID < 0)
		return -1;
	size_t len = (2*nregs) + 2;
	void* pBuf;
	MALLOC_MUST_SUCCEED(pBuf, len);
	len = modbus_RecvPacket(device, pBuf, len, tID);
	modbus_DisconnectDevice(device);

	if(len < 0)
		return -1;
	
	uint8_t code = *(uint8_t*)pBuf;
	if(*(uint8_t*)pBuf == 0x83)
	{
		LOG_ERROR("An error occurred while reading holding registers.");
		return *(uint8_t*)pBuf+1;
	}

	/* we need to swap all the bytes around */
	for(int i = 2; i < len; i += 2)
		((uint16_t*)(pBuf))[i] = BIG_TO_LITTLE_ENDIAN(((uint16_t*)pBuf)[i]);

	memcpy(pOutBuf, (pBuf+2), len-2);
	*nOutRegs = *((uint8_t*)pBuf+1) / 2;
	free(pBuf);
	return 0;
}

//======================================================//
// Name: modbus_ReadInputRegisters
// Purpose: Read up to 125 registers
// Notes:
//		-	items in the output buffer are endian corrected
//======================================================//
struct modbus_ReadInputRegisters_req
{
	uint8_t code;
	uint16_t addr;
	uint16_t count;
};
int modbus_ReadInputRegisters(modbus_device_t* device, uint16_t addr, uint16_t nregs, uint16_t* pOutBuf, uint8_t* pOutRegs)
{
	if(!device)
	{
		LOG_ERROR("Invalid device.");
		return -1;
	}

	struct modbus_ReadInputRegisters_req packet;
	packet.code = MB_RD_INP_REG_CODE;
	packet.addr = LITTLE_TO_BIG_ENDIAN(addr);
	packet.count = LITTLE_TO_BIG_ENDIAN(nregs);
	modbus_ConnectDevice(device);
	int tID = modbus_SendPacket(device, &packet, sizeof(struct modbus_ReadInputRegisters_req));
	if(tID < 0)
		return -1;

	size_t len = nregs * 2 + 2;
	void* pBuf;
	MALLOC_MUST_SUCCEED(pBuf, len);
	len = modbus_RecvPacket(device, pBuf, len, tID);
	if(len < 0)
		return -1;
	if(*(uint8_t*)pBuf == 0x84)
	{
		char buf[64];
		ipAddrToA(&device->addr, buf, 64);
		LOG_ERROR_FORMATTED("An error ocurred while reading from the device at %s", buf);
		return *((uint8_t*)pBuf+1);
	}

	for(int i = 2; i < len; i += 2)
		((uint8_t*)pBuf)[i] = BIG_TO_LITTLE_ENDIAN(((uint8_t*)pBuf)[i]);

	pOutRegs = (*(uint8_t*)(pBuf+1) / 2);
	memcpy(pOutBuf, (pBuf+2), len-2);
	free(pBuf);
	return 0;
}