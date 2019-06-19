//======================================================//
// Name: drvModbus.h
// Purpose: Simple modbus driver
// Authors: Jeremy L.
// Date Created: June 14, 2019
//======================================================//
#ifndef _DRV_MODBUS_H_
#define _DRV_MODBUS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Note: this code might not be portable to compilers other than GCC (__attribute__ is used a bit) */

/* standard includes */
#include <stddef.h>
#include <stdint.h>

/* EPICS includes */
#include <osiSock.h>
#include <epicsMutex.h>

/* Offset of the fn error code and the actual function code in modbus_excpt_pdu_t */
#define MB_ERRCODE_OFFSET 0x80

/* Function codes */
#define MB_RD_DISC_INPUTS_CODE	0x02
#define MB_RD_COILS_CODE		0x01
#define MB_WR_SIN_COIL_CODE		0x05
#define MB_WR_MUL_COIL_CODE		0x0F
#define MB_RD_INP_REG_CODE		0x04
#define MB_RD_HOL_REG_CODE		0x03
#define MB_WR_SIN_REG_CODE		0x06
#define MB_WR_MULT_REG_CODE		0x10
#define MB_RW_MULT_REG_CODE		0x23
#define MB_MSK_WRT_REG_CODE		0x16
#define MB_RD_FIFO_QUEUE_CODE	0x18
#define MB_RD_FILE_REC_CODE		0x14
#define MB_WR_FILE_REC_CODE		0x15
#define MB_RD_ERR_STAT_CODE		0x07
#define MB_DIAGNOSTIC_CODE		0x08
#define MB_RD_COM_EV_CNT_CODE	0x0B
#define MB_RD_COM_EV_LOG_CODE	0x0C
#define MB_RD_SRV_ID_CODE		0x11
#define MB_RD_DEV_ID			0x2B

#define MODBUS_ERR_ILLEGAL_FUNCTION		0x01
#define MODBUS_ERR_ILLEGAL_ADDR			0x02
#define MODBUS_ERR_ILLEGAL_VAL			0x03
#define MODBUS_ERR_DEVICE_FAILURE		0x04
#define MODBUS_ERR_ACKNOWLEDGE			0x05
#define MODBUS_ERR_DEVICE_BUSY			0x06
#define MODBUS_ERR_MEM_PARITY			0x07
#define MODBUS_ERR_GATEWAY_UNAVAIL		0x0A
#define MODBUS_ERR_GATEWAY_UNRESP		0x0B

/* Socket that modbus should use */
#define MODBUS_PORT 502

typedef struct
{
	uint16_t trans_id;
	uint16_t protocol_id;
	uint16_t len;
	uint8_t unit_id;
} __attribute__((packed)) modbus_mbap_header_t;

/* Forgive me for the long names */

typedef struct
{
	uint8_t func;
	uint8_t* data;
} __attribute__((packed)) modbus_pdu_t;

typedef struct
{
	uint8_t err_fn_code;
	uint8_t err_code;
} __attribute__((packed)) modbus_excpt_pdu_t;

/* Simple device connected via modbus tcp */
typedef struct
{
	epicsMutexId mutex;
	struct sockaddr_in addr;
} modbus_device_t;

/* Create a device with the specified IP */
modbus_device_t* modbus_CreateDevice(const struct sockaddr_in* ip);

/* Destroy a device */
void modbus_DestroyDevice(modbus_device_t* device);

/* Init the modbus stuff */
void modbus_Init();


/*
Name: modbus_ReadCoils
Desc: Modbus function 0x01. Read from n coils and store them in a buffer.
Params:
	-	device: the device to read from
	-	addr: the address of the first coil
	-	ncoils: the number of coils to read
	-	pOutBuf: the output buffer to store the coils in
	-	nOutCoils: the actual number of coils read
Notes:
	-	On error, this will return the error code. If OK, it will return 0. -1 means application error, otherwise it's a modbus error
	-	nCoils needs to be less than 0x7D0 (See modbus docs)
*/
int modbus_ReadCoils(modbus_device_t* device, uint16_t addr, uint16_t ncoils, uint8_t* pOutBuf, uint8_t* nOutCoils);

/*
Name: modbus_ReadDiscreteInputs
Desc: Read 1 to 2000 discrete inputs on the target device
Params:
	-	device: the target device
	-	addr: the address of the first coil
	-	ncoils: the number of coils to read
	-	poOutBuf: the output buffer to store everything into
	-	nOutCoils: the actual number of output coils
Notes:
	-	On error, this will return the error code. If OK, it will return 0. -1 means application error, otherwise it's a modbus error
	-	nCoils needs to be less than 0x7D0 (See modbus docs)
*/
int modbus_ReadDiscreteInputs(modbus_device_t* device, uint16_t addr, uint16_t ncoils, uint8_t* pOutBuf, uint8_t* nOutCoils);

/*
Name: modbus_ReadHoldingRegisters
Desc: Read 1-16 of the holding registers
Params:
	-	device: tharget device
	-	addr: starting addr
	-	nregs: register count
	-	pOutBuf: output buffer to hold everything in
	-	nOutRegs: actual number of registers saved
Notes:
	-	On error, this will return the error code. If OK, it will return 0. -1 means application error, otherwise it's a modbus thing
	-	nregs must be less than 0x7D
	-	Items in the output buffer are all little-endian (or big-endian, depending on the system), regardless of modbus's endianess
*/
int modbus_ReadHoldingRegisters(modbus_device_t* device, uint16_t addr, uint16_t nregs, uint16_t* pOutBuf, uint16_t* nOutRegs);

/*
Name: modbus_WriteSingleRegister
Desc: Write to a single register (function code 0x06)
Params:
	-	device: the target device 
	-	addr: the address of the register
	-	value: the new value
Notes:
	-	On error, this will return the error code, if OK, it will return 0. -1 means application error, otherwise it's a modbus error
*/
int modbus_WriteSingleRegister(modbus_device_t* device, uint16_t addr, uint16_t value);

/*
Name: modbus_ReadInputRegisters
Desc: Read from 1 to 125 contiguous input registers
Params:
	-	device: the target device 
	-	addr: the starting address
	-	nregs: the number of regs to read
	-	pOutBuf: the output buffer
	-	pOutRegs: the number of registers actually read
Notes:
	-	On error, this will return the error code, if OK, it will return 0. -1 means application error, otherwise it's a modbus error
	-	All values in pOutBuf are corrected for endianness, so they match the host platform.
	-	nregs must be 0x1-0x7D
*/
int modbus_ReadInputRegisters(modbus_device_t* device, uint16_t addr, uint16_t nregs, uint16_t* pOutBuf, uint8_t* pOutRegs);


#ifdef __cplusplus
}
#endif

#endif // _DRV_MODBUS_H_