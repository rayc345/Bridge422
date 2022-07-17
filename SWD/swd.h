#ifndef __SWD_H__
#define __SWD_H__

#include <stdint.h>
#include <stdbool.h>
#include "swd_pin.h"
#include "debug_cm.h"

// DAP Transfer Response
typedef uint8_t DAP_ACK;

#define DAP_TRANSFER_OK (1 << 0)
#define DAP_TRANSFER_WAIT (1 << 1)
#define DAP_TRANSFER_ERROR (1 << 2)

#define SWD_REG_AP (1)
#define SWD_REG_DP (0)
#define SWD_REG_R (1 << 1)
#define SWD_REG_W (0 << 1)
#define SWD_REG_ADR(a) (a & 0x0C)

#define DBG_CPUID 0xE000ED00 // CPUID寄存器
#define DBG_ICSR 0xE000ED04  //中断控制及状态寄存器
#define DBG_VTOR 0xE000ED08  //向量表偏移量寄存器
#define DBG_AIRCR 0xE000ED0C //应用程序中断及复位控制寄存器

// #define DBG_DHCSR 0xE000EDF0 //调试停机控制及状态寄存器    R/W
// #define DBG_DCRSR 0xE000EDF4 //调试内核寄存器选择者寄存器   W
// #define DBG_DCRDR 0xE000EDF8 //调试内核寄存器数据寄存器    R/W
// #define DBG_DEMCR 0xE000EDFC //调试及监视器控制寄存器       R/W

bool SW_InitDebug(void);

bool SW_ReadDP(const uint8_t adr, uint32_t *val);
bool SW_WriteDP(const uint8_t adr, uint32_t val);
bool SW_ReadAP(const uint32_t adr, uint32_t *val);
bool SW_WriteAP(const uint32_t adr, uint32_t val);

bool SW_WriteData(const uint32_t addr, uint32_t data);
bool SW_ReadData(const uint32_t addr, uint32_t *data);

bool SW_ReadBlock(const uint32_t addr, uint32_t *buf, const uint32_t len);
bool SW_WriteBlock(const uint32_t addr, uint32_t *buf, const uint16_t len);

bool SW_ReadMem(uint32_t address, uint32_t *data, uint32_t size);
bool SW_WriteMem(uint32_t address, uint32_t *data, uint32_t size);

// bool SW_HaltCore(void);
bool SW_RestoreCore(void);

bool SW_WriteCoreReg(const uint32_t n, uint32_t val);
bool SW_ReadCoreReg(const uint32_t n, uint32_t *val);

void SendIdle(void);

#endif
