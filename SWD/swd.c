#include "swd.h"
#include <stdio.h>

uint8_t uAP = 0, uAPBank = 0;

bool SWD_Transfer(const uint8_t request, uint32_t *data)
{
    uint8_t bit;
    uint32_t val;
    uint8_t n;
    uint8_t parity, ack;
    bool bSuccess = true;
    for (uint8_t uNumTry = 3; uNumTry > 0; uNumTry--)
    {
        parity = 0;
        /* Packet Request */
        SW_WRITE_BIT(1); /* Start Bit */
        bit = request & 0x01;
        SW_WRITE_BIT(bit); /* APnDP Bit */
        parity += bit;
        bit = request >> 1;
        SW_WRITE_BIT(bit); /* RnW Bit */
        parity += bit;
        bit = request >> 2;
        SW_WRITE_BIT(bit); /* A2 Bit */
        parity += bit;
        bit = request >> 3;
        SW_WRITE_BIT(bit); /* A3 Bit */
        parity += bit;
        SW_WRITE_BIT(parity); /* Parity Bit */
        SW_WRITE_BIT(0);      /* Stop Bit */
        SW_WRITE_BIT(1);      /* Park Bit */

        SWDIO_SET_INPUT();
        /* Turnaround */
        for (n = 1; n; n--)
            SW_CLOCK_CYCLE();
        /* Acknowledge response */
        ack = 0;
        uint8_t ack0, ack1, ack2;
        for (n = 0; n < 3; n++)
        {
            bit = SW_READ_BIT();
            if (n == 0)
                ack0 = bit;
            else if (n == 1)
                ack1 = bit;
            else if (n == 2)
                ack2 = bit;
            ack |= bit << n;
        }
        if (ack == DAP_TRANSFER_OK)
        {
            bSuccess = true;
            break;
        }
        else if (ack == DAP_TRANSFER_WAIT)
        {
            SW_CLOCK_CYCLE();
            SWDIO_SET_OUTPUT();
            continue;
        }
        else
        {
            SW_CLOCK_CYCLE();
            SWDIO_SET_OUTPUT();
            printf("Failed ACK: %08X / %d %d %d\n", ack, ack0, ack1, ack2);
            continue;
            // SWDIO_SET_OUTPUT();
            /* Protocol error */
            // for (n = 40; n; n--)
            //     SW_WRITE_BIT(0);
            // printf("ACK: %08X / %d %d %d\n", ack, ack0, ack1, ack2);
            // return false;
        }
    }
    if (request & SWD_REG_R) /* read data */
    {
        val = 0;
        parity = 0;
        for (n = 32; n; n--)
        {
            bit = SW_READ_BIT(); /* Read RDATA[0:31] */
            parity += bit;
            val >>= 1;
            val |= bit << 31;
        }
        bit = SW_READ_BIT(); /* Read Parity */

        if ((parity ^ bit) & 1)
        {
            bSuccess = false;
            printf("Failed Parity\n");
        }
        if (data)
            *data = val;

        /* Turnaround */
        for (n = 1; n; n--)
            SW_CLOCK_CYCLE();
        SWDIO_SET_OUTPUT();
    }
    else /* write data */
    {
        /* Turnaround */
        for (n = 1; n; n--)
            SW_CLOCK_CYCLE();
        SWDIO_SET_OUTPUT(); // SWDIO_SET_OUTPUT

        /* Write data */
        val = *data;
        parity = 0;

        for (n = 32; n; n--)
        {
            SW_WRITE_BIT(val); /* Write WDATA[0:31] */
            parity += val;
            val >>= 1;
        }
        SW_WRITE_BIT(parity); /* Write Parity Bit */
    }
    return bSuccess;
}

// Data send out LSB first
static void SW_SendData(uint16_t data)
{
    for (uint8_t i = 0; i < 16U; i++)
    {
        SW_WRITE_BIT(data & 0x01);
        data >>= 1;
    }
}

// JTAG Switch to SWD
void SW_JTAG2SWD(void)
{
    uint16_t i;
    SWDIO_SET();
    for (i = 0; i < 52; i++)
        SW_CLOCK_CYCLE();
    SW_SendData(0xE79E);
    for (i = 0; i < 52; i++)
        SW_CLOCK_CYCLE();
    SWDIO_CLR();
    for (i = 0; i < 16; i++)
        SW_CLOCK_CYCLE();
}

bool SW_InitDebug(void)
{
    SW_JTAG2SWD();
    if (!SW_ReadDP(DP_IDCODE, NULL))
        return false;

    if (!SW_WriteDP(DP_ABORT, STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR))
        return false;

    /* Power ups */
    if (!SW_WriteDP(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ))
        return false;

    uint32_t uTemp;
    // /* Ensure CTRL/STAT register selected in DPBANKSEL */
    // if (!SW_WriteDP(DP_SELECT, 0))
    //     return false;
    uAP = 0;
    uAPBank = 0;

    // if (!SW_WriteAP(AP_CSW, 0xB000000 | CSW_SIZE32 | CSW_SADDRINC))
    //     return false;

    if (!SW_ReadAP(AP_IDR, &uTemp))
        return false;
    printf("AP_IDR:0x%08X\r\n", uTemp);

    if (!SW_ReadAP(AP_CSW, &uTemp))
        return false;
    uTemp = uTemp & 0xFFFFFFC8 | CSW_SIZE32 | CSW_SADDRINC;
    if (!SW_WriteAP(AP_CSW, uTemp))
        return false;

    // Enable debug
    if (!SW_WriteData(DBG_HCSR, DBGKEY | C_HALT | C_DEBUGEN))
        return false;

    if (!SW_ReadData(DBG_HCSR, &uTemp))
        return false;
    if ((uTemp & S_HALT) == 0)
        return false;

    if (!SW_WriteData(DBG_EMCR, VC_CORERESET))
        return false;

    // reset core
    if (!SW_WriteAP(AP_TAR, DBG_AIRCR))
        return false;

    if (!SW_WriteAP(AP_DRW, 0x05FA0004))
        return false;

    HAL_Delay(10);

    SW_JTAG2SWD();

    HAL_Delay(5);

    SW_ReadDP(DP_IDCODE, &uTemp);

    // if (!SW_ReadDP(DP_IDCODE, &uTemp))
    //     return false;

    printf("DP_IDCODE:0x%08X\r\n", uTemp);

    if (!SW_WriteDP(DP_ABORT, STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR))
        return false;

    /* Power ups */
    if (!SW_WriteDP(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ))
        return false;

    // /* Ensure CTRL/STAT register selected in DPBANKSEL */
    // if (!SW_WriteDP(DP_SELECT, 0))
    //     return false;
    uAP = 0;
    uAPBank = 0;

    // if (!SW_WriteAP(AP_CSW, 0xB000000 | CSW_SIZE32 | CSW_SADDRINC))
    //     return false;

    if (!SW_ReadAP(AP_IDR, &uTemp))
        return false;
    printf("AP_IDR:0x%08X\r\n", uTemp);

    // if (!SW_WriteData(DBG_AIRCR, 0x05FA0004))
    //     return false;

    // if (!SW_ReadData(DBG_HCSR, &uTemp))
    //     return false;
    // if ((uTemp & (S_RESET_ST | S_HALT)) == (S_RESET_ST | S_HALT))
    //     return true;
    // printf("DBG_HCSR:0x%08X\r\n", uTemp);

    return true;
}

bool SW_ReadDP(const uint8_t adr, uint32_t *val)
{
    uint8_t tmp_in = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(adr);
    return SWD_Transfer(tmp_in, val);
}

bool SW_WriteDP(const uint8_t adr, uint32_t val)
{
    uint8_t req = SWD_REG_DP | SWD_REG_W | SWD_REG_ADR(adr);
    return SWD_Transfer(req, &val);
}

bool SW_ReadAP(const uint32_t adr, uint32_t *val)
{
    if (uAP != (adr & APSEL) >> 24 || uAPBank != (adr & APBANKSEL) >> 4)
    {
        /* write DP select */
        if (!SW_WriteDP(DP_SELECT, (adr & APSEL) | (adr & APBANKSEL)))
            return false;
        uAP = (adr & APSEL) >> 24;
        uAPBank = (adr & APBANKSEL) >> 4;
    }

    /* first dummy read */
    SWD_Transfer(SWD_REG_AP | SWD_REG_R | SWD_REG_ADR(adr), val);

    return SW_ReadDP(DP_RDBUFF, val);
}

bool SW_WriteAP(const uint32_t adr, uint32_t val)
{
    if (uAP != (adr & APSEL) >> 24 || uAPBank != (adr & APBANKSEL) >> 4)
    {
        /* write DP select */
        if (!SW_WriteDP(DP_SELECT, (adr & APSEL) | (adr & APBANKSEL)))
            return false;
        uAP = (adr & APSEL) >> 24;
        uAPBank = (adr & APBANKSEL) >> 4;
    }

    /* write AP data */
    return SWD_Transfer(SWD_REG_AP | SWD_REG_W | SWD_REG_ADR(adr), &val);
}

bool SW_WriteData(const uint32_t addr, uint32_t data)
{
    if (!SW_WriteAP(AP_TAR, addr))
        return false;
    if (!SW_WriteAP(AP_DRW, data))
        return false;

    /* read DP buff */
    if (!SWD_Transfer(SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF), NULL))
        return false;

    uint32_t uTemp;
    if (!SW_ReadDP(DP_CTRL_STAT, &uTemp))
        return false;
    return (uTemp & (STICKYORUN | STICKYCMP | STICKYERR | WDATAERR)) == 0;
}

bool SW_ReadData(const uint32_t addr, uint32_t *data)
{
    if (!SW_WriteAP(AP_TAR, addr))
        return false;
    if (!SW_ReadAP(AP_DRW, data))
        return false;

    /* read DP buff */
    if (!SWD_Transfer(SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF), NULL))
        return false;

    uint32_t uTemp;
    if (!SW_ReadDP(DP_CTRL_STAT, &uTemp))
        return false;
    return (uTemp & (STICKYORUN | STICKYCMP | STICKYERR | WDATAERR)) == 0;
}

bool SW_ReadBlock(const uint32_t addr, uint32_t *buf, const uint32_t len)
{
    if (len == 0)
        return 0;

    if (!SW_WriteAP(AP_TAR, addr))
        return false;

    if (!SWD_Transfer(SWD_REG_AP | SWD_REG_R | SWD_REG_ADR(AP_DRW), buf))
        return false;

    /* DRW Read */
    for (uint16_t i = 0; i < len; i++)
    {
        if (!SWD_Transfer(SWD_REG_AP | SWD_REG_R | SWD_REG_ADR(AP_DRW), buf++))
            return false;
    }

    /* read DP buff */
    if (!SWD_Transfer(SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF), NULL))
        return false;

    uint32_t uTemp;
    if (!SW_ReadDP(DP_CTRL_STAT, &uTemp))
        return false;
    return (uTemp & (STICKYORUN | STICKYCMP | STICKYERR | WDATAERR)) == 0;
}

bool SW_WriteBlock(const uint32_t addr, uint32_t *buf, const uint16_t len)
{
    if (len == 0)
        return false;

    if (!SW_WriteAP(AP_TAR, addr))
        return false;

    /* DRW write */
    for (uint16_t i = 0; i < len; i++)
    {
        if (!SWD_Transfer(SWD_REG_AP | SWD_REG_W | SWD_REG_ADR(AP_DRW), buf++))
            return false;
    }

    /* read DP buff */
    if (!SWD_Transfer(SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF), NULL))
        return false;

    uint32_t uTemp;
    if (!SW_ReadDP(DP_CTRL_STAT, &uTemp))
        return false;
    return (uTemp & (STICKYORUN | STICKYCMP | STICKYERR | WDATAERR)) == 0;
}

#define TARGET_AUTO_INCREMENT_PAGE_SIZE (0x400)

bool SW_ReadMem(uint32_t address, uint32_t *data, uint32_t size)
{
    while (size > 0)
    {
        // Limit to auto increment page size
        uint32_t n = TARGET_AUTO_INCREMENT_PAGE_SIZE - (address & (TARGET_AUTO_INCREMENT_PAGE_SIZE - 1));
        if (size < n)
            n = size & 0xFFFFFFFC; // Only count complete words remaining
        if (!SW_ReadBlock(address, data, n / 4))
            return false;
        address += n;
        data += n / 4;
        size -= n;
    }
    return true;
}

bool SW_WriteMem(uint32_t address, uint32_t *data, uint32_t size)
{
    // Write word aligned blocks
    while (size > 0)
    {
        // Limit to auto increment page size
        uint32_t n = TARGET_AUTO_INCREMENT_PAGE_SIZE - (address & (TARGET_AUTO_INCREMENT_PAGE_SIZE - 1));
        if (size < n)
            n = size & 0xFFFFFFFC; // Only count complete words remaining
        if (!SW_WriteBlock(address, data, n / 4))
            return false;
        address += n;
        data += n / 4;
        size -= n;
    }
    return true;
}

#define REGWnR (1 << 16)
#define MAX_TIMEOUT 10000

bool SW_HaltCore(void)
{
    if (!SW_WriteData(DBG_HCSR, DBGKEY | C_HALT | C_DEBUGEN))
        return false;
    uint32_t val;
    if (!SW_ReadData(DBG_HCSR, &val))
        return false;
    return val & S_HALT;
}

bool SW_RestoreCore(void)
{
    if (!SW_WriteData(DBG_HCSR, DBGKEY | C_DEBUGEN))
        return false;
    if (!SW_WriteData(DBG_HCSR, DBGKEY))
        return false;
    // if (!SW_WriteDP(DP_CTRL_STAT, 0))
    //     return false;
    // return true;
    uint32_t val;
    if (!SW_ReadData(DBG_HCSR, &val))
        return false;
    return (val & S_HALT) == 0;
}

bool SW_WriteCoreReg(const uint32_t n, uint32_t val)
{
    if (!SW_WriteData(DBG_CRDR, val))
        return false;
    if (!SW_WriteData(DBG_CRSR, n | REGWnR))
        return false;
    // wait for S_REGRDY
    TIM6->CNT = 0;
    while (TIM6->CNT < MAX_TIMEOUT)
    {
        if (!SW_ReadData(DBG_HCSR, &val))
            return false;
        if (val & S_REGRDY)
            return true;
    }
    return false;
}

bool SW_ReadCoreReg(const uint32_t n, uint32_t *val)
{
    if (!SW_WriteData(DBG_CRSR, n))
        return false;
    TIM6->CNT = 0;
    while (TIM6->CNT < MAX_TIMEOUT)
    {
        if (!SW_ReadData(DBG_HCSR, val))
            return false;
        if (*val & S_REGRDY)
            break;
    }
    return SW_ReadData(DBG_CRDR, val);
}

void SendIdle(void)
{
    SWDIO_OUT(0);
    for (uint8_t n = 40; n; n--)
        SW_CLOCK_CYCLE();
}