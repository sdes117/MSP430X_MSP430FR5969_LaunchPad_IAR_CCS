/*
 * supervisor_i2c.c
 *
 * Polling I2C master for MSP430FR5969 UCB0.
 * Mutex-protected so multiple FreeRTOS tasks can share the bus safely.
 */

#include "supervisor_i2c.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "driverlib.h"
#include <msp430.h>

/* Software spin limit per wait — at 8 MHz this is ~12 ms per iteration.
 * The hardware 31 ms CLK-low timeout fires before this expires on a stuck bus. */
#define I2C_SPIN_LIMIT  (100000UL)

/* Wait for (reg & mask)==expected; on timeout or NACK set rc and jump to lbl. */
#define WAIT_OR_GOTO(reg, mask, expected, err, lbl)         \
    do {                                                     \
        uint32_t _n = I2C_SPIN_LIMIT;                        \
        while (((reg) & (mask)) != (expected)) {             \
            if (--_n == 0UL) { rc = (err); goto lbl; }      \
            if (UCB0IFG & UCNACKIFG) {                       \
                UCB0CTLW0 |= UCTXSTP;                        \
                UCB0IFG   &= (uint16_t)~UCNACKIFG;           \
                rc = I2C_ERR_NACK; goto lbl;                 \
            }                                                \
        }                                                    \
    } while (0)

#define WAIT_TXIFG(lbl) WAIT_OR_GOTO(UCB0IFG,   UCTXIFG0, UCTXIFG0, I2C_ERR_TIMEOUT, lbl)
#define WAIT_RXIFG(lbl) WAIT_OR_GOTO(UCB0IFG,   UCRXIFG0, UCRXIFG0, I2C_ERR_TIMEOUT, lbl)
#define WAIT_STOP(lbl)  WAIT_OR_GOTO(UCB0CTLW0, UCTXSTP,  0,        I2C_ERR_TIMEOUT, lbl)
#define WAIT_START(lbl) WAIT_OR_GOTO(UCB0CTLW0, UCTXSTT,  0,        I2C_ERR_TIMEOUT, lbl)

static SemaphoreHandle_t xI2CMutex = NULL;

void supervisor_i2c_init(void)
{
    EUSCI_B_I2C_initMasterParam p = {
        .selectClockSource    = EUSCI_B_I2C_CLOCKSOURCE_SMCLK,
        .i2cClk               = 8000000UL,
        .dataRate             = EUSCI_B_I2C_SET_DATA_RATE_400KBPS,
        .byteCounterThreshold = 0u,
        .autoSTOPGeneration   = EUSCI_B_I2C_NO_AUTO_STOP,
    };
    EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &p);
    EUSCI_B_I2C_setTimeout(EUSCI_B0_BASE, EUSCI_B_I2C_TIMEOUT_31_MS);
    EUSCI_B_I2C_enable(EUSCI_B0_BASE);

    xI2CMutex = xSemaphoreCreateMutex();
    configASSERT(xI2CMutex != NULL);
}

void supervisor_i2c_recover(void)
{
    /* Disable UCB0, reset all state, re-enable.
     * Call after an RP reset to clear any stuck-bus state left by an
     * interrupted I2C transaction. Does NOT touch the mutex. */
    EUSCI_B_I2C_disable(EUSCI_B0_BASE);

    EUSCI_B_I2C_initMasterParam p = {
        .selectClockSource    = EUSCI_B_I2C_CLOCKSOURCE_SMCLK,
        .i2cClk               = 8000000UL,
        .dataRate             = EUSCI_B_I2C_SET_DATA_RATE_400KBPS,
        .byteCounterThreshold = 0u,
        .autoSTOPGeneration   = EUSCI_B_I2C_NO_AUTO_STOP,
    };
    EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &p);
    EUSCI_B_I2C_setTimeout(EUSCI_B0_BASE, EUSCI_B_I2C_TIMEOUT_31_MS);
    EUSCI_B_I2C_enable(EUSCI_B0_BASE);
}

int8_t i2c_write_reg(uint8_t addr, uint8_t reg, const uint8_t *data, uint8_t len)
{
    int8_t  rc = I2C_OK;
    uint8_t i;

    if (!data || !len) return I2C_ERR_ARG;

    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(200u)) != pdTRUE)
        return I2C_ERR_BUSY;

    if (EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE) == EUSCI_B_I2C_BUS_BUSY)
        { rc = I2C_ERR_BUSY; goto done; }

    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, addr);
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    UCB0IFG   &= (uint16_t)~(UCTXIFG0 | UCNACKIFG);
    UCB0CTLW0 |= UCTR | UCTXSTT;

    WAIT_TXIFG(done);
    UCB0TXBUF = reg;

    for (i = 0u; i < len; i++) {
        WAIT_TXIFG(done);
        UCB0TXBUF = data[i];
    }

    WAIT_TXIFG(done);
    UCB0CTLW0 |= UCTXSTP;
    WAIT_STOP(done);

done:
    xSemaphoreGive(xI2CMutex);
    return rc;
}

int8_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    int8_t  rc = I2C_OK;
    uint8_t i;

    if (!buf || !len) return I2C_ERR_ARG;

    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(200u)) != pdTRUE)
        return I2C_ERR_BUSY;

    if (EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE) == EUSCI_B_I2C_BUS_BUSY)
        { rc = I2C_ERR_BUSY; goto done; }

    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, addr);

    /* Phase 1: write register address then STOP.
     * Using STOP+START (not repeated-START) so that CircuitPython I2CTarget
     * sees two fully independent transactions.  With repeated-START the RP
     * may serve the read phase before on_i2c_write() commits _reg_ptr, which
     * causes the read to return data at the wrong offset. */
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    UCB0IFG   &= (uint16_t)~(UCTXIFG0 | UCNACKIFG);
    UCB0CTLW0 |= UCTR | UCTXSTT;
    WAIT_TXIFG(done);
    UCB0TXBUF = reg;
    WAIT_TXIFG(done);
    UCB0CTLW0 |= UCTXSTP;
    WAIT_STOP(done);

    /* Phase 2: new START for read.
     * Clock-stretching on the target holds SCL until req.write() is called,
     * so no explicit delay is needed between the STOP and this START. */
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
    UCB0IFG   &= (uint16_t)~(UCRXIFG0 | UCNACKIFG);

    if (len == 1u) {
        UCB0CTLW0 |= UCTXSTT;
        WAIT_START(done);
        UCB0CTLW0 |= UCTXSTP;
        WAIT_RXIFG(done);
        buf[0] = UCB0RXBUF;
        WAIT_STOP(done);
        goto done;
    }

    UCB0CTLW0 |= UCTXSTT;
    for (i = 0u; i < (uint8_t)(len - 1u); i++) {
        WAIT_RXIFG(done);
        if (i == (uint8_t)(len - 2u)) UCB0CTLW0 |= UCTXSTP;
        buf[i] = UCB0RXBUF;
    }
    WAIT_RXIFG(done);
    buf[len - 1u] = UCB0RXBUF;
    WAIT_STOP(done);

done:
    xSemaphoreGive(xI2CMutex);
    return rc;
}
