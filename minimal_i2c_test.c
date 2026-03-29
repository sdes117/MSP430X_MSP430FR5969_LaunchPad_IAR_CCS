/*
 * minimal_i2c_test.c
 *
 * Minimal I2C test for INA219 detection and reading.
 * Integrates as option 2 in main.c (mainDEMO_TYPE = 2)
 *
 * LED Patterns:
 * - Slow blink (1Hz): INA219 detected, reading values continuously
 * - Fast blink (4Hz): INA219 NOT detected
 *
 * Debug: Check g_ina_* global variables in CCS debugger for real-time values
 */

#include <msp430.h>
#include "driverlib.h"
#include <stdint.h>
#include "supervisor_i2c.h"

/* INA219 at address 0x44 */
#define INA219_ADDR         (0x44u)
#define INA219_REG_CONFIG   (0x00u)
#define INA219_REG_SHUNT_V  (0x01u)
#define INA219_REG_BUS_V    (0x02u)
#define INA219_REG_POWER    (0x03u)
#define INA219_REG_CURRENT  (0x04u)
#define INA219_REG_CALIB    (0x05u)

/* INA219 configuration: 32V range, /8 gain, 12-bit, continuous */
#define INA219_CONFIG_VALUE (0x3FFFu)
/* Calibration for 0.1 ohm shunt, 2A max */
#define INA219_CALIB_VALUE  (6710u)
#define INA219_CURRENT_LSB_UA (61u)
#define INA219_BUS_V_LSB_MV (4u)

/* ============================================================
 * Global variables for CCS debugger inspection
 * ============================================================ */
volatile int8_t  g_ina_status = -99;      /* I2C result code */
volatile uint16_t g_ina_config = 0;       /* Config register readback */
volatile uint16_t g_ina_vbus_mv = 0;      /* Bus voltage in mV */
volatile int16_t  g_ina_current_ma = 0;   /* Current in mA (signed) */
volatile uint16_t g_ina_shunt_uv = 0;     /* Shunt voltage in 10uV units */
volatile uint32_t g_ina_read_count = 0;   /* Number of successful reads */

/* ============================================================
 * I2C read/write (bypasses FreeRTOS mutex for bare-metal test)
 * ============================================================ */
#define I2C_SW_TIMEOUT_ITERS (100000UL)

static int8_t prv_i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    int8_t rc = I2C_OK;
    uint8_t i;
    uint32_t timeout;
    
    if ((buf == (void *)0) || (len == 0u)) {
        return I2C_ERR_ARG;
    }
    
    if (EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE) == EUSCI_B_I2C_BUS_BUSY) {
        return I2C_ERR_BUSY;
    }
    
    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, addr);
    
    /* Phase 1: write register address (TX) */
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    UCB0IFG &= (uint16_t)~(UCTXIFG0 | UCNACKIFG);
    UCB0CTLW0 |= UCTR | UCTXSTT;
    
    /* Wait for TX ready */
    timeout = I2C_SW_TIMEOUT_ITERS;
    while (!(UCB0IFG & UCTXIFG0)) {
        if (--timeout == 0) { rc = I2C_ERR_TIMEOUT; goto cleanup; }
        if (UCB0IFG & UCNACKIFG) { 
            UCB0CTLW0 |= UCTXSTP;
            UCB0IFG &= ~UCNACKIFG;
            rc = I2C_ERR_NACK; goto cleanup;
        }
    }
    UCB0TXBUF = reg;
    
    /* Wait for reg byte to shift out */
    timeout = I2C_SW_TIMEOUT_ITERS;
    while (!(UCB0IFG & UCTXIFG0)) {
        if (--timeout == 0) { rc = I2C_ERR_TIMEOUT; goto cleanup; }
        if (UCB0IFG & UCNACKIFG) { 
            UCB0CTLW0 |= UCTXSTP;
            UCB0IFG &= ~UCNACKIFG;
            rc = I2C_ERR_NACK; goto cleanup;
        }
    }
    
    /* Phase 2: repeated START, switch to RX */
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
    UCB0CTLW0 |= UCTXSTT;
    
    /* Wait for START to complete */
    timeout = I2C_SW_TIMEOUT_ITERS;
    while (UCB0CTLW0 & UCTXSTT) {
        if (--timeout == 0) { rc = I2C_ERR_TIMEOUT; goto cleanup; }
        if (UCB0IFG & UCNACKIFG) { 
            UCB0CTLW0 |= UCTXSTP;
            UCB0IFG &= ~UCNACKIFG;
            rc = I2C_ERR_NACK; goto cleanup;
        }
    }
    
    /* Read bytes */
    for (i = 0u; i < len; i++) {
        if (i == (len - 1u)) {
            UCB0CTLW0 |= UCTXSTP;  /* Send STOP after last byte */
        }
        timeout = I2C_SW_TIMEOUT_ITERS;
        while (!(UCB0IFG & UCRXIFG0)) {
            if (--timeout == 0) { rc = I2C_ERR_TIMEOUT; goto cleanup; }
        }
        buf[i] = UCB0RXBUF;
    }
    
    /* Wait for STOP */
    timeout = I2C_SW_TIMEOUT_ITERS;
    while (UCB0CTLW0 & UCTXSTP) {
        if (--timeout == 0) { rc = I2C_ERR_TIMEOUT; goto cleanup; }
    }
    
cleanup:
    return rc;
}

static int8_t prv_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, uint8_t len)
{
    int8_t rc = I2C_OK;
    uint8_t i;
    uint32_t timeout;
    
    if ((data == (void *)0) || (len == 0u)) {
        return I2C_ERR_ARG;
    }
    
    if (EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE) == EUSCI_B_I2C_BUS_BUSY) {
        return I2C_ERR_BUSY;
    }
    
    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, addr);
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    UCB0IFG &= (uint16_t)~(UCTXIFG0 | UCNACKIFG);
    UCB0CTLW0 |= UCTR | UCTXSTT;
    
    /* Wait for TX ready, send register address */
    timeout = I2C_SW_TIMEOUT_ITERS;
    while (!(UCB0IFG & UCTXIFG0)) {
        if (--timeout == 0) { rc = I2C_ERR_TIMEOUT; goto cleanup; }
        if (UCB0IFG & UCNACKIFG) {
            UCB0CTLW0 |= UCTXSTP;
            UCB0IFG &= ~UCNACKIFG;
            rc = I2C_ERR_NACK; goto cleanup;
        }
    }
    UCB0TXBUF = reg;
    
    /* Send data bytes */
    for (i = 0u; i < len; i++) {
        timeout = I2C_SW_TIMEOUT_ITERS;
        while (!(UCB0IFG & UCTXIFG0)) {
            if (--timeout == 0) { rc = I2C_ERR_TIMEOUT; goto cleanup; }
            if (UCB0IFG & UCNACKIFG) {
                UCB0CTLW0 |= UCTXSTP;
                UCB0IFG &= ~UCNACKIFG;
                rc = I2C_ERR_NACK; goto cleanup;
            }
        }
        UCB0TXBUF = data[i];
    }
    
    /* Wait for last byte, then STOP */
    timeout = I2C_SW_TIMEOUT_ITERS;
    while (!(UCB0IFG & UCTXIFG0)) {
        if (--timeout == 0) { rc = I2C_ERR_TIMEOUT; goto cleanup; }
    }
    UCB0CTLW0 |= UCTXSTP;
    
    timeout = I2C_SW_TIMEOUT_ITERS;
    while (UCB0CTLW0 & UCTXSTP) {
        if (--timeout == 0) { rc = I2C_ERR_TIMEOUT; goto cleanup; }
    }
    
cleanup:
    return rc;
}

/* ============================================================
 * INA219 helper functions
 * ============================================================ */
static int8_t ina_write_reg16(uint8_t reg, uint16_t val)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(val >> 8);    /* MSB first (big-endian) */
    buf[1] = (uint8_t)(val & 0xFF);
    return prv_i2c_write(INA219_ADDR, reg, buf, 2);
}

static int8_t ina_read_reg16(uint8_t reg, uint16_t *out)
{
    uint8_t buf[2] = {0, 0};
    int8_t rc = prv_i2c_read(INA219_ADDR, reg, buf, 2);
    if (rc == I2C_OK) {
        *out = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    }
    return rc;
}

static int8_t ina_init(void)
{
    int8_t rc;
    
    /* Write configuration register */
    rc = ina_write_reg16(INA219_REG_CONFIG, INA219_CONFIG_VALUE);
    if (rc != I2C_OK) return rc;
    
    /* Write calibration register */
    rc = ina_write_reg16(INA219_REG_CALIB, INA219_CALIB_VALUE);
    return rc;
}

static int8_t ina_read_values(void)
{
    uint16_t bus_raw, cur_raw, shunt_raw;
    int8_t rc;
    
    /* Read bus voltage */
    rc = ina_read_reg16(INA219_REG_BUS_V, &bus_raw);
    if (rc != I2C_OK) return rc;
    
    /* Check overflow flag */
    if (bus_raw & 0x0001u) return -1;
    
    /* Convert: shift out CNVR/OVF bits, multiply by 4mV/LSB */
    g_ina_vbus_mv = (uint16_t)((bus_raw >> 3) * INA219_BUS_V_LSB_MV);
    
    /* Read current */
    rc = ina_read_reg16(INA219_REG_CURRENT, &cur_raw);
    if (rc != I2C_OK) return rc;
    
    /* Convert to mA */
    {
        int16_t cur_signed = (int16_t)cur_raw;
        int32_t cur_ua = (int32_t)cur_signed * (int32_t)INA219_CURRENT_LSB_UA;
        g_ina_current_ma = (int16_t)(cur_ua / 1000);
    }
    
    /* Read shunt voltage (optional, for debug) */
    rc = ina_read_reg16(INA219_REG_SHUNT_V, &shunt_raw);
    if (rc == I2C_OK) {
        g_ina_shunt_uv = shunt_raw;  /* 10uV/LSB */
    }
    
    return I2C_OK;
}

/* ============================================================
 * LED control and delay
 * ============================================================ */
static void delay_ms(uint16_t ms)
{
    /* Rough delay at 8MHz DCO */
    while (ms--) {
        __delay_cycles(8000);
    }
}

static void led_on(void)
{
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
}

static void led_off(void)
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
}

/* ============================================================
 * Main test function (called from main.c when mainDEMO_TYPE=2)
 * ============================================================ */
void main_minimal_i2c_test(void)
{
    int8_t rc;
    uint8_t detected = 0;
    
    /* Probe INA219 by reading config register */
    rc = ina_read_reg16(INA219_REG_CONFIG, &g_ina_config);
    g_ina_status = rc;
    
    if (rc == I2C_OK) {
        /* INA219 detected! Initialize it */
        detected = 1;
        ina_init();
    }
    
    /* Main loop */
    while (1) {
        if (detected) {
            /* Slow blink (1Hz) + continuous reading */
            led_on();
            delay_ms(500);
            
            /* Read INA219 values */
            rc = ina_read_values();
            g_ina_status = rc;
            if (rc == I2C_OK) {
                g_ina_read_count++;
            }
            
            led_off();
            delay_ms(500);
        } else {
            /* Fast blink (4Hz) - INA219 not found */
            led_on();
            delay_ms(125);
            led_off();
            delay_ms(125);
        }
    }
}
