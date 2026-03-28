/*
 * mcp9808.c
 *
 * MCP9808 digital temperature sensor I2C driver.
 * See mcp9808.h for full description.
 *
 * Temperature register (0x05) layout (16-bit, big-endian):
 *   Bit 15     : TCRIT_A  (alert flag — ignored here)
 *   Bit 14     : TUPPER_A (alert flag — ignored here)
 *   Bit 13     : TLOWER_A (alert flag — ignored here)
 *   Bit 12     : Sign bit (1 = negative)
 *   Bits [11:4]: Integer part of temperature
 *   Bits [3:0] : Fractional part (0.0625 °C per LSB)
 */

#include "mcp9808.h"
#include "supervisor_i2c.h"

/* ------------------------------------------------------------------
 * Internal helper: read a 16-bit register (big-endian).
 * ------------------------------------------------------------------ */
static int8_t prv_read_reg16(uint8_t reg, uint16_t *out)
{
    uint8_t buf[2] = {0u, 0u};
    int8_t  rc;

    rc = i2c_read_reg(MCP9808_ADDR, reg, buf, 2u);
    if (rc == I2C_OK)
    {
        *out = ((uint16_t)buf[0] << 8u) | (uint16_t)buf[1];
    }
    return rc;
}

/* ------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------ */

int8_t mcp9808_check_id(void)
{
    uint16_t manuf_id;
    uint16_t dev_id;

    if (prv_read_reg16(MCP9808_REG_MANUF_ID, &manuf_id) != I2C_OK)
    {
        return MCP9808_ERR;
    }
    if (manuf_id != MCP9808_MANUF_ID)
    {
        return MCP9808_ERR;
    }

    if (prv_read_reg16(MCP9808_REG_DEVICE_ID, &dev_id) != I2C_OK)
    {
        return MCP9808_ERR;
    }
    if ((dev_id & MCP9808_DEVICE_ID_MASK) != MCP9808_DEVICE_ID_VAL)
    {
        return MCP9808_ERR;
    }

    return MCP9808_OK;
}

int8_t mcp9808_read_temp(int16_t *temp_cc_out)
{
    uint16_t raw;
    int8_t   rc;

    rc = prv_read_reg16(MCP9808_REG_TEMP, &raw);
    if (rc != I2C_OK)
    {
        return MCP9808_ERR;
    }

    /* Strip the three alert flag bits [15:13]; keep sign + temperature [12:0] */
    raw &= 0x1FFFu;

    /*
     * Convert to centi-degC:
     *   Each LSB = 0.0625 °C = 6.25 centi-degC = 25/4 centi-degC
     *
     * For positive temperatures (bit 12 = 0):
     *   temp_cc = raw * 25 / 4
     *
     * For negative temperatures (bit 12 = 1):
     *   raw is in two's complement within 13 bits.
     *   Sign-extend to int16_t first, then apply the same scale.
     */
    {
        int16_t signed_raw;

        if (raw & 0x1000u)
        {
            /* Negative: sign-extend from 13 bits to 16 bits */
            signed_raw = (int16_t)(raw | 0xE000u);
        }
        else
        {
            signed_raw = (int16_t)raw;
        }

        /* Scale: * 25 / 4 using 32-bit intermediate */
        *temp_cc_out = (int16_t)(((int32_t)signed_raw * 25) / 4);
    }

    return MCP9808_OK;
}
