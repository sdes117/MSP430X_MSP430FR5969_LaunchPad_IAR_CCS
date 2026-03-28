/*
 * rp_liveness.c
 *
 * RP2350 liveness monitor FreeRTOS task.
 * See rp_liveness.h for full description.
 */

#include "rp_liveness.h"
#include "supervisor_i2c.h"
#include "rp_regmap.h"
#include "fault_counters.h"
#include "driverlib.h"
#include <msp430.h>

/* ------------------------------------------------------------------
 * GPIO pin definitions (matching Init_GPIO in main.c)
 * ------------------------------------------------------------------ */
#define WDT_RP2MSP1_PORT    GPIO_PORT_P3
#define WDT_RP2MSP1_PIN     GPIO_PIN7

#define WDT_RP2MSP2_PORT    GPIO_PORT_P3
#define WDT_RP2MSP2_PIN     GPIO_PIN1

/* ~RESET_RP: low = RP held in reset, high = released */
#define RESET_RP_PORT       GPIO_PORT_P2
#define RESET_RP_PIN        GPIO_PIN6

/* EN_3V3: high = powered, low = off */
#define EN_3V3_PORT         GPIO_PORT_P3
#define EN_3V3_PIN          GPIO_PIN0

/* WDT_MSP2RP: MSP heartbeat output to RP */
#define WDT_MSP2RP_PORT     GPIO_PORT_P3
#define WDT_MSP2RP_PIN      GPIO_PIN5

/* ------------------------------------------------------------------
 * Exported state
 * ------------------------------------------------------------------ */
volatile rp_live_state_t g_rp_live_state     = RP_LIVE_OK;
volatile uint8_t         g_rp_powercycle_count = 0u;

/* ------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------ */

static void rp_assert_reset(void)
{
    GPIO_setOutputLowOnPin(RESET_RP_PORT, RESET_RP_PIN);
}

static void rp_release_reset(void)
{
    GPIO_setOutputHighOnPin(RESET_RP_PORT, RESET_RP_PIN);
}

static void rp_power_off(void)
{
    rp_assert_reset();                          /* hold reset first */
    GPIO_setOutputLowOnPin(EN_3V3_PORT, EN_3V3_PIN);
}

static void rp_power_on(void)
{
    GPIO_setOutputHighOnPin(EN_3V3_PORT, EN_3V3_PIN);
    /* Give rails time to stabilise before releasing reset */
    vTaskDelay(pdMS_TO_TICKS(200u));
    rp_release_reset();
}

/*
 * Read the 16-bit heartbeat counter from the RP regmap.
 * Returns 0xFFFF on I2C error (counter should never be 0xFFFF in normal use
 * since it wraps at 16-bit, but use separately tracked error flag for safety).
 */
static uint16_t read_rp_hb_counter(int8_t *i2c_err_out)
{
    uint8_t buf[2] = {0u, 0u};
    int8_t  rc;

    rc = i2c_read_reg_retry(RP_I2C_ADDR, REG_HB0, buf, 2u);
    if (i2c_err_out != (void *)0) {
        *i2c_err_out = rc;
    }
    if (rc != I2C_OK) {
        return 0xFFFFu;
    }
    return (uint16_t)((uint16_t)buf[1] << 8u) | (uint16_t)buf[0];
}

/* ------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------ */

static void prvRpLivenessTask(void *pvParameters)
{
    TickType_t xNextWake = xTaskGetTickCount();

    /* Snapshot state for edge detection */
    uint8_t  gpio1_last  = GPIO_getInputPinValue(WDT_RP2MSP1_PORT, WDT_RP2MSP1_PIN);
    uint8_t  gpio2_last  = GPIO_getInputPinValue(WDT_RP2MSP2_PORT, WDT_RP2MSP2_PIN);
    uint16_t hb_last     = 0u;
    uint8_t  msp2rp_state = 0u;  /* current MSP heartbeat output level */

    /* Stall counters (seconds without activity) */
    uint32_t gpio_stall_s  = 0u;
    uint32_t i2c_stall_s   = 0u;

    /* Rung-3 reset attempt counter; resets to 0 on confirmed RP recovery */
    uint8_t  reset_count       = 0u;

    /* Level-2 bus recovery attempt counter; resets to 0 on I2C success */
    uint8_t  bus_recover_count = 0u;

    int8_t   i2c_err       = I2C_OK;
    uint8_t  first_poll    = 1u;

    (void)pvParameters;

    /* Power on RP (enables EN_3V3, waits 200 ms, then releases ~RESET_RP). */
    rp_power_on();

    /* Seed HB counter before entering the loop */
    hb_last = read_rp_hb_counter(&i2c_err);

    for (;;)
    {
        vTaskDelayUntil(&xNextWake, pdMS_TO_TICKS(1000u));  /* 1 Hz poll */

        /* --- Toggle MSP heartbeat output to RP --- */
        msp2rp_state ^= 1u;
        if (msp2rp_state != 0u) {
            GPIO_setOutputHighOnPin(WDT_MSP2RP_PORT, WDT_MSP2RP_PIN);
        } else {
            GPIO_setOutputLowOnPin(WDT_MSP2RP_PORT, WDT_MSP2RP_PIN);
        }

        /* -------------------------------------------------------
         * 1. GPIO heartbeat check
         * ------------------------------------------------------- */
        uint8_t gpio1_now = GPIO_getInputPinValue(WDT_RP2MSP1_PORT, WDT_RP2MSP1_PIN);
        uint8_t gpio2_now = GPIO_getInputPinValue(WDT_RP2MSP2_PORT, WDT_RP2MSP2_PIN);

        uint8_t gpio_toggled = ((gpio1_now != gpio1_last) || (gpio2_now != gpio2_last));

        if (gpio_toggled || (first_poll != 0u)) {
            gpio_stall_s = 0u;
            gpio1_last   = gpio1_now;
            gpio2_last   = gpio2_now;
        } else {
            gpio_stall_s++;
            /* Raise F_WDT_RP_MISS once the stall threshold is first crossed */
            if (gpio_stall_s == RP_LIVENESS_GPIO_TIMEOUT_S) {
                fault_set(MSP_FAULT_WDT_RP_MISS, MSP_FCNT_WDT_RP_MISS);
            }
        }

        /* -------------------------------------------------------
         * 2. I2C heartbeat check
         * ------------------------------------------------------- */
        uint16_t hb_now = read_rp_hb_counter(&i2c_err);

        if ((i2c_err == I2C_OK) && ((hb_now != hb_last) || (first_poll != 0u))) {
            i2c_stall_s       = 0u;
            bus_recover_count = 0u;   /* link healthy — reset recovery attempt counter */
            hb_last           = hb_now;
        } else {
            i2c_stall_s++;
            /* Level-2: attempt bus recovery on I2C error, capped at 3 per incident */
            if (i2c_err != I2C_OK) {
                fault_set(MSP_FAULT_I2C_LINK_ERR, MSP_FCNT_I2C_LINK_ERR);
                if (bus_recover_count < I2C_BUS_RECOVER_MAX_ATTEMPTS) {
                    bus_recover_count++;
                    i2c_bus_recover();
                }
            }
        }

        first_poll = 0u;

        /* -------------------------------------------------------
         * 3. Escalation state machine
         * ------------------------------------------------------- */
        switch (g_rp_live_state)
        {
        case RP_LIVE_OK:
            if (gpio_stall_s >= RP_LIVENESS_GPIO_TIMEOUT_S) {
                g_rp_live_state = RP_LIVE_GPIO_STALL;
                gpio_stall_s    = 0u;
            } else if (i2c_stall_s >= RP_LIVENESS_I2C_TIMEOUT_S) {
                g_rp_live_state = RP_LIVE_I2C_STALL;
                i2c_stall_s     = 0u;
            } else if (gpio_toggled) {
                /* RP actively heartbeating — confirmed healthy, clear reset tally */
                reset_count = 0u;
            }
            break;

        case RP_LIVE_I2C_STALL:
            /* I2C stall only: wait for GPIO to confirm RP is still alive */
            if (gpio_toggled) {
                /* GPIO heartbeat present → I2C issue only, not a full CPU stall */
                /* Recovery: do nothing, let I2C recover on its own */
            }
            if (i2c_stall_s == 0u) {
                /* I2C recovered */
                g_rp_live_state = RP_LIVE_OK;
            } else if (gpio_stall_s >= RP_LIVENESS_GPIO_TIMEOUT_S) {
                /* Both channels stalled → escalate to GPIO stall */
                g_rp_live_state = RP_LIVE_GPIO_STALL;
                gpio_stall_s    = 0u;
            }
            break;

        case RP_LIVE_GPIO_STALL:
            /* Both I2C and GPIO unresponsive → assert reset */
            rp_assert_reset();
            g_rp_live_state = RP_LIVE_RESET;
            /* Hold reset for RP_RESET_HOLD_S seconds (handled next iterations) */
            gpio_stall_s = 0u;
            i2c_stall_s  = 0u;
            break;

        case RP_LIVE_RESET:
            /*
             * Hold reset for RP_RESET_HOLD_S seconds (rung 3).
             * gpio_stall_s is repurposed as the hold-time counter here.
             *
             * After releasing, escalate to POWERCYCLE (rung 4) if we have
             * already exceeded RP_RESET_MAX_ATTEMPTS without recovery;
             * otherwise return to OK and let the liveness checks run again.
             */
            if (gpio_stall_s >= RP_RESET_HOLD_S) {
                rp_release_reset();
                gpio_stall_s    = 0u;
                i2c_stall_s     = 0u;
                hb_last         = read_rp_hb_counter(&i2c_err);
                first_poll      = 1u;
                reset_count++;

                if (reset_count >= RP_RESET_MAX_ATTEMPTS) {
                    /* Rung 3 exhausted → escalate to rung 4 (power-cycle) */
                    g_rp_live_state = RP_LIVE_POWERCYCLE;
                } else {
                    g_rp_live_state = RP_LIVE_OK;
                }
            } else {
                /* Early-recovery check: RP heartbeat returned during hold */
                if (gpio_toggled) {
                    rp_release_reset();
                    gpio_stall_s    = 0u;
                    i2c_stall_s     = 0u;
                    /* Don't increment reset_count — RP recovered on its own */
                    g_rp_live_state = RP_LIVE_OK;
                } else {
                    gpio_stall_s++;
                }
            }
            break;

        case RP_LIVE_POWERCYCLE:
        {
            static uint32_t pc_off_counter = 0u;

            if (pc_off_counter == 0u) {
                /* Start the power-off phase */
                rp_power_off();
                g_rp_powercycle_count++;
            }
            pc_off_counter++;

            if (pc_off_counter >= RP_POWERCYCLE_OFF_S) {
                pc_off_counter  = 0u;
                gpio_stall_s    = 0u;
                i2c_stall_s     = 0u;
                reset_count     = 0u;   /* fresh start after power-cycle */
                rp_power_on();
                first_poll      = 1u;
                hb_last         = read_rp_hb_counter(&i2c_err);

                if (g_rp_powercycle_count >= RP_POWERCYCLE_MAX_ATTEMPTS) {
                    g_rp_live_state = RP_LIVE_SURVIVAL;
                } else {
                    /*
                     * Rung 4: "restart in minimum operation mode (SAFE mode)".
                     * Write MODE_CMD_ENTER_SAFE to RP regmap so RP boots into
                     * SAFE rather than its default NOMINAL path.
                     */
                    uint8_t safe_cmd = MODE_CMD_ENTER_SAFE;
                    (void)i2c_write_reg(RP_I2C_ADDR, REG_MODE_CMD,
                                        &safe_cmd, 1u);
                    g_rp_live_state = RP_LIVE_OK;
                }
            }
        }
        break;

        case RP_LIVE_SURVIVAL:
        {
            /*
             * RP powered off; supervisor-only safety monitoring active.
             * Xlsx rung 5: "Exit only when conditions recover + policy allows."
             *
             * Recovery conditions (all must hold for RP_SURVIVAL_EXIT_STABLE_S):
             *   - No active P0 faults (battery UV, overtemp, etc.)
             *   - Battery above recovery voltage (checked via fault bitmap)
             *
             * If RP_SURVIVAL_EXIT_STABLE_S == 0 the exit is disabled and only
             * a full MSP reset can escape SURVIVAL.
             */
            static uint32_t surv_stable_s = 0u;

            rp_power_off();

#if (RP_SURVIVAL_EXIT_STABLE_S > 0u)
            {
                static const uint16_t P0_MASK =
                      MSP_FAULT_PWR_UV_BATT
                    | MSP_FAULT_WDT_EXT_TRIP
                    | MSP_FAULT_THERM_OVERTEMP;

                if ((g_msp_fault_bitmap & P0_MASK) == 0u)
                {
                    surv_stable_s++;
                    if (surv_stable_s >= RP_SURVIVAL_EXIT_STABLE_S)
                    {
                        /* Conditions stable long enough — re-enable RP in SAFE mode */
                        surv_stable_s       = 0u;
                        g_rp_powercycle_count = 0u;
                        reset_count         = 0u;
                        gpio_stall_s        = 0u;
                        i2c_stall_s         = 0u;
                        first_poll          = 1u;
                        rp_power_on();
                        hb_last             = read_rp_hb_counter(&i2c_err);

                        uint8_t safe_cmd = MODE_CMD_ENTER_SAFE;
                        (void)i2c_write_reg(RP_I2C_ADDR, REG_MODE_CMD,
                                            &safe_cmd, 1u);
                        g_rp_live_state = RP_LIVE_OK;
                    }
                }
                else
                {
                    surv_stable_s = 0u;   /* reset soak timer on any P0 fault */
                }
            }
#else
            surv_stable_s = 0u;  /* suppress unused-variable warning */
            /* No exit from SURVIVAL except MSP POR. */
#endif
        }
        break;

        default:
            g_rp_live_state = RP_LIVE_OK;
            break;
        }

    }
}

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */

void rp_liveness_task_create(void)
{
    xTaskCreate(prvRpLivenessTask,
                "RpLive",
                RP_LIVENESS_STACK_SIZE,
                NULL,
                RP_LIVENESS_PRIORITY,
                NULL);
}
