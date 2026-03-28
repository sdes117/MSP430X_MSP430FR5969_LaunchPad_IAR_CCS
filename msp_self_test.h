/*
 * msp_self_test.h
 *
 * Power-On Self Test (POST) for the MSP430FR supervisor.
 *
 * msp_self_test_run() is called synchronously before the FreeRTOS
 * scheduler starts.  It probes the critical subsystems, logs the
 * result bitmap into g_post_result, and raises fault bits for any
 * critical sensor that is absent.
 *
 * POST checks (bitmap bits in g_post_result):
 *   POST_WDT1_HIGH     — WDT1 output (P2.2) is high (not stuck low).
 *   POST_WDT2_HIGH     — WDT2 output (P3.4) is high (not stuck low).
 *   POST_I2C_BUS_FREE  — SDA (P1.6) and SCL (P1.7) both sampled high
 *                        (bus not stuck).
 *   POST_INA219_FOUND  — INA219 battery sensor responded on I2C.
 *   POST_MCP9808_FOUND — MCP9808 temperature sensor responded on I2C.
 *   POST_RP_MAGIC_OK   — RP2350 I2C regmap magic bytes and version valid.
 *
 * A bit set = check PASSED.  A bit clear = check FAILED or not run.
 *
 * Critical failures (POST_INA219_FOUND clear) immediately raise
 * MSP_FAULT_I2C_LINK_ERR so the fault subsystem is aware before
 * the first task poll.
 *
 * Note: if g_last_sysrstiv indicates an external WDT reset
 * (SYSRSTIV_WDTKEY), POST also sets MSP_FAULT_WDT_EXT_TRIP.
 */

#ifndef MSP_SELF_TEST_H_
#define MSP_SELF_TEST_H_

#include <stdint.h>

/* ------------------------------------------------------------------
 * POST result bitmap
 * ------------------------------------------------------------------ */
#define POST_WDT1_HIGH      (0x01u)  /* bit0 */
#define POST_WDT2_HIGH      (0x02u)  /* bit1 */
#define POST_I2C_BUS_FREE   (0x04u)  /* bit2 */
#define POST_INA219_FOUND   (0x08u)  /* bit3 */
#define POST_MCP9808_FOUND  (0x10u)  /* bit4 */
#define POST_RP_MAGIC_OK    (0x20u)  /* bit5 */

/* Bits [7:6] reserved for future checks */

/*
 * POST result bitmap from the last boot.
 * Written by msp_self_test_run() before the scheduler starts.
 */
extern volatile uint8_t g_post_result;

/* ------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------ */

/*
 * Run POST synchronously.
 * reset_cause : value of g_last_sysrstiv (captured in main.c before
 *               SYSRSTIV is cleared by the first read in main_blinky).
 * boot_count  : current boot counter value.
 *
 * Must be called before vTaskStartScheduler().
 */
void msp_self_test_run(uint16_t reset_cause, uint16_t boot_count);

#endif /* MSP_SELF_TEST_H_ */
