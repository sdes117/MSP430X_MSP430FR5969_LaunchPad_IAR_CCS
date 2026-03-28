/*
 * eps_3v3_monitor.h
 *
 * EPS 3.3 V rail current-limit monitor for the RP2350 supply.
 * Implements the eps_current_limit_control_3v3 task.
 *
 * Reads the INA219 at I2C address 0x44 ("3V3 REG") on the BATT_SENSE bus.
 * This sensor measures current on the 3.3 V rail supplying the RP2350.
 *
 * On sustained overcurrent (current > EPS_3V3_OC_THRESHOLD_MA for
 * EPS_3V3_OC_TRIP_THRESHOLD consecutive polls):
 *   1. Assert ~RESET_RP (P2.6 LOW) to hold the RP in reset.
 *   2. De-assert EN_3V3 (P3.0 LOW) to cut the 3.3 V supply.
 *   3. Wait EPS_3V3_CYCLE_OFF_MS for capacitors to discharge.
 *   4. Re-assert EN_3V3 (P3.0 HIGH) to restore power.
 *   5. Wait EPS_3V3_SETTLE_MS for rail to stabilise.
 *   6. Release ~RESET_RP (P2.6 HIGH).
 *
 * If the rail does not return to normal within EPS_3V3_MAX_ATTEMPTS
 * consecutive recovery cycles, MSP_FAULT_PWR_UV_LOAD is set and cycling
 * stops.  The fault clears when the measured current stays below threshold
 * for EPS_3V3_STABLE_POLLS consecutive polls.
 *
 * Pin definitions mirror rp_liveness.c for independence; the power_policy_
 * enforcer 1 Hz tick re-asserts the mode-correct EN_3V3 state after any
 * cycle that leaves the pin in the wrong state.
 *
 * Calibration defaults assume a 100 mΩ shunt and 61 µA/LSB — the same as
 * the battery INA219.  Override EPS_3V3_INA219_CALIB and
 * EPS_3V3_INA219_CURRENT_LSB_UA if the 3V3 REG shunt differs.
 */

#ifndef EPS_3V3_MONITOR_H_
#define EPS_3V3_MONITOR_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------
 * INA219 I2C address for the 3V3 REG sensor (A1=GND, A0=VCC → 0x44)
 * ------------------------------------------------------------------ */
#define EPS_3V3_INA219_ADDR         (0x44u)

/* ------------------------------------------------------------------
 * INA219 calibration
 *   CAL = trunc(0.04096 / (Current_LSB * Rshunt))
 *   Default: Rshunt = 0.100 Ω, max ~2 A, Current_LSB = 61 µA/LSB
 *   → CAL = 6710  (same as battery INA219; adjust if shunt differs)
 * ------------------------------------------------------------------ */
#define EPS_3V3_INA219_CALIB        (6710u)
#define EPS_3V3_INA219_CURRENT_LSB_UA (61u)  /* µA per current register LSB */

/* ------------------------------------------------------------------
 * GPIO: EN_3V3 — active high (P3.0)
 * ------------------------------------------------------------------ */
#define EPS_3V3_EN_PORT             GPIO_PORT_P3
#define EPS_3V3_EN_PIN              GPIO_PIN0

/* ------------------------------------------------------------------
 * GPIO: ~RESET_RP — active low (P2.6)
 * ------------------------------------------------------------------ */
#define EPS_3V3_RESET_RP_PORT       GPIO_PORT_P2
#define EPS_3V3_RESET_RP_PIN        GPIO_PIN6

/* ------------------------------------------------------------------
 * Tuning parameters
 * ------------------------------------------------------------------ */

/* Overcurrent trip level in milliamps */
#define EPS_3V3_OC_THRESHOLD_MA     (800)

/* Consecutive over-limit polls before attempting a recovery cycle */
#define EPS_3V3_OC_TRIP_THRESHOLD   (2u)

/* EN_3V3 off time during a recovery cycle (ms) */
#define EPS_3V3_CYCLE_OFF_MS        (2000u)

/* Settle time after EN_3V3 is re-asserted before re-checking (ms) */
#define EPS_3V3_SETTLE_MS           (500u)

/* Maximum recovery attempts before latching MSP_FAULT_PWR_UV_LOAD */
#define EPS_3V3_MAX_ATTEMPTS        (3u)

/* Consecutive normal-current polls required to clear fault + reset count */
#define EPS_3V3_STABLE_POLLS        (5u)

/* ------------------------------------------------------------------
 * Task configuration
 * ------------------------------------------------------------------ */
#define EPS_3V3_STACK_SIZE  (configMINIMAL_STACK_SIZE + 48u)
#define EPS_3V3_PRIORITY    (tskIDLE_PRIORITY + 2u)
#define EPS_3V3_POLL_MS     (500u)   /* 2 Hz poll rate */

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */
void eps_3v3_monitor_task_create(void);

#endif /* EPS_3V3_MONITOR_H_ */
