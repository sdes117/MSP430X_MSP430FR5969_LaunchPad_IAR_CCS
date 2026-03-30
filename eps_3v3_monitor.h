/*
 * eps_3v3_monitor.h
 *
 * MSP 3.3 V supply current monitor.
 * Reads INA219 @ 0x41 ("3.3V_MSP REG") which sits on the power line
 * feeding the WDTs and MSP itself.
 *
 * The eFuse hardware autonomously limits overcurrent on this rail.
 * This task's role is purely to observe and raise a fault bit so the
 * event is logged and visible in the MSP status bytes — it does NOT
 * attempt to cycle the rail (that would kill the MSP itself).
 *
 * On sustained overcurrent (> EPS_3V3_OC_THRESHOLD_MA for
 * EPS_3V3_OC_TRIP_THRESHOLD consecutive polls):
 *   - Sets MSP_FAULT_PWR_OC_MCU + increments MSP_FCNT_PWR_OC_MCU.
 *
 * Fault clears when current stays below threshold for
 * EPS_3V3_STABLE_POLLS consecutive polls.
 */

#ifndef EPS_3V3_MONITOR_H_
#define EPS_3V3_MONITOR_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------
 * INA219 I2C address — 0x41 = 3.3V_MSP REG (A1=GND, A0=VS)
 * ------------------------------------------------------------------ */
#define EPS_3V3_INA219_ADDR           (0x41u)

/* ------------------------------------------------------------------
 * INA219 calibration (same shunt as battery INA219: 100 mΩ, 61 µA/LSB)
 * ------------------------------------------------------------------ */
#define EPS_3V3_INA219_CALIB          (6710u)
#define EPS_3V3_INA219_CURRENT_LSB_UA (61u)

/* ------------------------------------------------------------------
 * Tuning
 * ------------------------------------------------------------------ */

/* Overcurrent trip level in milliamps */
#define EPS_3V3_OC_THRESHOLD_MA     (500)

/* Consecutive over-limit polls before latching the fault */
#define EPS_3V3_OC_TRIP_THRESHOLD   (2u)

/* Consecutive normal-current polls required to clear the fault */
#define EPS_3V3_STABLE_POLLS        (5u)

/* ------------------------------------------------------------------
 * Task configuration
 * ------------------------------------------------------------------ */
#define EPS_3V3_STACK_SIZE  (configMINIMAL_STACK_SIZE + 32u)
#define EPS_3V3_PRIORITY    (tskIDLE_PRIORITY + 2u)
#define EPS_3V3_POLL_MS     (500u)   /* 2 Hz */

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */
void eps_3v3_monitor_task_create(void);

#endif /* EPS_3V3_MONITOR_H_ */
