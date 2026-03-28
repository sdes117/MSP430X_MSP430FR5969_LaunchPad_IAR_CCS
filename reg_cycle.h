/*
 * reg_cycle.h
 *
 * Regulator power-good monitor and recovery cycling task.
 *
 * Monitors the RegA_PG (P1.0) and RegB_PG (P1.1) signals from the
 * on-board voltage regulators.  If a PG line drops (regulator lost
 * regulation), the task attempts to recover by briefly cycling the
 * regulator's enable pin:
 *
 *   1. Disable regulator (~EN HIGH).
 *   2. Wait REG_CYCLE_OFF_MS milliseconds.
 *   3. Re-enable regulator (~EN LOW).
 *   4. Wait REG_CYCLE_SETTLE_MS for output to stabilise.
 *
 * If PG does not return after REG_CYCLE_MAX_ATTEMPTS consecutive
 * cycles the task sets MSP_FAULT_PWR_UV_LOAD and stops attempting
 * recovery for that rail until PG recovers naturally.
 *
 * Only rails that should be powered in the current MSP mode are
 * monitored.  Specifically:
 *   - NOMINAL: both RegA and RegB monitored.
 *   - SAFE:    only RegA monitored (RegB is disabled by policy).
 *   - SURVIVAL/STARTUP: neither monitored.
 *
 * PG monitoring uses a debounce counter: REG_PG_MISS_THRESHOLD
 * consecutive PG-low polls must occur before a cycle is attempted,
 * to avoid false triggers from brief transients during startup.
 *
 * Rail enable/disable pin definitions are shared from
 * power_policy_enforcer.h.  A cycle in reg_cycle is a brief
 * (<200 ms) perturbation; power_policy_enforcer re-asserts the
 * correct enable state on its next 1 Hz tick.
 */

#ifndef REG_CYCLE_H_
#define REG_CYCLE_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------
 * PG input pin definitions
 * ------------------------------------------------------------------ */
#define REGA_PG_PORT    GPIO_PORT_P1
#define REGA_PG_PIN     GPIO_PIN0   /* P1.0 — RegA power-good (active HIGH) */

#define REGB_PG_PORT    GPIO_PORT_P1
#define REGB_PG_PIN     GPIO_PIN1   /* P1.1 — RegB power-good (active HIGH) */

/* ------------------------------------------------------------------
 * Tuning parameters
 * ------------------------------------------------------------------ */

/* Consecutive PG-low polls before attempting a cycle */
#define REG_PG_MISS_THRESHOLD    (3u)

/* Off-time during a cycle (ms) */
#define REG_CYCLE_OFF_MS         (100u)

/* Settle time after re-enable before re-checking PG (ms) */
#define REG_CYCLE_SETTLE_MS      (50u)

/* Max consecutive cycles before raising MSP_FAULT_PWR_UV_LOAD */
#define REG_CYCLE_MAX_ATTEMPTS   (5u)

/* Stable polls (PG high) required to clear MSP_FAULT_PWR_UV_LOAD */
#define REG_PG_STABLE_POLLS      (3u)

/* ------------------------------------------------------------------
 * Task configuration
 * ------------------------------------------------------------------ */
#define REG_CYCLE_STACK_SIZE  (configMINIMAL_STACK_SIZE + 32u)
#define REG_CYCLE_PRIORITY    (tskIDLE_PRIORITY + 2u)
#define REG_CYCLE_PERIOD_MS   (1000u)  /* 1 Hz base poll rate */

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */
void reg_cycle_task_create(void);

#endif /* REG_CYCLE_H_ */
