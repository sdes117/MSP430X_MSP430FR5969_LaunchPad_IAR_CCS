/*
 * efuse_cycle.h
 *
 * eFuse fault monitor and recovery cycling task.
 * Implements the eps_current_limit_control_3v3 task from the task list.
 *
 * Monitors the eFuseA_~FLT (P2.0) and eFuseB_~FLT (P2.1) open-drain
 * fault outputs.  An eFuse asserts ~FLT LOW when it has tripped due to
 * overcurrent, overtemperature, or reverse current.
 *
 * On FLT assertion the task attempts recovery by cycling the eFuse:
 *
 *   1. Assert SHDN (eFuse shutdown) HIGH to reset the internal latch.
 *   2. Wait EFUSE_CYCLE_OFF_MS milliseconds.
 *   3. De-assert SHDN (SHDN LOW) to re-enable the eFuse.
 *   4. Wait EFUSE_CYCLE_SETTLE_MS for FLT to de-assert if load is healthy.
 *
 * Only eFuses that should be powered in the current MSP mode are cycled:
 *   - NOMINAL: both eFuseA and eFuseB monitored.
 *   - SAFE:    only eFuseA monitored (eFuseB is disabled by policy).
 *   - SURVIVAL/STARTUP: neither monitored.
 *
 * If FLT does not clear after EFUSE_CYCLE_MAX_ATTEMPTS consecutive cycles,
 * MSP_FAULT_PWR_OC_MCU is set.  Cycling stops until the eFuse recovers
 * naturally (FLT returns HIGH for EFUSE_FLT_STABLE_POLLS polls).
 *
 * SHDN pin definitions are also in power_policy_enforcer.h; both files
 * define the same pin constants for independence.  power_policy_enforcer
 * re-applies the mode-correct SHDN state on every 1 Hz tick, which acts
 * as a safe fallback if a cycle leaves a pin in the wrong state.
 */

#ifndef EFUSE_CYCLE_H_
#define EFUSE_CYCLE_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------
 * eFuse fault input pin definitions (active-low fault)
 * ------------------------------------------------------------------ */
#define EFUSEA_FLT_PORT     GPIO_PORT_P2
#define EFUSEA_FLT_PIN      GPIO_PIN0   /* P2.0 — eFuseA ~FLT */

#define EFUSEB_FLT_PORT     GPIO_PORT_P2
#define EFUSEB_FLT_PIN      GPIO_PIN1   /* P2.1 — eFuseB ~FLT */

/* eFuse shutdown output pin definitions (active-high shutdown) */
#define EFUSEA_SHDN_PORT    GPIO_PORT_P3
#define EFUSEA_SHDN_PIN     GPIO_PIN2   /* P3.2 — eFuseA SHDN */

#define EFUSEB_SHDN_PORT    GPIO_PORT_P1
#define EFUSEB_SHDN_PIN     GPIO_PIN5   /* P1.5 — eFuseB SHDN */

/* ------------------------------------------------------------------
 * Tuning parameters
 * ------------------------------------------------------------------ */

/* Consecutive FLT-asserted polls before attempting a cycle */
#define EFUSE_FLT_TRIP_THRESHOLD    (1u)   /* immediate: FLT is latching */

/* SHDN assertion time during a cycle (ms) */
#define EFUSE_CYCLE_OFF_MS          (100u)

/* Settle time after SHDN de-assertion before re-checking FLT (ms) */
#define EFUSE_CYCLE_SETTLE_MS       (50u)

/* Max consecutive cycles before raising MSP_FAULT_PWR_OC_MCU */
#define EFUSE_CYCLE_MAX_ATTEMPTS    (5u)

/* Stable FLT-high polls required to clear MSP_FAULT_PWR_OC_MCU */
#define EFUSE_FLT_STABLE_POLLS      (3u)

/* ------------------------------------------------------------------
 * Task configuration
 * ------------------------------------------------------------------ */
#define EFUSE_CYCLE_STACK_SIZE  (configMINIMAL_STACK_SIZE + 32u)
#define EFUSE_CYCLE_PRIORITY    (tskIDLE_PRIORITY + 2u)
#define EFUSE_CYCLE_PERIOD_MS   (1000u)  /* 1 Hz base poll rate */

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */
void efuse_cycle_task_create(void);

#endif /* EFUSE_CYCLE_H_ */
