/*
 * power_policy_enforcer.h
 *
 * MSP-local power rail GPIO policy task (1 Hz).
 * Sets RegA, RegB, eFuseA, eFuseB GPIO outputs based on MSP operating mode.
 * Also controls g_heater_permitted for battery_monitor.
 *
 * eFuse FLT monitoring and recovery cycling: see efuse_cycle.h.
 * Regulator PG monitoring and recovery cycling: see reg_cycle.h.
 *
 * Local power GPIO pins (MSP430FR5969):
 *
 *   RegA_~EN    P2.4 — LOW  = RegA enabled,  HIGH = RegA disabled
 *   RegB_~EN    P4.4 — LOW  = RegB enabled,  HIGH = RegB disabled
 *   eFuseA_SHDN P3.2 — LOW  = eFuse A active, HIGH = eFuse A shutdown
 *   eFuseB_SHDN P1.5 — LOW  = eFuse B active, HIGH = eFuse B shutdown
 *
 * eFuse fault inputs (active low = fault asserted):
 *   eFuseA_~FLT P2.0 — LOW  = eFuse A overcurrent / fault
 *   eFuseB_~FLT P2.1 — LOW  = eFuse B overcurrent / fault
 *
 * Rail policy per mode (Mode Authority Matrix):
 *
 *   STARTUP / NOMINAL:
 *     All rails enabled: RegA LOW, RegB LOW, eFuseA LOW, eFuseB LOW.
 *     g_heater_permitted = 1.
 *
 *   SAFE:
 *     RegA + eFuseA: enabled  (RegA LOW, eFuseA LOW)
 *     RegB + eFuseB: disabled (RegB HIGH, eFuseB HIGH)
 *     g_heater_permitted = 1.
 *
 *   SURVIVAL:
 *     All RP-side rails disabled: RegA HIGH, RegB HIGH, eFuseA HIGH, eFuseB HIGH.
 *     g_heater_permitted = 0.
 *     (RP is already powered off by rp_liveness; this ensures rail drivers
 *      do not float or back-power the RP through any load path.)
 *
 * Overcurrent detection (eps_current_limit_3v3):
 *   eFuseA_~FLT or eFuseB_~FLT reading LOW → set MSP_FAULT_PWR_OC_MCU.
 *   Fault auto-clears when both FLT lines return HIGH for
 *   OC_CLEAR_STABLE_POLLS consecutive polls.
 */

#ifndef POWER_POLICY_ENFORCER_H_
#define POWER_POLICY_ENFORCER_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------
 * GPIO pin definitions
 * ------------------------------------------------------------------ */

/* Rail enable/disable outputs (active-low enable) */
#define REGA_EN_PORT        GPIO_PORT_P2
#define REGA_EN_PIN         GPIO_PIN4

#define REGB_EN_PORT        GPIO_PORT_P4
#define REGB_EN_PIN         GPIO_PIN4

#define EFUSEA_SHDN_PORT    GPIO_PORT_P3
#define EFUSEA_SHDN_PIN     GPIO_PIN2

#define EFUSEB_SHDN_PORT    GPIO_PORT_P1
#define EFUSEB_SHDN_PIN     GPIO_PIN5

/* ------------------------------------------------------------------
 * Task configuration
 * ------------------------------------------------------------------ */
#define POWER_POLICY_STACK_SIZE (configMINIMAL_STACK_SIZE + 32u)
#define POWER_POLICY_PRIORITY   (tskIDLE_PRIORITY + 2u)
#define POWER_POLICY_PERIOD_MS  (1000u)  /* 1 Hz */

/* ------------------------------------------------------------------
 * Exported state
 * ------------------------------------------------------------------ */

/*
 * Heater permission flag.
 *   1 = battery_monitor may enable the heater.
 *   0 = heater must remain off (SURVIVAL mode).
 * Written by power_policy_enforcer; read by battery_monitor.
 */
extern volatile uint8_t g_heater_permitted;

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */
void power_policy_enforcer_task_create(void);

#endif /* POWER_POLICY_ENFORCER_H_ */
