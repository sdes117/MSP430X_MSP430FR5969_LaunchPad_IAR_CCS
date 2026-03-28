/*
 * rp_liveness.h
 *
 * rp_liveness_monitor FreeRTOS task.
 *
 * Monitors RP2350 liveness via two channels:
 *   1. GPIO heartbeats: WDT_RP2MSP1 (P3.7) and WDT_RP2MSP2 (P3.1)
 *      - RP toggles each line periodically (~1 Hz from its side).
 *      - MSP checks for edge transitions every second.
 *   2. I2C heartbeat counter: REG_HB0/HB1 (0x15-0x16 in RP regmap)
 *      - Monotonic counter incremented by RP firmware.
 *      - Sampled each poll cycle; stall detected if it stops incrementing.
 *
 * Escalation ladder (each stage adds RP_LIVENESS_TIMEOUT_S without recovery):
 *   Stage 0  Normal                   — no action
 *   Stage 1  Soft fault (I2C stall)   — log, increment fault counter
 *   Stage 2  Hard fault (GPIO stall)  — assert ~RESET_RP (P2.6, active low)
 *   Stage 3  Power-cycle              — deassert EN_3V3 (P3.0) then reassert
 *
 * GPIO pin assignments (from schematic / Init_GPIO):
 *   P3.7  WDT_RP2MSP1  — RP heartbeat input 1
 *   P3.1  WDT_RP2MSP2  — RP heartbeat input 2
 *   P2.6  ~RESET_RP    — active-low RP reset output (low = RP held in reset)
 *   P3.0  EN_3V3       — RP 3.3 V power enable output (low = off)
 *   P3.5  WDT_MSP2RP   — MSP heartbeat output to RP (optional, toggled each poll)
 */

#ifndef RP_LIVENESS_H_
#define RP_LIVENESS_H_

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

/* Seconds without a GPIO toggle before liveness is considered lost. */
#define RP_LIVENESS_GPIO_TIMEOUT_S    (5u)

/* Seconds without HB counter change before I2C liveness is flagged. */
#define RP_LIVENESS_I2C_TIMEOUT_S     (3u)

/* Seconds in reset/power-cycle hold before RP is allowed to restart. */
#define RP_RESET_HOLD_S               (1u)
#define RP_POWERCYCLE_OFF_S           (3u)

/* Maximum logic resets (rung 3) before escalating to power-cycle (rung 4). */
#define RP_RESET_MAX_ATTEMPTS         (3u)

/*
 * Maximum power-cycle attempts before entering SURVIVAL (rung 4 → rung 5).
 * Xlsx "Recovery Escalation" tab specifies: "up to 2 cycles per hour".
 */
#define RP_POWERCYCLE_MAX_ATTEMPTS    (2u)

/*
 * Minimum seconds battery voltage must be above BATT_VREC_MV and all P0
 * faults absent before SURVIVAL automatically exits and RP is re-enabled.
 * Set to 0 to disable automatic SURVIVAL exit (MSP reset required).
 */
#define RP_SURVIVAL_EXIT_STABLE_S     (300u)   /* 5 min recovery soak */

/*
 * Maximum consecutive I2C bus-recovery attempts per stall incident (Level 2).
 * After this many attempts without recovery, the liveness escalation timer
 * is allowed to expire naturally and drive Level 3 (RP reset).
 */
#define I2C_BUS_RECOVER_MAX_ATTEMPTS  (3u)

/* Task stack and priority */
#define RP_LIVENESS_STACK_SIZE        (configMINIMAL_STACK_SIZE)
#define RP_LIVENESS_PRIORITY          (tskIDLE_PRIORITY + 2u)

/* Escalation state */
typedef enum {
    RP_LIVE_OK          = 0,
    RP_LIVE_I2C_STALL   = 1,
    RP_LIVE_GPIO_STALL  = 2,
    RP_LIVE_RESET       = 3,
    RP_LIVE_POWERCYCLE  = 4,
    RP_LIVE_SURVIVAL    = 5
} rp_live_state_t;

/* Liveness status exported for other modules */
extern volatile rp_live_state_t g_rp_live_state;
extern volatile uint8_t         g_rp_powercycle_count;

/*
 * Create and start the rp_liveness_monitor FreeRTOS task.
 * Call from main_blinky() before vTaskStartScheduler().
 */
void rp_liveness_task_create(void);

#endif /* RP_LIVENESS_H_ */
