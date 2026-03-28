/*
 * mode_state_machine.h
 *
 * Supervisor mode state machine and power policy enforcer.
 * Runs at 0.5 Hz; computes MSP operating mode from faults, RP liveness,
 * and battery health, then enforces the Mode Authority Matrix rail policy.
 */

#ifndef MODE_STATE_MACHINE_H_
#define MODE_STATE_MACHINE_H_

#include <stdint.h>

/* ======================================================================
 * Supervisor operating modes
 * Published in MSP_STATUS0 bits [2:0] for RP downlink inclusion.
 * ====================================================================== */
typedef enum {
    MSP_MODE_STARTUP  = 0x00u,  /* Initialising — not yet ready */
    MSP_MODE_NOMINAL  = 0x01u,  /* Fully operational */
    MSP_MODE_SAFE     = 0x02u,  /* Degraded: RP alive but constrained */
    MSP_MODE_SURVIVAL = 0x03u,  /* RP off; supervisor-only safety loop */
} msp_mode_t;

/* Current operating mode — read by other tasks (status publish, policy). */
extern volatile msp_mode_t g_msp_mode;

/*
 * Rail allow-mask sent to RP via MODE_CMD/RAIL_MASK registers.
 * Bit definitions match RAIL_*_EN macros in rp_regmap.h.
 *   bit0 = 5V_PAYLOAD
 *   bit1 = 12V_PAYLOAD
 *   bit2 = UHF_AUX
 *   bit3 = SENSOR_BUS
 *   bit4 = HEATER_AUX
 *   bit5 = STORAGE_AUX
 *   bit6 = EXPANDER_DOMAIN
 */
extern volatile uint8_t g_rail_mask;

/* Rail masks per mode (Mode Authority Matrix) */
#define RAIL_MASK_NOMINAL    (0x7Fu)  /* all rails allowed by policy/TC */
#define RAIL_MASK_SAFE       (0x38u)  /* sensor + heater + storage; no payload, no UHF aux */
#define RAIL_MASK_SURVIVAL   (0x10u)  /* heater only — thermal safety must remain */

/* ======================================================================
 * Task configuration
 * ====================================================================== */
#define MODE_SM_STACK_SIZE   (configMINIMAL_STACK_SIZE + 64u)
#define MODE_SM_PRIORITY     (tskIDLE_PRIORITY + 3u)  /* above idle; below liveness */
#define MODE_SM_PERIOD_MS    (2000u)                  /* 0.5 Hz */

/* Number of consecutive OK polls before transitioning STARTUP → NOMINAL */
#define MODE_SM_STARTUP_POLLS (3u)

/* Task creation */
void mode_state_machine_task_create(void);

/*
 * Write a BOOT_CMD to RP via the full 7-byte mode/power block (0x28-0x2E)
 * with an incremented CMD_SEQ.  Call this instead of a bare i2c_write_reg
 * to REG_BOOT_CMD so RP's seq-change detection works correctly.
 *
 * Preserves the current g_msp_mode and g_rail_mask — only BOOT_CMD differs.
 * Not internally mutex-protected; in practice boot_image_manager calls this
 * at infrequent state transitions so there is no race with the SM task.
 */
void mode_sm_send_boot_cmd(uint8_t boot_cmd);

#endif /* MODE_STATE_MACHINE_H_ */
