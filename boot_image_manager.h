/*
 * boot_image_manager.h
 *
 * RP2350 boot image manager FreeRTOS task.
 *
 * After every RP power-on this task watches REG_RP_STATE until the
 * RP reaches a live operating state within BOOT_TIMEOUT_S seconds.
 * If the RP fails to boot BOOT_FAIL_THRESHOLD times in a row, the
 * task writes BOOT_CMD_BOOT_GOLDEN_NEXT so the RP falls back to the
 * golden (factory) image on the next power cycle.
 *
 * On success (RP reaches NOMINAL or SAFE), it writes
 * BOOT_CMD_CONFIRM_CURRENT_IMAGE to commit the running image and
 * resets the consecutive-failure counter.
 *
 * g_rp_boot_fail_count is NOINIT so it persists across warm resets
 * but is cleared after a confirmed successful boot.
 *
 * State machine (per power-on cycle):
 *
 *   WAITING_FOR_BOOT
 *     Poll REG_RP_STATE every second.
 *     If RP_STATE_NOMINAL or RP_STATE_SAFE within BOOT_TIMEOUT_S
 *       → write CONFIRM; transition to IDLE.
 *     If timeout expires:
 *       g_rp_boot_fail_count++
 *       If g_rp_boot_fail_count >= BOOT_FAIL_THRESHOLD:
 *         → write BOOT_GOLDEN_NEXT; reset fail count.
 *       → transition to IDLE.
 *
 *   IDLE
 *     Wait for rp_liveness to cycle power again (g_rp_powercycle_count
 *     changes), then re-enter WAITING_FOR_BOOT.
 */

#ifndef BOOT_IMAGE_MANAGER_H_
#define BOOT_IMAGE_MANAGER_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------
 * Boot monitor parameters
 * ------------------------------------------------------------------ */
#define BOOT_TIMEOUT_S          (60u)   /* seconds allowed for RP to reach NOMINAL */
#define BOOT_FAIL_THRESHOLD     (3u)    /* consecutive fails before golden fallback */

/* ------------------------------------------------------------------
 * Task configuration
 * ------------------------------------------------------------------ */
#define BOOT_MGR_STACK_SIZE  (configMINIMAL_STACK_SIZE + 32u)
#define BOOT_MGR_PRIORITY    (tskIDLE_PRIORITY + 2u)
#define BOOT_MGR_PERIOD_MS   (1000u)  /* 1 Hz poll */

/* ------------------------------------------------------------------
 * Exported state
 * ------------------------------------------------------------------ */

/*
 * Consecutive boot-failure counter.
 * NOINIT — persists across warm resets; cleared on confirmed boot.
 */
extern volatile uint8_t g_rp_boot_fail_count;

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */
void boot_image_manager_task_create(void);

#endif /* BOOT_IMAGE_MANAGER_H_ */
