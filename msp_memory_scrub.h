/*
 * msp_memory_scrub.h
 *
 * Hourly CRC32 memory scrub over the MSP430FR code FRAM region.
 *
 * A CRC32 baseline is computed once at first boot (after POR, when
 * g_scrub_baseline_valid == 0) and stored in FRAM-PERSISTENT variables.
 * Every SCRUB_INTERVAL_S seconds thereafter the same region is re-checked.
 * A mismatch sets MSP_FAULT_MEM_CRC (bit9 in g_msp_fault_bitmap).
 *
 * FRAM region:
 *   SCRUB_REGION_START — first address included in the CRC computation.
 *   SCRUB_REGION_LEN   — number of bytes covered.
 *   Default covers 0x4400–0xC3FF (32 KB; adjust to match linker output).
 *
 * msp_memory_scrub_invalidate() clears the baseline valid flag.  Call
 * it before triggering a POR after a firmware update so the scrub
 * re-baselines with the new image.
 */

#ifndef MSP_MEMORY_SCRUB_H_
#define MSP_MEMORY_SCRUB_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------
 * Scrub region — adjust to match actual code + staging footprint in FRAM.
 *
 * The staging area (msp_update.h: 0xC000–0xFBFF) is intentionally
 * included in this region.  msp_update.c calls msp_memory_scrub_invalidate()
 * at the start of every update session (BEGIN command) so the scrub
 * re-baselines at its next hourly run instead of raising a false fault
 * while chunks are being written.
 * ------------------------------------------------------------------ */
#define SCRUB_REGION_START_ADDR  (0x4400u)   /* start of code section */
#define SCRUB_REGION_LEN         (0x8000u)   /* 32 KB — covers staging too */

/* ------------------------------------------------------------------
 * Scrub interval
 * ------------------------------------------------------------------ */
#define SCRUB_INTERVAL_S         (3600u)     /* 1 hour */

/* ------------------------------------------------------------------
 * Task configuration
 * ------------------------------------------------------------------ */
#define SCRUB_STACK_SIZE  (configMINIMAL_STACK_SIZE + 64u)
#define SCRUB_PRIORITY    (tskIDLE_PRIORITY + 1u)

/* ------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------ */

/*
 * Invalidate the stored baseline CRC.
 * Must be called before a firmware update commit + POR so the scrub
 * re-establishes the baseline after the reboot.
 */
void msp_memory_scrub_invalidate(void);

/*
 * Create the scrub task.
 */
void msp_memory_scrub_task_create(void);

#endif /* MSP_MEMORY_SCRUB_H_ */
