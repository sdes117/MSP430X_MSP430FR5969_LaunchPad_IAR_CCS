/*
 * ground_contact.h
 *
 * Ground contact processor and 48-hour deadman timer.
 *
 * Combines two task-list entries:
 *   - ground_contact_processor (5 Hz):
 *       Reads a pending RP contact event from the I2C regmap.
 *       Validates it (type, flags, sequence number deduplication).
 *       Accepts or rejects, and writes the ack back to RP.
 *       On acceptance, resets the authoritative contact age timer.
 *
 *   - deadman_48h (60 s tick):
 *       Increments g_contact_age_s each second via the clock task hook.
 *       If age exceeds DEADMAN_TIMEOUT_S without an accepted contact,
 *       triggers a full power-on reset (PMM_trigPOR) to force a recovery.
 *
 * Regulatory basis: GDIR-MIS-12 ("48-hour no-contact reset").
 *
 * The contact age timer is stored in FRAM (NOINIT) so it persists across
 * warm resets. A full POR resets NOINIT variables automatically.
 */

#ifndef GROUND_CONTACT_H_
#define GROUND_CONTACT_H_

#include <stdint.h>

/* ------------------------------------------------------------------
 * Deadman timeout and GPIO-aware recovery policy
 * ------------------------------------------------------------------ */
#define DEADMAN_TIMEOUT_S           (172800u)  /* 48 hours in seconds */

/*
 * If the deadman fires while RP GPIO heartbeat is still active (I2C dead
 * but RP alive), trigger an RP hard reset instead of MSP POR and postpone
 * the POR deadline by this many seconds.  Each RP reset attempt buys this
 * much additional time for the I2C link to recover.
 */
#define DEADMAN_RP_RESET_PERIOD_S   (14400u)   /* 4 h per RP-reset attempt */

/* ------------------------------------------------------------------
 * Torn-read protection for the contact event block
 * ------------------------------------------------------------------ */
/*
 * Maximum retries when the contact event block sequence number changes
 * between the pre- and post-read snapshots (torn-read guard).
 */
#define CONTACT_READ_MAX_RETRIES    (3u)

/* ------------------------------------------------------------------
 * Exported state
 * ------------------------------------------------------------------ */

/*
 * Seconds since last MSP-acknowledged ground contact.
 * Stored in FRAM NOINIT so it survives warm resets.
 * Incremented by ground_contact_tick_1hz() called from the clock task.
 */
extern volatile uint32_t g_contact_age_s;

/*
 * Last accepted contact sequence number (for deduplication).
 * NOINIT — persists across warm resets.
 */
extern volatile uint8_t g_last_contact_seq;

/* ------------------------------------------------------------------
 * Task configuration
 * ------------------------------------------------------------------ */
#define GROUND_CONTACT_STACK_SIZE   (configMINIMAL_STACK_SIZE + 64u)
#define GROUND_CONTACT_PRIORITY     (tskIDLE_PRIORITY + 2u)
#define GROUND_CONTACT_PERIOD_MS    (200u)  /* 5 Hz */

/* ------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------ */

/*
 * Called once per second by the clock task.
 * Increments g_contact_age_s; triggers POR if deadman threshold exceeded.
 */
void ground_contact_tick_1hz(void);

/*
 * FreeRTOS task creation.
 */
void ground_contact_task_create(void);

#endif /* GROUND_CONTACT_H_ */
