/*
 * timekeeper.h
 *
 * FRAM-persistent Unix time counter for the MSP430FR supervisor.
 *
 * g_unix_time_s is stored in NOINIT FRAM so it survives warm resets.
 * It is NOT cleared on watchdog reset, only on POR (power-on reset).
 *
 * API:
 *   timekeeper_tick_1hz()  — called once per second from prvClockTask.
 *   timekeeper_set(now)    — validates and sets the time from ground TC.
 *
 * Validation rules in timekeeper_set():
 *   - Reject now == 0.
 *   - Reject if new time is more than TIMEKEEPER_MAX_BACKWARD_S behind
 *     the current value (backward jump > 1 day is suspicious).
 *   - Reject if new time is more than TIMEKEEPER_MAX_FORWARD_S ahead of
 *     the current value (forward jump > 10 years is clearly wrong).
 *
 * Returns:
 *   0  — accepted and applied.
 *  -1  — rejected (value out of bounds).
 */

#ifndef TIMEKEEPER_H_
#define TIMEKEEPER_H_

#include <stdint.h>

/* Sanity limits for set-time validation */
#define TIMEKEEPER_MAX_BACKWARD_S   (86400u)       /* 1 day  */
#define TIMEKEEPER_MAX_FORWARD_S    (315360000u)   /* 10 years */

/*
 * Current Unix time in seconds.
 * NOINIT — survives warm resets; undefined on first power-on.
 * Initialised to 0 on POR in main_blinky() (bootCause check).
 */
extern volatile uint32_t g_unix_time_s;

/*
 * Increment g_unix_time_s by 1.
 * Called once per second from prvClockTask (after internal WDT kick).
 */
void timekeeper_tick_1hz(void);

/*
 * Validate and set g_unix_time_s.
 * Returns 0 on success, -1 on rejection.
 */
int8_t timekeeper_set(uint32_t now);

#endif /* TIMEKEEPER_H_ */
