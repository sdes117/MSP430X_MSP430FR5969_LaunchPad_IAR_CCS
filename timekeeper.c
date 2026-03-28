/*
 * timekeeper.c
 *
 * FRAM-persistent Unix time counter.
 * See timekeeper.h for description.
 */

#include "timekeeper.h"

/* ------------------------------------------------------------------
 * FRAM-persistent time (NOINIT — survives warm reset, not POR)
 * ------------------------------------------------------------------ */
#ifdef __ICC430__
__no_init volatile uint32_t g_unix_time_s;
#else
#pragma NOINIT(g_unix_time_s)
volatile uint32_t g_unix_time_s;
#endif

/* ------------------------------------------------------------------
 * 1 Hz tick — called from clock task
 * ------------------------------------------------------------------ */

void timekeeper_tick_1hz(void)
{
    g_unix_time_s++;
}

/* ------------------------------------------------------------------
 * Validated time set
 * ------------------------------------------------------------------ */

int8_t timekeeper_set(uint32_t now)
{
    uint32_t current;

    /* Reject zero — clearly invalid */
    if (now == 0u)
    {
        return -1;
    }

    current = g_unix_time_s;

    /* Reject if the new time is more than 1 day in the past */
    if ((current > TIMEKEEPER_MAX_BACKWARD_S) &&
        (now < (current - TIMEKEEPER_MAX_BACKWARD_S)))
    {
        return -1;
    }

    /* Reject if the new time is more than 10 years in the future */
    if ((now > current) &&
        ((now - current) > TIMEKEEPER_MAX_FORWARD_S))
    {
        return -1;
    }

    g_unix_time_s = now;
    return 0;
}
