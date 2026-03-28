/*
 * msp_status_publish.h
 *
 * MSP status publish task.
 * Packs the two supervisor status bytes (MSP_STATUS0, MSP_STATUS1) and
 * writes them to RP regmap addresses 0x20-0x21 at 1 Hz.
 *
 * RP includes these bytes in downlink beacon frames.
 * MSP has no direct involvement in RF; this is the only telemetry path.
 *
 * STATUS0 bit layout (from rp_regmap.h MSP_STATUS0_* defines):
 *   bits [2:0] = MSP mode (msp_mode_t)
 *   bit  [3]   = ext WDTs OK (TPS3435 lines not held low)
 *   bit  [4]   = battery voltage OK (above BATT_VCRIT_MV)
 *   bit  [5]   = 3V3 rail power-good
 *   bit  [7]   = any P0 fault active
 *
 * STATUS1 = count of currently active fault bits (saturated at 0xFF).
 */

#ifndef MSP_STATUS_PUBLISH_H_
#define MSP_STATUS_PUBLISH_H_

#include <stdint.h>

#define MSP_STATUS_STACK_SIZE   (configMINIMAL_STACK_SIZE + 32u)
#define MSP_STATUS_PRIORITY     (tskIDLE_PRIORITY + 1u)
#define MSP_STATUS_PERIOD_MS    (1000u)  /* 1 Hz */

void msp_status_publish_task_create(void);

#endif /* MSP_STATUS_PUBLISH_H_ */
