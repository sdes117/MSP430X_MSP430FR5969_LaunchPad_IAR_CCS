/*
 * i2c_census.h
 *
 * I2C device census task (60 s).
 * Polls known I2C device addresses, verifies identity registers where possible,
 * and maintains a bitmap of present/missing devices.
 *
 * Missing devices trigger fault bits or log events depending on severity.
 * Used for msp_self_test (POST) and ongoing background monitoring.
 */

#ifndef I2C_CENSUS_H_
#define I2C_CENSUS_H_

#include <stdint.h>

/* ------------------------------------------------------------------
 * Device bitmap bits (g_i2c_census_bitmap)
 *   1 = device present and ID verified (or at least ACKing)
 *   0 = device absent or not responding
 * ------------------------------------------------------------------ */
#define I2C_DEV_RP2350      (0x01u)   /* bit0: RP2350 slave (0x42) */
#define I2C_DEV_INA219      (0x02u)   /* bit1: INA219 battery monitor (0x40) */
#define I2C_DEV_MCP9808     (0x04u)   /* bit2: MCP9808 temp sensor (0x18) */

/* ------------------------------------------------------------------
 * Exported census result
 * ------------------------------------------------------------------ */
extern volatile uint8_t g_i2c_census_bitmap;

/* ------------------------------------------------------------------
 * Task configuration
 * ------------------------------------------------------------------ */
#define I2C_CENSUS_STACK_SIZE   (configMINIMAL_STACK_SIZE + 32u)
#define I2C_CENSUS_PRIORITY     (tskIDLE_PRIORITY + 1u)
#define I2C_CENSUS_PERIOD_MS    (60000u)  /* 60 s */

/* ------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------ */

/*
 * Run one census poll synchronously (used by msp_self_test at boot).
 * Returns the census bitmap.
 */
uint8_t i2c_census_run(void);

/*
 * FreeRTOS task creation.
 */
void i2c_census_task_create(void);

#endif /* I2C_CENSUS_H_ */
