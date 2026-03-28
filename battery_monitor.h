/*
 * battery_monitor.h
 *
 * Battery monitoring FreeRTOS task.
 * Combines three task-list entries:
 *   - battery_iv_sample  (1 Hz)  : INA219 voltage + current
 *   - battery_temp_sample (5 s)  : MCP9808 temperature
 *   - battery_heater_control (5 s): hysteresis on/off
 *
 * Exports battery state globals used by mode_state_machine and
 * msp_status_publish.
 *
 * Thermal thresholds (centi-degC):
 *   BATT_TEMP_HEAT_ON_CC  : below this → heater ON
 *   BATT_TEMP_HEAT_OFF_CC : above this → heater OFF  (hysteresis gap)
 *   BATT_TEMP_OVERTEMP_CC : above this → F_THERM_OVERTEMP fault
 *
 * Battery voltage thresholds (mV):
 *   BATT_VCRIT_MV  : below this for N samples → F_PWR_UV_BATT fault
 *   BATT_VREC_MV   : above this for recovery countdown
 */

#ifndef BATTERY_MONITOR_H_
#define BATTERY_MONITOR_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------
 * GPIO: battery heater control output.
 * Adjust to match schematic pin assignment.
 * ------------------------------------------------------------------ */
#define BATT_HEAT_PORT          GPIO_PORT_P2
#define BATT_HEAT_PIN           GPIO_PIN7

/* ------------------------------------------------------------------
 * Thermal thresholds
 * ------------------------------------------------------------------ */
#define BATT_TEMP_HEAT_ON_CC    (0)       /*  0.00 °C — heater turns ON  */
#define BATT_TEMP_HEAT_OFF_CC   (500)     /*  5.00 °C — heater turns OFF */
#define BATT_TEMP_OVERTEMP_CC   (4500)    /* 45.00 °C — overtemp fault   */
#define BATT_TEMP_OVERTEMP_REC_CC (4000)  /* 40.00 °C — recovery after overtemp */

/* ------------------------------------------------------------------
 * Battery voltage thresholds (2S Li-Ion / LiPo, mV)
 * ------------------------------------------------------------------ */
#define BATT_VCRIT_MV           (6400u)   /* 3.20 V/cell × 2S — critical */
#define BATT_VREC_MV            (7200u)   /* 3.60 V/cell × 2S — recovery */

/* ------------------------------------------------------------------
 * Task rates
 * ------------------------------------------------------------------ */
#define BATTERY_IV_PERIOD_MS    (1000u)   /* 1 Hz — voltage/current */
#define BATTERY_TEMP_DIVIDER    (5u)      /* every 5 IV ticks = 5 s */

#define BATTERY_STACK_SIZE      (configMINIMAL_STACK_SIZE + 64u)
#define BATTERY_PRIORITY        (tskIDLE_PRIORITY + 2u)

/* ------------------------------------------------------------------
 * Exported battery state globals (updated by battery_monitor task)
 * ------------------------------------------------------------------ */

/* Battery bus voltage, millivolts (0 until first successful read) */
extern volatile uint16_t g_vbatt_mv;

/* Battery current, milliamps — signed: positive = discharging load current */
extern volatile int16_t  g_ibatt_ma;

/* Battery temperature, centi-degC (INT16_MIN until first successful read) */
extern volatile int16_t  g_tbatt_cc;

/* Heater state: 0 = off, 1 = on */
extern volatile uint8_t  g_heater_on;

/* ------------------------------------------------------------------
 * Task creation
 * ------------------------------------------------------------------ */
void battery_monitor_task_create(void);

#endif /* BATTERY_MONITOR_H_ */
