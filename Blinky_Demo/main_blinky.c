/*
 * main_blinky.c  —  MSP430FR5969 HIA supervisor (bring-up build)
 *
 * Tasks
 * -----
 *  CLK  (1 Hz)   : increments second counter, sends tick to queue
 *  Rx   (1 Hz)   : receives tick, toggles LEDs, pulses external WDT lines
 *  INA  (2 Hz)   : reads INA219 @ 0x41 (3V3_MSP supply current)
 *
 * Rail control
 * ------------
 *  Four volatile globals control the rail enable/shutdown pins.
 *  Set them to 0/1 in the debugger at runtime:
 *    g_rega_en   — RegA   ~EN  P2.4 (1=enabled, 0=disabled)
 *    g_regb_en   — RegB   ~EN  P4.4 (1=enabled, 0=disabled)
 *    g_efusea_en — eFuseA SHDN P3.2 (1=enabled, 0=shutdown)
 *    g_efuseb_en — eFuseB SHDN P1.5 (1=enabled, 0=shutdown)
 *  The Rx task applies these to the GPIO pins on every tick.
 *
 * RP regmap
 * ---------
 *  The CLK task writes MSP STATUS bytes to the RP regmap (I2C 0x42) once
 *  per second. Writes NACK with no RP connected — that is expected and safe.
 *  When the RP is powered and programmed it will start receiving immediately.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "partest.h"
#include "driverlib.h"
#include <msp430.h>
#include <string.h>

#include "supervisor_i2c.h"
#include "rp_regmap.h"
#include "ina219.h"

/* ------------------------------------------------------------------
 * Task priorities
 * ------------------------------------------------------------------ */
#define PRIORITY_CLK    ( tskIDLE_PRIORITY + 3u )
#define PRIORITY_RX     ( tskIDLE_PRIORITY + 2u )
#define PRIORITY_INA    ( tskIDLE_PRIORITY + 1u )

/* ------------------------------------------------------------------
 * WDT pulse config
 * ------------------------------------------------------------------ */
/* At 8 MHz, 8 cycles ≈ 1 µs  >  500 ns minimum pulse width */
#define WDT_LOW_CYCLES          ( 8U )
#define WDT_PULSE_PERIOD_S      ( 2U )   /* pulse WDT lines every N seconds */

/* INA219 instance — 3V3_MSP supply line, address 0x41 */
static const ina219_t g_ina_3v3_msp = {
    .addr           = 0x41u,
    .config         = INA219_CFG_3V3_MSP,
    .calib          = INA219_CALIB_3V3_MSP,
    .current_lsb_ua = INA219_LSB_UA_3V3_MSP,
};

/* ------------------------------------------------------------------
 * Rail enable/disable GPIO
 * ------------------------------------------------------------------ */
#define REGA_EN_PORT    GPIO_PORT_P2
#define REGA_EN_PIN     GPIO_PIN4   /* active-low enable */
#define REGB_EN_PORT    GPIO_PORT_P4
#define REGB_EN_PIN     GPIO_PIN4   /* active-low enable */
#define EFUSEA_SHDN_PORT GPIO_PORT_P3
#define EFUSEA_SHDN_PIN  GPIO_PIN2  /* HIGH = shutdown */
#define EFUSEB_SHDN_PORT GPIO_PORT_P1
#define EFUSEB_SHDN_PIN  GPIO_PIN5  /* HIGH = shutdown */

/* ------------------------------------------------------------------
 * Debugger-accessible rail control globals
 * 1 = rail enabled (default), 0 = rail disabled
 * ------------------------------------------------------------------ */
volatile uint8_t g_rega_en   = 1u;
volatile uint8_t g_regb_en   = 1u;
volatile uint8_t g_efusea_en = 1u;
volatile uint8_t g_efuseb_en = 1u;

/* ------------------------------------------------------------------
 * Shared INA219 result — stored in FRAM so values survive full power-off.
 * Inspect g_ina_data after standalone run (no JTAG/VCC_TOOL) to see
 * real shunt current when powered from the VBATT path.
 * ------------------------------------------------------------------ */
#pragma NOINIT(g_ina_data)
ina219_data_t g_ina_data;           /* shunt_uv, bus_mv, current_ma, power_mw */

#pragma NOINIT(g_ina_ok)
uint8_t g_ina_ok;                   /* 1 = last read succeeded */

/* ------------------------------------------------------------------
 * Internal state
 * ------------------------------------------------------------------ */
#pragma NOINIT(g_boot_count)
uint16_t g_boot_count;

#pragma NOINIT(g_current_second)
uint32_t g_current_second;

static QueueHandle_t xTickQueue = NULL;

extern volatile uint16_t g_last_sysrstiv;   /* captured in main.c */

/* ------------------------------------------------------------------
 * Forward declarations
 * ------------------------------------------------------------------ */
static void prvClockTask  (void *pvParameters);
static void prvRxTask     (void *pvParameters);
static void prvInaTask    (void *pvParameters);

static inline void prvApplyRailEnables(void);
static inline void prvPulseWdt1(void);
static inline void prvPulseWdt2(void);

/* ------------------------------------------------------------------
 * Entry point called from main()
 * ------------------------------------------------------------------ */
void main_blinky(void)
{
    uint16_t boot_cause = g_last_sysrstiv;

    if (boot_cause <= SYSRSTIV_DOBOR)
    {
        if (boot_cause <= SYSRSTIV_RSTNMI)
            g_current_second = 0u;
    }
    ++g_boot_count;

    /* Apply default rail state (all enabled) before scheduler starts */
    prvApplyRailEnables();

    xTickQueue = xQueueCreate(8u, sizeof(uint8_t));
    configASSERT(xTickQueue != NULL);

    xTaskCreate(prvClockTask, "CLK", configMINIMAL_STACK_SIZE, NULL, PRIORITY_CLK, NULL);
    xTaskCreate(prvRxTask,    "Rx",  configMINIMAL_STACK_SIZE, NULL, PRIORITY_RX,  NULL);
    xTaskCreate(prvInaTask,   "INA", configMINIMAL_STACK_SIZE, NULL, PRIORITY_INA, NULL);

    vTaskStartScheduler();

    /* Reach here only if heap was too small to create idle/timer tasks */
    for (;;);
}

/* ------------------------------------------------------------------
 * CLK task — 1 Hz heartbeat
 * Sends a tick token to the Rx task and attempts to write MSP status
 * to the RP regmap. Writes will NACK until RP is connected.
 * ------------------------------------------------------------------ */
static void prvClockTask(void *pvParameters)
{
    TickType_t xNext = xTaskGetTickCount();
    uint8_t    tick  = 1u;
    (void)pvParameters;

    for (;;)
    {
        vTaskDelayUntil(&xNext, pdMS_TO_TICKS(1000u));
        ++g_current_second;

        /* --- Build STATUS0: WDT lines OK (both P2.2 and P3.4 high) --- */
        uint8_t wdt_ok  = ((P2OUT & BIT2) && (P3OUT & BIT4)) ? 1u : 0u;
        uint8_t rail_ok = (g_rega_en && g_efusea_en) ? 1u : 0u;

        uint8_t s0 = (uint8_t)(
              (0u & STATUS0_MODE_MASK)           /* STARTUP mode until SM added */
            | (wdt_ok  ? STATUS0_WDT_OK  : 0u)
            | (rail_ok ? STATUS0_RAIL_OK : 0u));

        uint8_t s1 = (uint8_t)(g_ina_ok ? 0u : STATUS1_OC_MCU);  /* reuse bit: sensor absent = possible OC */

        /* Attempt to write STATUS bytes to RP regmap.
         * i2c_write_reg returns I2C_ERR_NACK if RP is not connected — ignore. */
        (void)i2c_write_reg(RP_I2C_ADDR, REG_MSP_STATUS0, &s0, 1u);
        (void)i2c_write_reg(RP_I2C_ADDR, REG_MSP_STATUS1, &s1, 1u);

        /* Also mirror to telemetry region */
        (void)i2c_write_reg(RP_I2C_ADDR, REG_TLM_MSP_STATUS0, &s0, 1u);

        /* Send tick to Rx task */
        xQueueSend(xTickQueue, &tick, 0u);
    }
}

/* ------------------------------------------------------------------
 * Rx task — LED toggle + WDT pulse on each 1 Hz tick
 * Also re-applies rail enable GPIO on every tick (debugger safe).
 * ------------------------------------------------------------------ */
static void prvRxTask(void *pvParameters)
{
    uint8_t  tick;
    uint32_t wdt1_counter = 0u;
    uint32_t wdt2_counter = 0u;
    (void)pvParameters;

    for (;;)
    {
        xQueueReceive(xTickQueue, &tick, portMAX_DELAY);

        /* Toggle activity LEDs */
        vParTestToggleLED(0);
        vParTestToggleLED(1);

        /* Apply rail enables (safe to call every tick — just GPIO writes) */
        prvApplyRailEnables();

        /* Pulse WDT1 (P2.2) */
        if (++wdt1_counter >= WDT_PULSE_PERIOD_S)
        {
            wdt1_counter = 0u;
            prvPulseWdt1();
        }

        /* Pulse WDT2 (P3.4) */
        if (++wdt2_counter >= WDT_PULSE_PERIOD_S)
        {
            wdt2_counter = 0u;
            prvPulseWdt2();
        }
    }
}

/* ------------------------------------------------------------------
 * INA task — read INA219 @ 0x41 at 2 Hz
 * Stores full result in g_ina_data / g_ina_ok for the CLK task.
 * ------------------------------------------------------------------ */
static void prvInaTask(void *pvParameters)
{
    TickType_t   xNext  = xTaskGetTickCount();
    uint8_t      inited = 0u;
    ina219_data_t data;
    (void)pvParameters;

    for (;;)
    {
        vTaskDelayUntil(&xNext, pdMS_TO_TICKS(500u));

        /* Write config + calibration on first successful contact */
        if (!inited)
        {
            if (ina219_init(&g_ina_3v3_msp) != INA219_OK)
                continue;   /* sensor not ready yet — try again next cycle */
            inited = 1u;
        }

        /* Read all four data registers */
        if (ina219_read(&g_ina_3v3_msp, &data) == INA219_OK)
        {
            g_ina_data = data;
            g_ina_ok   = 1u;
        }
        else
        {
            g_ina_ok = 0u;
            inited   = 0u;   /* re-init next time */
        }
    }
}

/* ------------------------------------------------------------------
 * Rail enable helper
 * ~EN pins: LOW = enabled, HIGH = disabled
 * SHDN pins: LOW = active, HIGH = shutdown
 * ------------------------------------------------------------------ */
static inline void prvApplyRailEnables(void)
{
    /* RegA ~EN */
    if (g_rega_en)   GPIO_setOutputLowOnPin (REGA_EN_PORT,    REGA_EN_PIN);
    else             GPIO_setOutputHighOnPin(REGA_EN_PORT,    REGA_EN_PIN);

    /* RegB ~EN */
    if (g_regb_en)   GPIO_setOutputLowOnPin (REGB_EN_PORT,    REGB_EN_PIN);
    else             GPIO_setOutputHighOnPin(REGB_EN_PORT,    REGB_EN_PIN);

    /* eFuseA SHDN */
    if (g_efusea_en) GPIO_setOutputLowOnPin (EFUSEA_SHDN_PORT, EFUSEA_SHDN_PIN);
    else             GPIO_setOutputHighOnPin(EFUSEA_SHDN_PORT, EFUSEA_SHDN_PIN);

    /* eFuseB SHDN */
    if (g_efuseb_en) GPIO_setOutputLowOnPin (EFUSEB_SHDN_PORT, EFUSEB_SHDN_PIN);
    else             GPIO_setOutputHighOnPin(EFUSEB_SHDN_PORT, EFUSEB_SHDN_PIN);
}

/* ------------------------------------------------------------------
 * WDT pulse helpers — brief low pulse then return high
 * Wrapped in critical section to keep pulse width tight.
 * ------------------------------------------------------------------ */
static inline void prvPulseWdt1(void)
{
    taskENTER_CRITICAL();
    P2OUT &= (uint8_t)~BIT2;
    __delay_cycles(WDT_LOW_CYCLES);
    P2OUT |= BIT2;
    taskEXIT_CRITICAL();
}

static inline void prvPulseWdt2(void)
{
    taskENTER_CRITICAL();
    P3OUT &= (uint8_t)~BIT4;
    __delay_cycles(WDT_LOW_CYCLES);
    P3OUT |= BIT4;
    taskEXIT_CRITICAL();
}
