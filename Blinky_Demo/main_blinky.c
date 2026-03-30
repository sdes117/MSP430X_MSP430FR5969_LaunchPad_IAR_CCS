/*
 * FreeRTOS V202112.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/******************************************************************************
 * NOTE 1:  This project provides two demo applications.  A simple blinky
 * style project, and a more comprehensive test and demo application.  The
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting in main.c is used to select
 * between the two.  See the notes on using mainCREATE_SIMPLE_BLINKY_DEMO_ONLY
 * in main.c.  This file implements the simply blinky style version.
 *
 * NOTE 2:  This file only contains the source code that is specific to the
 * basic demo.  Generic functions, such FreeRTOS hook functions, and functions
 * required to configure the hardware are defined in main.c.
 ******************************************************************************
 *
 * main_blinky() creates one queue, and two tasks.  It then starts the
 * scheduler.
 *
 * The Queue Send Task:
 * The queue send task is implemented by the prvClockTask() function in
 * this file.  prvClockTask() sits in a loop that causes it to repeatedly
 * block for 200 milliseconds, before sending the value 100 to the queue that
 * was created within main_blinky().  Once the value is sent, the task loops
 * back around to block for another 200 milliseconds...and so on.
 *
 * The Queue Receive Task:
 * The queue receive task is implemented by the prvQueueReceiveTask() function
 * in this file.  prvQueueReceiveTask() sits in a loop where it repeatedly
 * blocks on attempts to read data from the queue that was created within
 * main_blinky().  When data is received, the task checks the value of the
 * data, and if the value equals the expected 100, toggles an LED.  The 'block
 * time' parameter passed to the queue receive function specifies that the
 * task should be held in the Blocked state indefinitely to wait for data to
 * be available on the queue.  The queue receive task will only leave the
 * Blocked state when the queue send task writes to the queue.  As the queue
 * send task writes to the queue every 200 milliseconds, the queue receive
 * task leaves the Blocked state every 200 milliseconds, and therefore toggles
 * the LED every 200 milliseconds.
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Standard demo includes. */
#include "partest.h"
#include "serial.h"

/* TI includes. */
#include "driverlib.h"

/* CAN includes */
#include <msp430.h>
#include "mcp2515.h"

#include "mission.h"
#include <csp/csp.h>
#include <csp/drivers/can_socketcan.h>
#include <csp/interfaces/csp_if_can.h>
#include <csp/arch/csp_time.h>
#include <csp/arch/csp_clock.h>
#include <csp/arch/csp_system.h>

#include "csp_extensions.h"

/* Supervisor includes */
#include "rp_liveness.h"
#include "supervisor_i2c.h"
#include "rp_regmap.h"
#include "fault_counters.h"
#include "mode_state_machine.h"
#include "battery_monitor.h"
#include "ground_contact.h"
#include "msp_status_publish.h"
#include "fault_counter_update.h"
#include "rp_cmd_dispatch.h"
#include "event_logger.h"
#include "i2c_census.h"
#include "timekeeper.h"
#include "msp_self_test.h"
#include "power_policy_enforcer.h"
#include "msp_memory_scrub.h"
#include "boot_image_manager.h"
#include "msp_update.h"
#include "reg_cycle.h"
#include "efuse_cycle.h"
#include "eps_3v3_monitor.h"

/* g_last_sysrstiv captured in main.c before SYSRSTIV auto-clears */
extern volatile uint16_t g_last_sysrstiv;

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

/* Half-period for the clock task; two iterations = 1 s tick and 2 Hz WDT service. */
#define mainQUEUE_SEND_FREQUENCY_MS			( pdMS_TO_TICKS( 500 ) )

/* WDT pulse periods are in seconds (clock task runs once per second). */
#define WDT_PULSE_PERIOD_DEFAULT_S          ( 2U )
/* At 8 MHz MCLK, 8 cycles = 1 us, which exceeds the 500 ns minimum. */
#define WDT_EDGE_LOW_CYCLES                 ( 8U )

/* Fault-injection test mode: latch one WDT line after N timer blinks. */
#define WDT_FAULT_INJECT_ENABLE             ( 0U )
#define WDT_FAULT_AFTER_BLINKS              ( 10U )
#define WDT_FAULT_LINE                      ( 1U )   /* 1 = WDT1 (P2.2), 2 = WDT2 (P3.4) */
#define WDT_FAULT_LEVEL_HIGH                ( 0U )   /* 0 = force low, 1 = force high */

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 16 )

/* The LED toggled by the Rx task. */
#define mainTASK_LED						( 0 )
#define mainTASK_LED_2                      ( 1 )

#define MAX_TLM_PACKET_SIZE 48

/*----------------------------------------------------------*/

typedef enum
{
    eTimer = 0,
    eCAN   = 1,
    eADC   = 2
}eMsgType;

typedef enum
{
    eSOH_TLM = 0,
    eAPP_TLM = 1,
    eVER_TLM = 254,
    eSET_RATE_TLM = 255
}eTlmType;

struct AppMessage
{
    uint8_t  msgID;
    uint8_t  msgByte;
    void    *pvData;
};


extern int frame_tx_count;

/*
 * Called by main when mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 1 in
 * main.c.
 */
void main_blinky( void );
void can_tx_next_packet(BaseType_t *ptask_woken);
void handle_csp_tlm_request(csp_conn_t * conn, csp_packet_t * packet);
void handle_csp_timesync(csp_conn_t * conn, csp_packet_t * packet);
int  csp_reboot_function(void);
void can_start_tx(void);
static inline void prvPulseWdt1(void);
static inline void prvPulseWdt2(void);
static inline void prvForceWdtLine(uint8_t line, uint8_t level_high);

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvQueueReceiveTask( void *pvParameters );
static void prvClockTask( void *pvParameters );
static void prvCanTask( void *pvParameters );

/*-----------------------------------------------------------*/

/* The queue used by both tasks. */
static QueueHandle_t xQueue = NULL;
static QueueHandle_t xCanQueue = NULL;

struct AppMessage ADCResult = {0};
struct AppMessage CANflag = {0};

uint32_t rid;
uint32_t ridbuf[4];
uint8_t nirq = 0;
uint8_t mext;
uint8_t eflag;
uint8_t irq, buf[8];
uint16_t ADCdata[16];
uint32_t ADCtime;
uint8_t wakecount = 0;
uint8_t errorcount = 0;
#pragma NOINIT(bootCount)
uint16_t bootCount;
#pragma NOINIT(bootCause)
uint16_t bootCause;
uint32_t uptime= 0;
uint32_t wd_timeout = 86400;
#pragma NOINIT(wd_count)
uint16_t wd_count;
uint16_t rx_csp = 0;
uint16_t tx_csp = 0;


#define DEFAULT_TLM_PERIOD (30)

uint32_t tlm_period = DEFAULT_TLM_PERIOD;
uint32_t tlm_duration = 0;
uint32_t tlm_counter = 0;

volatile uint32_t wdt1_pulse_period_s = WDT_PULSE_PERIOD_DEFAULT_S;
volatile uint32_t wdt2_pulse_period_s = WDT_PULSE_PERIOD_DEFAULT_S;

csp_iface_t * default_interface = NULL;
csp_conn_t *conn = NULL;

uint8_t tlm_pkt[MAX_TLM_PACKET_SIZE] = {0,};

#pragma NOINIT(current_second)
uint32_t current_second;

/* This is the hardware-specific reboot function that is registered with the CSP library, and called on csp_reboot(). */
int csp_reboot_function(void)
{
    /* trigger a brown-out reset, this is equivalent to a initial power-up reset, and completely resets the MSP430.  */
    PMM_trigBOR();

    /* shouldn't get here */
    return CSP_ERR_NONE;
}

/* Keep the low pulse tightly bounded by avoiding scheduler/ISR preemption. */
static inline void prvPulseWdt1(void)
{
    taskENTER_CRITICAL();
    P2OUT &= (uint8_t) ~BIT2;
    __delay_cycles( WDT_EDGE_LOW_CYCLES );
    P2OUT |= BIT2;
    taskEXIT_CRITICAL();
}

/* Keep the low pulse tightly bounded by avoiding scheduler/ISR preemption. */
static inline void prvPulseWdt2(void)
{
    taskENTER_CRITICAL();
    P3OUT &= (uint8_t) ~BIT4;
    __delay_cycles( WDT_EDGE_LOW_CYCLES );
    P3OUT |= BIT4;
    taskEXIT_CRITICAL();
}

static inline void prvForceWdtLine(uint8_t line, uint8_t level_high)
{
    taskENTER_CRITICAL();

    if (line == 1U)
    {
        if (level_high != 0U)
        {
            P2OUT |= BIT2;
        }
        else
        {
            P2OUT &= (uint8_t) ~BIT2;
        }
    }
    else if (line == 2U)
    {
        if (level_high != 0U)
        {
            P3OUT |= BIT4;
        }
        else
        {
            P3OUT &= (uint8_t) ~BIT4;
        }
    }

    taskEXIT_CRITICAL();
}


void main_blinky( void )
{
    int error;

    /* g_last_sysrstiv was captured in main.c before SYSRSTIV auto-cleared.
     * Reading SYSRSTIV a second time here would always return 0. */
    bootCause = g_last_sysrstiv;

    if (bootCause <= SYSRSTIV_DOBOR)
    {
        /* power on reset, NMI(JTAG) reset or commanded (do BOR) reset - clear wd_count */
        wd_count = 0;

        if (bootCause <= SYSRSTIV_RSTNMI)
        {
            /* reset time */
            current_second = 0;
        }
    }

    ++bootCount;

    /* Create the event log mutex before any event_log_write() call so the
     * boot event from msp_self_test_run() is not silently dropped. */
    event_log_init();

    /* Power-On Self Test: probes I2C bus, sensors, WDT pin states.
     * Uses g_last_sysrstiv (captured in main.c) because reading SYSRSTIV
     * in main.c already cleared it before this function is reached. */
    msp_self_test_run(g_last_sysrstiv, bootCount);

    csp_sys_set_reboot(csp_reboot_function);

    error = csp_can_socketcan_open_and_add_interface("/dev/can", CSP_IF_CAN_DEFAULT_NAME, 0, false, &default_interface);
    if(error != CSP_ERR_NONE) {
        /* complain! */
        //csp_print("Error %d opening CAN interface", error);
    }

    csp_rtable_set(0, 0, default_interface, CSP_NO_VIA_ADDRESS);

	/* Create the queue. */
    xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( struct AppMessage ) );
    xCanQueue = xQueueCreate( 8, sizeof( struct AppMessage ) );

	if( xQueue != NULL )
	{
        /* Start the tasks. */
	    csp_route_start_task(configMINIMAL_STACK_SIZE, mainQUEUE_RECEIVE_TASK_PRIORITY+1);

		xTaskCreate( prvQueueReceiveTask,				/* The function that implements the task. */
					"Rx", 								/* The text name assigned to the task - for debug only as it is not used by the kernel. */
					configMINIMAL_STACK_SIZE, 			/* The size of the stack to allocate to the task. */
					NULL, 								/* The parameter passed to the task - not used in this case. */
					mainQUEUE_RECEIVE_TASK_PRIORITY, 	/* The priority assigned to the task. */
					NULL );								/* The task handle is not required, so NULL is passed. */

		xTaskCreate( prvClockTask, "CLK", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY+1, NULL );

        xTaskCreate( prvCanTask, "CN", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY+2, NULL ); // highest priority

        /* RP2350 liveness monitor (GPIO heartbeat + I2C heartbeat + escalation) */
        rp_liveness_task_create();

        /* Supervisor mode state machine (0.5 Hz) */
        mode_state_machine_task_create();

        /* Battery voltage/current (1 Hz) + temperature/heater (5 s) */
        battery_monitor_task_create();

        /* Ground contact processor + 48-hour deadman */
        ground_contact_task_create();

        /* MSP status bytes → RP (1 Hz) */
        msp_status_publish_task_create();

        /* RP fault counter reader (1 Hz) */
        fault_counter_update_task_create();

        /* Non-RF command dispatch + response collector (10 Hz) */
        rp_cmd_dispatch_task_create();

        /* FRAM event log flush (60 s) — also creates log mutex */
        event_log_flush_task_create();

        /* I2C device census (60 s) */
        i2c_census_task_create();

        /* MSP-local power rail GPIO policy (1 Hz) */
        power_policy_enforcer_task_create();

        /* Regulator PG monitor + recovery cycling (1 Hz) */
        reg_cycle_task_create();

        /* eFuse FLT monitor + recovery cycling / eps_current_limit_3v3 (1 Hz) */
        efuse_cycle_task_create();

        /* Hourly CRC32 memory scrub over code FRAM region */
        msp_memory_scrub_task_create();

        /* RP boot image confirmation / golden fallback manager (1 Hz) */
        boot_image_manager_task_create();

        /* EPS 3.3 V rail overcurrent monitor — INA219 @ 0x44 (2 Hz) */
        eps_3v3_monitor_task_create();

		/* Start the tasks and timer running. */
		vTaskStartScheduler();
	}

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was insufficient FreeRTOS heap memory available for the Idle and/or
	timer tasks to be created.  See the memory management section on the
	FreeRTOS web site for more details on the FreeRTOS heap
	http://www.freertos.org/a00111.html. */
	for( ;; );
}
/*-----------------------------------------------------------*/

static void prvClockTask( void *pvParameters )
{
TickType_t xNextWakeTime;
struct AppMessage msg;

	/* Remove compiler warning about unused parameter. */
	( void ) pvParameters;

	/* Initialise xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		/* Place this task in the blocked state until it is time to run again.
		 * Runs at 1 Hz; WDT_A timeout is ~1.05 s so kicking here gives ~1x margin.
		 * For the required 2 Hz service rate the kick is also done at the
		 * half-second point — see the second vTaskDelayUntil call below. */
		vTaskDelayUntil( &xNextWakeTime, pdMS_TO_TICKS( 500 ) );

		/* --- Half-second WDT kick --- */
		WDT_A_resetTimer( __MSP430_BASEADDRESS_WDT_A__ );

		vTaskDelayUntil( &xNextWakeTime, pdMS_TO_TICKS( 500 ) );

		/* --- Full second: kick again then do 1 Hz work --- */
		WDT_A_resetTimer( __MSP430_BASEADDRESS_WDT_A__ );

		++current_second;

		/* Advance FRAM-persistent Unix time counter (1 Hz tick) */
		timekeeper_tick_1hz();

		/* Advance fault counter decay timer (1 Hz tick) */
		fault_tick_1hz();

		/* Advance 48-hour deadman timer (1 Hz tick) */
		ground_contact_tick_1hz();

		/* Send to the queue - causing the queue receive task to unblock and
		toggle the LED.  0 is used as the block time so the sending operation
		will not block - it shouldn't need to block as the queue should always
		be empty at this point in the code. */
		msg.msgID = eTimer;
		msg.pvData = 0;
		xQueueSend( xQueue, &msg, 0U );

        /* Legacy 24-hour ground watchdog commented out.
         * The 48-hour deadman in ground_contact_tick_1hz() is authoritative.
        if(--wd_timeout <= 0)
        {
            ++wd_count;
            PMM_trigPOR();
        }
        */
  	}
}
/*-----------------------------------------------------------*/

static void prvQueueReceiveTask( void *pvParameters )
{
    struct AppMessage msg;
    csp_packet_t * packet;
    csp_socket_t *sock = csp_socket(CSP_SO_NONE);
    csp_conn_t *conn;
    uint32_t wdt1_counter = 0;
    uint32_t wdt2_counter = 0;
    uint32_t wdt_fault_blink_counter = 0;
    uint8_t wdt_fault_active = 0U;

    /* Remove compiler warning about unused parameter. */
    ( void ) pvParameters;

    /* Bind socket to all ports, e.g. all incoming connections will be handled here */
    csp_bind(sock, CSP_ANY);

    /* Create a backlog of 10 connections, i.e. up to 10 new connections can be queued */
    csp_listen(sock, 10);

    for( ;; )
    {
        /* Wait until something arrives in the queue - this task will block
        indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
        FreeRTOSConfig.h. */
        xQueueReceive( xQueue, &msg, portMAX_DELAY );

        /*  To get here something must have been received from the queue, but
        is it the expected value?  If it is, toggle the LED. */
        if( msg.msgID == eTimer )
        {
            if((tlm_period > 0) && (++tlm_counter >= tlm_period))
            {
                tlm_counter = 0;
                ADCtime = current_second;
#if 0   /* Init_ADC() is commented out — do not start conversion on uninitialised ADC */
                ADC12_B_startConversion(ADC12_B_BASE, ADC12_B_START_AT_ADC12MEM0, ADC12_B_SEQOFCHANNELS);
#endif
            }

            if(tlm_duration > 0)
            {
                /* countdown timer to revert to default tlm period */
                if(--tlm_duration == 0)
                {
                    /* timer expired */
                    tlm_period = DEFAULT_TLM_PERIOD;
                }
            }
            vParTestToggleLED( mainTASK_LED );
            vParTestToggleLED( mainTASK_LED_2 );

#if (WDT_FAULT_INJECT_ENABLE != 0U)
            if ((wdt_fault_active == 0U) && (++wdt_fault_blink_counter >= WDT_FAULT_AFTER_BLINKS))
            {
                wdt_fault_active = 1U;
                if (WDT_FAULT_LINE == 1U)
                {
                    wdt1_pulse_period_s = 0U;
                }
                else if (WDT_FAULT_LINE == 2U)
                {
                    wdt2_pulse_period_s = 0U;
                }
                prvForceWdtLine(WDT_FAULT_LINE, WDT_FAULT_LEVEL_HIGH);
            }
#endif

            /* Toggle each WDT output on its own configurable cadence. */
            if (wdt1_pulse_period_s == 0U)
            {
                wdt1_counter = 0;
            }
            else if (++wdt1_counter >= wdt1_pulse_period_s)
            {
                wdt1_counter = 0;
                prvPulseWdt1();
            }

            if (wdt2_pulse_period_s == 0U)
            {
                wdt2_counter = 0;
            }
            else if (++wdt2_counter >= wdt2_pulse_period_s)
            {
                wdt2_counter = 0;
                prvPulseWdt2();
            }
        }

        if( msg.msgID == eADC )
        {
            packet = csp_buffer_get(0);

            if(packet != NULL ) {
                /* copy ADCtime, followed by ADCdata */
                packet->data[0] = eAPP_TLM;
                packet->data[1] = 0; //OK
                packet->data[2] = MISSION_ID_0;
                packet->data[3] = MISSION_ID_1;
                packet->data[4] = 0;
                packet->data[5] = CSP_ID;
                memcpy(&(packet->data[6]), &ADCtime, 4);
                memcpy(&(packet->data[10]),ADCdata, 22);
                packet->length = 32;

                if((conn = csp_connect(CSP_PRIO_NORM, PC_CSP_ID, PC_BUFF_PORT, 0, CSP_SO_NONE)) != NULL) {
                    // send to PC, 3000ms timeout
                    if( csp_send(conn, packet, 3000) != CSP_ERR_NONE) {
                        csp_buffer_free(packet);
                    }
                    can_start_tx();
                    csp_close(conn);
                }
            }
        }

        /* Test for a new connection, 0 mS timeout */
        if ((conn = csp_accept(sock, 0)) != NULL) {
            /* handle connection */
            packet = csp_read(conn, 0);
            if (packet != NULL) {

                /* handle the CSP packet */
                ++rx_csp;

                switch(packet->id.dport){
                case CSP_TLM:
                    // CSP tlm request
                    handle_csp_tlm_request(conn,packet);
                    break;

                case CSP_TSYNC:
                    handle_csp_timesync(conn,packet);
                    break;

                case CSP_MSP_UPDATE:
                    handle_csp_msp_update(conn, packet);
                    break;

                default:
                    csp_service_handler(conn, packet);
                    break;
                }
                can_start_tx();
            }
            csp_close(conn);
        }
    }
}

static void prvCanTask( void *pvParameters )
{
    struct AppMessage msg;

    /* Remove compiler warning about unused parameter. */
    ( void ) pvParameters;

    for( ;; )
    {
        /* Wait until something arrives in the queue - this task will block
        indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
        FreeRTOSConfig.h. */
        xQueueReceive( xCanQueue, &msg, portMAX_DELAY /* pdMS_TO_TICKS( 100 )*/ );

        if (mcp2515_irq & MCP2515_IRQ_FLAGGED) {
            int i;
            //irq = can_irq_handler();
            while( (irq = can_irq_handler()) != 0) {
                if (irq & MCP2515_IRQ_RX && !(irq & MCP2515_IRQ_ERROR)) {
                    i = can_recv(&rid, &mext, buf);
                    if (i >= 0) {
                        if (mext) {

                            ridbuf[nirq++] = rid;
                            if(nirq>=4) {
                            nirq=0;
                            }

                            /* TODO: replace the following with CSP CAN packet handling when we have room */
                            /* Call RX callback */
                            //csp_can_rx(&ctx->iface, frame.can_id, frame.data, frame.can_dlc, NULL);
                            csp_can_rx(default_interface, rid, buf, i, NULL);
                        }
                    }
                } else if (irq & MCP2515_IRQ_TX && !(irq & MCP2515_IRQ_ERROR) ) {
                    /* successful transmit complete */
                    can_tx_next_packet(NULL);
                } else if (irq & MCP2515_IRQ_ERROR) {
                    can_r_reg(MCP2515_CANINTF, &mext, 1);
                    can_r_reg(MCP2515_EFLG, &eflag, 1);
                    // TODO  -assert? - reset? - handle error?
                }
            }
        }
        can_tx_next_packet(NULL);
    }
}

void can_start_tx(void)
{
    // TODO - replace with FreeRTOS task notification (plus task_woken)?
    xQueueSend( xCanQueue, &CANflag, 0U );
}


/**********************************************************************//**
 * @brief  ADC12 ISR
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
    uint8_t channel = 0;

  switch(__even_in_range(ADC12IV,12))
  {
    case ADC12IV_NONE: break;               // No interrupt
    case ADC12IV_ADC12OVIFG: break;         // conversion result overflow
    case ADC12IV_ADC12TOVIFG: break;        // conversion time overflow
    case ADC12IV_ADC12HIIFG: break;         // ADC12HI
    case ADC12IV_ADC12LOIFG: break;         // ADC12LO
    case ADC12IV_ADC12INIFG: break;         // ADC12IN
    //case ADC12IV_ADC12IFG0:
    case ADC12IV_ADC12IFG10:
        for(channel = 0; channel < 11; channel++)
        {
            /* 2*channel as reading 16-bit value with 8-bit offset */
            ADCdata[channel] = ADC12_B_getResults(ADC12_B_BASE, 2*channel);
        }
        ADCResult.msgID = eADC;
        /* now post result to xQueue */
        xQueueSendFromISR( xQueue, &ADCResult, 0U );
        __bic_SR_register_on_exit(CPUOFF);  //required?
        break;
    default: break;
  }
}


// ISR for PORT2
#pragma vector=PORT2_VECTOR
__interrupt void P2_ISR(void)
{
    //BaseType_t task_woken = 0;

    if (P2IFG & CAN_IRQ_PORTBIT) {
        P2IFG &= ~CAN_IRQ_PORTBIT;
        mcp2515_irq |= MCP2515_IRQ_FLAGGED;

        xQueueSendFromISR( xCanQueue, &CANflag, 0U );

        //__bic_SR_register_on_exit(LPM3_bits);
        __bic_SR_register_on_exit(CPUOFF);
    }
}


void handle_csp_tlm_request(csp_conn_t * conn, csp_packet_t * packet) {
    eTlmType packet_type = (eTlmType)packet->data[0];
    uint32_t tick = current_second;
    uint32_t scratch;

    switch(packet_type) {
    case eSOH_TLM:
        uptime = csp_get_uptime_s();
        tx_csp = (uint16_t)default_interface->tx;
        packet->data[0] = (uint8_t)packet_type;
        packet->data[1] = 0; //OK
        packet->data[2] = MISSION_ID_0;
        packet->data[3] = MISSION_ID_1;
        packet->data[4] = 0;
        packet->data[5] = CSP_ID;
        memcpy(&(packet->data[6]), &tick, sizeof(tick));
        memcpy(&(packet->data[10]),&bootCount, sizeof(bootCount));
        memcpy(&(packet->data[12]),&bootCause, sizeof(bootCause));
        memcpy(&(packet->data[14]),&uptime, sizeof(uptime));
        memcpy(&(packet->data[18]),&wd_timeout, sizeof(wd_timeout));
        memcpy(&(packet->data[22]),&wd_count, sizeof(wd_count));
        memcpy(&(packet->data[24]),&rx_csp, sizeof(rx_csp));
        memcpy(&(packet->data[26]),&tx_csp, sizeof(tx_csp));
        packet->length = 28;
        if (!csp_send(conn, packet, 0))
            csp_buffer_free(packet);
        break;

    case eAPP_TLM:
        // send application telemetry
        //memset();
        //bzero(tlm_pkt,MAX_TLM_PACKET_SIZE);
        packet->data[0] = (uint8_t)packet_type;
        packet->data[1] = 0; //OK
        packet->data[2] = MISSION_ID_0;
        packet->data[3] = MISSION_ID_1;
        packet->data[4] = 0;
        packet->data[5] = CSP_ID;
        memcpy(&(packet->data[6]), &ADCtime, 4);
        memcpy(&(packet->data[10]),ADCdata, 22);
        packet->length = 32;
        if (!csp_send(conn, packet, 0))
            csp_buffer_free(packet);
        break;

    case eVER_TLM:
        // send application version telemetry
        packet->data[0] = (uint8_t)packet_type;
        packet->data[1] = 0; //OK
        packet->data[2] = MISSION_ID_0;
        packet->data[3] = MISSION_ID_1;
        packet->data[4] = 0;
        packet->data[5] = CSP_ID;
        packet->data[6] = VER_MAJOR;
        packet->data[7] = VER_MINOR;
        packet->data[8] = VER_PATCH;
        packet->length = 9;
        if (!csp_send(conn, packet, 0))
            csp_buffer_free(packet);
        break;

    case eSET_RATE_TLM:
        // set application telemetry rate
        packet->data[0] = (uint8_t)packet_type;
        packet->length = 2;

        memcpy(&scratch,&(packet->data[1]),4);
        tlm_period = scratch;
        memcpy(&scratch,&(packet->data[5]),4);
        tlm_duration = scratch;
        packet->data[1] = 0; /* status = OK */

        if (!csp_send(conn, packet, 0))
             csp_buffer_free(packet);
        break;

    default:
        // unhandled tlm type - free packet.
        csp_buffer_free(packet);
        break;
    }

}

void handle_csp_timesync(csp_conn_t * conn, csp_packet_t * packet)
{
    switch(packet->data[0]) {

        case 0: {
            /* request time */
            int32_t now;
            //csp_timestamp_t ts;
            packet->data[1] = 0;
            //csp_clock_get_time(&ts);
            //now = htole32(ts.tv_sec);
            //now = csp_hton32(current_second);
            now = current_second;

            memcpy(&packet->data[2],&now, sizeof(int32_t));
            packet->length = 6;  /* cmd id + error code + time */
            if (!csp_send(conn, packet, 0))
                csp_buffer_free(packet);
        }
        break;

        case 1: {   /* set time */
            int32_t now;
            //csp_timestamp_t ts;
            memcpy(&now, &packet->data[1], sizeof(int32_t));
            //now = csp_ntoh32(now);
            current_second = now;
            /* Also update the FRAM-persistent Unix time with bounds check */
            (void)timekeeper_set((uint32_t)now);
            //ts.tv_sec = le32toh(now);
            //ts.tv_nsec = 0;
            packet->data[1] = 0; //csp_clock_set_time(&ts);
            packet->length = 2;
            if (!csp_send(conn, packet, 0))
                csp_buffer_free(packet);
        }
        break;

        case 2:     /* Last PPS time */
        case 3:     /* Get precise time */
        case 4:     /* Sync time request */
            csp_buffer_free(packet);
        break;

        default:
            csp_buffer_free(packet);
        break;
    }
}




