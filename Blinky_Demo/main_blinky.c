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
 * The queue send task is implemented by the prvQueueSendTask() function in
 * this file.  prvQueueSendTask() sits in a loop that causes it to repeatedly
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

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue.  The 200ms value is converted
to ticks using the portTICK_PERIOD_MS constant. */
#define mainQUEUE_SEND_FREQUENCY_MS			( pdMS_TO_TICKS( 500 ) )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 16 )

/* The LED toggled by the Rx task. */
#define mainTASK_LED						( 0 )

/*-----------------------------------------------------------*/
/* serial gubbins */
/* Dimensions the buffer into which input characters are placed. */
#define cmdMAX_INPUT_SIZE       50

/* Dimensions a buffer to be used by the UART driver, if the UART driver uses a
buffer at all. */
#define cmdQUEUE_LENGTH         64

/* DEL acts as a backspace. */
#define cmdASCII_DEL        ( 0x7F )

/* The maximum time to wait for the mutex that guards the UART to become
available. */
#define cmdMAX_MUTEX_WAIT       pdMS_TO_TICKS( 300 )

#ifndef configCLI_BAUD_RATE
    #define configCLI_BAUD_RATE 115200
#endif

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
    eAPP_TLM = 1
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

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );
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
uint8_t bootCause;
uint32_t uptime= 0;
uint32_t wd_timeout = 100;
uint16_t wd_count = 0;
uint16_t rx_csp = 0;
uint16_t tx_csp = 0;



csp_iface_t * default_interface = NULL;
csp_conn_t *conn = NULL;

uint8_t tlm_pkt[MAX_TLM_PACKET_SIZE] = {0,};


/*-----------------------------------------------------------*/

void main_blinky( void )
{
    int error;
    csp_conf_t conf;

    csp_conf_get_defaults(&conf);

    conf.address = CSP_ID;
    conf.hostname = "hostname";
    conf.model = "tmu";
    conf.revision = "revision";
    conf.conn_max = 2;
    conf.conn_queue_length = 4;
    conf.fifo_length = 4;
    conf.port_max_bind = 24;
    conf.rdp_max_window = 20;
    conf.buffers = 10;
    conf.buffer_data_size = 256;
    conf.conn_dfl_so = CSP_O_NONE;

    ++bootCount;

    /* initialise CSP */
    csp_init(&conf);

    error = csp_can_socketcan_open_and_add_interface("/dev/can", CSP_IF_CAN_DEFAULT_NAME, 0, false, &default_interface);
    if(error != CSP_ERR_NONE) {
        /* complain! */
        //csp_print("Error %d opening CAN interface", error);
    }

    csp_rtable_set(0, 0, default_interface, CSP_NO_VIA_ADDRESS);

	/* Create the queue. */
    //xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint32_t ) );
    xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( struct AppMessage ) );
    xCanQueue = xQueueCreate( 8, sizeof( struct AppMessage ) );

	if( xQueue != NULL )
	{
	    csp_route_start_task(configMINIMAL_STACK_SIZE,CSP_PRIO_NORM);

		/* Start the two tasks as described in the comments at the top of this
		file. */
		xTaskCreate( prvQueueReceiveTask,				/* The function that implements the task. */
					"Rx", 								/* The text name assigned to the task - for debug only as it is not used by the kernel. */
					configMINIMAL_STACK_SIZE, 			/* The size of the stack to allocate to the task. */
					NULL, 								/* The parameter passed to the task - not used in this case. */
					mainQUEUE_RECEIVE_TASK_PRIORITY, 	/* The priority assigned to the task. */
					NULL );								/* The task handle is not required, so NULL is passed. */

		xTaskCreate( prvQueueSendTask, "TX", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL );

        xTaskCreate( prvCanTask, "CN", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY+1, NULL ); // highest priority

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

static void prvQueueSendTask( void *pvParameters )
{
TickType_t xNextWakeTime;
struct AppMessage msg;

	/* Remove compiler warning about unused parameter. */
	( void ) pvParameters;

	/* Initialise xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		/* Place this task in the blocked state until it is time to run again. */
		vTaskDelayUntil( &xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS );

		/* Send to the queue - causing the queue receive task to unblock and
		toggle the LED.  0 is used as the block time so the sending operation
		will not block - it shouldn't need to block as the queue should always
		be empty at this point in the code. */
		msg.msgID = eTimer;
		msg.pvData = 0; //(void *)(xNextWakeTime/configTICK_RATE_HZ); /* nb: store time value, not pointer */
		xQueueSend( xQueue, &msg, 0U );

        if(msg.msgByte++ >= 30)
        {
            msg.msgByte = 0;
            ADCtime = (xNextWakeTime/configTICK_RATE_HZ); /* note rolls over at 1.x years! */
		    /* Enable and Start Sequence-of-Channel, Multiple Conversion Mode for channels 15-0: */
            ADC12_B_startConversion(ADC12_B_BASE, ADC12_B_START_AT_ADC12MEM0, ADC12_B_SEQOFCHANNELS);
        }

	}
}
/*-----------------------------------------------------------*/

static void prvQueueReceiveTask( void *pvParameters )
{
    struct AppMessage msg;
    csp_packet_t * packet;
    csp_socket_t *sock = csp_socket(CSP_SO_NONE);
    csp_conn_t *conn;

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
            vParTestToggleLED( mainTASK_LED );
            //ADCtime = (uint16_t)msg.pvData;
        }

        if( msg.msgID == eADC )
        {
            packet = csp_buffer_get(0);

            if(packet != NULL ) {
                /* copy ADCtime, followed by ADCdata */
                packet->data[0] = 'T';
                packet->data[1] = 'P';
                packet->data[2] = 'A';
                packet->data[3] = '1';
                memcpy(&(packet->data[4]), &ADCtime, 4);
                packet->data[8] = (uint8_t)eAPP_TLM;
                memcpy(&(packet->data[9]),ADCdata, 32);
                packet->length = 41;

                if((conn = csp_connect(CSP_PRIO_NORM, PC_CSP_ID, PC_BUFF_PORT, 0, CSP_SO_NONE)) != NULL) {
                    // send to PC, 3000ms timeout
                    if( csp_send(conn, packet, 3000) != CSP_ERR_NONE) {
                        csp_buffer_free(packet);
                    }
                    can_tx_next_packet(NULL);
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
                case 7:
                    // CSP tlm request
                    handle_csp_tlm_request(conn,packet);
                    break;

                    case 9:
                        // todo
                        csp_buffer_free(packet);
                        break;

                    default:
                        csp_service_handler(conn, packet);
                        break;
                }
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
                if (irq & MCP2515_IRQ_RX /*&& !(irq & MCP2515_IRQ_ERROR)*/) {
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
                } else if (irq & MCP2515_IRQ_TX /*&& !(irq & MCP2515_IRQ_ERROR)*/ ) {
                    /* successful transmit complete */
                    can_tx_next_packet(NULL);
                } else if (irq & MCP2515_IRQ_ERROR) {
                    can_r_reg(MCP2515_CANINTF, &mext, 1);
                    can_r_reg(MCP2515_EFLG, &eflag, 1);
                    // TODO  -assert? - reset? - handle error?
                }
            }
        }

        if(can_tx_available() == 0 ) {
            /* able to transmit pending packet */
            can_tx_next_packet(NULL);

        }
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
    case ADC12IV_ADC12IFG15:
        for(channel = 0; channel < 16; channel++)
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
#if 0 /*(moved back to main loop) */
//while (mcp2515_irq & MCP2515_IRQ_FLAGGED) {
        if (mcp2515_irq & MCP2515_IRQ_FLAGGED) {
            int i;
            irq = can_irq_handler();
            if (irq & MCP2515_IRQ_RX /*&& !(irq & MCP2515_IRQ_ERROR) */) {
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
                        csp_can_rx(default_interface, rid, buf, (uint8_t)i, &task_woken);
                        rxcount++;
                    }
                }
            } else if (irq & MCP2515_IRQ_TX && !(irq & MCP2515_IRQ_ERROR) ) {
                /* successful transmit complete */
                can_tx_next_packet(&task_woken);
                txcount++;
            } else if ( irq & MCP2515_CANINTF_WAKIF) {
                // TODO - what?
                wakecount++;
            } else if (irq & MCP2515_IRQ_ERROR) {
                can_r_reg(MCP2515_CANINTF, &mext, 1);
                can_r_reg(MCP2515_EFLG, &eflag, 1);
                // TODO  -assert?
                errorcount++;
            }
        }

        if ( !(mcp2515_irq & MCP2515_IRQ_FLAGGED) ) {
            //P1OUT ^= BIT0;
            // TODO: LPM3;
        }
#endif

        //__bic_SR_register_on_exit(LPM3_bits);
        __bic_SR_register_on_exit(CPUOFF);
    }
}


void handle_csp_tlm_request(csp_conn_t * conn, csp_packet_t * packet) {
    eTlmType packet_type = (eTlmType)packet->data[0];
    uint32_t tick = csp_get_s();

    switch(packet_type) {
    case eSOH_TLM:
        uptime = csp_get_uptime_s();
        tx_csp = (uint16_t)default_interface->tx;
        packet->data[0] = (uint8_t)packet_type;
        packet->data[1] = 'T';
        packet->data[2] = 'P';
        packet->data[3] = 'A';
        packet->data[4] = '1';
        packet->data[5] = 0; //OK
        memcpy(&(packet->data[6]), &tick, sizeof(tick));
        memcpy(&(packet->data[10]),&bootCount, sizeof(bootCount));
        packet->data[12] = bootCause;
        memcpy(&(packet->data[13]),&uptime, sizeof(uptime));
        memcpy(&(packet->data[17]),&wd_timeout, sizeof(wd_timeout));
        memcpy(&(packet->data[21]),&wd_count, sizeof(wd_count));
        memcpy(&(packet->data[23]),&rx_csp, sizeof(rx_csp));
        memcpy(&(packet->data[25]),&tx_csp, sizeof(tx_csp));
        packet->length = 27;
        if (!csp_send(conn, packet, 0))
            csp_buffer_free(packet);
        break;

    case eAPP_TLM:
        // send application telemetry
        //memset();
        //bzero(tlm_pkt,MAX_TLM_PACKET_SIZE);
        packet->data[0] = (uint8_t)packet_type;
        packet->data[1] = 'T';
        packet->data[2] = 'P';
        packet->data[3] = 'A';
        packet->data[4] = '1';
        packet->data[5] = 0; //OK
        memcpy(&(packet->data[6]), &ADCtime, 4);
        memcpy(&(packet->data[10]),ADCdata, 32);
        packet->length = 42;
        if (!csp_send(conn, packet, 0))
            csp_buffer_free(packet);
        break;

    default:
        // unhandled tlm type - free packet.
        csp_buffer_free(packet);
        break;
    }

}



