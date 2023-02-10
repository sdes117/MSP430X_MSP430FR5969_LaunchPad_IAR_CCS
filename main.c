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
 * This project provides two demo applications.  A simple blinky style project,
 * and a more comprehensive test and demo application.  The
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting (defined in this file) is used to
 * select between the two.  The simply blinky demo is implemented and described
 * in main_blinky.c.  The more comprehensive test and demo application is
 * implemented and described in main_full.c.
 *
 * This file implements the code that is not demo specific, including the
 * hardware setup and standard FreeRTOS hook functions.
 *
 * ENSURE TO READ THE DOCUMENTATION PAGE FOR THIS PORT AND DEMO APPLICATION ON
 * THE http://www.FreeRTOS.org WEB SITE FOR FULL INFORMATION ON USING THIS DEMO
 * APPLICATION, AND ITS ASSOCIATE FreeRTOS ARCHITECTURE PORT!
 *
 */

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Standard demo includes, used so the tick hook can exercise some FreeRTOS
functionality in an interrupt. */
#include "EventGroupsDemo.h"
#include "TaskNotify.h"
#include "ParTest.h" /* LEDs - a historic name for "Parallel Port". */

/* TI includes. */
#include "driverlib.h"

/* CAN includes */
#include <msp430.h>
#include "mcp2515.h"

#include "mission.h"

/* Set mainCREATE_SIMPLE_BLINKY_DEMO_ONLY to one to run the simple blinky demo,
or 0 to run the more comprehensive test and demo application. */
#define mainCREATE_SIMPLE_BLINKY_DEMO_ONLY	1

/*-----------------------------------------------------------*/

/*
 * Configure the hardware as necessary to run this demo.
 */
static void prvSetupHardware( void );

/*
 * main_blinky() is used when mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 1.
 * main_full() is used when mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 0.
 */
#if( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY == 1 )
	extern void main_blinky( void );
#else
	extern void main_full( void );
#endif /* #if mainCREATE_SIMPLE_BLINKY_DEMO_ONLY == 1 */

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file. */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );

/* The heap is allocated here so the "persistent" qualifier can be used.  This
requires configAPPLICATION_ALLOCATED_HEAP to be set to 1 in FreeRTOSConfig.h.
See http://www.freertos.org/a00111.html for more information. */
#ifdef __ICC430__
	__persistent 					/* IAR version. */
#else
	#pragma PERSISTENT( ucHeap ) 	/* CCS version. */
#endif
uint8_t ucHeap[ configTOTAL_HEAP_SIZE ] = { 0 };

uint8_t buf2[16];

/*-----------------------------------------------------------*/

int main( void )
{
	/* See http://www.FreeRTOS.org/MSP430FR5969_Free_RTOS_Demo.html */

	/* Configure the hardware ready to run the demo. */
	prvSetupHardware();

	/* The mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting is described at the top
	of this file. */
	#if( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY == 1 )
	{
		main_blinky();
	}
	#else
	{
		main_full();
	}
	#endif

	return 0;
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.
	See http://www.freertos.org/Stacks-and-stack-overflow-checking.html */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    __bis_SR_register( LPM4_bits + GIE );
    __no_operation();
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	#if( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY == 0 )
	{
		/* Call the periodic event group from ISR demo. */
		vPeriodicEventGroupsProcessing();

		/* Call the code that 'gives' a task notification from an ISR. */
		xNotifyTaskFromISR();
	}
	#endif
}
/* #define TMU (in project compile predefines) for TMU-specifc hardware configuration, otherwise configure for MSP_EXP430FR5739 experimenter board */
#ifdef TMU
/* The MSP430X port uses this callback function to configure its tick interrupt.
This allows the application to choose the tick interrupt source.
configTICK_VECTOR must also be set in FreeRTOSConfig.h to the correct
interrupt vector for the chosen tick interrupt source.  This implementation of
vApplicationSetupTimerInterrupt() generates the tick from timer A0, so in this
case configTICK_VECTOR is set to TIMER0_A0_VECTOR. */
void vApplicationSetupTimerInterrupt( void )
{
const unsigned long usACLK_Frequency_Hz = 4000000; /* (16MHz XT1)/4 */

    /* Ensure the timer is stopped. */
    TA0CTL = 0;

    /* Run the timer from the ACLK. */
    TA0CTL = TASSEL_1;

    /* Clear everything to start with. */
    TA0CTL |= TACLR;

    /* Set the compare match value according to the tick rate we want. */
    TA0CCR0 = usACLK_Frequency_Hz / configTICK_RATE_HZ;

    /* Enable the interrupts. */
    TA0CCTL0 = CCIE;

    /* Start up clean. */
    TA0CTL |= TACLR;

    /* Up mode. */
    TA0CTL |= MC_1;
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
    uint8_t i;

    /* Stop Watchdog timer. */
    WDT_A_hold( __MSP430_BASEADDRESS_WDT_A__ );

    /* Set required GPIO pins to output and low. */
    GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN6 | GPIO_PIN7 ); /* I2C */
    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7 );
    // no port 4: GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    //GPIO_setOutputLowOnPin( GPIO_PORT_PJ, GPIO_PIN0 );

    GPIO_setAsOutputPin( GPIO_PORT_P1, GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P3, GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    // GPIO_setAsOutputPin( GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    // VGL - not needed?: GPIO_setAsOutputPin( GPIO_PORT_PJ, GPIO_PIN0  );

    //GPIO_setAsInputPin( GPIO_PORT_PJ, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 ); /* JTAG inputs */

    /* Configure P2.0 - UCA0TXD and P2.1 - UCA0RXD. - TODO: configure as SPI1 I/O for dragsail MC */
    GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_P2, GPIO_PIN1, GPIO_SECONDARY_MODULE_FUNCTION );
    GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION );

    /* Configure P2.5 - UCA1SIMO and P2.6 - UCA1SOMI. */
    GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_P2, GPIO_PIN4, GPIO_SECONDARY_MODULE_FUNCTION );
    GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_P2, GPIO_PIN5, GPIO_SECONDARY_MODULE_FUNCTION );
    GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P2, GPIO_PIN6, GPIO_SECONDARY_MODULE_FUNCTION );

    /* Set PJ.4 and PJ.5 for HFXT. */
    GPIO_setAsPeripheralModuleFunctionInputPin(  GPIO_PORT_PJ, GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION  );

    /* set analog input pins */
    /* A12 = P3.0 = LM335D */
    GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 , GPIO_TERNARY_MODULE_FUNCTION );

    /* Set DCO frequency to 8 MHz. */
    CS_setDCOFreq( CS_DCORSEL_0, CS_DCOFSEL_3 );

    /* Set external clock frequency to 16.000 MHz. */
    CS_setExternalClockSource( 16000000, 0 );

    /* Set ACLK = XT1 with freq divider 4 (gives 4MHz ACLK). */
    CS_initClockSignal( CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_4 );
    // (internal DCO): CS_initClockSignal( CS_ACLK, CS_VLOCLK_SELECT, CS_CLOCK_DIVIDER_1 );

    /* Set SMCLK = DCO with frequency divider of 1. */
    CS_initClockSignal( CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );

    /* Set MCLK = DCO with frequency divider of 1. */
    CS_initClockSignal( CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );

    /* Start XT1 with no time out. */
    CS_turnOnXT1( XT1DRIVE_3 );

    /* Disable the GPIO power-on default high-impedance mode. */
    PMM_unlockLPM5();

    //Set P1.0 - P1.5 as input pins.
    /*
     * Select Port 1
     * Set Pins 0 - 5 as input
     * Set Ternary module function
     */
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P1,
            GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5,
            GPIO_TERNARY_MODULE_FUNCTION);

    //Initialize the ADC10B Module
    /*
     * Base Address for the ADC10B Module
     * Use internal ADC10B bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC10_B_init(ADC10_B_BASE,
        ADC10_B_SAMPLEHOLDSOURCE_SC,
        ADC10_B_CLOCKSOURCE_ADC10OSC,
        ADC10_B_CLOCKDIVIDER_1);

    ADC10_B_enable(ADC10_B_BASE);

    /*
     * Base Address for the ADC10B Module
     * Sample/hold for 16 clock cycles
     * Enable Multiple Sampling
     */
    ADC10_B_setupSamplingTimer(ADC10_B_BASE,
        ADC10_B_CYCLEHOLD_256_CYCLES,
        ADC10_B_MULTIPLESAMPLESENABLE);

    //Configure the Memory Buffer
    /*
     * Base Address for the ADC10B Module
     * Use input A0
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC10_B_configureMemory(ADC10_B_BASE,
        ADC10_B_INPUT_A15,
        //ADC10_B_INPUT_A12,
        ADC10_B_VREFPOS_AVCC,
        ADC10_B_VREFNEG_AVSS);

    ADC10_B_clearInterrupt(ADC10_B_BASE,
        ADC10IE0);

    //Enable the Memory Buffer Interrupt
    ADC10_B_enableInterrupt(ADC10_B_BASE,
        ADC10IE0);

    __delay_cycles(160000);
    can_init();
    if (can_speed(1000000, 1, 1) < 0) {
        //P1OUT |= BIT0;
        //LPM4;
        // TODO - error handling? assert?
    }

    can_rx_setmask(0, 0x00F80000, 1); // csp dest id mask
    can_rx_setfilter(0, 0, ((uint32_t)CSP_ID << 19));
    can_rx_setfilter(0, 1, 0x0000000F);

    can_rx_setmask(1, 0xFFFFFF0F, 1);
    can_rx_setfilter(1, 0, 0x00000000);
    can_rx_setfilter(1, 1, 0x00000000);
    can_rx_setfilter(1, 2, 0x00000000);
    can_rx_setfilter(1, 3, 0x00000000);

    can_rx_mode(0, MCP2515_RXB0CTRL_MODE_RECV_STD_OR_EXT);
    //can_rx_mode(0, MCP2515_RXB0CTRL_MODE_RECV_ALL);

    can_ioctl(MCP2515_OPTION_LOOPBACK, 0);
    //can_ioctl(MCP2515_OPTION_ONESHOT, 1);

    for (i=0; i < 16; i++)
        can_r_reg(i * 16, buf2, 16);

}
/*-----------------------------------------------------------*/

#else

/* The MSP430X port uses this callback function to configure its tick interrupt.
This allows the application to choose the tick interrupt source.
configTICK_VECTOR must also be set in FreeRTOSConfig.h to the correct
interrupt vector for the chosen tick interrupt source.  This implementation of
vApplicationSetupTimerInterrupt() generates the tick from timer A0, so in this
case configTICK_VECTOR is set to TIMER0_A0_VECTOR. */
void vApplicationSetupTimerInterrupt( void )
{
const unsigned short usACLK_Frequency_Hz = 12500;

    /* Ensure the timer is stopped. */
    TA0CTL = 0;

    /* Run the timer from the ACLK. */
    TA0CTL = TASSEL_1;

    /* Clear everything to start with. */
    TA0CTL |= TACLR;

    /* Set the compare match value according to the tick rate we want. */
    TA0CCR0 = usACLK_Frequency_Hz / configTICK_RATE_HZ;

    /* Enable the interrupts. */
    TA0CCTL0 = CCIE;

    /* Start up clean. */
    TA0CTL |= TACLR;

    /* Up mode. */
    TA0CTL |= MC_1;
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
    /* Stop Watchdog timer. */
    WDT_A_hold( __MSP430_BASEADDRESS_WDT_A__ );

    /* Set all GPIO pins to output and low. */
    GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setOutputLowOnPin( GPIO_PORT_PJ, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 | GPIO_PIN8 | GPIO_PIN9 | GPIO_PIN10 | GPIO_PIN11 | GPIO_PIN12 | GPIO_PIN13 | GPIO_PIN14 | GPIO_PIN15 );
    GPIO_setAsOutputPin( GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_PJ, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 | GPIO_PIN8 | GPIO_PIN9 | GPIO_PIN10 | GPIO_PIN11 | GPIO_PIN12 | GPIO_PIN13 | GPIO_PIN14 | GPIO_PIN15 );

    /* Configure P2.0 - UCA0TXD and P2.1 - UCA0RXD. */
    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN0 );
    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 );
    GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P2, GPIO_PIN1, GPIO_SECONDARY_MODULE_FUNCTION );
    GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION );

    /* Set PJ.4 and PJ.5 for LFXT. */
    //GPIO_setAsPeripheralModuleFunctionInputPin(  GPIO_PORT_PJ, GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION  );

    /* Set DCO frequency to 8 MHz. */
    CS_setDCOFreq( CS_DCORSEL_0, CS_DCOFSEL_3 );

    /* Set external clock frequency to 32.768 KHz. */
    //CS_setExternalClockSource( 32768, 0 );

    /* Set ACLK = VLO with freq divider TBD. */
    // VGL CS_initClockSignal( CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal( CS_ACLK, CS_VLOCLK_SELECT, CS_CLOCK_DIVIDER_1 );

    /* Set SMCLK = DCO with frequency divider of 1. */
    CS_initClockSignal( CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );

    /* Set MCLK = DCO with frequency divider of 1. */
    CS_initClockSignal( CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );

    /* Start XT1 with no time out. */
    // VGL CS_turnOnLFXT( CS_LFXT_DRIVE_0 );

    /* Disable the GPIO power-on default high-impedance mode. */
    PMM_unlockLPM5();
}
#endif

/*-----------------------------------------------------------*/

int _system_pre_init( void )
{
    /* Stop Watchdog timer. */
    WDT_A_hold( __MSP430_BASEADDRESS_WDT_A__ );

    /* Return 1 for segments to be initialised. */
    return 1;
}


