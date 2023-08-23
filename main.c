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
#include "ref_a.h"

/* CAN includes */
#include <msp430.h>
#include "mcp2515.h"

#include <csp/csp.h>

#include "mission.h"

/* Set mainCREATE_SIMPLE_BLINKY_DEMO_ONLY to one to run the simple blinky demo,
or 0 to run the more comprehensive test and demo application. */
#define mainCREATE_SIMPLE_BLINKY_DEMO_ONLY	1

/*-----------------------------------------------------------*/

/*
 * Configure the hardware as necessary to run this demo.
 */
static void prvSetupHardware( void );
static void Init_GPIO(void);
static void Init_Clock(void);
static void Init_ADC(void);
static void Init_CSP(void);
static void Init_CAN(void);

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

/* The MSP430X port uses this callback function to configure its tick interrupt.
This allows the application to choose the tick interrupt source.
configTICK_VECTOR must also be set in FreeRTOSConfig.h to the correct
interrupt vector for the chosen tick interrupt source.  This implementation of
vApplicationSetupTimerInterrupt() generates the tick from timer A0, so in this
case configTICK_VECTOR is set to TIMER0_A0_VECTOR. */
void vApplicationSetupTimerInterrupt( void )
{
const unsigned long usCLK_Frequency_Hz = 4000000; /* (8MHz SMCLK)/2 */

    /* Ensure the timer is stopped. */
    TA0CTL = 0;

    /* Run the timer from SMCLK/2. */
    TA0CTL = TASSEL_2 | ID__2;

    /* Clear everything to start with. */
    TA0CTL |= TACLR;

    /* Set the compare match value according to the tick rate we want. */
    TA0CCR0 = usCLK_Frequency_Hz / configTICK_RATE_HZ;

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

    Init_GPIO();
    Init_Clock();
    Init_ADC();
    Init_CSP();
    Init_CAN();

}

static void Init_GPIO(void)
{
    /* Set required GPIO pins to output and low. */
    GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN6 | GPIO_PIN7 ); /* I2C */
    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN7 );
    GPIO_setOutputHighOnPin( GPIO_PORT_P2, GPIO_PIN0 ); /* UCA0TXD */
    GPIO_setOutputLowOnPin( GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7 );

    GPIO_setAsOutputPin( GPIO_PORT_P1, GPIO_PIN6 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN7 );
    GPIO_setAsOutputPin( GPIO_PORT_P3, GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7 );

    // P2.2 is MCP2510 CAN interrupt.
    GPIO_setAsInputPin( GPIO_PORT_P2, GPIO_PIN2 );
    //GPIO_setAsInputPin( GPIO_PORT_PJ, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 ); /* JTAG inputs */

    /* Configure P2.0 - UCA0TXD and P2.1 - UCA0RXD. - TODO: configure as SPI1 I/O for dragsail MC */
    GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_P2, GPIO_PIN1, GPIO_SECONDARY_MODULE_FUNCTION );
    GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P2, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION );

    /* Configure P2.5 - UCA1SIMO and P2.6 - UCA1SOMI. */
    GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_P2, GPIO_PIN4, GPIO_SECONDARY_MODULE_FUNCTION );
    GPIO_setAsPeripheralModuleFunctionOutputPin( GPIO_PORT_P2, GPIO_PIN5, GPIO_SECONDARY_MODULE_FUNCTION );
    GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P2, GPIO_PIN6, GPIO_SECONDARY_MODULE_FUNCTION );

    /* Set PJ.6 and PJ.7 for HFXT. */
    GPIO_setAsPeripheralModuleFunctionInputPin(  GPIO_PORT_PJ, GPIO_PIN6 + GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION  );

    /* set analog input pins */
    /* A12 = P3.0 = LM335D */
    GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 , GPIO_TERNARY_MODULE_FUNCTION );

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

    /* Disable the GPIO power-on default high-impedance mode. */
    PMM_unlockLPM5();
}

static void Init_Clock(void)
{
    /* Set DCO frequency to 8 MHz. */
    CS_setDCOFreq( CS_DCORSEL_0, CS_DCOFSEL_6 );

    /* Set external clock frequency to 16.000 MHz. */
    CS_setExternalClockSource( 0, 16000000 );

    /* Set ACLK = VLOCLK (gives ~10 kHz ACLK). */
    CS_initClockSignal( CS_ACLK, CS_VLOCLK_SELECT, CS_CLOCK_DIVIDER_1 );

    /* Set SMCLK = HFXT with frequency divider of 2 => 8MHz. */
    CS_initClockSignal( CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_2 );

    /* Set MCLK = HFXT with frequency divider of 2 => 8MHz. */
    CS_initClockSignal( CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_2 );

    /* Start HFXT with no time out. */
    CS_turnOnHFXT( CS_HFXT_DRIVE_16MHZ_24MHZ );
}

static void Init_ADC(void)
{
    /* initialise the reference voltage module */
    Ref_A_setReferenceVoltage(REF_A_BASE, REF_A_VREF1_2V);
    Ref_A_enableReferenceVoltage(REF_A_BASE);

    //Initialize the ADC12B Module
    /*
    * Base address of ADC12B Module
    * Use internal ADC12B bit as sample/hold signal to start conversion
    * USE MODOSC 5MHZ Digital Oscillator as clock source
    * Use default clock divider/pre-divider of 1
    * Not use internal channel
    */
    ADC12_B_initParam initParam = {0};
    initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
    initParam.clockSourceSelect = ADC12_B_CLOCKSOURCE_ADC12OSC;
    initParam.clockSourceDivider = ADC12_B_CLOCKDIVIDER_1;
    initParam.clockSourcePredivider = ADC12_B_CLOCKPREDIVIDER__1;
    initParam.internalChannelMap = ADC12_B_TEMPSENSEMAP | ADC12_B_BATTMAP;

    ADC12_B_init(ADC12_B_BASE, &initParam);

    //Enable the ADC12B module
    ADC12_B_enable(ADC12_B_BASE);

    /*
    * Base address of ADC12B Module
    * For memory buffers 0-7 sample/hold for 256 clock cycles
    * For memory buffers 8-15 sample/hold for 256 clock cycles (default)
    * Enable Multiple Sampling
    */
    ADC12_B_setupSamplingTimer(ADC12_B_BASE,
      ADC12_B_CYCLEHOLD_256_CYCLES,
      ADC12_B_CYCLEHOLD_256_CYCLES,
      ADC12_B_MULTIPLESAMPLESENABLE);

    //Configure Memory Buffer
    /*
    * Base address of the ADC12B Module
    * Configure memory buffer 0
    * Map inputs A0-A15 to memory buffers 0-15,
    * Vref+ = AVcc
    * Vref- = AVss
    * Memory buffer 15 is the end of a sequence
    */
    /* VGL: TODO - include internal temp reference, concatenate channels */
    ADC12_B_configureMemoryParam configureMemoryParam = {0};

    configureMemoryParam.refVoltageSourceSelect = ADC12_B_VREFPOS_AVCC_VREFNEG_VSS;
    configureMemoryParam.endOfSequence = ADC12_B_NOTENDOFSEQUENCE;
    configureMemoryParam.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    configureMemoryParam.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;

    configureMemoryParam.memoryBufferControlIndex = 0; /* 16-bit data */
    configureMemoryParam.inputSourceSelect = 0; /* A0 = 0.5Vcc */
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    configureMemoryParam.memoryBufferControlIndex = 2; /* 16-bit data */
    configureMemoryParam.inputSourceSelect = 3; /* T1 == J4 input (A3) */
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    configureMemoryParam.memoryBufferControlIndex = 4; /* 16-bit data */
    configureMemoryParam.inputSourceSelect = 4; /* T2 == J5 input (A4) */
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    configureMemoryParam.memoryBufferControlIndex = 6; /* 16-bit data */
    configureMemoryParam.inputSourceSelect = 5; /* T3 == J6 input (A5) */
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    configureMemoryParam.memoryBufferControlIndex = 8; /* 16-bit data */
    configureMemoryParam.inputSourceSelect = 6; /* T4 == J7 input (A6) */
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    configureMemoryParam.memoryBufferControlIndex = 10; /* 16-bit data */
    configureMemoryParam.inputSourceSelect = 13; /* T5 == J8 input (A13) */
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    configureMemoryParam.memoryBufferControlIndex = 12; /* 16-bit data */
    configureMemoryParam.inputSourceSelect = 14; /* T6 == J9 input (A14) */
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    configureMemoryParam.memoryBufferControlIndex = 14; /* 16-bit data */
    configureMemoryParam.inputSourceSelect = 15; /* T7 == J10 input (A15) */
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    configureMemoryParam.memoryBufferControlIndex = 16; /* 16-bit data */
    configureMemoryParam.inputSourceSelect = 12; /* T8 == external LM85 input (A12) */
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    configureMemoryParam.memoryBufferControlIndex = 18; /* 16-bit data */
    configureMemoryParam.inputSourceSelect = 30; /* T9 == Internal temperature (A30) */
    configureMemoryParam.refVoltageSourceSelect = ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS; /* use 1.2V Vref */
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    configureMemoryParam.endOfSequence = ADC12_B_ENDOFSEQUENCE;
    configureMemoryParam.memoryBufferControlIndex = 20; /* 16-bit data */
    configureMemoryParam.inputSourceSelect = 31; /* T10 == Internal AVCC/2 (A31) */
    configureMemoryParam.refVoltageSourceSelect = ADC12_B_VREFPOS_AVCC_VREFNEG_VSS;
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    //configureMemoryParam.endOfSequence = ADC12_B_ENDOFSEQUENCE;
   // configureMemoryParam.memoryBufferControlIndex = ADC12_B_MEMORY_15;
    //configureMemoryParam.inputSourceSelect = ADC12_B_INPUT_A15;
    //ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    ADC12_B_clearInterrupt(ADC12_B_BASE,
        0,
        ADC12_B_IFG10
        );

    //Enable memory buffer 0 interrupt
    ADC12_B_enableInterrupt(ADC12_B_BASE,
      ADC12_B_IE10,
      0,
      0);
}

static void Init_CSP(void)
{
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

    /* initialise CSP */
    csp_init(&conf);
}


static void Init_CAN(void)
{
    // Set up CAN
    can_init();
    if (can_speed(1000000, 1, 1) < 0) {
        //P1OUT |= BIT0;
        //LPM4;
        // TODO - error handling? assert?
    }

    can_rx_setmask(0, 0x00F80000, 1); // CSP Destination ID mask
    can_rx_setfilter(0, 0, ((uint32_t)CSP_ID << 19));
    can_rx_setfilter(0, 1, 0x00000000);

    can_rx_setmask(1, 0xFFFFFF0F, 1);
    can_rx_setfilter(1, 0, 0x00000000);
    can_rx_setfilter(1, 1, 0x00000000);
    can_rx_setfilter(1, 2, 0x00000000);
    can_rx_setfilter(1, 3, 0x00000000);

    // Extended frames only
    can_rx_mode(0, MCP2515_RXB0CTRL_MODE_RECV_EXT);

    can_ioctl(MCP2515_OPTION_LOOPBACK, 0);

}

/*-----------------------------------------------------------*/

int _system_pre_init( void )
{
    /* Stop Watchdog timer. */
    WDT_A_hold( __MSP430_BASEADDRESS_WDT_A__ );

    /* Return 1 for segments to be initialised. */
    return 1;
}


