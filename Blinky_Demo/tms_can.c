/*
Cubesat Space Protocol - A small network-layer protocol designed for Cubesats
Copyright (C) 2012 GomSpace ApS (http://www.gomspace.com)
Copyright (C) 2012 AAUSAT3 Project (http://aausat3.space.aau.dk)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <csp/drivers/can_socketcan.h>

//#include <pthread.h>
#include <stdlib.h>
//#include <errno.h>

#include "FreeRTOS.h"
#include "queue.h"

#include <csp/csp.h>
#include <msp430.h>
#include "mcp2515.h"

typedef int pthread_t;

// CAN interface data, state, etc.
typedef struct {
    char name[CSP_IFLIST_NAME_MAX + 1];
    csp_iface_t iface;
    csp_can_interface_data_t ifdata;
    pthread_t rx_thread;
    int socket;
} can_context_t;

typedef struct {
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8];
} can_frame_t;


extern void can_start_tx(void);

void can_tx_next_packet(BaseType_t *ptask_woken);

int frame_tx_count = 0;

static QueueHandle_t cantxQueue = NULL;

static void socketcan_free(can_context_t * ctx) {

	if (ctx) {
		free(ctx);
	}
}

static void * socketcan_rx_thread(void * arg)
{
#if 0
	while (1) {
		/* Read CAN frame */
		struct can_frame frame;
		int nbytes = read(ctx->socket, &frame, sizeof(frame));
		if (nbytes < 0) {
			csp_log_error("%s[%s]: read() failed, errno %d: %s", __FUNCTION__, ctx->name, errno, strerror(errno));
			continue;
		}

		if (nbytes != sizeof(frame)) {
			csp_log_warn("%s[%s]: Read incomplete CAN frame, size: %d, expected: %u bytes", __FUNCTION__, ctx->name, nbytes, (unsigned int) sizeof(frame));
			continue;
		}

		/* Drop frames with standard id (CSP uses extended) */
		if (!(frame.can_id & CAN_EFF_FLAG)) {
			continue;
		}

		/* Drop error and remote frames */
		if (frame.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) {
			csp_log_warn("%s[%s]: discarding ERR/RTR/SFF frame", __FUNCTION__, ctx->name);
			continue;
		}

		/* Strip flags */
		frame.can_id &= CAN_EFF_MASK;

		/* Call RX callbacsp_can_rx_frameck */
		csp_can_rx(&ctx->iface, frame.can_id, frame.data, frame.can_dlc, NULL);
	}

	/* We should never reach this point */
	pthread_exit(NULL);
#endif
	return (NULL);
}


static int csp_can_tx_frame(void * driver_data, uint32_t id, const uint8_t * data, uint8_t dlc)
{
	if (dlc > 8) {
		return CSP_ERR_INVAL;
	}

	can_frame_t frame = {.can_id = id /*| CAN_EFF_FLAG*/,
                                  .can_dlc = dlc};
    memcpy(frame.data, data, dlc);

    xQueueSend( cantxQueue, &frame, pdMS_TO_TICKS( 100 ) );
    //TODO - Also kick CAN task intgo action (xQeueSend(xCanQueue? ...))

	return CSP_ERR_NONE;
}

void can_tx_next_packet(BaseType_t *ptask_woken)
{
    can_frame_t frame;

    BaseType_t got_msg;

    if(can_tx_available() == 0 ) {

        if(ptask_woken){
            got_msg =  xQueueReceiveFromISR( cantxQueue, &frame, ptask_woken);
        } else {
            got_msg = xQueueReceive( cantxQueue, &frame, 0 /*pdMS_TO_TICKS( 300 )*/ );
        }

        if(got_msg == pdPASS ){
            /* got a message to send */
            if(can_send( frame.can_id, 1, (void *)(frame.data), frame.can_dlc, 3) != -1) {
                frame_tx_count++;
                //can_start_tx();
            }
        }
    }
}


int csp_can_socketcan_open_and_add_interface(const char * device, const char * ifname, int bitrate, bool promisc, csp_iface_t ** return_iface)
{
	if (ifname == NULL) {
		ifname = CSP_IF_CAN_DEFAULT_NAME;
	}

	can_context_t * ctx = calloc(1, sizeof(*ctx));
	if (ctx == NULL) {
		return CSP_ERR_NOMEM;
	}

	/* create tx message queue */
	cantxQueue = xQueueCreate( 8, sizeof( can_frame_t ) );

	ctx->socket = -1;

	strncpy(ctx->name, ifname, sizeof(ctx->name) - 1);
	ctx->iface.name = ctx->name;
	ctx->iface.interface_data = &ctx->ifdata;
	ctx->iface.driver_data = ctx;
	ctx->ifdata.tx_func = csp_can_tx_frame;

	/* Add interface to CSP */
        int res = csp_can_add_interface(&ctx->iface);
	if (res != CSP_ERR_NONE) {
		//csp_log("%s[%s]: csp_can_add_interface() failed, error: %d", __FUNCTION__, ctx->name, res);
		socketcan_free(ctx);
		return res;
	}

#if 0
	/* Create receive thread */
	if (pthread_create(&ctx->rx_thread, NULL, socketcan_rx_thread, ctx) != 0) {
		csp_log_error("%s[%s]: pthread_create() failed, error: %s", __FUNCTION__, ctx->name, strerror(errno));
		//socketcan_free(ctx); // we already added it to CSP (no way to remove it)
		return CSP_ERR_NOMEM;
	}
#endif

	if (return_iface) {
		*return_iface = &ctx->iface;
	}

	return CSP_ERR_NONE;
}

csp_iface_t * csp_can_socketcan_init(const char * device, int bitrate, bool promisc)
{
	csp_iface_t * return_iface;
	int res = csp_can_socketcan_open_and_add_interface(device, CSP_IF_CAN_DEFAULT_NAME, bitrate, promisc, &return_iface);
	return (res == CSP_ERR_NONE) ? return_iface : NULL;
}

int csp_can_socketcan_stop(csp_iface_t *iface)
{
	can_context_t * ctx = iface->driver_data;

    socketcan_free(ctx);
	return CSP_ERR_NONE;
}

/* hooks! - TODO - move */

void csp_reboot_hook(void) {
}

void csp_shutdown_hook(void) {
}

size_t  strnlen(const char *string, int n) {
    return (strlen(string) > n ? n :strlen(string));
}

