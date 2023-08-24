#pragma once

/**
   @file
   NA extensions to standard CSP services.
*/

/**
   Additional ports for CSP services.
*/
typedef enum {
    CSP_TLM             = 7,   //!< Telemetry request
    CSP_TSYNC           = 8,   //!< Time Sync - used to synchronise CSP node time.
	CSP_CONFIG			= 9,   //!< Configuration commands
	CSP_FSYS			= 10,  //!< File Transfer port
	CSP_CLI				= 13,  //!< Command Line Interface 
	CSP_TCMD            = 14,  //!< Telecommand
	CSP_IMG				= 19,  //!< Capture an image 
	CSP_BUFF0			= 20,  //!< CSP buffering slot 0
	CSP_BUFF1			= 21,  //!< CSP buffering slot 1
	CSP_BUFF2			= 22,  //!< CSP buffering slot 2
} csp_add_port_t;
/*  */


typedef enum {
	BUFF_IDLE			= 0,   //!< No buffer transfer in progress.
	BUFF_ACTIVE			= 1,   //!< Buffer transfer in progress
} csp_buffer_state_t;

typedef struct {
    csp_buffer_state_t state;
    uint16_t src_id;
    uint16_t dst_id;
    uint8_t  src_port;
    uint8_t  dst_port;
    uint8_t  flags;
    uint8_t  retry_count;
    uint32_t bytes_read;
    uint32_t total_bytes;
    uint32_t timeout;
    FILE    *fd;
} csp_buffer_session_t;


csp_buffer_session_t *get_buffer_session(uint32_t thing);
csp_buffer_session_t *find_buffer_session(csp_packet_t *packet);
void handle_buffer_pkt(csp_packet_t *packet);
void do_buffer_fsm(csp_conn_t *conn);
void init_buffer_sessions(void);

