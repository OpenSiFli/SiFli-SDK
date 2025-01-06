/**
 ****************************************************************************************
 *
 * @file dbg_trc_tl.c
 *
 * @brief Tracer transport layer
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup TRACER
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"        // SW configuration

#if (TRACER_PRESENT)
#include <string.h>         // string definitions
#include "dbg_trc_int.h"     // tracer internal definitions
#include "dbg_trc.h"     // tracer definitions

#if (H4TL_SUPPORT)
    #include "h4tl.h"               // H4TL definitions
#endif // (H4TL_SUPPORT)

#include "ke_event.h"       // kernel event definition
#include "co_utils.h"


/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */
///Tracer TX states
enum dbg_trc_tx_state
{
    ///TRACER TX Trace Start State - when trace packet is ready to be sent
    TRACER_STATE_TX_TRC_ONGOING,
    ///TRACER TX Ack Start State - when acknowledgment packet is ready to be sent
    TRACER_STATE_TX_ACK_ONGOING,
    ///TRACER TX Idle State - No TX Pending
    TRACER_STATE_TX_IDLE
};

/*
 * STRUCTURES DEFINITIONS
 ****************************************************************************************
 */

///Tracer Environment context structure
struct dbg_trc_tl_env_tag
{
    /// Tx state - either transmitting or idle.
    uint8_t  tx_state;

    /// type of packet to send
    bool trc_ready;

    /// Store the received configuration word and the acknowledgment packet
    uint8_t trc_cfg_ack_buff[CHANNEL_ID_LEN + TRC_MSG_HDR_LEN + TRC_CFG_WORD_LEN];
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

///Tracer environment structure external global variable declaration
__STATIC struct dbg_trc_tl_env_tag dbg_trc_tl_env;

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#if (H4TL_SUPPORT)
    __STATIC void dbg_trc_tx_done(void);
#endif //(H4TL_SUPPORT)

/*
 * LOCAL FUNCTION DEFINITION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Check if a transmission has to be done and start it.
 *****************************************************************************************
 */
__STATIC void dbg_trc_tx_start(void)
{
    uint16_t len;
    uint8_t *buf;

    do
    {
        //check if an acknowledgment message has to be sent
        if (dbg_trc_tl_env.trc_ready == false)
        {
            //set the pointer to the trace packet type
            buf = dbg_trc_tl_env.trc_cfg_ack_buff;
            //move the pointer to the packet type
            buf += CHANNEL_ID_LEN;
            //compute the total length
            len = TRC_MSG_HDR_LEN + co_read16p(buf + TRC_OPCODE_LEN);

            dbg_trc_tl_env.tx_state = TRACER_STATE_TX_ACK_ONGOING;
        }
        //send the next trace otherwise
        else
        {
            //get the trace to be transmitted from the queue
            buf = dbg_trc_mem_read();

            if (buf == NULL)
            {
                break;
            }

            //get the total length of the trace
            len = co_read16p(buf);
            //remove channel ID length from total length because it is added by TL
            len -= CHANNEL_ID_LEN;
            //move the pointer to the beginning of the header
            buf += sizeof(len) + CHANNEL_ID_LEN;

            dbg_trc_tl_env.tx_state = TRACER_STATE_TX_TRC_ONGOING;
        }

#if (H4TL_SUPPORT)
        // Forward the trace to the H4TL for immediate transmission
        h4tl_write(TRC_MSG_TYPE, buf, len, &dbg_trc_tx_done);
#endif //(H4TL_SUPPORT)
    }
    while (0);
}

#if (H4TL_SUPPORT)
/**
 ****************************************************************************************
 * @brief Function called after sending message through UART push the next message for
 * transmission if any.
 *
 * The message is removed from memory if it is a trace message.
 *****************************************************************************************
 */
__STATIC void dbg_trc_tx_done(void)
{

    //If the sent packet is an acknowledgment packet
    if (dbg_trc_tl_env.tx_state == TRACER_STATE_TX_ACK_ONGOING)
    {
        //restore the transmission of traces
        dbg_trc_tl_env.trc_ready = true;
    }
    else
    {
        //Remove the trace from the memory
        dbg_trc_mem_dealloc();
    }

    dbg_trc_tl_env.tx_state = TRACER_STATE_TX_IDLE;

    // Check if there is a new message pending for transmission
    dbg_trc_tx_start();
}
#endif //(H4TL_SUPPORT)

/*
 * MODULES INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void dbg_trc_tx_trigger(void)
{
    // Check if there is no transmission ongoing
    if (dbg_trc_tl_env.tx_state == TRACER_STATE_TX_IDLE)
    {
        // Start the transmission
        dbg_trc_tx_start();
    }
}

void dbg_trc_tl_init()
{
    memset(&dbg_trc_tl_env, 0, sizeof(dbg_trc_tl_env));

    // Initialize TX state machine
    dbg_trc_tl_env.tx_state = TRACER_STATE_TX_IDLE;

    // Initialize type of message to send
    dbg_trc_tl_env.trc_ready = true;
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void dbg_trc_cfg_received()
{
    uint8_t *ack_buf;
    uint32_t cfg_word;

    cfg_word = co_read32p(dbg_trc_tl_env.trc_cfg_ack_buff);
    //Update the configuration word according to the supported traces
    dbg_trc_env.curr_cw = cfg_word & dbg_trc_env.compiled_cw;

    //Prepare the acknowledgment message
    ack_buf = dbg_trc_tl_env.trc_cfg_ack_buff;
    //leave space for the channel ID
    ack_buf += CHANNEL_ID_LEN;
    //fill the message
    *ack_buf++ = TRACER_ACK;
    co_write16p(ack_buf, TRC_CFG_WORD_LEN);
    ack_buf += 2;
    co_write32p(ack_buf, dbg_trc_env.curr_cw);

    //Set the next packet type to acknowledgment packet
    dbg_trc_tl_env.trc_ready = false;

    // trigger the event
    ke_event_set(KE_EVENT_TRC);
}

void dbg_trc_cfg_received_via_hci(uint32_t cfg)
{
    dbg_trc_env.curr_cw = cfg & dbg_trc_env.compiled_cw;
}

uint8_t *dbg_trc_pay_buff_get()
{
    return dbg_trc_tl_env.trc_cfg_ack_buff;
}

#endif /*(TRACER_PRESENT)*/
/// @} TRACER
