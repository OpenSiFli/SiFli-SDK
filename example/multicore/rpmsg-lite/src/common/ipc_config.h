#ifndef _IPC_CONFIG_H_
#define _IPC_CONFIG_H_
#include "board.h"



#define RPMSG_BUF_ADDR_REMOTE         (LPSYS_RAM_END - LPSYS_MBOX_BUF_SIZE - RPMSG_BUF_SIZE + 1)
#define RPMSG_BUF_ADDR_MASTER         (LCPU_ADDR_2_HCPU_ADDR(RPMSG_BUF_ADDR_REMOTE))

#define MASTER_EPT_ADDR               (30U)
#define REMOTE_EPT_ADDR               (40U)


#define RPMSG_LITE_LINK_ID  (0)

typedef struct
{
    uint32_t src;
    void *data;
    uint32_t len;
    void *priv;
} rpmsg_rx_cb_data_t;

#endif /* _IPC_CONFIG_H_ */

