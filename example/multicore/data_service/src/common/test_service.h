#ifndef _TEST_SERVICE_H_
#define _TEST_SERVICE_H_
#include "board.h"
#include "rtconfig.h"
#include "data_service.h"

enum
{
    MSG_SERVICE_TEST_DATA_REQ                = (MSG_SERVICE_CUSTOM_ID_BEGIN),
    MSG_SERVICE_TEST_DATA_RSP                = (MSG_SERVICE_TEST_DATA_REQ | RSP_MSG_TYPE),
    MSG_SERVICE_TEST_DATA2_IND               = ((MSG_SERVICE_TEST_DATA_REQ + 1) | RSP_MSG_TYPE),
};

typedef struct
{
    uint32_t data;
} test_service_data_rsp_t;

typedef struct
{
    uint32_t data;
} test_service_data_ntf_ind_t;

typedef struct
{
    uint32_t data;
} test_service_data2_ind_t;


#endif /* _TEST_SERVICE_H_ */

