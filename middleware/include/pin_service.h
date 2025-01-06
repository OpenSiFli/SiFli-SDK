#ifndef PIN_SERVICE_H
#define PIN_SERVICE_H
#include <rtthread.h>
#include "data_service.h"
#include "drivers/pin.h"

enum
{
    PIN_MSG_START = MSG_SERVICE_CUSTOM_ID_BEGIN, //0x30

    /*****Request messages*****/
    PIN_MSG_VALUE_GET_REQ,
    PIN_MSG_VALUE_SET_REQ,
    PIN_MSG_ENABLE_IRQ_REQ,
    PIN_MSG_DISABLE_IRQ_REQ,
    PIN_MSG_DETACH_IRQ_REQ,


    /*****Response messages*****/
    PIN_MSG_VALUE_GET_RSP = RSP_MSG_TYPE | PIN_MSG_VALUE_GET_REQ,
    PIN_MSG_VALUE_SET_RSP = RSP_MSG_TYPE | PIN_MSG_VALUE_SET_REQ,
    PIN_MSG_ENABLE_IRQ_RSP = RSP_MSG_TYPE | PIN_MSG_ENABLE_IRQ_REQ,
    PIN_MSG_DISABLE_IRQ_RSP = RSP_MSG_TYPE | PIN_MSG_DISABLE_IRQ_REQ,
    PIN_MSG_DETACH_IRQ_RSP = RSP_MSG_TYPE | PIN_MSG_DETACH_IRQ_REQ,
};

typedef enum
{
    PIN_SERVICE_FLAG_SET_IRQ_MODE       = (1 << 0), //Set pin irq mode
    PIN_SERVICE_FLAG_AUTO_DISABLE_IRQ   = (1 << 1), //Auto disable irq when it happened.

} pin_service_flag;



typedef struct
{
    uint16_t id;               //pin id same as pin.h
    uint16_t flag;             //See pin_service_flag
    uint8_t mode;              //See pin.h PIN_MODE_XXX definition
    uint8_t irq_mode;          //See pin.h PIN_IRQ_MODE_XXX definition

} pin_config_msg_t;


typedef struct
{
    uint16_t id;               //pin id same as pin.h
    uint16_t value;            //Value
} pin_common_msg_t;


#endif  /* PIN_SERVICE_H */

