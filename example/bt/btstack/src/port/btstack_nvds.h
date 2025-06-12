#ifndef BTSTACK_NVDS_H
#define BTSTACK_NVDS_H
#include <stdint.h>
int port_get_tag(void *context, uint32_t tag, uint8_t *buffer,
                 uint32_t buffer_size);
int port_store_tag(void *context, uint32_t tag, const uint8_t *data,
                   uint32_t data_size);
void port_delete_tag(void *context, uint32_t tag);
#endif