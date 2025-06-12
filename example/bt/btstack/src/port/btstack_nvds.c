#include <bf0_sibles_nvds.h>
#include <rtthread.h>

/**
 * Get Value for Tag
 * @param context
 * @param tag
 * @param buffer
 * @param buffer_size
 * @return size of value
 */
int port_get_tag(void *context, uint32_t tag, uint8_t *buffer,
                 uint32_t buffer_size)
{
    char key[9] = {0};
    rt_sprintf(key, "%x", tag);
    key[8] = 0;
    sifli_nvds_flash_read(key, buffer, buffer_size);
}

/**
 * Store Tag
 * @param context
 * @param tag
 * @param data
 * @param data_size
 * @return 0 on success
 */
int port_store_tag(void *context, uint32_t tag, const uint8_t *data,
                   uint32_t data_size)
{
    char key[9] = {0};
    rt_sprintf(key, "%x", tag);
    key[8] = 0;
    sifli_nvds_flash_write(key, data, data_size);
}

/**
 * Delete Tag
 *  @note it is not expected that delete operation fails, please use at least
 * log_error in case of errors
 * @param context
 * @param tag
 */
void port_delete_tag(void *context, uint32_t tag)
{
    char key[9] = {0};
    rt_sprintf(key, "%x", tag);
    key[8] = 0;
    sifli_nvds_flash_delete(key);
}