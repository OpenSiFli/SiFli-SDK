/*********************
 *      INCLUDES
 *********************/
#ifndef _APP_MESSAGE_H
#define _APP_MESSAGE_H

#include <stdint.h>

void app_message_data_init(void);
void app_message_set_app_name(const uint8_t *s);
void app_message_set_title(const uint8_t *s);
void app_message_set_content(const uint8_t *s);


#endif /* _APP_MESSAGE_H */


