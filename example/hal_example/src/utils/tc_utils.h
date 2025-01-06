#ifndef __TC_UTILS__
#define __TC_UTILS__


#include "stdlib.h"
#include "stdint.h"
#include "string.h"


#ifndef ABS
    #define ABS(_x) ((_x) < 0 ? -(_x) : (_x))
#endif

#ifndef MIN
    #define MIN(a, b) (a < b ? a : b)
#endif
#ifndef MAX
    #define MAX(a, b) (a > b ? a : b)
#endif



/**
 * @brief Return an random value of [0, max]
 * @param max - should be less than UINT32_MAX
 * @return Value from 0 ~ max
 */
uint32_t rand_within(uint32_t max);

/*
    Return value from min ~ max
*/
uint32_t rand_in_range(uint32_t min, uint32_t max);



/**
 * @brief Fill buffer with random data
 * @param p_buf - Pointer to buffer
 * @param size - buffer size
 */
void fill_with_random_data(uint8_t *p_buf, uint32_t size);


void dwtIpInit(void);
void dwtReset(void);
uint32_t dwtGetCycles(void);


#define ENTER_INTERRUPT() rt_interrupt_enter()            // RT-Thread requried interrupt enter function, other OS might be different
#define LEAVE_INTERRUPT() rt_interrupt_leave()            // RT-Thread requried interrupt leave function, other OS might be different

#endif /* __TC_UTILS__ */

