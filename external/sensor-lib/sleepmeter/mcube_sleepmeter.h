/**
 * @file slpmtr.h
 * @author Steven Chen (steven.chen@mcubemems.com)
 * @date 2015/12/09
 * @brief Sleep monitoring algorithm
 */
#ifndef SLPMTR_H
#define SLPMTR_H



/**
 * -----------------------------------------------------------------------------
 * Included headers
 * -----------------------------------------------------------------------------
 */
//#include <stdint.h>

/**
 * -----------------------------------------------------------------------------
 * Define  declarations
 * -----------------------------------------------------------------------------
 */
#define SLEEPMETER_ENABLE_FP_ALGORITHM
/**
* -----------------------------------------------------------------------------
* Type declarations
* -----------------------------------------------------------------------------
*/

typedef enum slpmtr_sens
{
    SLPMTR_SENS_LOW = 0,                        /**< Low sensitivity. */
    SLPMTR_SENS_MEDIUM,                         /**< Medium sensitivity. */
    SLPMTR_SENS_HIGH,                           /**< High sensitivity. */

    SLPMTR_SENS_END,
} slpmtr_sens_t;

typedef struct slpmtr_param
{
    slpmtr_sens_t sensitivity;                  /**< sensitivity of the library. */
#ifdef SLEEPMETER_ENABLE_FP_ALGORITHM
    int32_t idle_timeout;                         /**< Timeout for the idle status detection. (seconds) */
#else
    float idle_timeout;                         /**< Timeout for the idle status detection. (seconds) */
#endif
} slpmtr_param_t;

typedef union slpmtr_input
{
#ifdef SLEEPMETER_ENABLE_FP_ALGORITHM
    int32_t mpss[3];                              /**< Input in m/s^s. */
#else
    float mpss[3];                              /**< Input in m/s^s. */
#endif
} slpmtr_input_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * -----------------------------------------------------------------------------
 * External linkage variables
 * -----------------------------------------------------------------------------
 */

/**
 * -----------------------------------------------------------------------------
 * API declarations
 * -----------------------------------------------------------------------------
 */
/**
 * @brief Opens the sleepmeter libaray with the default setting.
 */
void mCubeSleep_open(void);

/**
 * @brief Opens the sleepmeter library with the sensitivity
 *
 * @param sensitivity The sensitivity of the sleepmeter libaray. It could be LOW
 *                    or MEDIUM or HIGH. Please refere the enumeration
 *                    slpmtr_sens in this header file. Higher sensitivity means
 *                    it is more sensitive when the libaray detecting the
 *                    activity.
 */
void mCubeSleep_open_with_sensitivity(const slpmtr_sens_t sensitivity);

/**
 * @brief Opens the sleepmeter libaray with the parameters.
 *
 * @param p_param Point to the structure of the sleepmeter parameter.
 */
void mCubeSleep_open_with_param(const slpmtr_param_t *p_param);

/**
 * @brief Closes the sleepmeter library.
 */
void mCubeSleep_close(void);

/**
 * @brief Determines the sleepmeter is opened or not.
 *
 * @return int8_t If it is opened, returns 1. Otherwise returns 0.
 */
unsigned char mCubeSleep_is_opened(void);

/**
 * @brief Detect the sleep status
 *
 * @param input Point to the input structure of the sleepmeter.
 * @param milliseconds The timestamp of the input in seconds.
 */
#ifdef SLEEPMETER_ENABLE_FP_ALGORITHM
void mCubeSleep_detect(const slpmtr_input_t *input, unsigned long milliseconds);
/*
*  @param seconds, seconds = 250 * N (N = number of input data)
*/
#else
void mCubeSleep_detect(const slpmtr_input_t *input, float seconds);
#endif

/**
 * @brief Get sleep status.
 *
 * @return uint8_t Returns the sleep status. 0 for awake, 1 for sleeping, 2 for
 *         restless, 3 for rollover and 4 for idle.
 */
unsigned char mCubeSleep_get_sleep_status(void);

/**
 * @brief Get the version.
 *
 * @return int32_t Return a 32 bits value it includes
 *         AlGOTITHM_MAJOR(4bits).MINOR(4bits).BUILD(4bits).Reserve(4bits).INTERFACE_MAJOR(4bits).MINOR(4bits).BUILD(4bits).Reserve(4bits)
 */
unsigned long mCubeSleep_get_version(void);



#ifdef __cplusplus
}
#endif


#endif /**< !MCUBE_SLEEPMETER_H */

